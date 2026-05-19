/*
 * main.c
 *
 * Tool gửi 1 chunk patch file qua LoRa E32 (Fixed Transmission).
 * Tự lo hoàn toàn: chia file → build chunk → gửi → đọc ACK → retry.
 *
 * Cú pháp:
 *   ./chunked <port> <baud> <gw_addl> <node_addl> <ch> <file> <index>
 *
 *   port      : /dev/ttyUSB0
 *   baud      : 9600
 *   gw_addl   : low byte địa chỉ gateway   (0x01) — reserved, không dùng
 *   node_addl : low byte địa chỉ node đích (0x02)
 *   ch        : channel                    (0x17)
 *   file      : đường dẫn patch binary
 *   index     : chunk cần gửi (0-based)
 *
 * Exit code:
 *   0  — gửi & ACK OK     → Python: index + 1, tiếp tục
 *   1  — thất bại hẳn     → Python: có thể retry hoặc dừng
 *   2  — index >= total   → Python: file đã gửi xong
 *
 * ADDH cố định = 0x00
 */

#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

/* ─── Cấu hình ──────────────────────────────────────────────── */
#define FIXED_ADDH      0x00u
#define CHUNK_DATA_SIZE 50
#define MAX_RETRY       5
#define ACK_TIMEOUT_MS  5000
#define INTER_RETRY_MS  500

#define ACK_OK   0xAAu
#define ACK_NACK 0xFFu

/* ─── Structs khớp node ESP32 ───────────────────────────────── */
typedef struct __attribute__((packed)) {
    uint16_t index;
    uint16_t total;
    uint8_t  len;
    uint8_t  data[CHUNK_DATA_SIZE];
} LoraChunk;   /* 55 bytes */

typedef struct __attribute__((packed)) {
    uint8_t  status;
    uint16_t index;
} AckPkt;      /* 3 bytes */

_Static_assert(sizeof(LoraChunk) == 55, "LoraChunk must be 55 bytes");
_Static_assert(sizeof(AckPkt)    ==  3, "AckPkt must be 3 bytes");

/* ══════════════════════════════════════════════════════════════
 *  Thời gian
 * ══════════════════════════════════════════════════════════════ */
static long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* ══════════════════════════════════════════════════════════════
 *  Serial
 * ══════════════════════════════════════════════════════════════ */
static speed_t baud_const(int baud)
{
    switch (baud) {
        case 1200:   return B1200;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:
            fprintf(stderr, "[serial] baud %d unknown, dùng 9600\n", baud);
            return B9600;
    }
}

static int serial_open(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "[serial] open %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[serial] tcgetattr: %s\n", strerror(errno));
        close(fd); return -1;
    }

    speed_t spd = baud_const(baud);
    cfsetospeed(&tty, spd);
    cfsetispeed(&tty, spd);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;  /* 8-bit             */
    tty.c_cflag &= ~(PARENB | CSTOPB);             /* no parity, 1 stop */
    tty.c_cflag |=  (CLOCAL | CREAD);              /* local, rx enable  */
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);        /* no sw flow ctrl   */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK |
                     ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag  =  0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;   /* 100 ms per read() call */

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[serial] tcsetattr: %s\n", strerror(errno));
        close(fd); return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int write_all(int fd, const uint8_t *buf, size_t n)
{
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = write(fd, buf + sent, n - sent);
        if (r < 0) {
            fprintf(stderr, "[serial] write: %s\n", strerror(errno));
            return -1;
        }
        sent += (size_t)r;
    }
    tcdrain(fd);   /* đợi hardware TX xong */

    /*
     * Flush RX echo: USB-UART / E32 thường loop-back bytes vừa TX
     * vào RX buffer. Chờ 60ms cho echo về đủ rồi xả sạch, tránh
     * nhầm 3 bytes đầu của frame 58 bytes với AckPkt.
     */
    sleep_ms(60);
    tcflush(fd, TCIFLUSH);
    return 0;
}

static int read_timed(int fd, uint8_t *buf, size_t n, int timeout_ms)
{
    size_t got      = 0;
    long   deadline = now_ms() + timeout_ms;

    while (got < n) {
        if (now_ms() >= deadline) break;
        ssize_t r = read(fd, buf + got, n - got);
        if (r > 0)
            got += (size_t)r;
        else if (r < 0 && errno != EAGAIN) {
            fprintf(stderr, "[serial] read: %s\n", strerror(errno));
            break;
        }
    }
    return (int)got;
}

/* ══════════════════════════════════════════════════════════════
 *  File
 * ══════════════════════════════════════════════════════════════ */
static long file_size_bytes(FILE *f)
{
    if (fseek(f, 0, SEEK_END) != 0) return -1;
    long sz = ftell(f);
    rewind(f);
    return sz;
}

static uint16_t calc_total(long sz)
{
    if (sz <= 0) return 0;
    return (uint16_t)((sz + CHUNK_DATA_SIZE - 1) / CHUNK_DATA_SIZE);
}

/* ══════════════════════════════════════════════════════════════
 *  main
 * ══════════════════════════════════════════════════════════════ */
int main(int argc, char *argv[])
{
    /* ── 1. Arguments ───────────────────────────────────────── */
    if (argc != 8) {
        fprintf(stderr,
            "Usage: %s <port> <baud> <gw_addl> <node_addl> <ch> <file> <index>\n"
            "\n"
            "  port      : e.g. /dev/ttyUSB0\n"
            "  baud      : e.g. 9600\n"
            "  gw_addl   : hex e.g. 0x01  (ADDH=0x00 fixed)\n"
            "  node_addl : hex e.g. 0x02  (địa chỉ node đích)\n"
            "  ch        : hex e.g. 0x17\n"
            "  file      : patch binary\n"
            "  index     : 0-based chunk index\n"
            "\n"
            "Exit: 0=OK  1=FAIL  2=DONE\n",
            argv[0]);
        return 1;
    }

    const char *port      = argv[1];
    int         baud      = atoi(argv[2]);
    /* gw_addl argv[3] — không dùng trong frame, giữ để API rõ ràng */
    uint8_t     node_addl = (uint8_t)strtoul(argv[4], NULL, 0);
    uint8_t     ch        = (uint8_t)strtoul(argv[5], NULL, 0);
    const char *filepath  = argv[6];
    uint16_t    idx       = (uint16_t)strtoul(argv[7], NULL, 0);

    /* ── 2. File & chunk ────────────────────────────────────── */
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        fprintf(stderr, "[send] open file: %s\n", strerror(errno));
        return 1;
    }

    long sz = file_size_bytes(f);
    if (sz <= 0) {
        fprintf(stderr, "[send] file rỗng hoặc lỗi\n");
        fclose(f); return 1;
    }

    uint16_t total = calc_total(sz);

    if (idx >= total) {
        fprintf(stdout, "DONE total=%u\n", total);
        fflush(stdout);
        fclose(f);
        return 2;   /* Python: file đã xong */
    }

    LoraChunk chunk;
    memset(&chunk, 0, sizeof(chunk));

    if (fseek(f, (long)idx * CHUNK_DATA_SIZE, SEEK_SET) != 0) {
        fprintf(stderr, "[send] fseek fail\n");
        fclose(f); return 1;
    }
    size_t nr = fread(chunk.data, 1, CHUNK_DATA_SIZE, f);
    fclose(f);
    if (nr == 0) { fprintf(stderr, "[send] fread=0\n"); return 1; }

    chunk.index = idx;
    chunk.total = total;
    chunk.len   = (uint8_t)nr;

    /* ── 3. Serial ──────────────────────────────────────────── */
    int fd = serial_open(port, baud);
    if (fd < 0) return 1;

    /*
     * Fixed TX frame → node:
     *   [0x00][node_addl][ch]  +  LoraChunk (55 bytes)
     *   = 58 bytes total
     *
     * Khớp với node:
     *   e32.receiveMessage(sizeof(LoraChunk))  ← nhận đúng 55 bytes
     *   sendFixedMessage(GW_ADDH, GW_ADDL, ch, &ack, 3) ← gửi lại ACK
     */
    uint8_t frame[3 + sizeof(LoraChunk)];
    frame[0] = FIXED_ADDH;
    frame[1] = node_addl;
    frame[2] = ch;
    memcpy(frame + 3, &chunk, sizeof(LoraChunk));

    /* ── 4. Gửi + ACK loop ──────────────────────────────────── */
    int result = 1;

    for (int attempt = 0; attempt <= MAX_RETRY; attempt++) {

        if (attempt > 0) {
            fprintf(stderr, "[send] retry %d/%d chunk=%u\n",
                    attempt, MAX_RETRY, idx);
            sleep_ms(INTER_RETRY_MS);
        }

        /* Gửi frame (write_all tự flush echo sau khi TX) */
        if (write_all(fd, frame, sizeof(frame)) != 0)
            continue;

        /* Đọc AckPkt (3 bytes).
         * Node delay 300ms trước khi gửi ACK → timeout 5000ms đủ dư. */
        uint8_t ack_buf[3] = {0};
        int got = read_timed(fd, ack_buf, 3, ACK_TIMEOUT_MS);

        if (got < 3) {
            fprintf(stderr, "[send] ACK timeout (got %d bytes) chunk=%u attempt=%d\n",
                    got, idx, attempt);
            tcflush(fd, TCIFLUSH);   /* xả bất kỳ rác còn trong RX */
            continue;
        }

        fprintf(stderr, "[send] ACK raw: %02X %02X %02X\n",
                ack_buf[0], ack_buf[1], ack_buf[2]);

        AckPkt ack;
        memcpy(&ack, ack_buf, sizeof(ack));

        if (ack.status == ACK_OK) {
            fprintf(stdout, "OK idx=%u total=%u len=%u\n",
                    idx, total, chunk.len);
            fflush(stdout);
            result = 0;
            break;
        }

        if (ack.status == ACK_NACK) {
            fprintf(stderr, "[send] NACK chunk=%u attempt=%d\n", idx, attempt);
            continue;
        }

        fprintf(stderr, "[send] unknown status=0x%02X\n", ack.status);
    }

    close(fd);

    if (result != 0)
        fprintf(stderr, "[send] FAILED chunk=%u sau %d attempts\n",
                idx, MAX_RETRY + 1);

    return result;
}
