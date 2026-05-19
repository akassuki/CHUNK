CC      = gcc
CFLAGS  = -std=c99 -O2 -Wall -Wextra -Wpedantic
TARGET  = chunked
SRC     = main.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^
	@echo "Build OK → ./$(TARGET)"

clean:
	rm -f $(TARGET)
rebuild: clean all
