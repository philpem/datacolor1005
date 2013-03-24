CC=gcc
CFLAGS=-O -Wall -Wextra -std=gnu99 -g -ggdb
LDFLAGS=-lusb-1.0

.PHONY: all clean

all: spectro

clean:
	-rm -f spectro spectro.o

spectro:	spectro.o
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

