CFLAGS  = -g -O3 -Wall
SOURCES = wteledsk.c 

## advanced compression
CFLAGS  += -DDCOMP
SOURCES += tdlzhuf.c

## CRC check
CFLAGS  += -DCRC
SOURCES += tdcrc.c 

wteledsk: $(SOURCES) Makefile
	$(CC) $(CFLAGS) -o $@ $(filter %.c,$^)

clean:
	rm -f wteledsk *~
