# single_chan_pkt_fwd
# Single Channel LoRaWAN Gateway

CC=gcc
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: single_chan_pkt_fwd

single_chan_pkt_fwd: main.o
	$(CC) main.o $(LIBS) -o single_chan_pkt_fwd

main.o: main.c
	$(CC) $(CFLAGS) main.c

clean:
	rm *.o single_chan_pkt_fwd	