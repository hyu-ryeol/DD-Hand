all:main client 
LDLIBS= -I/usr/local/include -L/usr/local/lib
LDFLAGS= -lPocoFoundation -lPocoUtil -lPocoNet -fpermissive

CC=g++
CFLAGS=-g -Wall

main: main.cpp
	$(CC) $(LDLIBS) -o $@ $< $(LDFLAGS)
client: client.cpp
	$(CC) $(LDLIBS) -o $@ $< $(LDFLAGS)
		
clean:
	rm main client 
