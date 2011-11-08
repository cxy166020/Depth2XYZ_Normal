CC = g++
CFLAGS = -Wall -O2 -c
LDFLAGS = #


PROG = depth2xyz_normal

OBJS = main.o 

$(PROG) : $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

main.o : main.cpp
	$(CC) $(CFLAGS) $^ 

clean :
	rm *.o $(PROG)