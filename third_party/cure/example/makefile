include ../../config

DIR=../..

INCLUDE=`cure-config -I`
LIB=`cure-config -L`
LINK=`cure-config -l`

CC=gcc
CFLAGS=-ansi -Wall -O3 $(LIB) $(INCLUDE) -funroll-loops

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean :
	rm -f *.o *~ core *test
