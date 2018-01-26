include ../config

ROOTDIR=..
LIBDIR=$(ROOTDIR)/lib
INCLUDE=-I$(ROOTDIR)/include

CC=gcc
CFLAGS=-ansi -Wall -fPIC -O3 $(INCLUDE) -funroll-loops
LD=gcc
LDFLAGS=-shared
SIGNUP=echo "CURE ver."$(VERSION)" Copyright (C) 1998 Tomomichi Sugihara (Zhidao)" >>

OBJ=cure_misc.o\
	cure_string.o \
	cure_bit.o cure_rand.o\
	cure_index.o cure_stream.o cure_list.o cure_rrtab.o\
	cure_strlist.o\
	cure_option.o
DLIB=libcure.so

TARGET=$(DLIB)

all: $(TARGET)

$(DLIB): $(OBJ)
	@echo " LD	" $^
	@$(LD) $(LDFLAGS) -o $@ $^ > /dev/null; $(SIGNUP) $@
	-@mv $@ $(LIBDIR)
%.o: %.c
	@echo " CC	" $<
	@$(CC) $(CFLAGS) -c $< > /dev/null
clean:
	@echo " CLEAN"
	-@rm -f *.o *~ \#* $(TARGET)
