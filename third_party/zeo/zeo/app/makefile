# default suffices
# .zph : 3D polyhedron
# .z3d : 3D shape

include ../config

INCLUDE=-I$(PREFIX)/include -I../include -I.
LIB=-L$(PREFIX)/lib -L../lib
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-ansi -Wall -O3 $(INCLUDE) $(LIB)

LINK=-lzeo `zm-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=cure-config-gen
CONFIG=zeo-config
TARGET=$(CONFIG) qhull gjk terra

#####################################################################
all : $(TARGET)

$(CONFIG):
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I $(PREFIX)/include -L $(PREFIX)/lib -l "$(LINK)" -v $(VERSION) > $@

%: %.c
	@echo " CC	" $<
	@$(COMPILE)
install: $(TARGET)
	@echo " INSTALL" $^
	@$(INSTALL) $^ $(BINDIR)

clean :
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET) *_out
