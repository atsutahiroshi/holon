# default suffices
# .z3d : 3D shape model
# .zrb : robot model
# .zri : robot initial state
# .zvs : vector sequence

include ../config

INCLUDE=-I. -I../include -I$(PREFIX)/include
LIB=-L../lib -L$(PREFIX)/lib
BINDIR=$(PREFIX)/bin

CC=gcc
CFLAGS=-Wall -O3 $(INCLUDE) $(LIB) -D__USE_GLEW

GLLINK=-lGLEW -lglut -lGLU -lGL

LINK=-lrokigl $(GLLINK) `zx11-config -l` `liw-config -l` `roki-config -l`

COMPILE=$(CC) $(CFLAGS) -o $@ $< $(LINK)

INSTALL=install -m 755

CONFIGGEN=cure-config-gen
CONFIG=rokigl-config
TARGET=$(CONFIG) rk_view rk_anim rk_seq rk_pen

#####################################################################
all: $(TARGET)

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

clean:
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
