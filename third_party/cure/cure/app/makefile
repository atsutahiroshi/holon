include ../config

BINDIR=$(PREFIX)/bin

INSTALL=install -m 755

LINK=-lcure -lm

CONFIG=cure-config
CONFIGGEN=cure-config-gen
TARGET=$(CONFIG)

#####################################################################
all: install

$(CONFIG):
	@echo " INSTALL" $(CONFIGGEN)
	@$(INSTALL) $(CONFIGGEN) $(BINDIR)
	@echo " GENERATE" $@
	-@rm -f $@
	@$(CONFIGGEN) -I $(PREFIX)/include -L $(PREFIX)/lib -l "$(LINK)" -v $(VERSION) > $@

install: $(TARGET)
	@echo " INSTALL" $^
	@$(INSTALL) $^ $(BINDIR)

clean:
	@echo " CLEAN"
	-@rm -f *.o *~ core $(TARGET)
