PKG_NAME=libcsp
PKG_URL=https://github.com/pi76r/libcsp.git
PKG_VERSION=cb0888aa75bf8c6670bda64da14a722900010a39
PKG_LICENSE=LGPL

include $(RIOTBASE)/pkg/pkg.mk

COMMA=,
EMPTY=
SPACE=$(EMPTY) $(EMPTY)

CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -Wno-unused-parameter

all:
	rm -rf $(BINDIR)/src
	cd $(PKG_SOURCE_DIR) && python3 waf configure --toolchain=arm-none-eabi- --with-os=riotos --out=$(BINDIR)/$(PKG_NAME) \
	--includes=$(subst $(SPACE),$(COMMA),$(subst -I,,$(subst -isystem ,-I,$(INCLUDES)))) \
	--with-driver-can=riotos
	cd $(PKG_SOURCE_DIR) && python3 waf build
	mv $(BINDIR)/$(PKG_NAME)/src $(BINDIR)/src
	mv $(BINDIR)/$(PKG_NAME)/$(PKG_NAME).a $(BINDIR)/$(PKG_NAME).a
