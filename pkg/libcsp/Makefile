PKG_NAME=libcsp
PKG_URL=git@gricad-gitlab.univ-grenoble-alpes.fr:thingsat/libcsp.git
PKG_VERSION=17e0e860df9d8aec3fcb00f2f1b76a13acf793eb
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
