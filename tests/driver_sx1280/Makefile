BOARD ?= nucleo-f446re

include ../Makefile.tests_common

USEMODULE += shell od
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += xtimer
USEMODULE += random

DRIVER ?= sx1280

USEMODULE += $(DRIVER)

include $(RIOTBASE)/Makefile.include
