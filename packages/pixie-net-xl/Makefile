#
# PixieNet XL Package
#

# Comment to see the build steps
QUIET = @

#
# Collect the files relative to the path the collection is from.
#
collect = $(shell cd $(1) && find . -type f -printf "%P\n")

#o
# Where we are in our built tree
#
TOP	= ../..
BUILD	= $(TOP)/build-linux
SRC	= $(TOP)

EXE	= \
	pncontrol \
	pnkintex

CONF	= \
	defaults.ini

BFILE	=

SYSD	= \
	systemd/system/pncontrol.socket \
	systemd/system/pncontrol@.service
	systemd/system/pixie-net-init.service

prefix	= /usr
systemd	= /usr/lib/systemd/system

INSTALL	= install
MKDIR_P	= mkdir -p
RM_RF	= rm -rf

BIN	= bin
ETC	= etc
CFG	= $(ETC)/xia/pixie-net-xl
BITFILE	= $(CFG)/bitfiles
SYSTEMD = systemd

VAR	 = var

DIRS	= $(BIN) $(ETC) $(CFG) $(SYSTEMD) $(WWWDS)
EXES	= $(foreach E,$(EXE),$(BIN)/$(notdir $(E)))
CONFS	= $(foreach C,$(CONF),$(CFG)/$(notdir $(C)))
SYSDS	= $(foreach S,$(SYSD),$(SYSTEMD)/$(notdir $(S)))
BFILES	= $(foreach B,$(BFILE),$(BITFILES)/$(notdir $(B)))

all: $(DIRS) $(CONFS) $(EXES) $(SYSDS) $(BFILES)

install: \
	install-conf \
	install-bitfiles \
	install-exe \
	install-systemd

install-conf: $(CONFS)
	$(QUIET)echo "Installing configs"; \
	for f in $(foreach C,$^,$(notdir $(C))); \
	do \
	 $(INSTALL) -m 664 -D $(CFG)/$$f $(DESTDIR)/$(CFG)/$$f; \
	done

install-bitfiles: $(BFILES)
	$(QUIET)echo "Installing bitfiles"; \
	for f in $(foreach B,$^,$(notdir $(B))); \
	do \
	 $(INSTALL) -D $(BITFILES)/$$f $(DESTDIR)/$(NESITES)/$$f; \
	done

install-exe: $(EXES)
	$(QUIET)echo "Installing executables"; \
	for f in $(foreach E,$^,$(notdir $(E))); \
	do \
	 $(INSTALL) -s -D $(BIN)/$$f $(DESTDIR)$(prefix)/$(BIN)/$$f; \
	 strip $(DESTDIR)$(prefix)/$(BIN)/$$f; \
	done

install-systemd: $(SYSDS)
	$(QUIET)echo "Installing systemd"; \
	for f in $(foreach S,$^,$(notdir $(S))); \
	do \
	 $(INSTALL) -D $(SYSTEMD)/$$f $(DESTDIR)$(systemd)/$$f; \
	done

clean:
	-$(QUIET)echo "Clean"; \
	$(RM_RF) $(DIRS)

distclean: clean

#
# Template out the rules
#
define COPY
$3: $2/$1
	$(QUIET)echo "Copying $1 -> $3"; \
	mkdir -p `dirname $3`; \
	cp $2/$1 $3
endef
define MKDIR
$1:
	$(QUIET)echo "Make $1"; \
	$(MKDIR_P) $1
endef

$(foreach D,$(DIRS),$(eval $(call MKDIR,$(strip $(D)))))
$(foreach E,$(EXE),$(eval $(call COPY,$(strip $(E)),$(BUILD),$(BIN)/$(notdir $(E)))))
$(foreach C,$(CONF),$(eval $(call COPY,$(strip $(C)),$(SRC),$(CFG)/$(notdir $(C)))))
$(foreach C,$(CONFEXE),$(eval $(call COPY,$(strip $(C)),$(SRC),$(CFG)/$(notdir $(C)))))
$(foreach S,$(SYSD),$(eval $(call COPY,$(strip $(S)),$(SRC),$(SYSTEMD)/$(notdir $(S)))))
$(foreach W,$(WUIB), \
        $(foreach WW,$(call collect,$(SRC)/$(WUI)/$(W)), \
		$(eval $(call COPY,$(strip $(WW)),$(SRC)/$(WUI)/$(W),$(WWWDS)/$(WW)))))
$(foreach W,$(WUIS),$(eval $(call COPY,$(strip $(W)),$(SRC)/$(WUI),$(WWWDS)/$(notdir $(W)))))
$(foreach N,$(NSITE),$(eval $(call COPY,$(strip $(N)),$(SRC),$(WWWDS)/$(notdir $(N)))))
