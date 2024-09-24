################################################################################
# Project makefile
################################################################################

# Build tool settings
RM := rm
SHARC_CC := cc21k
SHARC_AS := easm21k
LD := elfloader

# Project settings
PROJECT ?= EV-21569-EZKIT
PROC = ADSP-21569
SI_REVISION = any
LDR = $(PROJECT).ldr

# retrieve the command-line release flag (default no release)
ifeq ($(RELEASE), yes)
	DEBUG =
	OPTIMIZE = -O1
else
	DEBUG = -g
	OPTIMIZE ?=
endif

# Export RELEASEfor submakes

################################################################################
# SHARC0 section
################################################################################

SRC_PREFIX = ..

# All of the SHARC0 sources participating in the build are defined here
SHARC0_SRC_DIRS = \
	SHARC0/src \
	SHARC0/src/simple-drivers \
	SHARC0/src/simple-services/adau1962 \
	SHARC0/src/simple-services/adau1979 \
	SHARC0/src/simple-services/a2b-xml \
	SHARC0/src/simple-services/adi-a2b-cmdlist \
	SHARC0/src/simple-services/a2b-to-sport-cfg \
	SHARC0/src/simple-services/FreeRTOS-cpu-load \
	SHARC0/src/simple-services/syslog \
	SHARC0/src/simple-services/fs-dev \
	SHARC0/src/oss-services/shell \
	SHARC0/src/oss-services/umm_malloc \
	SHARC0/src/oss-services/xmodem \
	SHARC0/src/oss-services/crc \
	SHARC0/src/oss-services/yxml \
	SHARC0/src/oss-services/spiffs \
	SHARC0/src/oss-services/FreeRTOS-SHARC \
	SHARC0/src/oss-services/FreeRTOS-SHARC/portable/MemMang \
	SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/osal \
	SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/SHARC_215xx \
	SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/SHARC_215xx/osal

# Include directories
SHARC0_INCLUDE_DIRS = \
	-I. \
	-I"../SHARC0/include" \
	-I"../SHARC0/src" \
	-I"../SHARC0/src/simple-drivers" \
	-I"../SHARC0/src/simple-services/adau1962" \
	-I"../SHARC0/src/simple-services/adau1979" \
	-I"../SHARC0/src/simple-services/a2b-xml" \
	-I"../SHARC0/src/simple-services/FreeRTOS-cpu-load" \
 	-I"../SHARC0/src/simple-services/adi-a2b-cmdlist" \
 	-I"../SHARC0/src/simple-services/a2b-to-sport-cfg" \
 	-I"../SHARC0/src/simple-services/syslog" \
	-I"../SHARC0/src/simple-services/fs-dev" \
	-I"../SHARC0/src/oss-services/shell" \
	-I"../SHARC0/src/oss-services/umm_malloc" \
	-I"../SHARC0/src/oss-services/xmodem" \
	-I"../SHARC0/src/oss-services/crc" \
	-I"../SHARC0/src/oss-services/yxml" \
	-I"../SHARC0/src/oss-services/spiffs" \
	-I"../SHARC0/src/oss-services/FreeRTOS-SHARC/include" \
	-I"../SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/SHARC_215xx" \
	-I"../SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/SHARC_215xx/osal" \
	-I"../SHARC0/src/oss-services/FreeRTOS-SHARC/portable/CCES/osal"

# Library directories
SHARC0_LIB_DIRS = SHARC0/lib

SHARC0_C_SRC = $(foreach srcdir, $(SHARC0_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.c))
SHARC0_C_OBJ = $(subst $(SRC_PREFIX)/,,${SHARC0_C_SRC:%.c=%.doj})

SHARC0_ASM_SRC = $(foreach srcdir, $(SHARC0_SRC_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.asm))
SHARC0_ASM_OBJ = $(subst $(SRC_PREFIX)/,,${SHARC0_ASM_SRC:%.asm=%.doj})

SHARC0_STATIC_LIBS = $(foreach srcdir, $(SHARC0_LIB_DIRS), $(wildcard $(SRC_PREFIX)/$(srcdir)/*.dlb))

# retrieve dependencies
SHARC0_DEPS = $(foreach depdir, $(SHARC0_SRC_DIRS), $(wildcard $(depdir)/*.d))

# define the compiler flags
SHARC0_CFLAGS = $(OPTIMIZE) $(DEBUG) -proc $(PROC) $(SHARC0_INCLUDE_DIRS)
SHARC0_CFLAGS += -DFREE_RTOS -D__ADI_FREERTOS=1 -DPLAT=\"${PROJECT}\"
SHARC0_CFLAGS += -si-revision $(SI_REVISION) -DCORE1 -swc -char-size-8
SHARC0_CFLAGS += -structs-do-not-overlap -no-const-strings -no-multiline -warn-protos
SHARC0_CFLAGS += -double-size-32 -Wwarn 0177

# define the assembler flags
SHARC0_AFLAGS = $(BUILD_RELEASE) -proc $(PROC) $(SHARC0_INCLUDE_DIRS)
SHARC0_AFLAGS += -si-revision $(SI_REVISION) -DCORE1 -swc -char-size-8
SHARC0_AFLAGS += -double-size-32 -W2536

SHARC0_OBJS = $(SHARC0_ASM_OBJ) $(SHARC0_C_OBJ)

SHARC0_EXE = $(PROJECT)-SHARC0.dxe

# compile SHARC0 'C' files
SHARC0/%.doj: ../SHARC0/%.c
	$(SHARC_CC) $(SHARC0_CFLAGS) -gnu-style-dependencies -MD -Mo "$(basename $@).d" -o "$@" -c "$<"

# compile SHARC0 assembly files
SHARC0/%.doj: ../SHARC0/%.asm
	$(SHARC_AS) $(SHARC0_AFLAGS) -gnu-style-dependencies -MM -Mo "$(basename $@).d" -o "$@" "$<"

$(SHARC0_EXE): $(SHARC0_OBJS) $(SHARC0_STATIC_LIBS)
	$(SHARC_CC) -proc $(PROC) -si-revision $(SI_REVISION) -T"SHARC0-$(PROJECT).ldf" -no-mem -map linker_log_SHARC0.xml -flags-link -e -flags-link -ip -flags-link -MDDOLBY_DSP_IDK -flags-link -MDCORE0,-MDUSE_SDRAM,-MDOTHERCORE,-MDDEBUG -o "$@" $^

################################################################################
# Generic section
################################################################################

.DEFAULT_GOAL = all
all: builddirs $(LDR)

$(LDR):  $(SHARC0_EXE)
	$(LD) -proc $(PROC) -core0=$(SHARC0_EXE) -init ezkit21569_initcode.dxe -b SPI -f BINARY -Width 8 -bcode 0x1 -o $@

# Other Targets
clean:
	$(RM) -f *.xml $(PROJECT).* ../*.xml
	$(RM) -rf $(SHARC0_EXE) $(SHARC0_SRC_DIRS)

builddirs:
	@mkdir -p $(SHARC0_SRC_DIRS)

help:
	@echo 'usage:'
	@echo '    make [all|clean] [RELEASE=<yes/no>]'
	@echo ''
	@echo 'DEBUG:'
	@echo '    make all'
	@echo ''
	@echo 'RELEASE:'
	@echo '    make all RELEASE=yes [<other options>]'


.PHONY: all clean help builddirs
.SECONDARY:

# pull in and check dependencies
-include $(SHARC0_C_OBJ:.doj=.d)
-include $(SHARC0_ASM_OBJ:.doj=.d)
