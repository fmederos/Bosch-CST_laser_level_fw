#compiling settings
PROCESSOR	= 16F676
CC		= xc8
PROJ		= pll360-upgrade
PROJ_DIR	= .
GEN_DIR		= /tmp/$(PROJ)
BUILD_DIR	= ${GEN_DIR}/build
BIN_DIR		= ${GEN_DIR}/bin
OBJ_DIR		= ${GEN_DIR}/obj
CC_INCLUDE	= -I/Applications/microchip/xc8/v1.34/include/
CC_FLAGS	= -P -N255 --warn=0 --addrqual=ignore --mode=pro# --echo
OPTIMISATION	= --opt=default,+asm,-asmfile,+speed,-space,-debug,9
OPTIMISATION   += --runtime=+plib
OUTPUT		= --output=intel --outdir=${BUILD_DIR} --objdir=${OBJ_DIR}
OUTFILE		= main.hex
MKDIRS		= mkdir -p $(BUILD_DIR) $(BIN_DIR) $(OBJ_DIR)

CHIP		= --chip=${PROCESSOR}
ALL_C		= ${PROJ_DIR}/main.c

#programming settings
#PROGRAMMER	= pickit2
PROGRAM		= /Applications/pk2cmd/pk2cmd
DEVICE		= -pPIC${PROCESSOR}
#datasheet stuff
READER		= okular
PREFIX		= PIC
SUFFIX		= .pdf
DIR		= ~/datasheets/IC\'s/Microchip/PIC/
NICEDUMP	= egrep ^0000 | awk '{ print "Laser mode: " $$2 ", Locked: " $$3 ", Location mode: " $$4 "\n" }'


.PHONY : all clean clean_all prog program restore erase on off info help
.DEFAULT : all
${ALL_C} all:
	${MKDIRS}
	${CC} ${CC_FLAGS} ${CHIP} ${OPTIMISATION} ${ALL_C} ${OUTPUT}
	cp ${BUILD_DIR}/${OUTFILE} ${BIN_DIR}/${OUTFILE}
clean:
	rm -rf ${GEN_DIR}
prog program:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -f${BIN_DIR}/${OUTFILE} -m -j
restore:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -f${PROJ_DIR}/backup/original-firmware.hex -m -j
erase:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -e -j
on:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -t
off:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -w
dump:
	@$(PROGRAM) -B/Applications/pk2cmd/ ${DEVICE} -GE0-2 |$(NICEDUMP)
	$(info = = = = = = = = = = = = = = = = = = = = = = = =)
	$(info 0x00    Laser mode: 2 = Vert., 4 = Horiz., 6 = H+V )
	$(info 0x01    Lock mode: 0 = Unlocked, 1 = Locked )
	$(info 0x02    Location mode: 0 = Outdoor, 1 = Indoor)
	$(info = = = = = = = = = = = = = = = = = = = = = = = =)
info:
	${PROGRAM} -B/Applications/pk2cmd/ ${DEVICE} -I
datasheet:
	${READER} ${DIR}/${PREFIX}${PROCESSOR}${SUFFIX}
help:
	@echo "Type 'make' or 'make all' to make the program"
	@echo "Use the 'prog' or 'program' targets to program the device"
	@echo "Use 'erase' to erase the device"
	@echo "Use 'on' and 'off' to control the PicKit's power output"
	@echo "Use 'info' to get info on the device"
	@echo "'clean' cleans temp files"
	@echo "'clean_all' cleans temp and release files"
	@echo "Use 'datasheet' to open datasheet of part"

