#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile

# Environment
MKDIR=mkdir -p
RM=rm -f 
CP=cp 
# Macros
CND_CONF=pic32-starter-kit

ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf
else
IMAGE_TYPE=production
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf
endif
# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}
# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files
OBJECTFILES=${OBJECTDIR}/startup.o ${OBJECTDIR}/pic32bootloader.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

OS_ORIGINAL="Darwin"
OS_CURRENT="$(shell uname -s)"
############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
MP_CC=/Applications/microchip/mplabc32/v1.11a/bin/pic32-gcc
MP_AS=/Applications/microchip/mplabc32/v1.11a/bin/pic32-as
MP_LD=/Applications/microchip/mplabc32/v1.11a/bin/pic32-ld
MP_AR=/Applications/microchip/mplabc32/v1.11a/bin/pic32-ar
MP_CC_DIR=/Applications/microchip/mplabc32/v1.11a/bin
MP_AS_DIR=/Applications/microchip/mplabc32/v1.11a/bin
MP_LD_DIR=/Applications/microchip/mplabc32/v1.11a/bin
MP_AR_DIR=/Applications/microchip/mplabc32/v1.11a/bin
.build-conf: ${BUILD_SUBPROJECTS}
ifneq ($(OS_CURRENT),$(OS_ORIGINAL))
	@echo "***** WARNING: This make file contains OS dependent code. The OS this makefile is being run is different from the OS it was created in."
endif
	${MAKE}  -f nbproject/Makefile-pic32-starter-kit.mk dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
.PHONY: ${OBJECTDIR}/startup.o
${OBJECTDIR}/startup.o: startup.S  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} ${OBJECTDIR} 
	${MP_CC}  -D__DEBUG  -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=32MX360F512L  -o ${OBJECTDIR}/startup.o startup.S  -Wa,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--gdwarf-2
else
.PHONY: ${OBJECTDIR}/startup.o
${OBJECTDIR}/startup.o: startup.S  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} ${OBJECTDIR} 
	${MP_CC}  -c -mprocessor=32MX360F512L  -o ${OBJECTDIR}/startup.o startup.S  -Wa,--defsym=__MPLAB_BUILD=1
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/pic32bootloader.o: pic32bootloader.c  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} ${OBJECTDIR} 
	${RM} ${OBJECTDIR}/pic32bootloader.o.d 
	${MP_CC} -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -x c -c -mprocessor=32MX360F512L -ffunction-sections -fdata-sections -mips16 -Os -MMD -MF ${OBJECTDIR}/pic32bootloader.o.d -o ${OBJECTDIR}/pic32bootloader.o pic32bootloader.c 
ifneq (,$(findstring MINGW32,$(OS_CURRENT))) 
	 sed -e 's/\\$$/__EOL__/g' -e 's/\\ /__ESCAPED_SPACES__/g' -e 's/\\/\//g' -e 's/__ESCAPED_SPACES__/\\ /g' -e 's/__EOL__$$/\\/g' ${OBJECTDIR}/pic32bootloader.o.d > ${OBJECTDIR}/pic32bootloader.o.tmp
	${RM} ${OBJECTDIR}/pic32bootloader.o.d 
	${CP} ${OBJECTDIR}/pic32bootloader.o.tmp ${OBJECTDIR}/pic32bootloader.o.d 
	${RM} ${OBJECTDIR}/pic32bootloader.o.tmp}
endif
else
${OBJECTDIR}/pic32bootloader.o: pic32bootloader.c  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} ${OBJECTDIR} 
	${RM} ${OBJECTDIR}/pic32bootloader.o.d 
	${MP_CC}  -x c -c -mprocessor=32MX360F512L -ffunction-sections -fdata-sections -mips16 -Os -MMD -MF ${OBJECTDIR}/pic32bootloader.o.d -o ${OBJECTDIR}/pic32bootloader.o pic32bootloader.c 
ifneq (,$(findstring MINGW32,$(OS_CURRENT))) 
	 sed -e 's/\\$$/__EOL__/g' -e 's/\\ /__ESCAPED_SPACES__/g' -e 's/\\/\//g' -e 's/__ESCAPED_SPACES__/\\ /g' -e 's/__EOL__$$/\\/g' ${OBJECTDIR}/pic32bootloader.o.d > ${OBJECTDIR}/pic32bootloader.o.tmp
	${RM} ${OBJECTDIR}/pic32bootloader.o.d 
	${CP} ${OBJECTDIR}/pic32bootloader.o.tmp ${OBJECTDIR}/pic32bootloader.o.d 
	${RM} ${OBJECTDIR}/pic32bootloader.o.tmp}
endif
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC}  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=32MX360F512L -nostartfiles -o dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf ${OBJECTFILES}      -Wl,--defsym=__MPLAB_BUILD=1,--script=boot-linkerscript.ld,--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--gc-sections,-Map="map.txt",--cref,-Os
else
dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC}  -mprocessor=32MX360F512L -nostartfiles -o dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf ${OBJECTFILES}      -Wl,--defsym=__MPLAB_BUILD=1,--script=boot-linkerscript.ld,--gc-sections,-Map="map.txt",--cref,-Os
	${MP_CC_DIR}/pic32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/arduino-booloader.X.${IMAGE_TYPE}.elf 
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/pic32-starter-kit
	${RM} -r dist
# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
