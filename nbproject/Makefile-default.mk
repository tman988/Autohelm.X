#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1846003557/ADC.o ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o" ${OBJECTDIR}/_ext/1846003557/MotorControl.o ${OBJECTDIR}/_ext/1846003557/timers.o ${OBJECTDIR}/_ext/1846003557/Uart1.o ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1846003557/ADC.o.d ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o.d" ${OBJECTDIR}/_ext/1846003557/MotorControl.o.d ${OBJECTDIR}/_ext/1846003557/timers.o.d ${OBJECTDIR}/_ext/1846003557/Uart1.o.d ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1846003557/ADC.o ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o ${OBJECTDIR}/_ext/1846003557/Hall\ Encoder.o ${OBJECTDIR}/_ext/1846003557/MotorControl.o ${OBJECTDIR}/_ext/1846003557/timers.o ${OBJECTDIR}/_ext/1846003557/Uart1.o ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP256MC502
MP_LINKER_FILE_OPTION=,--script=p33EP256MC502.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1846003557/ADC.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/ADC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/ADC.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/ADC.c  -o ${OBJECTDIR}/_ext/1846003557/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/ADC.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/ADC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/AutohelmNode.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/AutohelmNode.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/AutohelmNode.c  -o ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/Hall\ Encoder.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Hall\ Encoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/Hall\ Encoder.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Hall Encoder.c"  -o "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/MotorControl.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/MotorControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/MotorControl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/MotorControl.c  -o ${OBJECTDIR}/_ext/1846003557/MotorControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/MotorControl.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/MotorControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/timers.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/timers.c  -o ${OBJECTDIR}/_ext/1846003557/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/timers.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/Uart1.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/Uart1.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Uart1.c  -o ${OBJECTDIR}/_ext/1846003557/Uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/Uart1.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/Uart1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/CircularBuffer.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/CircularBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/CircularBuffer.c  -o ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1846003557/ADC.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/ADC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/ADC.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/ADC.c  -o ${OBJECTDIR}/_ext/1846003557/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/ADC.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/ADC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/AutohelmNode.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/AutohelmNode.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/AutohelmNode.c  -o ${OBJECTDIR}/_ext/1846003557/AutohelmNode.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/AutohelmNode.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/Hall\ Encoder.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Hall\ Encoder.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/Hall\ Encoder.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  "C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Hall Encoder.c"  -o "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o"  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/Hall Encoder.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/MotorControl.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/MotorControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/MotorControl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/MotorControl.c  -o ${OBJECTDIR}/_ext/1846003557/MotorControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/MotorControl.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/MotorControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/timers.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/timers.c  -o ${OBJECTDIR}/_ext/1846003557/timers.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/timers.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/timers.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/Uart1.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Uart1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/Uart1.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/Uart1.c  -o ${OBJECTDIR}/_ext/1846003557/Uart1.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/Uart1.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/Uart1.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1846003557/CircularBuffer.o: C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/CircularBuffer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1846003557 
	@${RM} ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  C:/Users/tfurtado/Dropbox/Autohelm/Software/Autohelm.X/CircularBuffer.c  -o ${OBJECTDIR}/_ext/1846003557/CircularBuffer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d"        -g -omf=elf -O0 -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1846003557/CircularBuffer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Autohelm.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf 
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
