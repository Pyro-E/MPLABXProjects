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
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c Dev_Led.c MCU_Time.c Sys_Time_MCU_Specific.c Dev_Uart.c PulseCounter.c FlowMeter.c FlowLog.c FlowReport.c led_fsm_sysstate.c Compress_Pack_10_14.c Dev_Valve.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.p1 ${OBJECTDIR}/Dev_Led.p1 ${OBJECTDIR}/MCU_Time.p1 ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 ${OBJECTDIR}/Dev_Uart.p1 ${OBJECTDIR}/PulseCounter.p1 ${OBJECTDIR}/FlowMeter.p1 ${OBJECTDIR}/FlowLog.p1 ${OBJECTDIR}/FlowReport.p1 ${OBJECTDIR}/led_fsm_sysstate.p1 ${OBJECTDIR}/Compress_Pack_10_14.p1 ${OBJECTDIR}/Dev_Valve.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/main.p1.d ${OBJECTDIR}/Dev_Led.p1.d ${OBJECTDIR}/MCU_Time.p1.d ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d ${OBJECTDIR}/Dev_Uart.p1.d ${OBJECTDIR}/PulseCounter.p1.d ${OBJECTDIR}/FlowMeter.p1.d ${OBJECTDIR}/FlowLog.p1.d ${OBJECTDIR}/FlowReport.p1.d ${OBJECTDIR}/led_fsm_sysstate.p1.d ${OBJECTDIR}/Compress_Pack_10_14.p1.d ${OBJECTDIR}/Dev_Valve.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.p1 ${OBJECTDIR}/Dev_Led.p1 ${OBJECTDIR}/MCU_Time.p1 ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 ${OBJECTDIR}/Dev_Uart.p1 ${OBJECTDIR}/PulseCounter.p1 ${OBJECTDIR}/FlowMeter.p1 ${OBJECTDIR}/FlowLog.p1 ${OBJECTDIR}/FlowReport.p1 ${OBJECTDIR}/led_fsm_sysstate.p1 ${OBJECTDIR}/Compress_Pack_10_14.p1 ${OBJECTDIR}/Dev_Valve.p1

# Source Files
SOURCEFILES=main.c Dev_Led.c MCU_Time.c Sys_Time_MCU_Specific.c Dev_Uart.c PulseCounter.c FlowMeter.c FlowLog.c FlowReport.c led_fsm_sysstate.c Compress_Pack_10_14.c Dev_Valve.c



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
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F06Q40
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Led.p1: Dev_Led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Led.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Led.p1 Dev_Led.c 
	@-${MV} ${OBJECTDIR}/Dev_Led.d ${OBJECTDIR}/Dev_Led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCU_Time.p1: MCU_Time.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCU_Time.p1.d 
	@${RM} ${OBJECTDIR}/MCU_Time.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/MCU_Time.p1 MCU_Time.c 
	@-${MV} ${OBJECTDIR}/MCU_Time.d ${OBJECTDIR}/MCU_Time.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCU_Time.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Sys_Time_MCU_Specific.p1: Sys_Time_MCU_Specific.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d 
	@${RM} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 Sys_Time_MCU_Specific.c 
	@-${MV} ${OBJECTDIR}/Sys_Time_MCU_Specific.d ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Uart.p1: Dev_Uart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Uart.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Uart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Uart.p1 Dev_Uart.c 
	@-${MV} ${OBJECTDIR}/Dev_Uart.d ${OBJECTDIR}/Dev_Uart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Uart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/PulseCounter.p1: PulseCounter.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PulseCounter.p1.d 
	@${RM} ${OBJECTDIR}/PulseCounter.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/PulseCounter.p1 PulseCounter.c 
	@-${MV} ${OBJECTDIR}/PulseCounter.d ${OBJECTDIR}/PulseCounter.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/PulseCounter.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowMeter.p1: FlowMeter.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowMeter.p1.d 
	@${RM} ${OBJECTDIR}/FlowMeter.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowMeter.p1 FlowMeter.c 
	@-${MV} ${OBJECTDIR}/FlowMeter.d ${OBJECTDIR}/FlowMeter.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowMeter.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowLog.p1: FlowLog.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowLog.p1.d 
	@${RM} ${OBJECTDIR}/FlowLog.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowLog.p1 FlowLog.c 
	@-${MV} ${OBJECTDIR}/FlowLog.d ${OBJECTDIR}/FlowLog.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowLog.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowReport.p1: FlowReport.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowReport.p1.d 
	@${RM} ${OBJECTDIR}/FlowReport.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowReport.p1 FlowReport.c 
	@-${MV} ${OBJECTDIR}/FlowReport.d ${OBJECTDIR}/FlowReport.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowReport.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/led_fsm_sysstate.p1: led_fsm_sysstate.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/led_fsm_sysstate.p1.d 
	@${RM} ${OBJECTDIR}/led_fsm_sysstate.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/led_fsm_sysstate.p1 led_fsm_sysstate.c 
	@-${MV} ${OBJECTDIR}/led_fsm_sysstate.d ${OBJECTDIR}/led_fsm_sysstate.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/led_fsm_sysstate.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Compress_Pack_10_14.p1: Compress_Pack_10_14.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Compress_Pack_10_14.p1.d 
	@${RM} ${OBJECTDIR}/Compress_Pack_10_14.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Compress_Pack_10_14.p1 Compress_Pack_10_14.c 
	@-${MV} ${OBJECTDIR}/Compress_Pack_10_14.d ${OBJECTDIR}/Compress_Pack_10_14.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Compress_Pack_10_14.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Valve.p1: Dev_Valve.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Valve.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Valve.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit5   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Valve.p1 Dev_Valve.c 
	@-${MV} ${OBJECTDIR}/Dev_Valve.d ${OBJECTDIR}/Dev_Valve.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Valve.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Led.p1: Dev_Led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Led.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Led.p1 Dev_Led.c 
	@-${MV} ${OBJECTDIR}/Dev_Led.d ${OBJECTDIR}/Dev_Led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/MCU_Time.p1: MCU_Time.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MCU_Time.p1.d 
	@${RM} ${OBJECTDIR}/MCU_Time.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/MCU_Time.p1 MCU_Time.c 
	@-${MV} ${OBJECTDIR}/MCU_Time.d ${OBJECTDIR}/MCU_Time.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/MCU_Time.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Sys_Time_MCU_Specific.p1: Sys_Time_MCU_Specific.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d 
	@${RM} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Sys_Time_MCU_Specific.p1 Sys_Time_MCU_Specific.c 
	@-${MV} ${OBJECTDIR}/Sys_Time_MCU_Specific.d ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Sys_Time_MCU_Specific.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Uart.p1: Dev_Uart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Uart.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Uart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Uart.p1 Dev_Uart.c 
	@-${MV} ${OBJECTDIR}/Dev_Uart.d ${OBJECTDIR}/Dev_Uart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Uart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/PulseCounter.p1: PulseCounter.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PulseCounter.p1.d 
	@${RM} ${OBJECTDIR}/PulseCounter.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/PulseCounter.p1 PulseCounter.c 
	@-${MV} ${OBJECTDIR}/PulseCounter.d ${OBJECTDIR}/PulseCounter.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/PulseCounter.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowMeter.p1: FlowMeter.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowMeter.p1.d 
	@${RM} ${OBJECTDIR}/FlowMeter.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowMeter.p1 FlowMeter.c 
	@-${MV} ${OBJECTDIR}/FlowMeter.d ${OBJECTDIR}/FlowMeter.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowMeter.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowLog.p1: FlowLog.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowLog.p1.d 
	@${RM} ${OBJECTDIR}/FlowLog.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowLog.p1 FlowLog.c 
	@-${MV} ${OBJECTDIR}/FlowLog.d ${OBJECTDIR}/FlowLog.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowLog.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/FlowReport.p1: FlowReport.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FlowReport.p1.d 
	@${RM} ${OBJECTDIR}/FlowReport.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/FlowReport.p1 FlowReport.c 
	@-${MV} ${OBJECTDIR}/FlowReport.d ${OBJECTDIR}/FlowReport.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/FlowReport.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/led_fsm_sysstate.p1: led_fsm_sysstate.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/led_fsm_sysstate.p1.d 
	@${RM} ${OBJECTDIR}/led_fsm_sysstate.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/led_fsm_sysstate.p1 led_fsm_sysstate.c 
	@-${MV} ${OBJECTDIR}/led_fsm_sysstate.d ${OBJECTDIR}/led_fsm_sysstate.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/led_fsm_sysstate.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Compress_Pack_10_14.p1: Compress_Pack_10_14.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Compress_Pack_10_14.p1.d 
	@${RM} ${OBJECTDIR}/Compress_Pack_10_14.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Compress_Pack_10_14.p1 Compress_Pack_10_14.c 
	@-${MV} ${OBJECTDIR}/Compress_Pack_10_14.d ${OBJECTDIR}/Compress_Pack_10_14.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Compress_Pack_10_14.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/Dev_Valve.p1: Dev_Valve.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Dev_Valve.p1.d 
	@${RM} ${OBJECTDIR}/Dev_Valve.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -DXPRJ_default=$(CND_CONF)  $(COMPARISON_BUILD)      -o ${OBJECTDIR}/Dev_Valve.p1 Dev_Valve.c 
	@-${MV} ${OBJECTDIR}/Dev_Valve.d ${OBJECTDIR}/Dev_Valve.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/Dev_Valve.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=pickit5  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"        $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} ${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.hex 
	
	
else
${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"      $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/PIC18F06Q40_Template.X_V031_DeliverA.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
