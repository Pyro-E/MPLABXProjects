
	; Microchip MPLAB XC8 C Compiler V3.10
	; Copyright (C) 2025 Microchip Technology Inc.

	; Auto-generated runtime startup code for final link stage.

	;
	; Compiler options:
	;
	; -q --opt=none --chip=18f06q40 \
	; -Mdist/default/production/PIC18F06Q40_Template.X_V055_Pro.production.map \
	; -DXPRJ_default=default -L--defsym=__MPLAB_BUILD=1 \
	; --dfp=/Applications/microchip/mplabx/v6.30/packs/Microchip/PIC18F-Q_DFP/1.28.451/xc8 \
	; --summary=+xml --summarydir=dist/default/production/memoryfile.xml \
	; -oPIC18F06Q40_Template.X_V055_Pro.production.elf \
	; --objdir=dist/default/production --outdir=dist/default/production \
	; build/default/production/main.p1 build/default/production/Dev_Led.p1 \
	; build/default/production/MCU_Time.p1 \
	; build/default/production/Sys_Time_MCU_Specific.p1 \
	; build/default/production/Dev_Uart.p1 \
	; build/default/production/PulseCounter.p1 \
	; build/default/production/FlowMeter.p1 \
	; build/default/production/FlowLog.p1 \
	; build/default/production/FlowReport.p1 \
	; build/default/production/led_fsm_sysstate.p1 \
	; build/default/production/Compress_Pack_10_14.p1 \
	; build/default/production/Dev_Valve.p1 \
	; build/default/production/Flow_Control.p1 \
	; build/default/production/MValve_OP3.p1 \
	; build/default/production/Packet.p1 -L--fixupoverflow=error --std=c99 \
	; --rors --icl=auto --callgraph=none --warn=-3 \
	; --errformat=%f:%l:%c: error: (%n) %s \
	; --warnformat=%f:%l:%c: warning: (%n) %s \
	; --msgformat=%f:%l:%c: advisory: (%n) %s
	;


	processor	18F06Q40
	include "/Applications/microchip/mplabx/v6.30/packs/Microchip/PIC18F-Q_DFP/1.28.451/xc8/pic/include/proc/18f06q40.cgen.inc"

	GLOBAL	_main,start
	FNROOT	_main

	pic18cxx	equ	1

	psect	const,class=CONST,delta=1,reloc=2,noexec
	psect	smallconst,class=SMALLCONST,delta=1,reloc=2,noexec
	psect	mediumconst,class=MEDIUMCONST,delta=1,reloc=2,noexec
	psect	rbss,class=COMRAM,space=1,noexec
	psect	bss,class=RAM,space=1,noexec
	psect	rdata,class=COMRAM,space=1,noexec
	psect	irdata,class=CODE,space=0,reloc=2,noexec
	psect	bss,class=RAM,space=1,noexec
	psect	data,class=RAM,space=1,noexec
	psect	idata,class=CODE,space=0,reloc=2,noexec
	psect	nvrram,class=COMRAM,space=1,noexec
	psect	nvbit,class=COMRAM,bit,space=1,noexec
	psect	temp,ovrld,class=COMRAM,space=1,noexec
	psect	struct,ovrld,class=COMRAM,space=1,noexec
	psect	rbit,class=COMRAM,bit,space=1,noexec
	psect	bigbss,class=BIGRAM,space=1,noexec
	psect	bigdata,class=BIGRAM,space=1,noexec
	psect	ibigdata,class=CODE,space=0,reloc=2,noexec
	psect	farbss,class=FARRAM,space=0,reloc=2,delta=1,noexec
	psect	nvFARRAM,class=FARRAM,space=0,reloc=2,delta=1,noexec
	psect	fardata,class=FARRAM,space=0,reloc=2,delta=1,noexec
	psect	ifardata,class=CODE,space=0,reloc=2,delta=1,noexec

	psect	reset_vec,class=CODE,delta=1,reloc=2
	psect	powerup,class=CODE,delta=1,reloc=2
	psect	init,class=CODE,delta=1,reloc=2
	psect	text,class=CODE,delta=1,reloc=2
GLOBAL	intlevel0,intlevel1,intlevel2
intlevel0:
intlevel1:
intlevel2:
GLOBAL	intlevel3
intlevel3:
	psect	clrtext,class=CODE,delta=1,reloc=2

	psect	config,class=CONFIG,delta=1,space=4,noexec
	psect	idloc,class=IDLOC,delta=1,space=5,noexec
	psect	eeprom_data,class=EEDATA,delta=1,noexec
	PSECT	ramtop,class=RAM,noexec
	GLOBAL	__S1			; top of RAM usage
	GLOBAL	__ramtop
	GLOBAL	__LRAM,__HRAM
__ramtop:

	psect	reset_vec
reset_vec:
	; No powerup routine
	global start

; jump to start
	goto start
	GLOBAL __accesstop
__accesstop EQU 1376

;Initialize the stack pointer (FSR1)
	global stacklo, stackhi
	stacklo	equ	0
	stackhi	equ	0


	psect	stack,class=STACK,space=2,noexec
	global ___stack_lo, ___stack_hi, ___inthi_stack_lo, ___inthi_stack_hi, ___intlo_stack_lo, ___intlo_stack_hi
	global ___sp,___inthi_sp,___intlo_sp
___sp:
___inthi_sp:
___intlo_sp:
___stack_lo:
___stack_hi:
___inthi_stack_lo:
___inthi_stack_hi:
___intlo_stack_lo:
___intlo_stack_hi:

; No heap to be allocated
	psect	heap,class=HEAP,space=7,noexec
	global ___heap_lo
	___heap_lo	equ	0
	global ___heap_hi
	___heap_hi	equ	0



	psect	init
start:
	global start_initialization
	goto start_initialization	;jump to C runtime clear & initialization

psect comram,class=COMRAM,space=1
psect abs1,class=ABS1,space=1
psect bigram,class=BIGRAM,space=1
psect ram,class=RAM,space=1
psect bank5,class=BANK5,space=1
psect bank6,class=BANK6,space=1
psect bank7,class=BANK7,space=1
psect bank8,class=BANK8,space=1
psect bank9,class=BANK9,space=1
psect bank10,class=BANK10,space=1
psect bank11,class=BANK11,space=1
psect bank12,class=BANK12,space=1
psect bank13,class=BANK13,space=1
psect bank14,class=BANK14,space=1
psect bank15,class=BANK15,space=1
psect bank16,class=BANK16,space=1
psect bank17,class=BANK17,space=1
psect bank18,class=BANK18,space=1
psect bank19,class=BANK19,space=1
psect bank20,class=BANK20,space=1
psect sfr,class=SFR,space=1
psect bigsfr,class=BIGSFR,space=1


	end	start
