;3.0
;15 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;Subrutina de lectura del sensor NPA700
;
;Despues de leer el sensor se le resta 4383, para obtener la dirección del offset de la
;tabla de conversión, que contiene los valores en cmH2O
.equ		n_muestrasNPA700	=	4

rd_NPA700:	pushw
			pushx
			pushy
			pushr	sreg

			rcall	rd_i2c_NPA

			cpri	estado_i2cNPA,'0'
			breq	sensor_OK
;Si se deja de detectar el sensor de presion, se procede a liberar la presion
;error_liberar_presion:
			ASIGNA_PWM_FLUJO					0,0
			;ASIGNA_PWM_V_ex  					0,0;ELECTROVALVULAS						0
			outi	presion_liberada,'1'
			jmp		fin_NPA700

sensor_OK:	inr		xh,NPA_700H
			inr		xl,NPA_700L

			cpi		xl,low(8192)
			cpci	xh,high(8192)
			brlo	verifica_limite_lectura
			ldiw	xh,xl,8192
			jmp		acumula_dato
verifica_limite_lectura:
			cpi		xl,low(5335)	;Si la lectura del sensor es menor a 5335, se iguala
			cpci	xh,high(5335)	;a 5334
			brsh	acumula_dato
			ldiw	xh,xl,5334
			jmp		acumula_dato

acumula_dato:
			inr		yl,Acc_NPA700L
			inr		yh,Acc_NPA700M
			add		yl,xl
			adc		yh,xh
			outr	Acc_NPA700L,yl
			outr	Acc_NPA700M,yh
			brcc	skip_inc_Acc_H
			incr	Acc_NPA700H
skip_inc_Acc_H:
			decr	cont_muestrasNPA700
			rbrne	fin_NPA700

;el registro X, mantiene el valor acumulado

			inr		yl,Acc_NPA700L
			inr		yh,Acc_NPA700M
			inr		xl,Acc_NPA700H

			lsr		xl
			ror		yh
			ror		yl
			lsr		xl
			ror		yh
			ror		yl

;			lsr		xl
;			ror		yh
;			ror		yl
;			lsr		xl
;			ror		yh
;			ror		yl

;			lsr		xl
;			ror		yh
;			ror		yl
;			lsr		xl
;			ror		yh
;			ror		yl

			outr	Presion_NPA700L,yl
			outr	Presion_NPA700H,yh

			subi	yl,low(5335)
			sbci	yh,high(5335)
			lsl		yl
			rol		yh

			ldiw	zh,zl,(TABLA_CONVERSION_PRESION_NPA700*2)
			add		zl,yl
			adc		zh,yh
			outi	rampz,0x00
			lpm		xl,z+
			lpm		xh,z
;**********************************
;Sección de corrección de la lectura del sensor de presion
			subi	xl,low(Offset_NPA700)
			sbci	xh,high(Offset_NPA700)
			breq	asigna_x
			brpl	asigna_x
			ldiw	xh,xl,0x0000
asigna_x:
;**********************************
			outr	Presion_cmH2OL,xl
			outr	Presion_cmH2OH,xh

			outi	NPA700_valido,'1'	;Bandera para validar la lectura
			outi	cont_muestrasNPA700,n_muestrasNPA700
			outi	Acc_NPA700L,0x00
			outi	Acc_NPA700M,0x00
			outi	Acc_NPA700H,0x00

			call	control_presion

fin_NPA700:
			popr	sreg
			popy
			popx
			popw
			ret

#define		SDAO		ddrd,1
#define		SCLO		ddrd,0
#define		SDAI		pind,1
#define		SCLI		pind,0

;***********************************************************************
.macro		sda0
			CBI			PORTD,1
			sbi			SDAO
			.endm

.macro		sda1
			SBI			PORTD,1
			cbi			SDAO
			.endm

.macro		scl0
			CBI			PORTD,0
			sbi			SCLO
			.endm

.macro		scl1
			SBI			PORTD,0
			cbi			SCLO
			.endm

.macro		delay_I2C
			call delay
.endm

delay:		nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			nop
			ret
;***********************************************************************

;***********************************************************************
rd_i2c_NPA:	ldi		r25,(0x28*2)+1		;ID de la memoria
			scl1
			sda1
			sda0
			delay_I2C
			scl0
			delay_I2C
			rcall	wrbyte_i2c

			scl0
			scl1
			delay_I2C
;			sbic	SDAI
;			rjmp	@0
;			delay_I2C
			scl0
;read_i2c:
			rcall	rdbyte_i2c
			outr	NPA_700H,r24
/*
						outr	udr_debug,r24
						outi	cont_tx0,0x00
*/

			scl0			;Envia ACK
			sda0
			delay_I2C
			scl1
			delay_I2C
			scl0
			sda1

			rcall	rdbyte_i2c
			outr	NPA_700L,r24
/*
						outr	udr_debug,r24
						outi	cont_tx0,0x00
*/

			scl0			;Envia NAK
			sda1
			delay_I2C
			scl1
			delay_I2C
			scl0

			sda0			;STOP
			scl1
			delay_I2C
			sda1
			delay_I2C

			inr		r25,NPA_700H
			andi	r25,0b11000000
			cpi		r25,0b00000000
			breq	lectura_NPA_OK
			cpi		r25,0b10000000	;Si el bit más significativo esta activo, solo indica
			brne	ERROR2			;que el sensor envió el dato anterior
			inr		r25,NPA_700H
			andi	r25,0b00111111
			outr	NPA_700H,r25
lectura_NPA_OK:
			outi	estado_i2cNPA,'0'

/*
					inr		XL,NPA_700L
					inr		XH,NPA_700H
					outi	cont_tx0,0
					outr	udr_debug,XH
					outr	udr_debug,XL
*/

			ret

ERROR2:		outi	estado_i2cNPA,'1'
			ret
;***********************************************************************

;***********************************************************************
wrbyte_i2c:	ldi		r24,0x08
			scl0
next_bit_wr:rol		r25
			brcc	pon0
			delay_I2C
			sda1
pulso:		delay_I2C
			scl1
			delay_I2C
			scl0
			delay_I2C
			dec		r24
			brne	next_bit_wr
			sda1
			ret
pon0:		sda0
			rjmp	pulso
;***********************************************************************

;***********************************************************************
rdbyte_i2c:	ldi		r24,0x08
;			sda1
			scl0
next_bit_rd:delay_I2C
			scl1
			delay_I2C
			clc
			sbic	SDAI
			sec
			rol		r25
			scl0
			delay_I2C
			dec		r24
			brne	next_bit_rd
						mov		r24,r25
			ret
;***********************************************************************

;***********************************************************************
;Valores del registro estado_control:
;
;	'0'		No se deben de modificar valvulas ni PWMs
;	'h'		Se deben de mantener los PWMs y cerrar la valvula de exhalacion
;	'+'		Incrementar el PWM de flujo, cerrar la valvula de exhalacion
;	'-'		Decrementar el PWM de flujo, abrir la valvula de exhalacion
;
;Primero se calcula PNPA/Pdeseada
;
;Pdeseada puede ser Pinspiracion, PS ó PEEP (CPAP), dependiendo del modo y fase en curso
;
;Modo	Nombre	Pdeseada
; 0		PCMV	Pinspiracion, PEEP
; 1		VCMV	PEEP					Por el momento no -> P alcanzada (mantener en tiempo pausa)
; 2		PSIMV	Pinspiracion, PEEP, PS
; 3		VSIMV	PEEP, PS				Por el momento no -> P alcanzada (mantener en tiempo pausa) 
; 4		PCPAP	CPAP (antes de APNEA), Pinspiracion, PS, PEEP (=CPAP)
; 5		VCPAP	CPAP (antes de APNEA), PS, PEEP (=CPAP)
;										Por el momento no -> P alcanzada (mantener en tiempo pausa)
;
;RELACION DE FASES VALIDAS PARA QUE ACTUE EL CONTROL
;FASE									ACCION				Pdeseada
;STANDBY								Libera la presion	0
;INSPIRACION							Control				Pinspiracion (PCMV, PSIMV)
;INSPIRACION_ASISTIDA (=INSPIRACION)	Control				Pinspiracion (PSIMV, PCPAP apnea)
;PRESION_SOPORTE						Control				PS
;CPAP_SIN_APNEA							Control				CPAP
;TIEMPO_PAUSA							Control				Se mantiene
;																	Por el momento no P alcanzada (mantener en tiempo pausa)
;TIEMPO_PAUSA_PS						Control				Se mantiene
;																	Por el momento no P alcanzada (mantener en tiempo pausa)
;EXHALACION								Control				PEEP
control_presion:
									outi	ucsr1b,0x18
			cpri	FASE,STANDBY
			rbreq	salir_control
			cpri	FASE,CALIBRA_O2_21
			rbreq	salir_control
			cpri	FASE,CALIBRA_O2_100
			rbreq	salir_control

			cpri	FASE,EXHALACION
			rbreq	tst_PEEP

			cpri	presion_maxima,'1'
			rbreq	tst_PEEP;sin_flujo
/*
			inr		xL,reg_PmaxL
			inr		xH,reg_PmaxH
			inr		r16,Presion_cmH2OL
			inr		r17,Presion_cmH2OH
			cp		r16,Xl
			cpc		r17,Xh		
			rbrsh	sobre_presion
*/
			cpri	FASE,TST_ELECTROVALVULAS
			rbreq	tst_P_PRUEBA

			cpri	FASE,INSPIRACION	;Aplica tambien para INSPIRACION_ASISTIDA
			rbreq	tst_P_INSPIRACION

			cpri	FASE,TIEMPO_PAUSA
			rbreq	sin_flujo
			cpri	FASE,TIEMPO_PAUSA_PS
			rbreq	sin_flujo



			cpri	FASE,PRESION_SOPORTE
			rbreq	ini_Pr_Sp;tst_PS;

;			cpri	FASE,CPAP_APNEA
;			rbreq	salir_control;tst_P_INSPIRACION;tst_CPAP
			cpri	FASE,CPAP_SIN_APNEA
			rbreq	peep_CPAP;tst_CPAP
salir_control:
									outi	ucsr1b,0x98
			ret
/*
libera_presion:
			OFF_PRESION_0p
			rjmp	salir_control
*/
tst_P_PRUEBA:

			cpri	B_PR_TST,'0'
			rbreq	salir_control
			;outi	B_PR_TST,'0'
			;delay_ms	30
			inr		r17,Presion_cmH2OH
			inr		r16,Presion_cmH2OL
			cpi		r16,low(30*100)
			cpci	r17,high(30*100)
			rbrlo	salir_control
			OFF_PRESION_TST
			;outi	B_PR_TST,'0'
			;cpri	B_PR_TST,'1'
			;breq	salir_control
SDS:		outi	envia_trama_U,'1';Se habilita el envió de la trama U para la primera vez
			outi	tmr_U,150		;10ms
			outi	tx_U,'1'		;Validación de tx cada (ya transcurrieron los 300ms)
			
			rjmp	salir_control


;************************************************************
tst_P_INSPIRACION:
control_P_INS_MODO_PRESION:
;			cpri	tmr_de_presion,0x00
;			rbrne	salir_control

;			cpri	presion_maxima,'1'
;			rbreq	salir_control

			cli
			inr		r16,cont_10msL	;Si el tiempo de la fase es menor a 20ms
			inr		r17,cont_10msH	;ya no se ejecuta el control
			sei
			cpi		r16,low(2)
			cpci	r17,high(2)
			rbrlo	salir_control

			cpri	Modo_Op,P_CMV
			rbreq	sigue_control_P_inspiracion
			cpri	Modo_Op,P_SIMV
			rbreq	sigue_control_P_inspiracion
			cpri	Modo_Op,P_CPAP
			rbreq	sigue_control_P_inspiracion
			cpri	B_Vol,'1'
			rbreq	sin_Flujo
			ASIGNA_PWM_FLUJO					reg_PWM0H,reg_PWM0L
			rjmp	salir_control





sigue_control_P_inspiracion:

			
/*			ldiw	xh,xl,0x0064
			inr		r16,Presion_cmH2OL
			inr		r17,Presion_cmH2OH
			cp		r16,Xl
			cpc		r17,Xh		
			rbrlo	pip_nul
			;cpri	inhibe_compensacion,'1'
*/			;rbreq	sin_flujo;salir_control
			;rjmp	PIP_control

;************************************************************
			;inr		yl,CNT_P_PIP
			;cpi		yl,0x03
			;brlo	PIP_Baja
			
;************************************************************
;************************************************************

;************************************************************
PIP_control:
			;outi	inhibe_compensacion,'1'
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			subi	XL,low(100)
			sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PIP_Alta
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			subi	XL,low(100)
			sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PIP_Baja_P
;************************************************************	
;			rjmp		salir_control
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PIP_Ok_P
;************************************************************
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPIOK_A
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			outi	B_Dec_PEEP,'1'
SRPPIOK_A:;Skip_read_Pres_PIP_OK
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-10)
			sbci	XH,high(-10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	out_PIP_OK
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(50)
			cpci	XH,high(50)
			rbrsh	Skip_PIP_V_OK_B
			sbiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
Skip_PIP_V_OK_B:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(40)
			cpci	XH,high(40)
			rbrlo	salir_control
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	salir_control
;************************************************************
PIP_Ok_P:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	Skip_PIP_V_OK
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
Skip_PIP_V_OK:
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPIOK
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			outi	B_Dec_PEEP,'1'
SRPPIOK:;Skip_read_Pres_PIP_OK
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-10)
			sbci	XH,high(-10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	out_PIP_OK
			inr		ZL,offset_COMFLPIPL
			inr		ZH,offset_COMFLPIPH
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cp		XL,ZL
			cpc		XH,ZH
			rbrlo	salir_control
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	salir_control
;************************************************************
;************************************************************
out_PIP_OK:
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
;************************************************************
;************************************************************



PIP_Ok:					
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			inr		XL,offset_COMFLPIPL
			inr		XH,offset_COMFLPIPH
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	pip_OK_out
			sbiw	ZL,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control

			
;************************************************************
pip_OK_out:
			movr	PWM_FLUJOH,offset_COMFLPIPH
			movr	PWM_FLUJOL,offset_COMFLPIPL
			rjmp		salir_control
;************************************************************
;************************************************************
PIP_Baja_P:
			outi	reg_PWM_Val_ExH,0x01
			outi	reg_PWM_Val_ExL,0xF4
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPIA
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			outi	B_Dec_PEEP,'1'
SRPPIA:;Skip_read_Pres_PIP_Alta
			call	CARGA_PR_OFFSET
			outr	Presion_antL,R24
			outr	Presion_antH,R25 
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-10)
			sbci	XH,high(-10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	out_PIP_Baja
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(450)
			cpci	XH,high(450)
			rbrsh	salir_control
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	salir_control
;************************************************************
;************************************************************
out_PIP_Baja:
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
;************************************************************
;************************************************************


;************************************************************
;************************************************************

low_PWM_PIP:
			outi	B_P_PIP,'1'
			outi	reg_PWM2_AireH,high(2)
			outi	reg_PWM2_AireL,low(2)
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL

			rjmp	salir_control
;************************************************************
;************************************************************
Inc_PWM_PIP:
			inr		ZL,Cont_CF_L
			inr		ZH,Cont_CF_H
			cpi		ZL,low(2)
			cpci	ZH,high(2)
			brlo	R_PWM
			sbiw	Z,1
			outr	Cont_CF_L,ZL
			outr	Cont_CF_H,ZH
			rjmp	salir_control
;************************************************************
;************************************************************
R_PWM:		
			outi	Cont_CF_H,high(50)
			outi	Cont_CF_L,low(50)
			;rjmp	salir_control

;************************************************************
			cpri	C_PEEP_DOWN,5
			rbrsh	R_PWM_1
;************************************************************
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			sub		XL,r24
			sbc		XH,r25	
			ldiw	zh,zl,tabla_PIP*2
			;outr		xl,low(500)
			;outr		xl,low(500)
			lsl		xl
			rol		xh
			add		zl,xl
			adc		zh,xh
			outi	rampz,0x00
			lpm		xl,z+
			lpm		xh,z
;************************************************************
			
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			;adiw	XL,3			;subi	XL,low(-10)
			;sbci	XH,high(-10)
			add		zl,XL
			adc		zh,XH
			cpi		ZL,low(480)
			cpci	ZH,high(480)
			rbrsh	PMW_Inc_out
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
			
;************************************************************
PMW_Inc_out:
			outi	PWM_FLUJOH,high(480)
			outi	PWM_FLUJOL,low(480)
			rjmp	salir_control
;************************************************************
PIP_Alta:
			;outi	B_PEEP_DOWN,'0'
			;outi	reg_PWM_Val_ExH,0x00
			;outi	reg_PWM_Val_ExL,0x00
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(60)
			cpci	ZH,high(60)
			rbrlo	CHK_PIP_Alta
			sbiw	Z,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			;rjmp	salir_control		///nuevo
;************************************************************
CHK_PIP_Alta:
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			subi	XL,low(-500)
			sbci	XH,high(-500)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	salir_control
			;ASIGNA_PWM_V_ex  				0,0
			outi	reg_PWM_Val_ExH,0x00
			outi	reg_PWM_Val_ExL,0x00
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			
			rjmp	salir_control
;************************************************************
;************************************************************
;************************************************************
R_PWM_1:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			adiw	ZL,5			
			cpi		ZL,low(480)
			cpci	ZH,high(480)
			rbrsh	PMW_Inc_out
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			;outi	PWM_FLUJOH,high(140)
			;outi	PWM_FLUJOL,low(140)
			rjmp	salir_control
;************************************************************
;************************************************************
CARGA_PR_OFFSET:
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		zl,offset_PreL
			inr		zh,offset_PreH
			sub		r24,zl
			sbc		r25,zh
			subi	r24,low(-300)
			sbci	r25,high(-300)
			ret

;************************************************************
;************************************************************
;************************************************************
pip_nul:				
			outi	PWM_FLUJOH,high(160)
			outi	PWM_FLUJOL,low(160)
			rjmp	salir_control

			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(450)
			cpci	ZH,high(450)
			rbrsh	salir_control;CHK_PEEP_Baja
			adiw	Z,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			;rjmp	salir_control	

			rjmp	salir_control
;************************************************************
;************************************************************

;************************************************************
////////////////fin_control_pip

;************************************************************
.macro		tst_PEEP_0
			cpri	reg_PEEPH,0x00			;Si PEEP = 0, no se ejecuta
			brne	sigue_control_PEEP
			cpri	reg_PEEPL,0x00
			brne	sigue_control_PEEP
			;ASIGNA_PWM_V_ex  				0,0;ELECTROVALVULAS			0
			outi		PWM_FLUJOH,high(PWMmin)
			outi		PWM_FLUJOL,low(PWMmin)
			rjmp		@0
sigue_control_PEEP:
.endm

.macro		tst_tiempo_final_fase
			cli
			inr		r16,cont_10msL	;Si el tiempo de la fase es menor a 20ms
			inr		r17,cont_10msH	;ya no se ejecuta el control
			sei
			cpi		r16,low(@0)
			cpci	r17,high(@0)
			rbrlo	@1
.endm

.macro		cp_Presion_cmH2OH
			inr		r17,Presion_cmH2OH
			inr		r16,Presion_cmH2OL
			inr		r25,@0
			inr		r24,@1
			cp		r16,r24
			cpc		r17,r25
.endm

.macro		cjeq_modo_presion
			cpri	Modo_Op,P_CMV
			breq	@0
			cpri	Modo_Op,P_SIMV
			breq	@0
			cpri	Modo_Op,P_CPAP
			breq	@0
.endm
;************************************************************

;************************************************************
PEEP_PrSp:
			cpri	B_PEEP_D,'1'
			rbreq	tst_PEEP
			outi	primer_cruce,'0'
			outi		B_PR_SP,'1'
			;ASIGNA_PWM_V_ex  				0,0;ELECTROVALVULAS		0
			;delay_ms	30
			;outi		PWM_FLUJOH,high(0)
			;outi		PWM_FLUJOL,low(0)
			ldiw	zh,zl,tabla_PIP*2;tabla_inc_rangoPEEP_PARKER*2
			inr		xh,Presion_cmH2OH
			inr		xl,Presion_cmH2OL;low(500);reg_PEEPL
			;low(500);reg_PEEPH
			;outr		xl,low(500)
			;outr		xl,low(500)
			lsl		xl
			rol		xh
			add		zl,xl
			adc		zh,xh
			outi	rampz,0x00
			lpm		xl,z+
			lpm		xh,z
			;ldiw	zh,zl,1
			;low(500);reg_PEEPL
			inr		zh,offset_PEEPH;low(500);reg_PEEPH
			inr		zL,offset_PEEPL
			add		xl,zl
			adc		xh,zh
			;subi	xl,low(-50);zl
			;sbci	xh,high(-50)
			outr		Ctrl_CF_H,xh
			outr		Ctrl_CF_L,xl
			outi		CCF_H,high(60)
			outi		CCF_L,low(60)
			;outi		Ctrl_CF_H,high(114)
			;outi		Ctrl_CF_L,low(114)
			outi		Cont_CF_H,high(300)
			outi		Cont_CF_L,low(300)
			outi		reg_FLUJO_compensacionH,high(flujo_inicial_compensacion)
			outi		reg_FLUJO_compensacionL,low(flujo_inicial_compensacion)
			outi		bandera_inc_compensacion,'0'
			outi		B_Dec,'0'
			outi		B_PEEP_AC,'0'
			outi		B_PEEP_D,'0'
			outi		B_PEEP_DOWN,'0'
			outi		B_PEEP_UP,'0'
			outi		B_PEEP_DC,'0'	
			outi		B_Cont,'0'
			movr		VV_X,VV_OFF
			outi		Cont_VV,50
								
			
			
			ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
			outi	PWM_FLUJOH,high(PWMZERO);;*************modificacion
			outi	PWM_FLUJOL,low(PWMZERO)

;************************************************************
tst_PEEP2:			;PEEP_Control
tst_PEEP:
			tst_PEEP_0					salir_control
			tst_tiempo_final_fase	10,	sin_flujo

peep_CPAP:

;Si es algún modo de volumen, se considera que en los primeros ciclos respiratorios
;se debe de alcanzar el nivel de PEEP, independientemente de si aun no se alcanza
;el volumen programado

		
;************************************************************
PEEP_control:
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(-100)
			sbci	XH,high(-100)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Alta
;************************************************************
			;call	CARGA_PR_OFFSET
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(100)
			sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PEEP_Baja_P
;************************************************************	
;			rjmp		salir_control
PEEP_Ok:
			outi	B_PEEP_DOWN,'0'
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			cp		R24,XL
			cpc		R25,XH
			rbrsh	check_Flujo_PEEP
			;rjmp	salir_control

;************************************************************
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPOk
			movr	Presion_antL,Presion_cmH2OL
			movr	Presion_antH,Presion_cmH2OH
			outi	B_Dec_PEEP,'1'
SRPPOk:;Skip_read_Pres_PEEP_OK
			inr		ZL,Presion_cmH2OL
			inr		ZH,Presion_cmH2OH
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(10)
			sbci	XH,high(10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	check_Flujo_PEEP
			rbreq	check_Flujo_PEEP
;************************************************************

			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	salir_control
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
check_Flujo_PEEP:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			inr		XL,offset_COMFLPEEPL
			inr		XH,offset_COMFLPEEPH
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	PEEP_Ok_out;salir_control
			sbiw	ZL,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
;************************************************************
PEEP_Ok_out:
			outi	B_Dec_PEEP,'0'
			movr	PWM_FLUJOH,offset_COMFLPEEPH
			movr	PWM_FLUJOL,offset_COMFLPEEPL
			rjmp	salir_control
;************************************************************
PEEP_Baja_P:
			sbi		led_run
			cpri	B_PEEP_DOWN,'1'
			rbreq	skip_PEEP_baja_A
			movr	Presion_antL,Presion_cmH2OL
			movr	Presion_antH,Presion_cmH2OH
			outi	B_PEEP_DOWN,'1'
skip_PEEP_baja_A:			
			inr		ZL,Presion_cmH2OL
			inr		ZH,Presion_cmH2OH
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(10)
			sbci	XH,high(10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	salir_control
;******************anterior PEEP baja*************************************
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			sbiw	XL,5
			outr	Presion_antH,XH
			outr	Presion_antL,XL
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	skip_PEEP_Baja
			adiw	XL,2
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
skip_PEEP_Baja:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(480)
			cpci	ZH,high(480)
			rbrsh	salir_control
			adiw	ZL,1
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
;*********************aca termina PEEP baja anterior**************************************
;************************************************************		
;************************************************************
PEEP_Alta:
			outi	B_PEEP_DOWN,'0'
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	skip_PEEP_Alta
			sbiw	ZL,1
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
skip_PEEP_Alta:
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPEA
			movr	Presion_antL,Presion_cmH2OL
			movr	Presion_antH,Presion_cmH2OH
			outi	B_Dec_PEEP,'1'
SRPPEA:;Skip_read_Pres_PEEP_Alta
			inr		ZL,Presion_cmH2OL
			inr		ZH,Presion_cmH2OH
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(10)
			sbci	XH,high(10)
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	out_PEEP_Alta
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(50)
			cpci	XH,high(50)
			rbrlo	salir_control
			sbiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	salir_control
;************************************************************
;************************************************************
out_PEEP_Alta:
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
;************************************************************
;************************************************************



;************************************************************
;************************************************************

sin_modificar_flujo:	;outi		bandera_inc_compensacion,'0'
						outi		reg_FLUJO_compensacionH,high(flujo_inicial_compensacion)
						outi		reg_FLUJO_compensacionL,low(flujo_inicial_compensacion)
						;ASIGNA_PWM_V_ex  			0,0;ELECTROVALVULAS		0
						;outi		PWM_FLUJOH,high(PWMmin)
						;outi		PWM_FLUJOL,low(PWMmin)
						rjmp		salir_control

;Presion es igual al umbral superior                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   

;************************************************************
;************************************************************



sobre_presion:
						outi		b_sp,'1'
						outi		tx_b_sp,'1'
						outi		presion_maxima,'1'
						
		
sin_flujo:
											
						outi	PWM_FLUJOH,high(PWMZERO);(PWMmin)
						outi	PWM_FLUJOL,low(PWMZERO)
						;outi	Cont_VV,10
						;call	Valv_ON
						;ASIGNA_PWM_V_ex  			0x01,0x00;ELECTROVALVULAS		200
						;outi 	bandera_inc_compensacion,'0'
						rjmp	salir_control
;************************************************************

libera_PS:			
			;outi		B_PR_SP,'1'
			;ASIGNA_PWM_V_ex  				0,0;ELECTROVALVULAS				0
			;delay_ms	100
			outi		PWM_FLUJOH,high(60)
			outi		PWM_FLUJOL,low(60)
			PIN_PS0
			
			cli
			outi		cont_10msL,0x0C
			outi		cont_10msH,0x00
			sei

			rjmp		salir_control
;************************************************************
;************************************************************
;************************************************************
;************************************************************
ini_Pr_Sp:
			cli
			inr		r16,cont_10msL	;Si el tiempo de la fase es menor a 20ms
			inr		r17,cont_10msH	;ya no se ejecuta el control
			sei
			cpi		r16,low(2)
			cpci	r17,high(2)
			rbrlo	sin_flujo
;************************************************************
;************************************************************
			cpri	B_PS_Ok,'1'
			rbreq	Pr_Sp_Ok
;************************************************************
;************************************************************
			inr		r16,reg_PSx100L
			inr		r17,reg_PSx100H
			inr		R24,Presion_cmH2OL
			inr		R25,Presion_cmH2OH
			;call	CARGA_PR_OFFSET
			subi	R24,low(-700)
			sbci	R25,high(-700)
			cp		R24,r16
			cpc		R25,r17
			;cpi		XL,low(2800)
			;cpci	XH,high(2800)
			rbrsh	Pr_Sp_Ok
;************************************************************
			movr	Reg_PS_TempL,Presion_cmH2OL
			movr	Reg_PS_TempH,Presion_cmH2OH
;************************************************************
;************************************************************
			inr		ZL,Cont_CF_L
			inr		ZH,Cont_CF_H
			cpi		ZL,low(2)
			cpci	ZH,high(2)
			brlo	sum_PS_PWM
			sbiw	Z,1
			outr	Cont_CF_L,ZL
			outr	Cont_CF_H,ZH
			rjmp	salir_control
;+++++++++++++++++++++++++++++++++++++++++
;+++++++++++++++++++++++++++++++++++++++++
sum_PS_PWM:
			outi	Cont_CF_H,high(20)
			outi	Cont_CF_L,low(20)
			;inr		r24,Presion_cmH2OL
			;inr		r25,Presion_cmH2OH
			call	CARGA_PR_OFFSET
			inr		XL,reg_PSx100L
			inr		XH,reg_PSx100H
			sub		XL,r24
			sbc		XH,r25	
			ldiw	zh,zl,tabla_PIP*2
			;outr		xl,low(500)
			;outr		xl,low(500)
			lsl		xl
			rol		xh
			add		zl,xl
			adc		zh,xh
			outi	rampz,0x00
			lpm		xl,z+
			lpm		xh,z
;************************************************************
			
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			;adiw	XL,3			;subi	XL,low(-10)
			;sbci	XH,high(-10)
			add		zl,XL
			adc		zh,XH
			cpi		ZL,low(480)
			cpci	ZH,high(480)
			rbrsh	PMW_Inc_out
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control

;************************************************************
Pr_Sp_Ok:
			outi	B_PS_Ok,'1'
			inr		r16,reg_PSx100L
			inr		r17,reg_PSx100H
/*			inr		R24,Presion_cmH2OL
			inr		R25,Presion_cmH2OH
			subi	R24,low(-150)
			sbci	R25,high(-150)*/
			call	CARGA_PR_OFFSET
			cp		R24,r16
			cpc		R25,r17
			rbrsh	libera_PS
;************************************************************
			inr		ZL,Cont_CF_L
			inr		ZH,Cont_CF_H
			cpi		ZL,low(2)
			cpci	ZH,high(2)
			brlo	Dec_PS_PWM
			sbiw	Z,1
			outr	Cont_CF_L,ZL
			outr	Cont_CF_H,ZH
			rjmp	salir_control
;************************************************************
Dec_PS_PWM:
			inr		r16,Reg_PS_TempL
			inr		r17,Reg_PS_TempH
			inr		R24,Presion_cmH2OL
			inr		R25,Presion_cmH2OH
			;call	CARGA_PR_OFFSET
			subi	R24,low(10)
			sbci	R25,high(10)
			cp		R24,r16
			cpc		R25,r17
			rbrlo	salir_control
;************************************************************
;************************************************************
			movr	Reg_PS_TempL,Presion_cmH2OL
			movr	Reg_PS_TempH,Presion_cmH2OH
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			inr		XL,offset_COMFLPIPL
			inr		XH,offset_COMFLPIPH
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	pip_OK_out
			sbiw	ZL,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
;************************************************************
;************************************************************
			
			outi	Cont_CF_H,high(15)
			outi	Cont_CF_L,low(15)
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			inr		XL,offset_COMFLPIPL
			inr		XH,offset_COMFLPIPH
			cp		ZL,XL
			cpc		ZH,XH
			rbrlo	pip_OK_out
			sbiw	ZL,5
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
;************************************************************
;************************************************************
