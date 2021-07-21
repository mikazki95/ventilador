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
			ELECTROVALVULAS						0
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

			scl0
;read_i2c:
			rcall	rdbyte_i2c
			outr	NPA_700H,r24


			scl0			;Envia ACK
			sda0
			delay_I2C
			scl1
			delay_I2C
			scl0
			sda1

			rcall	rdbyte_i2c
			outr	NPA_700L,r24


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

			
			ldiw	xh,xl,0x01F4
			inr		r16,Presion_cmH2OL
			inr		r17,Presion_cmH2OH
			cp		r16,Xl
			cpc		r17,Xh		
			rbrlo	pip_nul
			;cpri	inhibe_compensacion,'1'
			;rbreq	sin_flujo;salir_control
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
			subi	XL,low(25)
			sbci	XH,high(25)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PIP_Alta
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,reg_Pinspiracionx100L
			inr		xh,reg_Pinspiracionx100H
			subi	XL,low(75)
			sbci	XH,high(75)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PIP_Baja
;************************************************************	
;			rjmp		salir_control
			outi	B_PEEP_DOWN,'0'
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
PIP_Baja:
			;rjmp	salir_control
			;call	Valv_ON
			electrovalvulas		200
			;inr		yl,CNT_P_PIP
			;cpi		yl,0x03
			;brsh	PIP_Baja_C
			cpri	B_PEEP_DOWN,'0'
			brne	Inc_PWM_PIP
			;movr	PWM_FLUJOL,Ctrl_CF_L;offset_COMFLPIPL
			;movr	PWM_FLUJOH,Ctrl_CF_H;offset_COMFLPIPH
			;inr		yl,CNT_P_PIP
			;inc		yl
			;outr	CNT_P_PIP,yl
			outi	B_PEEP_DOWN,'1'
			inr		yl,C_PEEP_DOWN
			inc		yl
			outr	C_PEEP_DOWN,yl
			outi	Cont_CF_H,high(50)
			outi	Cont_CF_L,low(50)
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
;			sbi		led_run	
			outi	Cont_CF_H,high(50)
			outi	Cont_CF_L,low(50)
			;rjmp	salir_control

;************************************************************
			cpri	C_PEEP_DOWN,4
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
			electrovalvulas	0
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
			ret

;************************************************************
;************************************************************
;************************************************************
pip_nul:				
			outi	PWM_FLUJOH,high(160);(PWMmin)
			outi	PWM_FLUJOL,low(160)
			rjmp		salir_control
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
			ELECTROVALVULAS			0
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
			ELECTROVALVULAS		0
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

			cjeq_modo_presion	skip_tst_primer_PEEP
			cpri				reg_primer_PEEP,'1'
			rbreq				primer_PEEP

skip_tst_primer_PEEP:

						cpri	B_PEEP_D,'1'
						rbreq	peep_control
skip_CPAP://		cp_Presion_cmH2OH	reg_PEEPH_100porciento,reg_PEEPL_100porciento

						inr		r17,Presion_cmH2OH
						inr		r16,Presion_cmH2OL
						inr		r25,reg_PEEPH_100porciento
						inr		r24,reg_PEEPL_100porciento
						subi	r24,low(100)						;modificacion
						sbci	r25,high(100)
						cp		r16,r24
						cpc		r17,r25
						rbrlo	Primer_PEEP_D;peep_control;



						ELECTROVALVULAS		0						
						inr		XH,Presion_cmH2OH
						inr		XL,Presion_cmH2OL
						inr		r16,reg_PEEPL_100porciento
						inr		r17,reg_PEEPH_100porciento
						subi	r16,low(-1000)						;modificacion
						sbci	r17,high(-1000)
						rbrsh	salir_control;Mayor
						

						cpri	primer_cruce,'1'
						breq	dec_CF
						outi	primer_cruce,'1'
						movr	PWM_FLUJOH,Ctrl_CF_H;(PWMmin)
						movr	PWM_FLUJOL,Ctrl_CF_L;
						rjmp	salir_control


dec_CF:					
						inr		XL,Cont_CF_L
						inr		XH,Cont_CF_H
						cpi		XL,low(2)
						cpci	XH,high(2)	
						rbrlo	amr_curva
						subi	XL,low(1)
						sbci	XH,high(1)
						outr	Cont_CF_H,XH
						outr	Cont_CF_L,XL
						rjmp	salir_control

amr_curva:
						;call	valv_on
						;rjmp	salir_control
						inr		ZL,PWM_FLUJOL
						inr		ZH,PWM_FLUJOH
						cpi		ZL,low(60)
						cpci	ZH,high(60)
						rbrlo	salir_control
						sbiw	Z,1
						outr	PWM_FLUJOH,ZH
						outr	PWM_FLUJOL,ZL
						rjmp	salir_control
;************************************************************
;************************************************************

Primer_PEEP_D:			;ELECTROVALVULAS		200
						;call	Valv_ON
;************************************************************
						inr		r25,Presion_cmH2OH
						inr		r24,Presion_cmH2OL
						inr		xl,reg_PEEPL_100porciento
						inr		xh,reg_PEEPH_100porciento
						subi	XL,low(200)
						sbci	XH,high(200)
						cp		R24,XL
						cpc		R25,XH
						rbrsh	PEEP_Control_in
;************************************************************
						inr		XL,Cont_CF_L
						inr		XH,Cont_CF_H
						cpi		XL,low(2)
						cpci	XH,high(2)	
						rbrlo	PEEP_Control
						subi	XL,low(2)
						sbci	XH,high(2)
						outr	Cont_CF_H,XH
						outr	Cont_CF_L,XL
						cpri	B_PEEP_UP,'1'
						breq	amr_curva
;************************************************************
						outi	B_PEEP_UP,'1'
						outi	Cont_CF_H,high(200)
						outi	Cont_CF_L,low(200)
						rjmp	amr_curva
						//delay_ms	35
						;outi		PWM_FLUJOH,high(40);(PWMmin)
						;outi		PWM_FLUJOL,low(40);(PWMmin)

						;outi 	B_PEEP_D,'1'
						rjmp		salir_control

;************************************************************
PEEP_Control_in:		
						outi	B_PEEP_D,'1'
						rjmp	sin_flujo
;************************************************************
PEEP_control:
			;outi	inhibe_compensacion,'1'
			;call	CARGA_PR_OFFSET
			outi	B_PEEP_D,'1'
			;rjmp	salir_control
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(-10)
			sbci	XH,high(-10)
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
			rbrlo	PEEP_Baja
;************************************************************	
;			rjmp		salir_control
PEEP_Ok:
			call	Valv_ON
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
			movr	PWM_FLUJOH,offset_COMFLPEEPH
			movr	PWM_FLUJOL,offset_COMFLPEEPL
			rjmp	salir_control
;************************************************************
;************************************************************
PEEP_Baja:
			;electrovalvulas		200
			;rjmp	salir_control
			;outi	B_C_VV,'0'

			call	Valv_ON
			;rjmp	salir_control
			;electrovalvulas		200
			cpri	B_PEEP_DOWN,'1'
			breq	Inc_PWM_PEEP
			;movr	PWM_FLUJOL,Ctrl_CF_L;offset_COMFLPIPL
			;movr	PWM_FLUJOH,Ctrl_CF_H;offset_COMFLPIPH
			inr		ZL,offset_COMFLPEEPL
			inr		ZH,offset_COMFLPEEPH
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			outi	B_PEEP_DOWN,'1'
			outi	Cont_CF_H,high(50)
			outi	Cont_CF_L,low(50)
			;rjmp	salir_control
;************************************************************
;************************************************************
Inc_PWM_PEEP:
			inr		ZL,Cont_CF_L
			inr		ZH,Cont_CF_H
			cpi		ZL,low(2)
			cpci	ZH,high(2)
			brlo	R_PWM_PEEP
			sbiw	Z,1
			outr	Cont_CF_L,ZL
			outr	Cont_CF_H,ZH

			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(200)
			sbci	XH,high(200)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	cerrar_valvula
			rjmp	salir_control
;************************************************************
cerrar_valvula:
			electrovalvulas	200
			rjmp	salir_control
;************************************************************
R_PWM_PEEP:		
;			sbi		led_run	
			outi	Cont_CF_H,high(50)
			outi	Cont_CF_L,low(50)
			;rjmp	salir_control

;************************************************************			
			
			;call	CARGA_PR_OFFSET
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
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
			cpi		ZL,low(250)
			cpci	ZH,high(250)
			rbrsh	PMW_Inc_out_PEEP
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
			
;************************************************************
PMW_Inc_out_PEEP:
			outi	PWM_FLUJOH,high(254)
			outi	PWM_FLUJOL,low(254)
			rjmp	salir_control
;************************************************************
PEEP_Alta:
			
			;rjmp	salir_control
			call	valv_off
			;outi 	recarga_PWM,0x10
			;rjmp	salir_control
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	CHK_PEEP_Alta
			sbiw	Z,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			;rjmp	salir_control		///nuevo
;************************************************************
CHK_PEEP_Alta:
			;call	CARGA_PR_OFFSET
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(-150)
			sbci	XH,high(-150)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	salir_control
			call	valv_off
			;electrovalvulas	0
			rjmp	salir_control
;************************************************************
Valv_ON:	
			outi	B_C_VV,'0'
			inr		xl,recarga_PWM
			cpi		xl,199
			brsh	skip_valv_on
			inc		xl
			outr	recarga_PWM,xl
			ret
;************************************************************
skip_valv_on:
			outi	recarga_PWM,200
			ret
;************************************************************
valv_off:
;B_Cont
			;movr	recarga_PWM,VV_X
			;ret


			

			cpri	B_C_VV,'0'
			breq	Valv_off_s_mod

			inr		ZL,Cont_CF_L
			inr		ZH,Cont_CF_H
			cpi		ZL,low(2)
			cpci	ZH,high(2)
			brlo	dec_vv
			sbiw	Z,1
			outr	Cont_CF_L,ZL
			outr	Cont_CF_H,ZH
			ret


;************************************************************
;************************************************************
dec_vv:
			
			outi	Cont_CF_H,high(20)
			outi	Cont_CF_L,low(20)
			inr		xl,recarga_PWM
			cpi		xl,20
			brlo	skip_valv_off
			inr		xl,recarga_PWM
			subi	xl,1
			outr	recarga_PWM,xl
			ret
;************************************************************
;************************************************************
skip_valv_off:
			outi	recarga_PWM,10
			ret
;************************************************************
AR_valv_on:	movr	recarga_PWM,VV_OFF
			ret
;************************************************************
Valv_off_s_mod:
			outi	Cont_CF_H,high(15)
			outi	Cont_CF_L,low(15)
			outi	B_C_VV,'1'
			movr	recarga_PWM,VV_OFF
			ret
;************************************************************
;************************************************************
Cp_PEEP_UP:				
					//	rjmp	salir_control
						cpri	B_PEEP_UP,'2'
						rbrne	salir_control //Cp_PEEP_UP_2
						outi	B_PEEP_UP,'3'
						rjmp	salir_control



D_CCF:					;rjmp	salir_control
						;delay_ms	1
						inr		xl,Cont_CF_L
						inr		xh,Cont_CF_H
						cpi		xl,0x01
						cpci	xh,0x00
						rbrlo	D_CF
						sbiw	xl,1
						outr	Cont_CF_L,xl
						outr	Cont_CF_H,xh
						rjmp	salir_control
D_CF:					outi	Cont_CF_H,high(20)
						outi	Cont_CF_L,low(20)
						inr		xl,CCF_L
						inr		xh,CCF_H
						subi	xl,low(90)
						sbci	xh,high(90)
						rbreq	salir_control//sin_modificar_flujo
						rbrlo	salir_control//sin_modificar_flujo

						inr		xl,CCF_L
						inr		xh,CCF_H
						sbiw	xl,3
						outr	CCF_L,xl
						outr	CCF_H,xh
						;outi	PWM_FLUJOH,high(0)
						;outi	PWM_FLUJOL,low(0)
						movr	PWM_FLUJOH,CCF_H	
						movr	PWM_FLUJOL,CCF_L
						//ret

						rjmp		salir_control

PEEP_BAJO:
						//outi	B_PEEP_BAJO_AC,'1'
						outi	CCF_H,high(150)
						outi	CCF_L,low(150)
						rjmp		salir_control	


sin_modificar_flujo:	;outi		bandera_inc_compensacion,'0'
						outi		reg_FLUJO_compensacionH,high(flujo_inicial_compensacion)
						outi		reg_FLUJO_compensacionL,low(flujo_inicial_compensacion)
						ELECTROVALVULAS		0
						;outi		PWM_FLUJOH,high(PWMmin)
						;outi		PWM_FLUJOL,low(PWMmin)
						rjmp		salir_control

;Presion es igual al umbral superior                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
cierra_valvula_PEEP:
						;outi		bandera_inc_compensacion,'0'
						outi		B_PEEP_AC,'1'
						ELECTROVALVULAS		200
						delay_ms	30
						outi		PWM_FLUJOH,high(PWMmin)
						outi		PWM_FLUJOL,low(PWMmin)
;						delay_ms	100
						rjmp		salir_control

;Presion es menor al umbral superior, se procede a verificar el umbral inferior








;Presion es menor al umbral inferior
activa_flujo:			cpri	bandera_inc_compensacion,'0'
						breq	skip_inc_compensacion
						inr		r17,reg_FLUJO_compensacionH
						inr		r16,reg_FLUJO_compensacionL
						subi	r16,low(-1)		;(-10)
						sbci	r17,high(-1)	;(-10)
						cpi		r16,low(limite_de_compensacion)
						cpci	r17,high(limite_de_compensacion)
						brsh	skip_actualiza
						outr	reg_FLUJO_compensacionH,r17
						outr	reg_FLUJO_compensacionL,r16
skip_actualiza:			rjmp	skip_bandera_1

skip_inc_compensacion:	;outi	bandera_inc_compensacion,'1'
skip_bandera_1:			ELECTROVALVULAS		200
;////////////////////delay
;						delay_ms	100
						inr		r17,reg_FLUJO_compensacionH
						inr		r16,reg_FLUJO_compensacionL
						outr	PWM_FLUJOH,r17
						outr	PWM_FLUJOL,r16
;						outi		PWM_FLUJOH,high(PWMmin)	;420 para 5 a 10 PEEP
;						outi		PWM_FLUJOL,low(PWMmin)
						rjmp				salir_control
;************************************************************

;************************************************************
;La siguiente sección se ejecuta solo en la siguientes condiciónes:
;	1. Algún modo de volumen
;	2. Aun no se alcanza la primera vez el nivel de PEEP
primer_PEEP:
			ELECTROVALVULAS						200
			inr		XH,Presion_cmH2OH
			inr		XL,Presion_cmH2OL
			inr		ZL,reg_PEEPL_100porciento
			inr		ZH,reg_PEEPH_100porciento
			cp		XL,ZL
			cpc		XH,ZH
			rbrsh	primer_PEEP_ok
			outi		PWM_FLUJOH,high(160)/////modifica
			outi		PWM_FLUJOL,low(160)			
			rjmp	salir_control

primer_PEEP_ok:
			outi		PWM_FLUJOH,high(PWMmin)
			outi		PWM_FLUJOL,low(PWMmin)
			outi	reg_primer_PEEP,'0'
			rjmp	salir_control
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
						ELECTROVALVULAS		200
						;outi 	bandera_inc_compensacion,'0'
						rjmp	salir_control
;************************************************************

libera_PS:			
			;outi		B_PR_SP,'1'
			ELECTROVALVULAS				0
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
