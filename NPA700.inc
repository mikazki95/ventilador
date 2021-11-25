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
			rbreq	prueba_aire100;salir_control;libera_pres
			cpri	FASE,pre_CALIBRA_O2_21
			rbreq	aire_100_ON
			cpri	FASE,CALIBRA_O2_100;
			rbreq	salir_control
			cpri	FASE,pre_CALIBRA_O2_100
			rbreq	O2_100_ON

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

libera_pres:
			outi	reg_PWM_Val_ExH,0x00
			outi	reg_PWM_Val_ExL,0x00
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200
			outi		PWM_FLUJOH,high(60)
			outi		PWM_FLUJOL,low(60)
			rjmp	salir_control

tst_P_PRUEBA:
			cpri				B_TST_Run,'0'
			rbreq				fin_tst
			
			inr		r16,cont_10msL	;Si el tiempo de la fase es menor a 20ms
			inr		r17,cont_10msH	;ya no se ejecuta el control
			cpi		r16,low(50)
			cpci	r17,high(50)
			rbrlo	verf_pres
			;sbi		led_run
			outi		tmr_U,150
			jmp		sigue_control_P_inspiracion

verf_pres:
			;outiw	cont_10msH,cont_10msL,0x000A
			;sbi		led_run
			;rjmp	salir_control
			cpri	B_PR_TST,'0'
			rbrne	sin_flujo
			outi	envia_trama_U,'1';Se habilita el envió de la trama U para la primera vez
			outi	tmr_U,150		;10ms
			outi	tx_U,'1'
			outi	B_PR_TST,'1'
			jmp		sin_flujo

			OFF_PRESION_TST
fin_tst:
			;sbi		led_run
			outi	PWM_FLUJOH,high(PWMZERO);;*************modificacion
			outi	PWM_FLUJOL,low(PWMZERO)
			outi	reg_PWM_Val_ExH,0x00
			outi	reg_PWM_Val_ExL,0x0A
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200
			jmp					salir_control

;************************************************************
prueba_aire100:
			;sbi		led_run
			jmp					salir_control
;************************************************************
aire_100_ON:
			
			
			
			jmp		control_pres_cal_gases
;************************************************************
;************************************************************
O2_100_ON:
			outi		Pres_objL,low(2000)
			outi		Pres_objH,high(2000)
			outi		B_TST_Run,'1'
			outi	reg_PWM1_O2H,high(480);;*************modificacion
			outi	reg_PWM1_O2L,low(480)
			outi	reg_PWM2_AireH,0x00;;*************modificacion
			outi	reg_PWM2_AireL,0x00
			ASIGNA_PWM_O2					reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE					reg_PWM2_AireH,reg_PWM2_AireL
			
			jmp		control_pres_cal_gases
;************************************************************
;************************************************************
control_pres_cal_gases:
			outi	reg_PWM_Val_ExH,0x00
			outi	reg_PWM_Val_ExL,0x00
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200

			call	CARGA_PR_OFFSET
			cpi		R24,low(45)
			cpci	R25,high(45)
			rbrlo	pip_nul
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			subi	XL,low(100)
			sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	Pres_cal_Alta
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			subi	XL,low(100)
			sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	salir_control
;************************************************************
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPCGA
			call	CARGA_PR_OFFSET
			outr	Presion_antH,R25 
			outr	Presion_antL,R24
			outi	B_Dec_PEEP,'1'
SRPCGA:;Skip_read_Pres_calibracion de gases
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
out_PCG_Baja:
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
;************************************************************
Pres_cal_Alta:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(60)
			cpci	ZH,high(60)
			rbrlo	CHK_PIP_Alta
			sbiw	Z,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
			rjmp	salir_control
;************************************************************
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

			;ASIGNA_PWM_FLUJO		offset_PreH,offset_PreL
			inr		YL,Modo_TST
			cpi		YL,0x03
			rbreq	control_manual
			ASIGNA_PWM_FLUJO					reg_PWM0H,reg_PWM0L
			rjmp	salir_control

control_manual:
			ASIGNA_PWM_FLUJO		Vol_TST_H,Vol_TST_L
			rjmp	salir_control




sigue_control_P_inspiracion:
			inr		Xl,Pres_objL
			inr		Xh,Pres_objH
			cpi		XL,0x00
			cpci	XH,0x00
			rbreq	libera_pres
			call	CARGA_PR_OFFSET
			cpi		R24,low(45)
			cpci	R25,high(45)
			rbrlo	pip_nul
;************************************************************
;************************************************************
PIP_control:
			
			;sbi		led_run
			;outi	inhibe_compensacion,'1'
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			subi	XL,low(50)
			sbci	XH,high(50)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PIP_Alta
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			subi	XL,low(200)
			sbci	XH,high(200)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PIP_Baja_P
;************************************************************	
			rjmp	PIP_Ok_P;	salir_control
;************************************************************
;************************************************************
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
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
			;outr	Presion_antL,R24
			;outr	Presion_antH,R25 
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-10)
			sbci	XH,high(-10)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	actualizar_Pres_PIP;;;apagado de basndera de actualizacion 
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(50)
			cpci	XH,high(50)
			rbrsh	Skip_PIP_V_OK_B
			sbiw	XL,1
			;outr	reg_PWM_Val_ExH,XH
			;outr	reg_PWM_Val_ExL,XL
Skip_PIP_V_OK_B:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(110)
			cpci	XH,high(110)
			rbrlo	actualizar_Pres_PIP
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PIP
;************************************************************
PIP_Ok_P:	
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PIP_Ok_1;actualizar_Pres_PIP
;************************************************************
			call	CARGA_PR_OFFSET
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-1)
			sbci	XH,high(-1)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PIP_Ok_1;actualizar_Pres_PIP
			call	CARGA_PR_OFFSET
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(2)
			sbci	XH,high(2)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	actualizar_Pres_PIP
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(480)
			cpci	XH,high(480)
			rbrsh	actualizar_Pres_PIP
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PIP

	
;************************************************************
PIP_Ok_1:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(40)
			cpci	XH,high(40)
			rbrlo	actualizar_Pres_PIP
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PIP
;************************************************************
;************************************************************			
;************************************************************
;pip_OK_out:
;			movr	PWM_FLUJOH,offset_COMFLPEEPH;offset_COMFLPIPH
;			movr	PWM_FLUJOL,offset_COMFLPEEPL;offset_COMFLPIPL
;			rjmp		salir_control
;************************************************************
;************************************************************
PIP_Baja_P:
			;outi	reg_PWM_Val_ExH,0x01
			;outi	reg_PWM_Val_ExL,0xF4
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200
			cpri	B_Dec_PEEP,'1'
			rbreq	SRPPIA
			call	CARGA_PR_OFFSET
			outr	Presion_antH,R25 
			outr	Presion_antL,R24
			outi	B_Dec_PEEP,'1'
SRPPIA:;Skip_read_Pres_PIP_Alta
			call	CARGA_PR_OFFSET
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-1)
			sbci	XH,high(-1)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	actualizar_Pres_PIP
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	ChF_PIP_B
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL
ChF_PIP_B:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(450)
			cpci	XH,high(450)
			rbrsh	actualizar_Pres_PIP
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PIP
;************************************************************
;************************************************************
out_PIP_Baja:
			outi	B_Dec_PEEP,'0'
			rjmp	salir_control
;************************************************************
;************************************************************


;************************************************************
PIP_Alta:
;************************************************************
			call	CARGA_PR_OFFSET
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(1)
			sbci	XH,high(1)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	actualizar_Pres_PIP
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(60)
			cpci	ZH,high(60)
			rbrlo	CHK_PIP_Alta
			sbiw	Z,3
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
CHK_PIP_Alta:
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			subi	XL,low(-500)
			sbci	XH,high(-500)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	actualizar_Pres_PIP
			sbi		led_run
			inr		Xl,PWM_FLUJOL
			inr		Xh,PWM_FLUJOH
			cpi		XL,low(50)
			cpci	XH,high(50)
			rbrlo	CHK_Flujo_PIP_Alta
			sbiw	XL,5
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;outi	PWM_FLUJOH,high(PWMZERO);;*************modificacion
			;outi	PWM_FLUJOL,low(PWMZERO)
CHK_Flujo_PIP_Alta:
			inr		Xl,reg_PWM_Val_ExL
			inr		Xh,reg_PWM_Val_ExH
			cpi		XL,low(50)
			cpci	XH,high(50)
			rbrlo	actualizar_Pres_PIP
			sbiw	XL,5
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PIP
;************************************************************
;************************************************************		
actualizar_Pres_PIP:
			call	CARGA_PR_OFFSET
			outr	Presion_antH,R25
			outr	Presion_antL,R24
			rjmp	salir_control
;************************************************************
;************************************************************
CARGA_PR_OFFSET:
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		ZL,offset_PIPL
			inr		ZH,offset_PIPH
			cp		R24,ZL
			cpc		R25,ZH
			rbrlo	out_carga_offset
			sub		r24,zl
			sbc		r25,zh
			subi	r24,low(-200)
			sbci	r25,high(-200)
			ret

;************************************************************
out_carga_offset:
			inr		R24,0x00
			inr		R25,0x00
			ret
;************************************************************
;************************************************************
pip_nul:			
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(450)
			cpci	ZH,high(450)
			rbrsh	salir_control;CHK_PEEP_Baja
			adiw	Z,1
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

prueba_gases:
			outi		B_Ctrl_Parker,'1'
			;tst_tiempo_final_fase	10,	sin_flujo
			jmp	peep_CPAP
;******************;PEEP_Control******************************
tst_PEEP:
			tst_PEEP_0					salir_control
			tst_tiempo_final_fase	20,	prueba_gases;sin_flujo

peep_CPAP:

;Si es algún modo de volumen, se considera que en los primeros ciclos respiratorios
;se debe de alcanzar el nivel de PEEP, independientemente de si aun no se alcanza
;el volumen programado

;************************************************************
PEEP_control:
			;inr		r24,Presion_cmH2OL
			;inr		r25,Presion_cmH2OH
			inr		Xl,reg_PEEPL_100porciento
			inr		Xh,reg_PEEPH_100porciento
			cpi		XL,0x00
			cpci	XH,0x00
			rbreq	libera_pres
			
			call	Carga_offset_PEEP
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(-150)
			sbci	XH,high(-150)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Alta
			
			call	Carga_offset_PEEP
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(-50)
			sbci	XH,high(-50)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Alta ;PEEP_Alta_lim
;************************************************************
			;call	CARGA_PR_OFFSET
			;inr		r24,Presion_cmH2OL
			;inr		r25,Presion_cmH2OH
			call	Carga_offset_PEEP
			inr		xl,reg_PEEPL_100porciento
			inr		xh,reg_PEEPH_100porciento
			subi	XL,low(50)
			sbci	XH,high(50)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	PEEP_Baja_P
;***********************peep ok*************************************	
;************************************************************
PEEP_Ok:	
			call	Carga_offset_PEEP
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Ok_1;actualizar_Pres_PIP
;************************************************************
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-1)
			sbci	XH,high(-1)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Ok_1;actualizar_Pres_PIP
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(2)
			sbci	XH,high(2)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	actualizar_Pres_PEEP
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(480)
			cpci	XH,high(480)
			rbrsh	actualizar_Pres_PEEP
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
	
;************************************************************
PEEP_Ok_1:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(40)
			cpci	XH,high(40)
			rbrlo	actualizar_Pres_PEEP
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			;ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
;*******************Fin peep ok*****************************************

;************************************************************
PEEP_Baja_P:
			;sbi		led_run
			;jmp		sin_flujo
;************************************************************			
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-1)
			sbci	XH,high(-1)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	PEEP_Baja_1;actualizar_Pres_PEEP;salir_control
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	PEEP_Baja_2
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL
PEEP_Baja_2:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(450)
			cpci	XH,high(450)
			rbrsh	actualizar_Pres_PEEP;salir_control
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL	
			jmp		actualizar_Pres_PEEP
;************************************************************
PEEP_Baja_1:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	PEEP_Baja_3
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL
PEEP_Baja_3:
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(-10)
			sbci	XH,high(-10)
			cp		R24,XL
			cpc		R25,XH
			rbrlo	actualizar_Pres_PEEP
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(40)
			cpci	XH,high(40)
			rbrlo	actualizar_Pres_PEEP;salir_control
			sbiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL
			jmp		actualizar_Pres_PEEP
;************************************************************
;******************anterior PEEP baja*************************************
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	skip_PEEP_Baja
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
skip_PEEP_Baja:
			inr		XL,PWM_FLUJOL
			inr		XH,PWM_FLUJOH
			cpi		XL,low(450)
			cpci	XH,high(450)
			rbrsh	actualizar_Pres_PEEP;salir_control
			adiw	XL,1
			outr	PWM_FLUJOH,XH
			outr	PWM_FLUJOL,XL


			;rjmp	salir_control
;*********************aca termina PEEP baja anterior**************************************
;************************************************************		
actualizar_Pres_PEEP:
			movr	Presion_antH,Prs_Aux9H
			movr	Presion_antL,Prs_Aux9L
			movr	Prs_Aux9H,Prs_Aux8H
			movr	Prs_Aux9L,Prs_Aux8L
			movr	Prs_Aux8H,Prs_Aux7H
			movr	Prs_Aux8L,Prs_Aux7L
			movr	Prs_Aux7H,Prs_Aux6H
			movr	Prs_Aux7L,Prs_Aux6L
			movr	Prs_Aux6H,Prs_Aux5H
			movr	Prs_Aux6L,Prs_Aux5L
			movr	Prs_Aux5H,Prs_Aux4H
			movr	Prs_Aux5L,Prs_Aux4L
			movr	Prs_Aux4H,Prs_Aux3H
			movr	Prs_Aux4L,Prs_Aux3L
			movr	Prs_Aux3H,Prs_Aux2H
			movr	Prs_Aux3L,Prs_Aux2L
			movr	Prs_Aux2H,Prs_Aux1H
			movr	Prs_Aux2L,Prs_Aux1L
			movr	Prs_Aux1H,Prs_Aux0H
			movr	Prs_Aux1L,Prs_Aux0L
			call	Carga_offset_PEEP
			outr	Prs_Aux0H,R25
			outr	Prs_Aux0L,R24
			rjmp	salir_control
;************************************************************		
;************************************************************
PEEP_Alta:
			;rjmp	salir_control
			
;************************************************************
			call	Carga_offset_PEEP ;44
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			inr		ZL,Vol_TST_L
			inr		ZH,Vol_TST_H
			sub		XL,ZL
			sbc		XH,ZH
			;add		XL,ZL
			;adc		XH,ZH
			;subi	XL,low(6)
			;sbci	XH,high(6)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	CHK_PEEP_Alta_2;out;actualizar_Pres_PEEP;CHK_PEEP_Alta;SRPPEA
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	CHK_PEEP_Alta_out;CHK_PEEP_Alta
			sbiw	ZL,2
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
;************************************************************
CHK_PEEP_Alta_out:
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			inr		ZL,PWM_O2_TST_L
			inr		ZH,PWM_O2_TST_H
			sub		XL,ZL
			sbc		XH,ZH
			;subi	XL,low(100)
			;sbci	XH,high(100)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	actualizar_Pres_PEEP
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	actualizar_Pres_PEEP;salir_control
			adiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
;************************************************************
CHK_PEEP_Alta_2:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	CHK_PEEP_Alta_3;CHK_PEEP_Alta
			sbiw	ZL,2
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
CHK_PEEP_Alta_3:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			inr		ZL,offset_COMFLPEEPL
			inr		ZH,offset_COMFLPEEPH
			cp		XL,ZL
			cpc		XH,ZH
			rbrlo	actualizar_Pres_PEEP;salir_control
			;sbi		led_run
			sbiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
;************************************************************
CHK_PEEP_Alta:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			inr		ZL,offset_COMFLPIPL
			inr		ZH,offset_COMFLPIPH
			cp		XL,ZL
			cpc		XH,ZH
			rbrlo	actualizar_Pres_PEEP;salir_control
			;sbiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP;salir_control
;************************************************************

PEEP_Alta_lim:
			;rjmp	actualizar_Pres_PEEP
;************************************************************
			call	Carga_offset_PEEP ;44
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(1)
			sbci	XH,high(1)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	CHK_PEEP_Alta_lim_2;actualizar_Pres_PEEP;out;CHK_PEEP_Alta;SRPPEA
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	CHK_PEEP_Alta_lim_out;CHK_PEEP_Alta
			sbiw	ZL,1
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
;************************************************************
CHK_PEEP_Alta_lim_out:
			call	Carga_offset_PEEP
			inr		XL,Presion_antL
			inr		XH,Presion_antH
			subi	XL,low(4)
			sbci	XH,high(4)
			cp		R24,XL
			cpc		R25,XH
			rbrsh	actualizar_Pres_PEEP
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			cpi		XL,low(500)
			cpci	XH,high(500)
			rbrsh	actualizar_Pres_PEEP;salir_control
			adiw	XL,3
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
;************************************************************
CHK_PEEP_Alta_lim_2:
			inr		ZL,PWM_FLUJOL
			inr		ZH,PWM_FLUJOH
			cpi		ZL,low(40)
			cpci	ZH,high(40)
			rbrlo	CHK_PEEP_Alta_lim_3;CHK_PEEP_Alta
			sbiw	ZL,2
			outr	PWM_FLUJOH,ZH
			outr	PWM_FLUJOL,ZL
CHK_PEEP_Alta_lim_3:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			inr		ZL,offset_COMFLPEEPL
			inr		ZH,offset_COMFLPEEPH
			cp		XL,ZL
			cpc		XH,ZH
			rbrlo	actualizar_Pres_PEEP;salir_control
			sbiw	XL,2
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP
;************************************************************
CHK_PEEP_Alta_lim:
			inr		XL,reg_PWM_Val_ExL
			inr		XH,reg_PWM_Val_ExH
			inr		ZL,offset_COMFLPIPL
			inr		ZH,offset_COMFLPIPH
			cp		XL,ZL
			cpc		XH,ZH
			rbrlo	actualizar_Pres_PEEP;salir_control
			sbiw	XL,1
			outr	reg_PWM_Val_ExH,XH
			outr	reg_PWM_Val_ExL,XL
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	
			rjmp	actualizar_Pres_PEEP;salir_control
;************************************************************
Carga_offset_PEEP:
			inr		r24,Presion_cmH2OL
			inr		r25,Presion_cmH2OH
			inr		ZL,offset_PEEPL
			inr		ZH,offset_PEEPH
			cp		R24,ZL
			cpc		R25,ZH
			rbrlo	out_carga_offset
			sub		r24,zl
			sbc		r25,zh
			subi	r24,low(-300)
			sbci	r25,high(-300)
			ret
;************************************************************
;***************************fin control pep******************



sobre_presion:
						outi		b_sp,'1'
						outi		tx_b_sp,'1'
						outi		presion_maxima,'1'
						
		
sin_flujo:
											
			outi	PWM_FLUJOH,high(PWMZERO);(PWMmin)
			outi	PWM_FLUJOL,low(PWMZERO)
			outi	reg_PWM_Val_ExH,0x01
			outi	reg_PWM_Val_ExL,0xF4
			ASIGNA_PWM_V_ex  				reg_PWM_Val_ExH,reg_PWM_Val_ExL	;electrovalvulas		200
			rjmp	salir_control
;************************************************************
;************************************************************
;********************sobre presion***************************
;************************************************************
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
;*********************Presion soporte ***********************
;************************************************************
ini_Pr_Sp:
			rjmp	sigue_control_P_inspiracion
			cli
			inr		r16,cont_10msL	;Si el tiempo de la fase es menor a 20ms
			inr		r17,cont_10msH	;ya no se ejecuta el control
			sei
			cpi		r16,low(4)
			cpci	r17,high(4)
			rbrlo	salir_control


			cpri	B_PS_Ok,'0'
			breq	Pr_Sp_control
			cpri	B_PS_Ok,'2'
			breq	Pr_Sp_control
			;jmp		Pr_Sp_control
			outiw	cont_10msH,cont_10msL,0x0040
			outi	B_PS_Ok,'2'
			;rjmp	sin_flujo
Pr_Sp_control:
			call	CARGA_PR_OFFSET
			inr		xl,Pres_objL
			inr		xh,Pres_objH
			cp		R24,XL
			cpc		R25,XH
			rbrsh	out_Pr_Sp
			rjmp	sigue_control_P_inspiracion
out_Pr_Sp:
			outi	B_PS_Ok,'1'
			rjmp	salir_control
;+++++++++++++++++++++++++++++++++++++++++
;+++++++++++++++++++++++++++++++++++++++++


;************************************************************
;************************************************************
