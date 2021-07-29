;3.0
;15 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;Asignación de pines
;#define		CPAP_0				porta,0
;#define		CPAP_1				porta,1
;#define		CPAP_2				porta,2

#define		SCK					pinb,1	;SCK, MOSI y MISO, son solo para programación
#define		MOSI				pinb,2
#define		MISO				portb,3
#define		PCINT_5				pinb,5	;Int de sobrepresíón
#define		PCINT_6				pinb,6	;Int para validar el control de volumen
#define		PWM0				portb,7	;OC1C, control de flujo y presión

#define		NO_BREAK_AC_BAT		pinc,0

;#define		ALARMA_PRESION_MAX	pind,0	;Señal proveniente de la tarjeta SV. Si se detecta
										;un pulso en este pin, se cancela la fase de
										;inspiración sin importar el modo de operación.
;#define		INT_RETRO			pind,1	;Señal de disparo de la retroalimentación
										;La función a realizar, dependera de
										;la combinación de los pines PL0 y PL1
#define		RXD_SV				pind,2
#define		TXD_SV				portd,3
#define		T_PAUSA				portd,4	;Indica que esta activo el tiempo pausa
#define		ALARMA				portd,5	;Salida de alarma audible
#define		ELECTROVALVULA0		portd,6
#define		ELECTROVALVULA1		portd,7

#define		RXD_Raspberry		pine,0
#define		TXD_Raspberry		porte,1

#define		NO_BREAK_VBAT		pinf,1
#define		O2					pinf,2

#define		led_run				ping,0

#define		INC_PWM				pinl,0
#define		DEC_PWM				pinl,1
#define		PIN_PS				portl,2		;Indicador de presión soporte
#define		PWM1				portl,4		;OC5B, control de concentración de O2
#define		PWM2				portl,5		;OC5C, control de paso de aire
#define		PIN_INSP_EXHA		portl,6		;Señalización de inspiración/exhalación para
											;la tarjeta SV

#define		STANDBY				'0'
#define		INSPIRACION			'1'
#define		TIEMPO_PAUSA		'2'
#define		EXHALACION			'3'
#define		PRESION_SOPORTE		'4'
;#define		PAUSA				'5'	;Pausa generada en caso de que terine el ciclo
									;respiratorio con la presion soporte activa
									;y debe de iniciar la inspiracion
#define		TIEMPO_PAUSA_PS		'6'

#define		CPAP_SIN_APNEA		'7'
;#define		CPAP_APNEA			'8'

#define		TST_ELECTROVALVULAS	'9'

;#define		TST_ELEC_1			'5'
;#define		ESPERA				'E'

#define		PRE_TST_ELECTROVALVULAS	'P'


#define		CALIBRA_O2_21		'H'
#define		CALIBRA_O2_100		'L'

#define		PWM_FLUJOH			ocr1ch
#define		PWM_FLUJOL			ocr1cl
#define		PWM_O2H				ocr5bh
#define		PWM_O2L				ocr5bl
#define		PWM_AIREH			ocr5ch
#define		PWM_AIREL			ocr5cl

.macro		INSP_EXHA
			push		r25
			ldi			r25,@0
			cpi			r25,1
			brne		borra_insp_exha
			inr			r25,portl
			ori			r25,0b01000000
			outr		portl,r25
			rjmp		salir_macro_insp_exha
borra_insp_exha:
			inr			r25,portl
			andi		r25,0b10111111
			outr		portl,r25
salir_macro_insp_exha:
			pop			r25
.endm

.macro		PIN_PS1
			inr			r25,portl
			ori			r25,0b00000100
			outr		portl,r25
.endm

.macro		PIN_PS0
			inr			r25,portl
			andi		r25,0b11111011
			outr		portl,r25
.endm

.macro		ASIGNA_TIEMPO
			.if		@0==0 & @1==0
					outiw		cont_10msH,cont_10msL,0x0000
			.else
					inr			r24,@1
					inr			r25,@0
					outr		cont_10msH,r25
					outr		cont_10msL,r24
			.endif
.endm

.macro		ASIGNA_TIEMPO_10ms
					outiw		cont_10msH,cont_10msL,0x0001
.endm

.macro		ASIGNA_TIEMPO_3p99s
					outiw		cont_10msH,cont_10msL,399
.endm

.macro		ASIGNA_PWM_FLUJO
			.if		@0==0 & @1==0
					outiw		PWM_FLUJOH,PWM_FLUJOL,0x0000
			.else
					inr			r24,@1
					inr			r25,@0
					outr		PWM_FLUJOH,r25
					outr		PWM_FLUJOL,r24
			.endif
.endm

.macro		ASIGNA_PWM_FLUJO_MAX
					ldi			r24,low(PWMmax)
					ldi			r25,high(PWMmax)
					outr		PWM_FLUJOH,r25
					outr		PWM_FLUJOL,r24
.endm

.macro		ASIGNA_PWM_O2
			.if		@0==0 & @1==0
					outiw		PWM_O2H,PWM_O2L,0x0000
			.else
;					outiw		PWM_O2H,PWM_O2L,0x0000
					inr			r24,@1
					inr			r25,@0
					outr		PWM_O2H,r25
					outr		PWM_O2L,r24
			.endif
.endm

.macro		ASIGNA_PWM_AIRE
			.if		@0==0 & @1==0
					outiw		PWM_AIREH,PWM_AIREL,0x0000
			.else
					inr			r24,@1
					inr			r25,@0
					outr		PWM_AIREH,r25
					outr		PWM_AIREL,r24
			.endif
.endm

;************************************************************
;cp_limites	regH,regL,'n',Valor_max
;Si se especifica 'n', indica que solo hay limite maximo.
;En este caso si el valor de los registros (reg) es menor o igual a Valor_max,
;continua en la siguiente linea, en caso contrario salta a @4
;
;cp_limites	regH,regL,Valor_min,Valor_max
;En este caso si el valor de los registros (reg) es mayor o igual a Valor_min
; "Y" menor o igual a Valor_max, continua en la siguiente linea, en caso
;contrario salta a @4
.macro		cp_limites
			inr			r16,@1
			inr			r17,@0
			.if		@2=='n'
				cpi			r16,low(@3)
				cpci		r17,high(@3)
				rbrsh		@4
			.else
				cpi			r16,low(@2)
				cpci		r17,high(@2)
				rbrlo		@4
				cpi			r16,low(@3+1)			;El valor no debe de exceder a 59.9seg
				cpci		r17,high(@3+1)
				rbrsh		@4
			.endif
.endm
;*********************************************************************

;*********************************************************************
.macro		FASE_STANDBY
;			outi	inhibe_inc_dec_pwm,'1'
			outi	tmr_inhibe_on_off_pwm,		0
			outi	inhibe_on_off_pwm,			'0'
			OUTI	FASE,						STANDBY
			INSP_EXHA							0
			PIN_PS0
			CBI		T_PAUSA
			ASIGNA_TIEMPO						0,0
;			cpri	presion_liberada,'1'
;			breq	fin_macro_standby
			outi	reg_primer_PEEP,'1'			;Este registro se usa en los modos
												;de control por volumen, para que
												;el primer control del PEEP, se
												;controle diferente
			outi	inhibe_compensacion_primer_PEEP,'0'

			ASIGNA_PWM_FLUJO					0,0
			ELECTROVALVULAS						0
			ASIGNA_PWM_O2						0,0
			ASIGNA_PWM_AIRE						0,0
fin_macro_standby:
			.endm

.macro		FASE_INSPIRACION
			outi		B_Vol,'0'
			outi		B_C_VV,'0'
			outi		b_sp,'0'
			outi		ventana,'1'
			outi		B_PIP_OK,'0'
			outi		B_PEEP_DOWN,'1'
			outi		B_PEEP_UP,'0'
			outi		tmr_inhibe_on_off_pwm,		3			; -> 3 int de 10ms
			outi		inhibe_on_off_pwm,			'1'
			OUTI		FASE,						INSPIRACION
			INSP_EXHA			1
			PIN_PS0
			////nuevo
			outi		Cont_CF_H,high(50)
			outi		Cont_CF_L,low(50)
			





			CBI		T_PAUSA
			outi	presion_maxima,				'0'
					outi		bandera_Tinsp_50p,'0'
					outi	bandera_inc_PWM_inspiracion,'0'
			outi	inhibe_compensacion,'0'
			ASIGNA_TIEMPO						tmr_TIEMPO_INSH,tmr_TIEMPO_INSL
			cpri	Modo_Op,P_CMV
			breq	asigna_inspiracion_0
			cpri	Modo_Op,P_SIMV
			breq	asigna_inspiracion_0
			cpri	Modo_Op,P_CPAP
			breq	asigna_inspiracion_0
					ASIGNA_PWM_FLUJO					reg_PWM0H,reg_PWM0L
			rjmp	skip_asigna_0
asigna_inspiracion_0:

			;outi		PWM_FLUJOH,high(120)
			;outi		PWM_FLUJOL,low(120)
			movr	PWM_FLUJOH,offset_PiPH
			movr	PWM_FLUJOL,offset_PiPL
			;ASIGNA_PWM_FLUJO					0,0
skip_asigna_0:
			ELECTROVALVULAS						PWM_VV_Max
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL

fin_macro_inspiracion:
;					outi		tmr_de_presion,0
			.endm

.macro		FASE_PRESION_SOPORTE
			outi	B_PR_SP,'0'
			outi	B_PS_Ok,'0'
			movr	PWM_FLUJOH,offset_PiPH
			movr	PWM_FLUJOL,offset_PiPL
			outi	Cont_CF_H,high(10)
			outi	Cont_CF_L,low(10)

			;outi		reg_PSx100H,high(3000)
			;outi		reg_PSx100L,low(3000)



c_PS:		outi	tmr_inhibe_on_off_pwm,		3
			outi	inhibe_on_off_pwm,			'1'
			OUTI	FASE,						PRESION_SOPORTE
			INSP_EXHA							1
			PIN_PS1
			CBI		T_PAUSA
			
			;outiw		cont_10msH,cont_10msL,0x0196
			ASIGNA_TIEMPO						tmr_TIEMPO_INSH,tmr_TIEMPO_INSL;0x01,0x90;0x17,0x2F;
			ELECTROVALVULAS						PWM_VV_Max
			ASIGNA_PWM_O2					  	reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
fin_macro_presion_soporte:
			.endm

.macro		FASE_CPAP_SIN_APNEA
			outi	tmr_inhibe_on_off_pwm,		3			; -> 3 int de 10ms
			outi	inhibe_on_off_pwm,			'1'
			OUTI	FASE,						CPAP_SIN_APNEA
			;INSP_EXHA							1	;0
			PIN_PS0
			CBI		T_PAUSA
			ASIGNA_TIEMPO_10ms					;tmr_TIEMPO_INSH,tmr_TIEMPO_INSL

			;outi		PWM_FLUJOH,high(PWMmin)
			;outi		PWM_FLUJOL,low(PWMmin)
			;ELECTROVALVULAS						100
			outi	primer_cruce,'0'

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
			outi	PWM_FLUJOL,low(PWMZERO);;		




			outi	CPAP_alarma_APNEA,			'0'
		//			outi	porta,				0b00000001
fin_macro_sin_apnea:
			.endm

.macro		FASE_TIEMPO_PAUSA
			outi	tmr_inhibe_on_off_pwm,		3
			outi	inhibe_on_off_pwm,			'1'
			OUTI	FASE,						TIEMPO_PAUSA
			PIN_PS0
			SBI									T_PAUSA
			ASIGNA_TIEMPO						tmr_TIEMPOpausaH,tmr_TIEMPOpausaL

			outi	PWM_FLUJOH,high(PWMmin);(PWMmin)
			outi	PWM_FLUJOL,low(PWMmin)
			ELECTROVALVULAS		PWM_VV_Max;
fin_macro_tiempo_pausa:
			.endm

;El tiempo pausa de PS en realidad es parte de la exhalación
;Se combinan estados del tiempo pausa de un ciclo normal con los valores
;para exhalación
.macro		FASE_TIEMPO_PAUSA_PS
			outi	tmr_inhibe_on_off_pwm,		3
			outi	inhibe_on_off_pwm,			'1'
			OUTI	FASE,						TIEMPO_PAUSA_PS
			;INSP_EXHA							0
			PIN_PS0
			SBI									T_PAUSA
			ASIGNA_TIEMPO						tmr_TIEMPOpausaH,tmr_TIEMPOpausaL
//			cpri	presion_liberada,'1'
//			breq	fin_macro_tiempo_pausa
			ASIGNA_PWM_FLUJO					0,0;reg_PWM0H_6p25p,reg_PWM0L_6p25p
			ELECTROVALVULAS						0		;Combinación para Tiempo pausa
			ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
fin_macro_tiempo_pausa:
			.endm

;*********************************************************************
.macro		electrovalvulas
			outi	recarga_PWM,@0
.endm
;*********************************************************************

.macro		FASE_EXHALACION
			outi	tmr_inhibe_on_off_pwm,		3
			outi	inhibe_on_off_pwm,			'1'
			;outi	b_sp,'0'
			OUTI	FASE,						EXHALACION
			INSP_EXHA							0
			PIN_PS0
			CBI		T_PAUSA
			ASIGNA_TIEMPO						tmr_TIEMPO_EXPH,tmr_TIEMPO_EXPL
;			cpri	presion_liberada,'1'
;			breq	fin_macro_exhalacion
			
					outi	primer_cruce,'0'

					ELECTROVALVULAS		0
					outi		B_C_VV,'0'
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
					outi		C_PEEP_DOWN,0
					
					
					ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
					ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
					outi	PWM_FLUJOH,high(PWMZERO);;*************modificacion
					outi	PWM_FLUJOL,low(PWMZERO);;


fin_macro_exhalacion:
			.endm

;Esta fase se definio para el caso particular de rx el comando M, mientras el ventilador
;ya estaba trabajando, por lo cual se inhibe la interrupción mientras se asignan los
;nuevos valores o se borran si se recibio la orden de detener el ventilador.
/*
.macro		FASE_ESPERA
			outi		cont_10msH,high(20)
			outi		cont_10msL,low (20)
			ASIGNA_PWM_FLUJO					0,0
;			ELECTROVALVULAS						0
			outi		stop_ventilador,0x00
			.endm
*/
.macro		FASE_O2_21
			OUTI	FASE,						CALIBRA_O2_21
			INSP_EXHA							0
			PIN_PS0
			CBI		T_PAUSA
			ASIGNA_TIEMPO						0,0

			outi		PWM_FLUJOH,high(170)
			outi		PWM_FLUJOL,low(170)
		;	ASIGNA_PWM_FLUJO					90,90
			ELECTROVALVULAS						0	;Fase de expiración
			ASIGNA_PWM_O2						0,0
			ASIGNA_PWM_AIRE						ocr5ah,ocr5al
fin_macro_o2_21:
			.endm

.macro		FASE_O2_100
			OUTI	FASE,						CALIBRA_O2_100
			INSP_EXHA							0
			PIN_PS0
			CBI		T_PAUSA
			ASIGNA_TIEMPO						0,0

			outi		PWM_FLUJOH,high(170)
			outi		PWM_FLUJOL,low(170)
			ELECTROVALVULAS						0	;Fase de expiración
			ASIGNA_PWM_O2						ocr5ah,ocr5al
			ASIGNA_PWM_AIRE						0,0
fin_macro_o2_100:
			.endm

.macro		FASE_TST
			
			outi	inhibe_inc_dec_pwm,'0'
			outi	tmr_inhibe_on_off_pwm,		3
			INSP_EXHA							0
			PIN_PS0
			CBI		T_PAUSA
			outi	reg_PmaxH,high(5000);(PWMmin)
			outi	reg_PmaxL,low(5000)
			outi	presion_maxima,'0'
			ASIGNA_PWM_O2					reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE					reg_PWM2_AireH,reg_PWM2_AireL
			ASIGNA_PWM_FLUJO				reg_PWM0H,reg_PWM0L
			outi		FASE,				TST_ELECTROVALVULAS
			outi	B_PR_TST,'1'
			cpri	tst_electro_exp,'1'
			breq	activa_exp_tst
			ELECTROVALVULAS						0
			jmp		fin_macro_tst
activa_exp_tst:
			ELECTROVALVULAS						PWM_VV_Max
			
			;delay_ms	30
			;sbi		led_run
fin_macro_tst:
			.endm

.macro		OFF_PRESION_TST
//			cpri	presion_liberada,'1'
//			breq	fin_macro_off_presion
			outi	B_PR_TST,'0'

;				outi	PWM_FLUJOH,high(337)
;				outi	PWM_FLUJOL,low(337)
			;outi	B_PR_TST,'0'
			ASIGNA_PWM_FLUJO					0,0
			ELECTROVALVULAS						PWM_VV_Max
			ASIGNA_PWM_O2						0,0
			ASIGNA_PWM_AIRE						0,0
fin_macro_off_presion:
			.endm
/*
.macro		OFF_PRESION_0p
			;cpri	presion_liberada,'1'
			;breq	fin_macro_off_presion
			outi		PWM_FLUJOH,high(PWMmin)
			outi		PWM_FLUJOL,low(PWMmin)
			ELECTROVALVULAS						PWM_VV_Max
			ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
fin_macro_off_presion:
			.endm
*/
.macro		INC_VOLUMEN
			pushx
;			OUTI	FASE,						INSPIRACION
;			INSP_EXHA							1
;			PIN_PS0
;			CBI		T_PAUSA
;			ASIGNA_TIEMPO						tmr_TIEMPO_INSH,tmr_TIEMPO_INSL
;			outi	inhibe_inc_dec_pwm,'1'
			inr		r24,reg_PWM0L
			inr		r25,reg_PWM0H
			subi	r24,low(-1)
			sbci	r25,high(-1)
			ldiw	xh,xl,Val_PWMmax+1
			cp		r24,xl
			cpc		r25,xh
			brsh	skip_actualiza
			outr	reg_PWM0L,r24
			outr	reg_PWM0H,r25
skip_actualiza:
;			cpri	presion_liberada,'1'
;			breq	fin_macro_inc_volumen
;			ASIGNA_PWM_FLUJO					reg_PWM0H,reg_PWM0L
;			ELECTROVALVULAS						0b01
;			ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
;			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
fin_macro_inc_volumen:
			popx
			.endm

.macro		DEC_VOLUMEN
			pushx
;			OUTI	FASE,						INSPIRACION
;			INSP_EXHA							1
;			PIN_PS0
;			CBI		T_PAUSA
;			ASIGNA_TIEMPO						tmr_TIEMPO_INSH,tmr_TIEMPO_INSL
;			outi	inhibe_inc_dec_pwm,'1'
			inr		r24,reg_PWM0L
			inr		r25,reg_PWM0H
			subi	r24,low(1)
			sbci	r25,high(1)
			brmi	skip_actualiza			;Si el resultado es negativo, no se
											;actualiza el valor de la resta
			outr	reg_PWM0L,r24
			outr	reg_PWM0H,r25
skip_actualiza:
;			cpri	presion_liberada,'1'
;			breq	fin_macro_dec_volumen
;			ASIGNA_PWM_FLUJO					reg_PWM0H,reg_PWM0L
;			ELECTROVALVULAS						0b01
;			ASIGNA_PWM_O2						  reg_PWM1_O2H,reg_PWM1_O2L
;			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
fin_macro_dec_volumen:
			popx
			.endm
;*********************************************************************

;*********************************************************************
;Modos de operación del respirador
#define		P_CMV		'0'
#define		V_CMV		'1'
#define		P_SIMV		'2'
#define		V_SIMV		'3'
#define		P_CPAP		'4'
#define		V_CPAP		'5'
;*********************************************************************

;*********************************************************************
.equ		Reg_rx				=	0x0400
.equ		rx_Modo_op			=	Reg_rx+0
.equ		rx_TIEMPO_INSL		=	Reg_rx+1
.equ		rx_TIEMPO_INSML		=	Reg_rx+2
.equ		rx_TIEMPO_INSMH		=	Reg_rx+3
.equ		rx_TIEMPO_INSH		=	Reg_rx+4
.equ		rx_TIEMPOpausaL		=	Reg_rx+5
.equ		rx_TIEMPOpausaML	=	Reg_rx+6
.equ		rx_TIEMPOpausaMH	=	Reg_rx+7
.equ		rx_TIEMPOpausaH		=	Reg_rx+8
.equ		rx_TIEMPO_EXPL		=	Reg_rx+9
.equ		rx_TIEMPO_EXPML		=	Reg_rx+10
.equ		rx_TIEMPO_EXPMH		=	Reg_rx+11
.equ		rx_TIEMPO_EXPH		=	Reg_rx+12
.equ		rx_FRL				=	Reg_rx+13
.equ		rx_FRM				=	Reg_rx+14
.equ		rx_FRH				=	Reg_rx+15
.equ		rx_PWM0L			=	Reg_rx+16	;Valor de PWM para ventilar
.equ		rx_PWM0ML			=	Reg_rx+17
.equ		rx_PWM0MH			=	Reg_rx+18
.equ		rx_PWM0H			=	Reg_rx+19
.equ		rx_O2L				=	Reg_rx+20		;Valor de PWM para presion soporte
.equ		rx_O2M				=	Reg_rx+21
.equ		rx_O2H				=	Reg_rx+22
;.equ		rx_PWM0_psH			=	Reg_rx+23
.equ		rx_PWM1_O2L			=	Reg_rx+24		;Valor de PWM para O2
.equ		rx_PWM1_O2ML		=	Reg_rx+25
.equ		rx_PWM1_O2MH		=	Reg_rx+26
.equ		rx_PWM1_O2H			=	Reg_rx+27
.equ		rx_PWM2_AireL		=	Reg_rx+28		;Valor de PWM la valvula que controla
.equ		rx_PWM2_AireML		=	Reg_rx+29		;la entrada de aire
.equ		rx_PWM2_AireMH		=	Reg_rx+30
.equ		rx_PWM2_AireH		=	Reg_rx+31
.equ		rx_PinspiracionH	=	Reg_rx+32
.equ		rx_PinspiracionM	=	Reg_rx+33
.equ		rx_PinspiracionL	=	Reg_rx+34
.equ		rx_PEEPH			=	Reg_rx+35
.equ		rx_PEEPM			=	Reg_rx+36
.equ		rx_PEEPL			=	Reg_rx+37
.equ		rx_PSH				=	Reg_rx+38
.equ		rx_PSM				=	Reg_rx+39
.equ		rx_PSL				=	Reg_rx+40
.equ		rx_PmaxH			=	Reg_rx+41
.equ		rx_PmaxM			=	Reg_rx+42
.equ		rx_PmaxL			=	Reg_rx+43
.equ		rx_ON_OFF			=	Reg_rx+44

///****************************************************cadena configuracion mezclador 
.equ		rx_OPEEPMH			=	Reg_rx+45
.equ		rx_OPEEPML			=	Reg_rx+46
.equ		rx_OPEEPL			=	Reg_rx+47
.equ		rx_OPIPMH			=	Reg_rx+48
.equ		rx_OPIPML			=	Reg_rx+49
.equ		rx_OPIPL			=	Reg_rx+50
.equ		rx_O_CPEEPMH		=	Reg_rx+51
.equ		rx_O_CPEEPML		=	Reg_rx+52
.equ		rx_O_CPEEPL			=	Reg_rx+53
.equ		rx_O_CPIPMH			=	Reg_rx+54
.equ		rx_O_CPIPML			=	Reg_rx+55
.equ		rx_O_CPIPL			=	Reg_rx+56
.equ		rx_O_CPRESMH		=	Reg_rx+57
.equ		rx_O_CPRESML		=	Reg_rx+58
.equ		rx_O_CPRESL			=	Reg_rx+59
.equ		rx_O_BATMH			=	Reg_rx+60
.equ		rx_O_BATML			=	Reg_rx+61
.equ		rx_O_BATL			=	Reg_rx+62
.equ		rx_O_GAN_BATMH		=	Reg_rx+63
.equ		rx_O_GAN_BATML		=	Reg_rx+64
.equ		rx_O_GAN_BATL		=	Reg_rx+65
.equ		rx_OFF				=	Reg_rx+66
.equ		Tx_OFF				=	Reg_rx+67
.equ		rx_tst_02			=	Reg_rx+68
.equ		rx_VV_Off_MH		=	Reg_rx+69
.equ		rx_VV_Off_ML		=	Reg_rx+70
.equ		rx_VV_Off_L			=	Reg_rx+71
;.equ		Tx_OFF				=	Reg_rx+67
.equ		rx_Ctrl_Vol			=	Reg_rx+72
.equ		rx_Ctrl_FiO2		=	Reg_rx+73
;.equ		rx_flag_Test		=	Reg_rx+74

//*****************************************************


.equ		Reg_internos		=	0x0500
.equ		Modo_op				=	Reg_internos+0
.equ		TIEMPO_INSL			=	Reg_internos+1
.equ		TIEMPO_INSML		=	Reg_internos+2
.equ		TIEMPO_INSMH		=	Reg_internos+3
.equ		TIEMPO_INSH			=	Reg_internos+4
.equ		TIEMPOpausaL		=	Reg_internos+5
.equ		TIEMPOpausaML		=	Reg_internos+6
.equ		TIEMPOpausaMH		=	Reg_internos+7
.equ		TIEMPOpausaH		=	Reg_internos+8
.equ		TIEMPO_EXPL			=	Reg_internos+9
.equ		TIEMPO_EXPML		=	Reg_internos+10
.equ		TIEMPO_EXPMH		=	Reg_internos+11
.equ		TIEMPO_EXPH			=	Reg_internos+12
.equ		FRL					=	Reg_internos+13
.equ		FRM					=	Reg_internos+14
.equ		FRH					=	Reg_internos+15
.equ		PWM0L				=	Reg_internos+16
.equ		PWM0ML				=	Reg_internos+17
.equ		PWM0MH				=	Reg_internos+18
.equ		PWM0H				=	Reg_internos+19
.equ		O2L					=	Reg_internos+20
.equ		O2M					=	Reg_internos+21
.equ		O2H					=	Reg_internos+22
;.equ		PWM0_psH			=	Reg_internos+23
.equ		PWM1_O2L			=	Reg_internos+24
.equ		PWM1_O2ML			=	Reg_internos+25
.equ		PWM1_O2MH			=	Reg_internos+26
.equ		PWM1_O2H			=	Reg_internos+27
.equ		PWM2_AireL			=	Reg_internos+28
.equ		PWM2_AireML			=	Reg_internos+29
.equ		PWM2_AireMH			=	Reg_internos+30
.equ		PWM2_AireH			=	Reg_internos+31
.equ		PinspiracionH		=	Reg_internos+32
.equ		PinspiracionM		=	Reg_internos+33
.equ		PinspiracionL		=	Reg_internos+34
.equ		PEEPH				=	Reg_internos+35
.equ		PEEPM				=	Reg_internos+36
.equ		PEEPL				=	Reg_internos+37
.equ		PSH					=	Reg_internos+38
.equ		PSM					=	Reg_internos+39
.equ		PSL					=	Reg_internos+40
.equ		PmaxH				=	Reg_internos+41
.equ		PmaxM				=	Reg_internos+42
.equ		PmaxL				=	Reg_internos+43
.equ		ON_OFF				=	Reg_internos+44

.equ		Reg_hex				=	0x0600
.equ		tmr_TIEMPO_INSL		=	Reg_hex+0
.equ		tmr_TIEMPO_INSH		=	Reg_hex+1
.equ		tmr_TIEMPOpausaL	=	Reg_hex+2
.equ		tmr_TIEMPOpausaH	=	Reg_hex+3
.equ		tmr_TIEMPO_EXPL		=	Reg_hex+4
.equ		tmr_TIEMPO_EXPH		=	Reg_hex+5
.equ		reg_FRL				=	Reg_hex+6
.equ		reg_FRH				=	Reg_hex+7
.equ		reg_PWM0L			=	Reg_hex+8
.equ		reg_PWM0H			=	Reg_hex+9
.equ		reg_O2L				=	Reg_hex+10
.equ		reg_O2H				=	Reg_hex+11
.equ		reg_PWM1_O2L		=	Reg_hex+12
.equ		reg_PWM1_O2H		=	Reg_hex+13
.equ		reg_PWM2_AireL		=	Reg_hex+14
.equ		reg_PWM2_AireH		=	Reg_hex+15
.equ		reg_PinspiracionH	=	Reg_hex+16
.equ		reg_PinspiracionL	=	Reg_hex+17	
.equ		reg_PEEPH			=	Reg_hex+18
.equ		reg_PEEPL			=	Reg_hex+19
.equ		reg_PSH				=	Reg_hex+20
.equ		reg_PSL				=	Reg_hex+21
.equ		reg_PmaxH			=	Reg_hex+22
.equ		reg_PmaxL			=	Reg_hex+23

.equ		tmp_tmr_TIEMPO_INSL	=	Reg_hex+16+8
.equ		tmp_tmr_TIEMPO_INSH	=	Reg_hex+17+8
.equ		tmp_tmr_TIEMPOpausaL=	Reg_hex+18+8
.equ		tmp_tmr_TIEMPOpausaH=	Reg_hex+19+8
.equ		tmp_tmr_TIEMPO_EXPL	=	Reg_hex+20+8
.equ		tmp_tmr_TIEMPO_EXPH	=	Reg_hex+21+8
.equ		tmp_reg_FRL			=	Reg_hex+22+8
.equ		tmp_reg_FRH			=	Reg_hex+23+8
.equ		tmp_reg_PWM0L		=	Reg_hex+24+8
.equ		tmp_reg_PWM0H		=	Reg_hex+25+8
.equ		tmp_reg_O2L			=	Reg_hex+26+8
.equ		tmp_reg_O2H			=	Reg_hex+27+8
.equ		tmp_reg_PWM1_O2L	=	Reg_hex+28+8
.equ		tmp_reg_PWM1_O2H	=	Reg_hex+29+8
.equ		tmp_reg_PWM2_AireL	=	Reg_hex+30+8
.equ		tmp_reg_PWM2_AireH	=	Reg_hex+31+8
.equ		tmp_reg_PinspiracionH	=	Reg_hex+32+8
.equ		tmp_reg_PinspiracionL	=	Reg_hex+33+8
.equ		tmp_reg_PEEPH			=	Reg_hex+34+8
.equ		tmp_reg_PEEPL			=	Reg_hex+35+8
.equ		tmp_reg_PSH				=	Reg_hex+36+8
.equ		tmp_reg_PSL				=	Reg_hex+37+8
.equ		tmp_reg_PmaxH			=	Reg_hex+38+8
.equ		tmp_reg_PmaxL			=	Reg_hex+39+8





.equ		Reg_misc			=	0x0700
.equ		cont_10msL			=	Reg_misc+0
.equ		cont_10msH			=	Reg_misc+1
.equ		tmp_cont_10msAL		=	Reg_misc+2
.equ		tmp_cont_10msAH		=	Reg_misc+3
.equ		tmp_cont_10msRL		=	Reg_misc+4
.equ		tmp_cont_10msRH		=	Reg_misc+5

.equ		estado_electrovalvulas	=	Reg_misc+6

.equ		bandera_tx_estado	=	Reg_misc+7 ;Esta bandera se usa dentro de la interrupcion de conteo de pulsos

.equ		stop_ventilador		=	Reg_misc+8	;Bandera para indicar el final del ciclo de respiración

.equ		tmr_pingL			=	Reg_misc+9
.equ		tmr_pingH			=	Reg_misc+10

.equ		tmr_ping_run		=	Reg_misc+11

.equ		trigger				=	Reg_misc+12

.equ		tmp_udr1			=	Reg_misc+13

.equ		tmp_ocr3al_0		=	Reg_misc+14	;Comparador para el tiempo bajo de la alarma
.equ		tmp_ocr3ah_0		=	Reg_misc+15
.equ		tmp_ocr3al_1		=	Reg_misc+16	;Comparador para el tiempo alto de la alarma
.equ		tmp_ocr3ah_1		=	Reg_misc+17

.equ		tmr_alarma_onL		=	Reg_misc+18	;Temporizador de alarma encendida
.equ		tmr_alarma_onH		=	Reg_misc+19
.equ		tmr_alarma_muteL	=	Reg_misc+20	;Temporizador de alarma en mute
.equ		tmr_alarma_muteH	=	Reg_misc+21

.equ		tmr_alarmaL			=	Reg_misc+22
.equ		tmr_alarmaH			=	Reg_misc+23

.equ		estado_mute			=	Reg_misc+24

.equ		tmr_sv				=	Reg_misc+25
.equ		alarma_sv			=	Reg_misc+26
.equ		cont_alarma_sv		=	Reg_misc+27

.equ		estado_alarmas		=	Reg_misc+28

.equ		alarmas_criticas	=	Reg_misc+29

;Este registro mantiene el estado la fase activa de operación
;FASE	FASE_PS
;'0'			Mecanismo en estado de reposo
;'1'			Motor en movimiento durante inspiración
;'2'			Pausa de inspiración
;'3'			Motor en movimiento durante la expiración
;'4'			Ventana de tiempo para inhibir el trigger (solo en modo 2)
;'5'			Pausa de expiración
;'9'			Busqueda de origen
.equ		FASE				=	Reg_misc+30

.equ		ventana				=	Reg_misc+31

.equ		tmp_tmr_ventanaSIMV_L	=	Reg_misc+32
.equ		tmp_tmr_ventanaSIMV_H	=	Reg_misc+33
.equ		tmr_ventanaSIMV_L		=	Reg_misc+34
.equ		tmr_ventanaSIMV_H		=	Reg_misc+35

.equ		tmr_led				=	Reg_misc+36
.equ		tmp_led				=	Reg_misc+37
.equ		buff_rx_debug		=	Reg_misc+38
.equ		tmr_debug			=	Reg_misc+39

.equ		presion_liberada	=	Reg_misc+40

.equ		start_exha			=	Reg_misc+41

;***************************************************************
;***************************************************************
;***************************************************************
;La función de este registro esta deshabilitada (no se esta usando)
.equ		inhibe_inc_dec_pwm	=	Reg_misc+42	;Este registro permite que solo
												;se acepte una vez la orden de
												;inc o dec el PWM, por cada
												;ciclo de respiración.
;***************************************************************
;***************************************************************
;***************************************************************

.equ		reg_PWM0L_50p		=	Reg_misc+43
.equ		reg_PWM0H_50p		=	Reg_misc+44

.equ		reg_PWM0L_25p		=	Reg_misc+45
.equ		reg_PWM0H_25p		=	Reg_misc+46

.equ		reg_PWM0L_12p5p		=	Reg_misc+47
.equ		reg_PWM0H_12p5p		=	Reg_misc+48

.equ		reg_PWM0L_6p25p		=	Reg_misc+49
.equ		reg_PWM0H_6p25p		=	Reg_misc+50

.equ		rx_tst_electro_exp	=	Reg_misc+51
.equ		rx_tst_run			=	Reg_misc+52
.equ		tst_electro_exp		=	Reg_misc+53

.equ		tmr_inhibe_on_off_pwm=	Reg_misc+54	;Al inicio de inspiración se
.equ		inhibe_on_off_pwm	=	Reg_misc+55	;inhibe la función ON/OFF de la
												;presión (INT1 con combinación
												;2 y 3
.equ		reg_6p25p_escala_completaL	=	Reg_misc+56
.equ		reg_6p25p_escala_completaH	=	Reg_misc+57

.equ		tx_presion_aireH			=	Reg_misc+58
.equ		tx_presion_aireM			=	Reg_misc+59
.equ		tx_presion_aireL			=	Reg_misc+60

.equ		tx_presion_O2H				=	Reg_misc+61
.equ		tx_presion_O2M				=	Reg_misc+62
.equ		tx_presion_O2L				=	Reg_misc+63

.equ		presion_hex_aireH			=	Reg_misc+64
.equ		presion_hex_aireL			=	Reg_misc+65

.equ		presion_hex_O2H				=	Reg_misc+66
.equ		presion_hex_O2L				=	Reg_misc+67

;.equ		PSI_aire_HEX				=	Reg_misc+68
.equ		B_PR_SP						=	Reg_misc+68

.equ		restante_cont_10msL			=	Reg_misc+72
.equ		restante_cont_10msH			=	Reg_misc+73

.equ		inspiracion_asistida		=	Reg_misc+74

.equ		CPAP_alarma_APNEA			=	Reg_misc+75

.equ		CPAP_ACTIVO					=	Reg_misc+76

.equ		reg_12p5p_escala_completaL	=	Reg_misc+77
.equ		reg_12p5p_escala_completaH	=	Reg_misc+78

.equ		cont_rx0					=	Reg_misc+79
.equ		comando_rx0					=	Reg_misc+80
.equ		apuntador_buffer_rx0H		=	Reg_misc+81
.equ		apuntador_buffer_rx0L		=	Reg_misc+82

.equ		apuntador_buffer_tx0H		=	Reg_misc+83
.equ		apuntador_buffer_tx0L		=	Reg_misc+84

.equ		cont_tx0					=	Reg_misc+85

.equ		cmd_pendiente				=	Reg_misc+86

.equ		envia_trama_U				=	Reg_misc+87	;Esta bandera se activa en el modo
														;de prueba de electrovalvulas, una
														;vez que se alcanza la presión y
														;se empieza a tx de manera periodica
														;la trama 'U' con la lectura de la
														;presion
.equ		tmr_U						=	Reg_misc+88
.equ		tx_U						=	Reg_misc+89

;.equ		primer_flanco				=	Reg_misc+90

.equ		timer_electrovalvula		=	Reg_misc+91

.equ		PWM_exhalacion				=	Reg_misc+92
.equ		Periodo_exhalacion			=	Reg_misc+93
.equ		recarga_PWM					=	Reg_misc+94

.equ		TIEMPO_INSH_50p				=	Reg_misc+95
.equ		TIEMPO_INSL_50p				=	Reg_misc+96

.equ		bandera_Tinsp_50p			=	Reg_misc+97
.equ		bandera_inc_PWM_inspiracion	=	Reg_misc+98

.equ		B_PS_Ok						=	Reg_misc+99

.equ		Reg_PS_TempH				=	Reg_misc+100
.equ		Reg_PS_TempL				=	Reg_misc+101



.equ		datos_actualizados			=	Reg_misc+200

;.equ		tmr_de_presion				=	Reg_misc+201


.equ		CCF_H						= 	Reg_misc+202
.equ		CCF_L						= 	Reg_misc+203

.equ		Cont_CF_H					= 	Reg_misc+204
.equ		Cont_CF_L					= 	Reg_misc+205

.equ		B_Dec						= 	Reg_misc+206

.equ		B_PEEP_AC					= 	Reg_misc+208

.equ		Ctrl_CF_H					= 	Reg_misc+210
.equ		Ctrl_CF_L					= 	Reg_misc+211

.equ		B_PEEP_D					=	Reg_misc+212
.equ		B_PEEP_DC					=	Reg_misc+213

.equ		B_PIP_OK					=	Reg_misc+214


.equ		primer_cruce				=	Reg_misc+215
	

.equ		B_PEEP_UP					=	Reg_misc+216
.equ		B_PEEP_DOWN					=	Reg_misc+217
	
.equ		offset_PIPL					=	Reg_misc+218
.equ		offset_PIPH					=	Reg_misc+219

.equ		offset_PEEPL				=	Reg_misc+220
.equ		offset_PEEPH				=	Reg_misc+221


.equ		offset_PreL					=	Reg_misc+222
.equ		offset_PreH					=	Reg_misc+223

.equ		offset_COMFLPIPL			=	Reg_misc+224
.equ		offset_COMFLPIPH			=	Reg_misc+225

.equ		offset_COMFLPEEPL			=	Reg_misc+226
.equ		offset_COMFLPEEPH			=	Reg_misc+227

.equ		temp_H						=	Reg_misc+228
.equ		temp_L						=	Reg_misc+229

.equ		B_PR_TST					=	Reg_misc+230
.equ		B_TMR_TST					=	Reg_misc+231

.equ		offset_batL					=	Reg_misc+232
.equ		offset_batH					=	Reg_misc+233

.equ		G_batL						=	Reg_misc+234
.equ		G_batH						=	Reg_misc+235

.equ		B_ox_max					=	Reg_misc+236

.equ		cont_2L						=	Reg_misc+237
.equ		cont_2H						=	Reg_misc+238

.equ		b_sp						=	Reg_misc+239
.equ		tx_b_sp						=	Reg_misc+240

.equ		cont_peep_up_H				= 	Reg_misc+241
.equ		cont_peep_up_L				= 	Reg_misc+242

.equ		B_Cont						= 	Reg_misc+243

.equ		B_act						= 	Reg_misc+244

.equ		VV_OFF						= 	Reg_misc+245

.equ		VV_X						= 	Reg_misc+246

.equ		Cont_VV						= 	Reg_misc+247
.equ		Cont_VX						= 	Reg_misc+248

.equ		B_Vol						= 	Reg_misc+249
.equ		B_C_VV						= 	Reg_misc+250

.equ		C_PEEP_DOWN					=	Reg_misc+251


.equ		buffer_tx0					=	0x0800

.equ		buffer_rx0					=	0x0880

;Sección agregada para FiO2
.equ		FiO2					=	0x0900
.equ		Acc_FiO2L				=	FiO2+0	;Acumulador de lecturas
.equ		Acc_FiO2H				=	FiO2+1
.equ		cont_muestras			=	FiO2+2

.equ		tx_FiO2L				=	FiO2+3
.equ		tx_FiO2M				=	FiO2+4
.equ		tx_FiO2H				=	FiO2+5

.equ		FiO2_nuevoL				=	FiO2+6
.equ		FiO2_nuevoH				=	FiO2+7
.equ		FiO2_valido				=	FiO2+8

.equ		FiO2HEX_L				=	FiO2+9
.equ		FiO2HEX_H				=	FiO2+10

.equ		estado_21_O2			=	FiO2+11
.equ		estado_100_O2			=	FiO2+12

.equ		FiO2HEX_21L				=	FiO2+13
.equ		FiO2HEX_21H				=	FiO2+14

.equ		FiO2HEX_100L			=	FiO2+15
.equ		FiO2HEX_100H			=	FiO2+16

.equ		tmp_FP_ADC_O2L			=	FiO2+17
.equ		tmp_FP_ADC_O2ML			=	FiO2+18
.equ		tmp_FP_ADC_O2MH			=	FiO2+19
.equ		tmp_FP_ADC_O2H			=	FiO2+20

.equ		rx_CAL_O2				=	FiO2+21

.equ		timer_O2H				=	FiO2+22
.equ		timer_O2L				=	FiO2+23
.equ		habilita_control_O2		=	FiO2+24
.equ		FIO2_NH					=	FiO2+25
.equ		FIO2_NL					=	FiO2+26
.equ		C_FIO2					=	FiO2+27

;Sección agregada para lectura de la bateria
.equ		bateria					=	0x0A00
.equ		modo_alimentacion		=	bateria+0
.equ		nivel_bat				=	bateria+1
.equ		selector_O2_Bateria		=	bateria+2

#define		modo_Bateria			'0'
#define		modo_AC					'1'

;Definiciones del sensor NPA700
.equ		reg_NPA700				=	0x0B00
;.equ		Acc_NPA700L				=	reg_NPA700+0	;Acumulador de lecturas
;.equ		Acc_NPA700H				=	reg_NPA700+1
.equ		cont_muestrasNPA700		=	reg_NPA700+2

.equ		NPA_700H				=	reg_NPA700+3
.equ		NPA_700L				=	reg_NPA700+4

.equ		Presion_NPA700H			=	reg_NPA700+5
.equ		Presion_NPA700L			=	reg_NPA700+6
.equ		NPA700_valido			=	reg_NPA700+7

.equ		estado_i2cNPA			=	reg_NPA700+8

.equ		Presion_cmH2OH			=	reg_NPA700+9
.equ		Presion_cmH2OL			=	reg_NPA700+10

.equ		tx_PSI_4				=	reg_NPA700+11
.equ		tx_PSI_3				=	reg_NPA700+12
.equ		tx_PSI_2				=	reg_NPA700+13
.equ		tx_PSI_1				=	reg_NPA700+14
.equ		tx_PSI_0				=	reg_NPA700+15

.equ		reg_PinspiracionLFP		=	reg_NPA700+16
.equ		reg_PinspiracionMLFP	=	reg_NPA700+17
.equ		reg_PinspiracionMHFP	=	reg_NPA700+18
.equ		reg_PinspiracionHFP		=	reg_NPA700+19

.equ		reg_PEEPLFP				=	reg_NPA700+20
.equ		reg_PEEPMLFP			=	reg_NPA700+21
.equ		reg_PEEPMHFP			=	reg_NPA700+22
.equ		reg_PEEPHFP				=	reg_NPA700+23

.equ		reg_PSLFP				=	reg_NPA700+24
.equ		reg_PSMLFP				=	reg_NPA700+25
.equ		reg_PSMHFP				=	reg_NPA700+26
.equ		reg_PSHFP				=	reg_NPA700+27

.equ		reg_PmaxLFP				=	reg_NPA700+28
.equ		reg_PmaxMLFP			=	reg_NPA700+29
.equ		reg_PmaxMHFP			=	reg_NPA700+30
.equ		reg_PmaxHFP				=	reg_NPA700+31

.equ		reg_PinspiracionLFPx100	=	reg_NPA700+32
.equ		reg_PinspiracionMLFPx100=	reg_NPA700+33
.equ		reg_PinspiracionMHFPx100=	reg_NPA700+34
.equ		reg_PinspiracionHFPx100	=	reg_NPA700+35

;.equ		reg_PEEPLFPx100			=	reg_NPA700+36
;.equ		reg_PEEPMLFPx100		=	reg_NPA700+37
;.equ		reg_PEEPMHFPx100		=	reg_NPA700+38
;.equ		reg_PEEPHFPx100			=	reg_NPA700+39

.equ		reg_PSLFPx100			=	reg_NPA700+40
.equ		reg_PSMLFPx100			=	reg_NPA700+41
.equ		reg_PSMHFPx100			=	reg_NPA700+42
.equ		reg_PSHFPx100			=	reg_NPA700+43

.equ		reg_PmaxLFPx100			=	reg_NPA700+44
.equ		reg_PmaxMLFPx100		=	reg_NPA700+45
.equ		reg_PmaxMHFPx100		=	reg_NPA700+46
.equ		reg_PmaxHFPx100			=	reg_NPA700+47

;.equ		reg_PEEPLFPx1p05		=	reg_NPA700+48
;.equ		reg_PEEPMLFPx1p05		=	reg_NPA700+49
;.equ		reg_PEEPMHFPx1p05		=	reg_NPA700+50
;.equ		reg_PEEPHFPx1p05		=	reg_NPA700+51

;.equ		reg_PEEPLFPx0p95		=	reg_NPA700+52
;.equ		reg_PEEPMLFPx0p95		=	reg_NPA700+53
;.equ		reg_PEEPMHFPx0p95		=	reg_NPA700+54
;.equ		reg_PEEPHFPx0p95		=	reg_NPA700+55

;.equ		PEEP_en_rango			=	reg_NPA700+56

.equ		tmr_inicio_exhalacion	=	reg_NPA700+57	;Temporizador desde que inicia
											;la exhalación hasta que ya transcurrieron
											;varios ms de exhalación
.equ		tmr_t_inicial_exha_electro=	reg_NPA700+58	;Temporizador desde que inicia
											;la exhalación con la electrovalvula exha.
											;abierta y sin flujo, cuando termina, debe
											;de asignar los valores que siguen para
											;la pendiene de descenso


.equ		reg_PEEPH_100porciento	=	reg_NPA700+63
.equ		reg_PEEPL_100porciento	=	reg_NPA700+64

;Dirección de la presion, para el control del PEEP
;.equ		direccion_presion		=	reg_NPA700+69
;#define		subiendo			's'
;#define		bajando				'b'
;#define		desconocido			'd'

.equ		Acc_NPA700L				=	reg_NPA700+70	;Acumulador de lecturas
.equ		Acc_NPA700M				=	reg_NPA700+71
.equ		Acc_NPA700H				=	reg_NPA700+72

.equ		tmp_reg_PEEPH_100porciento	=	reg_NPA700+73
.equ		tmp_reg_PEEPL_100porciento	=	reg_NPA700+74

.equ		Presion_final_inspH		=	reg_NPA700+75
.equ		Presion_final_inspL		=	reg_NPA700+76



.equ		reg_FLUJO_compensacionH	=	reg_NPA700+87
.equ		reg_FLUJO_compensacionL	=	reg_NPA700+88
.equ		bandera_inc_compensacion=	reg_NPA700+89

.equ		presion_maxima			=	reg_NPA700+90

.equ		inhibe_compensacion		=	reg_NPA700+91

.equ		reg_Pinspiracionx100H	=	reg_NPA700+92
.equ		reg_Pinspiracionx100L	=	reg_NPA700+93

.equ		reg_primer_PEEP			=	reg_NPA700+94

.equ		inhibe_compensacion_primer_PEEP	=	reg_NPA700+95



.equ		ASCIIH					=	reg_NPA700+96
.equ		ASCIIM					=	reg_NPA700+97
.equ		ASCIIL					=	reg_NPA700+98



.equ		reg_Dif50p_presionH		=	reg_NPA700+99
.equ		reg_Dif50p_presionL		=	reg_NPA700+100


.equ		reg_PSx100H				=	reg_NPA700+101
.equ		reg_PSx100L				=	reg_NPA700+102

.equ		CONT_RET				=	reg_NPA700+103


.equ		C_A_FiO2				=	reg_NPA700+104
.equ		C_X_FiO2				=	reg_NPA700+105


.equ		FiO2_A_L				=	reg_NPA700+106
.equ		FiO2_A_H				=	reg_NPA700+107

.equ		B_FIO2_BAJO				=	reg_NPA700+108
.equ		B_FIO2_ALTO				=	reg_NPA700+109

;Definición base de la libreria de punto flotante
.equ		reg_lib					=	0x0D00

;********************************************************************

;********************************************************************
init_io:	outi	ddra, 0b11111111
			outi	porta,0b00000000

			outi	ddrb, 0b10000000	;PB7 = OC1C = PWM0, control de la electrovalvula
			outi	portb,0b01111111	;para el control del flujo.
										;PB5 = Entrada de interrupción
										;PB6 = Entrada de interrupción

			outi	ddrc, 0b00000000	;PC0 Entrada de estado de la alimentacion AC o Bat
			outi	portc,0b11111111

;PDO = SCL
;PD1 = SDA
			outi	ddrd, 0b11111000	;PD0 = Señal enviada por la tarjeta SV para indicar
			outi	portd,0b00000111	;una sobrepresión, por lo cual se cancela la fase
										;de inspiración y/o el tiempo pausa.
										;PD1 = Señal enviada por la tarjeta SV para indicar
										;que se debe de abrir la valvula PWM0
										;(sin cancelar modo ni fase)
										;PD2 y PD3 = Comunicacion con la tarjeta SV
										;PD4 = Señal enviada por la tarjeta SV para indicar
										;que se debe de cerrar la valvula PWM0
										;PD6 y PD7 = Salidas para electrovalvulas

			outi	ddre, 0b00000010	;PE0 y PE1 = Comunicación con la Raspberry
			outi	porte,0b11111111

			outi	ddrf, 0b00000000	;PF1 y PF2 = Entradas analogicas para
			outi	portf,0b00000000	;digitalizar el valor de la bateria del no break
										;y del sensor de O2
            outi	didr0,0xFF			;Se deshabilita la entrada digital de los pines
            outi	didr2,0xFF			;de los canales analogicos.

			outi	ddrg, 0b00000001	;PG0 = pin del led indicador
            outi	portg,0b00000001

					outi	ddrh, 0b11111111
					outi	porth,0b00000000

			outi	ddrl, 0b11111100	;PL0 = ICP4 = Interrupción externa.
			outi	portl,0b00000011	;En modo volumen indica que se debe de aumentar
										;el flujo con la valvula controlada con el PWM0,
										;para el siguiente ciclo respiratorio.
										;En modo presión indica que se debe de abrir la
										;valvula controlada con el PWM0, para permitir el
										;paso del aire.
										;PL1 = ICP5 = Interrupción externa.
										;En modo volumen indica que se debe de reducir el
										;flujo con la valvula controlada con el PWM0, para
										;el siguiente ciclo respiratorio.
										;En modo presión indica que se debe de cerrar la
										;valvula controlada con el PWM0, para no dejar
										;pasar mas aire.
										;PL4 = OC5B = PWM1 controla la electrovalvula
										;para O2
										;PL5 = OC5C = PWM2 controla la electrovalvula
										;para paso el paso del aire
										;PL6 = INSPIRACIÓN/EXHALACIÓN, le indica a la
										;tarjeta SV en que fase esta (1/0 respectivaente)
;***************************************

;***************************************
;Configura las interrupciones externas
;			outi		eicra,0b00001111	;INTF0 e INTF1 activas por flanco asc
;			outi		eifr, 0b00000011	;Borra bandera INTF0
;			outi		eimsk,0b00000011	;Habilita la interrupción
					outi		pcmsk0,	0b01100000	;Habilita las interrupciones de los
					outi		pcicr,	0b00000001	;pines PCINT5 y PCINT6 (PB5 y PB6)
					outi		pcifr,	0b00000001
;***************************************

;***************************************
;Configura los puertos de comunicaciones
			usart0		115200,8,n,1,rxtx,irxtx	;USART para Rx de comandos
			outi		cont_rx0,0x00			;Inicializa el contador de datos de la Rx
												;para la USART 0
			usart1		115200,8,n,1,rxtx,irx	;USART para deteccion del Trigger
			outi		tmr_debug,0x00
;***************************************

;***************************************
;Timer para base de tiempo de la salida de alarma
;	tmp_ocr3a_0 + ocr3a_1 = 0.002272*fxtal
            outiw	tcnt3h,tcnt3l,0x0000
			outiw	tmp_ocr3ah_0,tmp_ocr3al_0,(0.003034*fxtal)*.9
			outiw	tmp_ocr3ah_1,tmp_ocr3al_1,(0.003034*fxtal)*.1
			outiw	ocr3ah,ocr3al,(0.003034*fxtal)*.1
            outi	tccr3a,0x00
            outi	tccr3c,0x00
            outi	tccr3b,0x08		;Timer detenido
            outi	timsk3,0x02		;Habilita la interrupcion de comparacion del Timer 3

			outiw	tmr_alarma_onH,tmr_alarma_onL,440
			outiw	tmr_alarma_muteH,tmr_alarma_muteL,440*5*10
			outiw	tmr_alarmaH,tmr_alarmaL,440
			outi	estado_mute,0x00
;***************************************

;***************************************
;Timer para los PWM's de control de O2 y paso de aire
;Modo PWM con corrección de fase y frecuencia
;El valor de OCR5A determina la frecuencia de PWM (20KHz)
;El valor de OCR5B y OCR5C, depende del ajuste deseado para las electrovalvulas
;Inicia el timer corriendo con PWM al 0%
            outi        tccr5a,0b00101011		;Salidas B y C
            outiw       tcnt5h,tcnt5l,0x0000
			outiw		ocr5ah,ocr5al,0x0000
            outiw       ocr5bh,ocr5bl,0x0000
            outiw       ocr5ch,ocr5cl,0x0000
            outi        tccr5b,0b11011010		;Configura la captura en flanco asc prescaler PWM
			outi		timsk5,0b00000000
;***************************************

;***************************************
;Timer para el PWM de control flujo principal
;Modo PWM con corrección de fase y frecuencia
;El valor de OCR1A determina la frecuencia de PWM (20KHz)
;El valor de OCR1C, depende del ajuste deseado para las electrovalvulas
;Inicia el timer corriendo con PWM al 0%
            outi        tccr1a,0b00001011		;Salida C
            outiw       tcnt1h,tcnt1l,0x0000
			outiw		ocr1ah,ocr1al,0x0000
			outiw       ocr1ch,ocr1cl,0x0000
            outi        tccr1b,0b11011001
			outi		timsk1,0b00000000
;***************************************

;***************************************
			outiw		ocr1ah,ocr1al,Fxtal/Fpwm
			outiw		ocr5ah,ocr5al,Fxtal/Fpwm
;***************************************

;***************************************
;Temporizador de 10ms
;Para el led, para el tiempo de envio de dato "S" y demas temporizaciones basadas en 10ms
.equ		tiempo_ping_20s	=	20000
;.equ		tiempo_ping_10s	=	10000
.equ		tiempo_ping_1s	=	1000
			outi		tccr0a,0x02
			outi		tcnt0,0x00
			outi		ocr0a,0.010*(fxtal/1024)
			outi		timsk0,0x02
			outi		tmp_led,10
			movr		tmr_led,tmp_led
			outi		tccr0b,0x05
			outi		tmr_debug,0x00

			outi		tmr_pingL,low(tiempo_ping_20s/10)		;Temporizador para ping
			outi		tmr_pingH,high(tiempo_ping_20s/10)		;0.1seg x tmr_ping
			outi		tmr_ping_run,0x00

;Inicializa parametros para el ADC
;El canal a usar es el ADC2 (PF2) para O2
;Se usa la interrupción del Timer para la frecuencia de muestreo
;Se efectuta un promedio de las lecturas (sin eliminar valor mas alto y bajo)
.equ		n_muestras	=	16
.equ		n_shift		=	4
			outi		tx_FiO2L,'0'	;Este registro conserva la lectura que se
			outi		tx_FiO2M,'0'	;puede enviar (ASCII)
			outi		tx_FiO2H,'0'	;puede enviar (ASCII)

			outi		FiO2_nuevoL,0x00;En este registro se prepara la lectura
			outi		FiO2_nuevoH,0x00;ya promediada antes de validarla (Hex)
			outi		FiO2_valido,'0'	;Bandera para validar la lectura

			outi		cont_muestras,n_muestras
			outi		Acc_FiO2L,0x00
			outi		Acc_FiO2H,0x00
			outi		adcsra,0b10000111
			outi		adcsrb,0b00000000
			outi		admux, 0b00000010
			outi		adcsra,0b11000111	;Arranque a la conversión A/D

;Sección de inicialización para lectura de la bateria
;Nota: ADC1 (PF1) para el voltaje de la bateria
			outi		selector_O2_Bateria,'0'
			outi		modo_alimentacion,modo_Bateria
			outi		nivel_bat,'0'

;Inicialización de variables generales
			outi		estado_electrovalvulas,'1'		;Aun no se realiza el tst

			outi		bandera_tx_estado,0x00
			outi		stop_ventilador,0x00

			outi		modo_op,'0'
			outi		trigger,0x00			;Este registro puede ser 0x00, '0' ó '1'
;			outiw		timer_ventanaH,timer_ventanaL,0x0001
;			outi		run_timer_ventana,0x00

;			outiw		add_cont_10msH,add_cont_10msL,0x0000

			outi		tmr_sv,0x00
			outi		alarma_sv,0x00
			outi		cont_alarma_sv,0x00
			outi		estado_alarmas,'0'
			outi		ON_OFF,'0'
			outi		alarmas_criticas,'0'
;			outiw		alarma_cont_10msH,alarma_cont_10msL,0x0000
;			outi		bandera_pasos_alarma,'0'

			FASE_STANDBY

			outi		estado_21_O2,'?'		;Valor inicial, que indica que no se ha
			outi		estado_100_O2,'?'		;verificado la calibración del sensor de
												;O2
			outi		ventana,'0'

			outi		presion_liberada,'0'

			outi	presion_maxima,'0'


			outi	start_exha,'0'

			outi	inhibe_inc_dec_pwm,'1'

			outi	inhibe_on_off_pwm,'1'
			outi	tmr_inhibe_on_off_pwm,0

			outi	reg_6p25p_escala_completaL,low(Val_6p25p_escala_completa)
			outi	reg_6p25p_escala_completaH,high(Val_6p25p_escala_completa)

			outi	reg_12p5p_escala_completaL,low(Val_12p5p_escala_completa)
			outi	reg_12p5p_escala_completaH,high(Val_12p5p_escala_completa)

			outi	inspiracion_asistida,		'0'

			outi	CPAP_ACTIVO,'0'

;****************************************************
;****************************************************
;****************************************************
;La siguiente sección se debe de modificar con el valor
;real de la presión de entrada
;****************************************************
;****************************************************
;****************************************************
				outi	presion_hex_aireH,high(404)
				outi	presion_hex_aireL,low(404)
				outi	tx_presion_aireH,'4'
				outi	tx_presion_aireM,'0'
				outi	tx_presion_aireL,'0'

				outi	presion_hex_O2H,high(404)
				outi	presion_hex_O2L,low(404)
				outi	tx_presion_O2H,'4'
				outi	tx_presion_O2M,'0'
				outi	tx_presion_O2L,'0'
;****************************************************
;****************************************************
;****************************************************
				outi	cont_muestrasNPA700,n_muestrasNPA700
				outi	NPA700_valido,'0'
				outi	Acc_NPA700L,0x00
				outi	Acc_NPA700M,0x00
				outi	Acc_NPA700H,0x00
				outi	estado_i2cNPA,'0'
;				outi	twbr,132			;Inicializa la interface I2C
;				outi	twcr,0x04
;				outi	twsr,0x00
;****************************************************
;****************************************************
;****************************************************

				outi	tccr2a,0b00000010	;El timer 2 se usa para el PWM de la
				outi	tcnt2,0x00			;electrovalvula de exhalacion
				outi	timsk2,0x02
				outi	ocr2a,69;Val_ocr2a


				outi	PWM_exhalacion,0
				outi	recarga_PWM,0
				outi	Periodo_exhalacion,200
				outi	tccr2b,0b00000010
				ELECTROVALVULAS 0

				outi	cmd_pendiente,0x00

			outi	envia_trama_U,'0';Se habilita el envió de la trama U para la primera vez
			outi	tx_U,'0'		;Validación de tx cada (ya transcurrieron los 300ms)

			outi	tmr_inicio_exhalacion,0
			outi	tmr_t_inicial_exha_electro,0

			outi	timer_O2H,high(1500)
			outi	timer_O2L,low(1500)
			outi	habilita_control_O2,'0'

			outi	reg_O2H,0
			outi	reg_O2L,0

			outi	datos_actualizados,'0'
			outi	B_C_VV,'0'

;			outi	tmr_de_presion,0

//*******************valores a calibrar por equipo 
			outi 	offset_PEEPH,high(100)
			outi 	offset_PEEPL,low(100)
			
			outi 	offset_PIPH,high(115)
			outi 	offset_PIPL,low(115)

			outi 	offset_COMFLPEEPH,high(100)
			outi 	offset_COMFLPEEPL,low(100)
			
			outi 	offset_COMFLPIPH,high(120)
			outi 	offset_COMFLPIPL,low(120)

			outi 	offset_PreH,high(150)
			outi 	offset_PreL,low(150)
			
			outi 	offset_batH,high(585)
			outi 	offset_batL,low(585)
			
			outi 	G_batH,high(16)
			outi 	G_batL,low(16)

			outi	rx_Ctrl_Vol,'0'
			outi	rx_Ctrl_FiO2,'0'

			outi	VV_OFF,60
			
			outi	CPAP_alarma_APNEA,'0'
			OUTI	FASE,						STANDBY
			outi		tx_b_sp,'0'

			;ldE2	r16,dt_O2
			;ldE2	r17,dt_O2+1

			;stE2	dt_O2+2,r16
			;stE2	dt_O2+3,r17

			outiw		cont_2H,cont_2L,0
			outi		b_sp,'0'
			outi		B_act,'0'

			outi		B_FIO2_BAJO,'0'				
			outi		B_FIO2_ALTO,'0'	
			outi		C_A_FiO2,0
			outi		B_ox_max,0


            sei                                 ;Habilita de manera global las interrupciones
            ret

