;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;************************************************************
int_PCINT4y5:
			pushw
			pushr	sreg
			sbic	PCINT_4
			rjmp	tst_retro
			sbic	PCINT_5
			rjmp	int_cancela_inspiracion


salir_int_PCINT:
			popr	sreg
			popw
			reti

tst_retro:	inr		r25,pinl
			andi	r25,0b00000011
			cpi		r25,0x00
			rbreq	int_INC_PWM0
			cpi		r25,0x01
			rbreq	int_DEC_PWM0
			cpi		r25,0x02
			rbreq	int_CORTA_PWM0_MODO_VOLUMEN
			rjmp	salir_int_PCINT
;************************************************************

;************************************************************
;Esta subrutina solo se tiene que ejecutar en modo volumen, en caso de que la
;tarjeta SV determine que el volumen se esta pasando demasiado.
int_CORTA_PWM0_MODO_VOLUMEN:

			cpri	FASE,STANDBY
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN
			cpri	FASE,CALIBRA_O2_21
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN
			cpri	FASE,CALIBRA_O2_100
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN
			cpri	FASE,TST_ELECTROVALVULAS
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN

			cpri	Modo_op,P_CMV
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN
			cpri	Modo_op,P_SIMV
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN
			cpri	Modo_op,P_CPAP
			rbreq	salir_int_CORTA_PWM0_MODO_VOLUMEN

			cpri	Modo_op,V_CMV
			rbreq	corta_PWM0_volumen
			cpri	Modo_op,V_SIMV
			rbreq	corta_PWM0_volumen
			cpri	Modo_op,V_CPAP
			rbreq	corta_PWM0_volumen

corta_PWM0_volumen:
			;ASIGNA_PWM_FLUJO					0,0
			outi	B_Vol,'1'

salir_int_CORTA_PWM0_MODO_VOLUMEN:
			popr	sreg
			popw
			reti
;************************************************************

;************************************************************
int_INC_PWM0:
			
;			cpri	inhibe_inc_dec_pwm,'1'
;			rbreq	salir_int_inc
			cpri	b_sp,'1'
			rbreq	salir_int_PCINT;int_DEC_PWM0;
			cpri	presion_maxima,'1'
			rbreq	salir_int_PCINT;int_DEC_PWM0;
			cpri	tx_b_sp,'1'
			rbreq	salir_int_PCINT;int_DEC_PWM0;
			cpri	tx_b_sp,'2'
			rbreq	salir_int_PCINT;int_DEC_PWM0;
;			cpri	rx_Ctrl_Vol,'1'
;			rbreq	salir_int_PCINT


			cpri	FASE,STANDBY
			rbreq	salir_int_inc
			cpri	FASE,CALIBRA_O2_21
			rbreq	salir_int_inc
			cpri	FASE,CALIBRA_O2_100
			rbreq	salir_int_inc
			cpri	FASE,TST_ELECTROVALVULAS
			rbreq	salir_int_inc

			cpri	Modo_op,P_CMV
			rbreq	salir_int_inc
			cpri	Modo_op,P_SIMV
			rbreq	salir_int_inc
			cpri	Modo_op,P_CPAP
			rbreq	salir_int_inc

			cpri	Modo_op,V_CMV
			rbreq	Incrementa_volumen
			cpri	Modo_op,V_SIMV
			rbreq	Incrementa_volumen
			cpri	Modo_op,V_CPAP
			rbreq	Incrementa_volumen

Incrementa_volumen:
			INC_VOLUMEN

salir_int_inc:
			popr	sreg
			popw
			reti
;************************************************************

;************************************************************
int_DEC_PWM0:
;			cpri	inhibe_inc_dec_pwm,'1'
;			rbreq	salir_int_dec

			cpri	FASE,STANDBY
			rbreq	salir_int_dec
			cpri	FASE,CALIBRA_O2_21
			rbreq	salir_int_dec
			cpri	FASE,CALIBRA_O2_100
			rbreq	salir_int_dec
			cpri	FASE,TST_ELECTROVALVULAS
			rbreq	salir_int_dec

			cpri	Modo_op,P_CMV
			rbreq	salir_int_dec
			cpri	Modo_op,P_SIMV
			rbreq	salir_int_dec
			cpri	Modo_op,P_CPAP
			rbreq	salir_int_dec


			cpri	Modo_op,V_CMV
			rbreq	Decrementa_volumen
			cpri	Modo_op,V_SIMV
			rbreq	Decrementa_volumen
			cpri	Modo_op,V_CPAP
			rbreq	Decrementa_volumen

Decrementa_volumen:
			DEC_VOLUMEN
salir_int_dec:
			popr	sreg
			popw
			reti
;************************************************************

;************************************************************
int_cancela_inspiracion:
			cpri	FASE,TST_ELECTROVALVULAS
			rbreq	libera_tst
			rjmp	alarma_sobrepresion

;			cpri	FASE,EXHALACION
;			brne	alarma_sobrepresion
;			ASIGNA_PWM_FLUJO					0,0;reg_PWM0H_6p25p,reg_PWM0L_6p25p
;			ELECTROVALVULAS						0b00	;Fase de expiración
;			rjmp	salir_int0

;Si durante el test se presenta una presión excesiva y al parecer la Raspberry no dio
;la orden de liberar, se ejecuta el siguiente código
libera_tst:	ASIGNA_PWM_FLUJO					0,0
			;ASIGNA_PWM_V_ex  					0,0;ELECTROVALVULAS						0b00
			ASIGNA_PWM_O2						0,0
			ASIGNA_PWM_AIRE						0,0
			rjmp	salir_int0

alarma_sobrepresion:
;			ASIGNA_PWM_FLUJO					0,0
;			ELECTROVALVULAS						0b00
			outi	presion_maxima,'1'

		;			outi	presion_liberada,'1'
salir_int0:	popr	sreg
			popw
			reti
;************************************************************
