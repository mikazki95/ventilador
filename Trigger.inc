;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;************************************************************
rx_trigger:
			pushw
			pushr	sreg
			inr		r25,udr1
			outr	tmp_udr1,r25

			cpi		r25,'1'
			rbreq	si_es_trigger
			cpi		r25,'0'
			rbreq	rx_ping_sv
						cpi		r25,'X'
						rbrne	tst_modo_CPAP
						cpri	datos_actualizados,'1'	;Verifica si se actualizaron
						rbreq	borra_bandera_datos		;parametros del ventilador
						outi	tmp_udr1,'0'			;De no ser así, responde con
						jmp		rx_ping_sv				;'0'
borra_bandera_datos:	outi	datos_actualizados,'0'
						jmp		rx_ping_sv				;Responde con 'X' (el mismo
														;dato que se Rx

;En modo CPAP, puede Rx '2' si se detecta la alarma de APNEA
;Tambien puede Rx '3' si se determina que ya no hay alarma de APNEA
tst_modo_CPAP:		cpri	Modo_Op,P_CPAP
					breq	tst_rx_2_3
					cpri	Modo_Op,V_CPAP
					breq	tst_rx_2_3
					rjmp	tst_alarma_sv


tst_rx_2_3:			cpri	tmp_udr1,'2'
					rbreq	entra_CPAP_asistido
					cpri	tmp_udr1,'3'
					rbreq	salir_CPAP_asistido

tst_alarma_sv:
			inr		r25,cont_alarma_sv
			cpi		r25,0x05		;5 intentos consecutivos
			rbreq	salir_int_rx1
			inc		r25
			outr	cont_alarma_sv,r25
			outi	tmr_sv,50
			outi	alarma_sv,0x01
			jmp		salir_int_rx1

si_es_trigger:
			;sbi		led_run


			cpri	FASE,CPAP_SIN_APNEA
			rbreq	tst_P_CPAP

			cpri	FASE,PRESION_SOPORTE
			rbrne	tst_tiempo_pausa_ps
;Si esta en la fase de PS, verifica si el PIN_PS = 1 (PORTL,2), de ser asi, se
;rechaza el trigger, en caso contrario se acepta, debido a que indica que esta
;dentro de la temporizacion de PS, pero se recibio el pulso para liberar la
;presión, lo cual en realidad indica que ya es exhalación, por lo tanto debe de
;admitir un nuevo trigger
				inr		r25,portl
				andi	r25,0b00000100
				cpi		r25,0b00000100
				rbreq	rx_ping_sv
				rjmp	tst_SIMV

tst_tiempo_pausa_ps:
			
			cpri	FASE,TIEMPO_PAUSA_PS;Si esta en fase de tiempo pausa pero
			rbreq	tst_SIMV			;de presión soporte, se acepta el trigger
										;(es implicito que es dentro de uno de los
										;modos SIMV)
										;EN CASO CONTRARIO SIGUE:
			cpri	FASE,EXHALACION		;Solo si esta en la fase de exhalacion se acepta
			rbrne	rx_ping_sv			;el trigger
/*
			cpri	FASE,INSPIRACION	;Si se recibe el dato del trigger, pero esta en la
			breq	rx_ping_sv			;fase de inspiración o tiempo pausa, solo se
			cpri	FASE,TIEMPO_PAUSA	;considera como ping con la tarjeta sv
			breq	rx_ping_sv
*/
tst_CMV:
			outi	trigger,'1'

			cpri	Modo_op,V_CMV
			breq	asigna_inspiracion
			cpri	Modo_op,P_CMV
			breq	asigna_inspiracion
			jmp		tst_SIMV
asigna_inspiracion:
			outi		presion_liberada,'0'

			FASE_INSPIRACION
			jmp		rx_ping_sv

tst_SIMV:	cpri	Modo_op,V_SIMV
			breq	tst_ventana
			cpri	Modo_op,P_SIMV
			breq	tst_ventana
			jmp		tst_P_CPAP
tst_ventana:cpri	ventana,'0'			;En SIMV si ventana = '0', se aplica un ciclo
			rbreq	asigna_inspiracion_asistida	;normal de inspiración, de no ser asi, se

			outi		presion_liberada,'0'
			FASE_PRESION_SOPORTE		;aplica presion soporte
			jmp		rx_ping_sv

asigna_inspiracion_asistida:
			;outi		ventana,'1'

;			inr			r24,tmp_tmr_ventanaSIMV_L
;			inr			r25,tmp_tmr_ventanaSIMV_H
;			outr		tmr_ventanaSIMV_L,r24
;			outr		tmr_ventanaSIMV_H,r25

			FASE_INSPIRACION;_ASISTIDA
			jmp		rx_ping_sv
;			jmp			asigna_inspiracion

;*************************************************************
;FUNCIONALIDAD DEL TRIGGER EN LOS MODOS CPAP (SIN APNEA)
;*************************************************************
tst_P_CPAP:
					;sbi		led_run
;					cpri	Modo_op,P_CPAP
;					rbreq	asigna_PS_CPAP
;tst_V_CPAP:		cpri	Modo_op,V_CPAP
;					rbreq	asigna_PS_CPAP
;					jmp		asigna_PS_CPAP
asigna_PS_CPAP:		outi		presion_liberada,'0'
					outi	porta,				0b00000100
					FASE_PRESION_SOPORTE		;aplica presion soporte
					jmp		rx_ping_sv

entra_CPAP_asistido:
					cpri	CPAP_alarma_APNEA,'1'
					rbreq	rx_ping_sv
					outi	porta,				0b00000010
					outi	CPAP_alarma_APNEA,'1'
					FASE_INSPIRACION;_ASISTIDA
					jmp		rx_ping_sv

salir_CPAP_asistido:
					cpri	CPAP_alarma_APNEA,'0'
					rbreq	rx_ping_sv
					outi	porta,				0b00000001
					outi	CPAP_alarma_APNEA,'0'
					FASE_CPAP_SIN_APNEA
					jmp		rx_ping_sv

rx_ping_sv:
			outi		tmr_sv,50
			outi		alarma_sv,0x00
			outi		cont_alarma_sv,0x00



			cpri	tx_b_sp,'1'
			brne	pres_ok
			;sbi		led_run
			outi	tx_b_sp,'2'//outi
			outi	tmp_udr1,'P'
			jmp		pres_ok_2
pres_ok:	
;			sbi		led_run
			cpri	tx_b_sp,'0'
			breq	pres_ok_2
			inr		r13,portL
			sbrc	R13,6
			jmp		pres_ok_2	

			cpri	b_sp,'1'
			breq	pres_ok_2
			outi	tx_b_sp,'0'//outi
			outi	tmp_udr1,'B'
pres_ok_2:
			inr		r25,tmp_udr1
			outr	udr1,r25
salir_int_rx1:
			popr	sreg
			popw
			reti

alarma_nivel1:
			outr		alarmas_criticas,r25

			rjmp	rx_ping_sv
;************************************************************
