;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;************************************************************

;************************************************************

;************************************************************
timer_10ms:	
//////////////////////////////////////////////////////////////////////////////////////////////
;			reti
//////////////////////////////////////////////////////////////////////////////////////////////
			pushw
			pushr	sreg
			pushx
			pushz
;			cpri	tmr_de_presion,0
;			breq	skip_tmr_presion
;			decr	tmr_de_presion



;*******************timer control de fio2********************
;************************************************************
/*			inr		XL,portl
			sbrs	XL,6	
			jmp		skip_timer_ctrl_fio2
			inr		XH,En_Ctrl_FiO2H
			inr		XL,En_Ctrl_FiO2L
			cpi		XL,0x00
			cpci	XH,0x00
			rbreq	s_tmr_Ctrl_FiO2
			sbiw	X,1
			outr	En_Ctrl_FiO2H,XH
			outr	En_Ctrl_FiO2L,XL
			inr		ZH,Set_Ctrl_FiO2H
			inr		ZL,Set_Ctrl_FiO2L
			cp		XL,ZL
			cpc		XH,ZH
			brlo	set_O2




			outi	B_ctrl_FiO2,0x00

			outi	reg_PWM1_O2H,high(0)
			outi	reg_PWM1_O2L,low(0)
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			outi	reg_PWM2_AireH,high(450)
			outi	reg_PWM2_AireL,low(450)
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL


			Jmp		skip_timer_ctrl_fio2

set_O2:	

		
			outi	B_ctrl_FiO2,0x01


			movr	reg_PWM1_O2H,PWM_FiO2_O2H
			movr	reg_PWM1_O2L,PWM_FiO2_O2L
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L

			outi	reg_PWM2_AireH,high(0)
			outi	reg_PWM2_AireL,low(0)
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
		

*/
			;Jmp		skip_timer_ctrl_fio2
;************************************************************
;************************************************************

s_tmr_Ctrl_FiO2:
			;movr	En_Ctrl_FiO2H,Set_Ctrl_FiO2H
			;movr	En_Ctrl_FiO2L,Set_Ctrl_FiO2L
			outi	En_Ctrl_FiO2H,0x00;17
			outi	En_Ctrl_FiO2L,0x32
skip_timer_ctrl_fio2:

			cpri	inhibe_on_off_pwm,'1'
			brne	timer_adc
			decr	tmr_inhibe_on_off_pwm
			brne	timer_adc
			outi	inhibe_on_off_pwm,'0'

;Sección agregada para FiO2 y lectura de Vbat
timer_adc:	cpri	admux,0b00000001
			breq	lee_Bateria
			cpri	admux,0b00000010
			rbreq	lee_O2
			cpri	admux,0b00000011
			breq	lee_presion_aire
;Si admux no tiene ninguno de los valores anteriores, quiere decir que:
;	admux = 0b00000101
;			cpri	admux,0b00000101
;			breq	lee_presion_O2

;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
;Lectura del ADC correspondiente a la presion de entrada de O2
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
lee_presion_O2:
			inr		r24,ADCL
			inr		r25,ADCH
			outr	Presion_hex_O2H,r25
			outr	Presion_hex_O2L,r24
fin_adc_presion_O2:
			outi	admux, 0b00000010	;Selecciona el canal de Presion de aire
			rjmp	sigue_TMR0
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
;Lectura del ADC correspondiente a la presion de entrada de aire
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
lee_presion_aire:
			inr		r24,ADCL
			inr		r25,ADCH
			outr	Presion_hex_aireH,r25
			outr	Presion_hex_aireL,r24
fin_adc_presion_aire:
			outi	admux, 0b00000101	;Selecciona el canal de Presion de aire
			rjmp	sigue_TMR0
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
;Lectura del ADC correspondiente al voltaje de la bateria del
;No Break
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
lee_Bateria:inr		r24,ADCL
			inr		r25,ADCH
cp_nivel0:	;ldiw	xh,xl,642
			inr		xH,offset_batH
			inr		xL,offset_batL
			cp		r24,xl
			cpc		r25,xh
			brlo	cp_nivel1
			outi	nivel_bat,'0'
			rjmp	fin_adc_bat
cp_nivel1:	;ldiw	xh,xl,631
			inr		zL,G_batL;low(500);reg_PEEPL
			inr		zh,G_batH;low(500);reg_PEEPH
			inr		xL,offset_batL;low(500);reg_PEEPL
			inr		xH,offset_batH
			sub		xl,zL
			sbc		xh,zH
			cp		r24,xL
			cpc		r25,xH
			brlo	cp_nivel2
			outi	nivel_bat,'1'
			rjmp	fin_adc_bat
cp_nivel2:	;ldiw	xh,xl,611
			inr		zL,G_batL;low(500);reg_PEEPL
			inr		zH,G_batH;low(500);reg_PEEPH
			inr		xL,offset_batL;low(500);reg_PEEPL
			inr		xH,offset_batH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			cp		r24,xL
			cpc		r25,xH
			brlo	cp_nivel3
			outi	nivel_bat,'2'
			rjmp	fin_adc_bat
cp_nivel3:	;ldiw	xh,xl,576
			inr		zL,G_batL;low(500);reg_PEEPL
			inr		zh,G_batH;low(500);reg_PEEPH
			inr		xL,offset_batL;low(500);reg_PEEPL
			inr		xH,offset_batH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			cp		r24,xl
			cpc		r25,xh
			brlo	cp_nivel4
			outi	nivel_bat,'3'
			rjmp	fin_adc_bat
cp_nivel4:	;ldiw	xh,xl,533
			inr		zL,G_batL;low(500);reg_PEEPL
			inr		zh,G_batH;low(500);reg_PEEPH
			inr		xL,offset_batL;low(500);reg_PEEPL
			inr		xH,offset_batH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			sub		xL,zL
			sbc		xH,zH
			cp		r24,xL
			cpc		r25,xH
			brlo	es_nivel5
			outi	nivel_bat,'4'
			rjmp	fin_adc_bat
es_nivel5:	outi	nivel_bat,'5'

fin_adc_bat:;outi	selector_O2_Bateria,'0'
			outi	admux, 0b00000011	;Selecciona el canal de Presion de aire
			rjmp	sigue_TMR0
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
;Lectura del ADC correspondiente a O2
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
lee_O2:		inr		r24,ADCL
			inr		r25,ADCH
			inr		xl,Acc_FiO2L
			inr		xh,Acc_FiO2H
			add		xl,r24
			adc		xh,r25
			outr	Acc_FiO2L,xl
			outr	Acc_FiO2H,xH
			decr	cont_muestras
			brne	sigue_TMR0
			outi	cont_muestras,n_shift

;el registro X, mantiene el valor acumulado
siguiente_shift:
			lsr		xh
			ror		xl
			decr	cont_muestras
			brne	siguiente_shift

			subi	xl,low(6)
			sbci	xh,high(6)

			outr	FiO2_nuevoL,xl
			outr	FiO2_nuevoH,xh
			outi	FiO2_valido,'1'	;Bandera para validar la lectura
			outi	cont_muestras,n_muestras
			outi	Acc_FiO2L,0x00
			outi	Acc_FiO2H,0x00
;			outi	selector_O2_Bateria,'1'
			outi	admux, 0b00000001	;Selecciona el canal de la bateria
;*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

sigue_TMR0:	outi	adcsra,0b11000111	;Arranque a la conversión A/D

;Sección para verificar el time-out de la comunicación con la tarjeta SV
			inr		r25,tmr_sv
			cpi		r25,0x00
			breq	dec_tmr_led
			dec		r25
			outr	tmr_sv,r25
			rbrne	dec_tmr_led
			outi	tmr_sv,50
			outi	alarma_sv,0x01

;Sección de la temporización para el parpadeo del led
dec_tmr_led:
			decr	tmr_led
			brne	salir_tmr_led
			movr	tmr_led,tmp_led
			;sbi		led_run
salir_tmr_led:

;Sección del time-out de la USART "debug"
			cpri	tmr_debug,0x00
			breq	fin_timer_debug
			decr	tmr_debug
			brne	fin_timer_debug
			outi	cont_rx0,0x00	;Desecha los datos que se Rx

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'E'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'T'
			outi	buffer_tx0+2,0x0A
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)
			outi	udr_debug,'E'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx
			outi	cont_tx0,2
fin_timer_debug:

;Sección de temporización del ping con la tarjeta Raspberry
tst_ping_time:
			cpri		tmr_ping_run,0x00
			breq		tiempo_fase
			inr			r24,tmr_pingL
			inr			r25,tmr_pingH
			sbiw		r24,1
			outr		tmr_pingL,r24
			outr		tmr_pingH,r25
			brne		tiempo_fase
			outi		tmr_pingL,low(tiempo_ping_20s/10)		;Temporizador para ping
			outi		tmr_pingH,high(tiempo_ping_20s/10)	;0.01seg x tmr_ping
			outi		tmr_ping_run,0x02

;Sección de temporización de las FASES de operación
tiempo_fase:

//falla_cal_100
			;rjmp	salir_contar_pulsos
;Si esta en la fase de prueba de electrovalvulas y esta activa la bandera "envia_U",
;se procede a tx la cadena U
			cpri		FASE,TST_ELECTROVALVULAS
			brne		verifica_otras_fases
			cpri		envia_trama_U,'0'
			rbreq		temporizacion_general
			;rjmp		salir_contar_pulsos
			;decr		tmr_U
			inr			R25,tmr_U 	////nuevo
			cpi			R25,0x02	////nuevo
			rbrlo		rst_tmr_U///rbrne
			subi		R25,1
			outr		tmr_U,R25
			rjmp		salir_contar_pulsos
rst_tmr_U:
			outi		tmr_U,150		;30 x 10ms = 300ms
			outi		tx_U,'1'
			rjmp		salir_contar_pulsos

verifica_otras_fases:
			
			outi	start_exha,'0'
			
	/*		cpri		FASE,STANDBY		;Si la fase es STANDBY, se cancela el resto
			rbreq		salir_contar_pulsos	;del proceso.

			cpri		FASE,CALIBRA_O2_21	;En caso de que se esten ejecutando alguna
			rbreq		salir_contar_pulsos	;de las calibraciones de O2, tambien se
			;cpri		FASE,CALIBRA_O2_21	;cancela el resto del proceso.
			;rbreq		salir_contar_pulsos
			cpri		FASE,CALIBRA_O2_100	;cancela el resto del proceso.
			rbreq		salir_contar_pulsos
*/			cpri		ON_OFF,'0'
			rbreq		salir_contar_pulsos
			cpri		ON_OFF,'2'
			rbreq		salir_contar_pulsos
			cpri		Modo_op,P_SIMV
			breq		temporizacion_ventana
			cpri		Modo_op,V_SIMV
			breq		temporizacion_ventana
			jmp			temporizacion_general
temporizacion_ventana:
			inr			r24,tmr_ventanaSIMV_L
			inr			r25,tmr_ventanaSIMV_H
			sbiw		r24,1
			outr		tmr_ventanaSIMV_L,r24
			outr		tmr_ventanaSIMV_H,r25
			rbrne		temporizacion_general

			outi		ventana,'0'		;Este registro se usa para determinar a función
										;del trigger en modo SIMV
			inr			r24,tmp_tmr_ventanaSIMV_L
			inr			r25,tmp_tmr_ventanaSIMV_H
			outr		tmr_ventanaSIMV_L,r24
			outr		tmr_ventanaSIMV_H,r25

		;	rjmp		temporizacion_general

temporizacion_general:
			;sbi		led_run
			inr			r24,cont_10msL
			inr			r25,cont_10msH
			ldiw		xh,xl,6001
			cp			r24,xl
			cpc			r25,xh
			brsh		Fin_ciclo
			ldiw		xh,xl,0000
			cp			r24,xl
			cpc			r25,xh
			breq		Fin_ciclo
			sbiw		r24,1
			;sbi		led_run
			outr		cont_10msL,r24
			outr		cont_10msH,r25
			

			rjmp		salir_contar_pulsos
;TST_FASE:	

fin_ciclo:	
			cpri		FASE,pre_CALIBRA_O2_21
			rbreq		W_CALIBRA_O2_21
			cpri		FASE,pre_CALIBRA_O2_100
			rbreq		W_CALIBRA_O2_100
			cpri		FASE,TST_ELECTROVALVULAS
			rbreq		salir_contar_pulsos
			outi		presion_maxima,'0'
			cpri		CPAP_alarma_APNEA,'1'
			rbreq		tst_fase_inspiracion ;breq =/brsh >/brlo<
			cpri		FASE,CPAP_SIN_APNEA
			rbrne		tst_fase_inspiracion
			INSP_EXHA	0
			ASIGNA_TIEMPO_10ms;3p99s
			outi	stop_ventilador,0x00
			rjmp	salir_contar_pulsos

W_CALIBRA_O2_21:
			;sbi		led_run
			;outi	FASE,					CALIBRA_O2_21
			FASE_O2_21
			rjmp	salir_contar_pulsos

W_CALIBRA_O2_100:
			
			;outi	FASE,					CALIBRA_O2_21
			FASE_O2_100
			;sbi		led_run
salir_contar_pulsos:
			;ret
/*			cpri	timer_O2H,0
			brne	dec_timer_O2
			cpri	timer_O2L,0
			breq	recupera_registros

       antiguo control de FiO2
dec_timer_O2:

			inr		YL,PWM_FLUJOL
			inr		YH,PWM_FLUJOH
			inr		xh,offset_COMFLPEEPH
			inr		xl,offset_COMFLPEEPL
			cp		YL,XL
			cpc		YH,XH
			rbrlo	recupera_registros
	

			inr		xh,timer_O2H
			inr		xl,timer_O2L
			sbiw	xl,1
			outr	timer_O2H,xh
			outr	timer_O2L,xl
			brne	recupera_registros
			outi	habilita_control_O2,'1';'1'*/
;Se verifica el número de fase en curso, para cambiarla


			inr		xh,timer_O2H
			inr		xl,timer_O2L
			cpi		XL,0X00
			cpci	XH,0X00
			breq	dec_timer_O2
			sbiw	xl,1
			outr	timer_O2H,xh
			outr	timer_O2L,xl
			rjmp	recupera_registros

dec_timer_O2:
			outi	habilita_control_O2,'1'

recupera_registros:
			popz
			popx
			popr		sreg
			popw
			reti
;activa_INSP_EXHA:	
					

tst_fase_inspiracion:
			;sbi		led_run
			cpri		FASE,INSPIRACION
			rbreq		inicia_tiempo_pausa
				cpri		FASE,PRESION_SOPORTE
				rbreq		finaliza_PS
			cpri		FASE,TIEMPO_PAUSA
			rbreq		inicia_exhalacion
				cpri		FASE,TIEMPO_PAUSA_PS
				rbreq		inicia_exhalacion
			cpri		FASE,EXHALACION
			rbreq		termino_exhalacion
			jmp			inicia_inspiracion

termino_exhalacion:
;			outi		ventana,'0'
					outi		inspiracion_asistida,'0'

inicia_inspiracion:

			cpri	B_act,'1'
			rbrne	inicia_fase_inspiracion
			outi	B_act,'0'
			outi	cont_10msH,high(20)
			outi	cont_10msL,low (20)
			rjmp	salir_contar_pulsos
/*
			cpri	stop_ventilador,0x01	;Antes de iniciar la fase de inspiración, se
			rbrne	inicia_fase_inspiracion	;verifica si hay una solicitud para detener
											;el respirador o bien para actualizar valores
			FASE_ESPERA						;Esta condición la indica el registro
			rjmp	salir_contar_pulsos		;stop_ventilador = 0x01, de ser asi, se borra
											;el registro y se da por terminada la
											;interrupción, lo siguiente dependera de la
											;ejecución del comando 'M'
*/
inicia_fase_inspiracion:
			outi		presion_liberada,'0'

			FASE_INSPIRACION
			rjmp		salir_contar_pulsos

;PASO 2
inicia_tiempo_pausa:
			inr		r24,tmr_TIEMPOpausaL
			inr		r25,tmr_TIEMPOpausaH
			cpi		r24,0x00
			cpci	r25,0x00
			rbreq	no_hay_pausa
;Asigna valores para Tpausa
			FASE_TIEMPO_PAUSA
			;	sbi		led_run
			rjmp	salir_contar_pulsos

finaliza_PS:
			cpri	Modo_Op,P_CPAP
			rbreq	verifica_alarma_APNEA
			cpri	Modo_Op,V_CPAP
			rbreq	verifica_alarma_APNEA
			jmp		inicia_exhalacion		;Finalizo PS en modo SIMV

verifica_alarma_APNEA:
			cpri		CPAP_alarma_APNEA,'1'
			rbreq		inicia_exhalacion
regresa_a_CPAP_sin_APNEA:



;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
						
;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
;;;;;+++++++++++++++++++++++++++++++
			FASE_CPAP_SIN_APNEA
			rjmp		salir_contar_pulsos

;PASO 3
no_hay_pausa:
inicia_exhalacion:
exhalacion_completa:
			outi	start_exha,'1'
			FASE_EXHALACION
			rjmp		salir_contar_pulsos

final_de_espera:
			outi		stop_ventilador,0x00




;************************************************************

;************************************************************
timer_frecuencia:
			pushw
			pushr	sreg
			sbis	ALARMA
			rjmp	pin_alarma_0
			cbi		ALARMA
			inr		r24,tmp_ocr3al_0
			inr		r25,tmp_ocr3ah_0
			outr	ocr3ah,r25
			outr	ocr3al,r24
			rjmp	salir_alarma

pin_alarma_0:
			inr		r24,tmr_alarmaL
			inr		r25,tmr_alarmaH
			subi	r24,0x01
			sbci	r25,0x00
			outr	tmr_alarmaL,r24
			outr	tmr_alarmaH,r25
			brne	set_pin
			inr		r25,estado_mute
			cpi		r25,0x01
			breq	desactiva_mute
			outi		estado_mute,0x01
			inr			r24,tmr_alarma_muteL
			inr			r25,tmr_alarma_muteH
			outr		tmr_alarmaL,r24
			outr		tmr_alarmaH,r25
			rjmp		set_pin
desactiva_mute:
			outi		estado_mute,0x00
			inr			r24,tmr_alarma_onL
			inr			r25,tmr_alarma_onH
			outr		tmr_alarmaL,r24
			outr		tmr_alarmaH,r25
			rjmp		set_pin

set_pin:	inr		r25,estado_mute
			cpi		r25,0x01
			breq	solo_carga
			sbi		ALARMA
solo_carga:	inr		r24,tmp_ocr3al_1
			inr		r25,tmp_ocr3ah_1
			outr	ocr3ah,r25
			outr	ocr3al,r24
			rjmp	salir_alarma
salir_alarma:
			popr	sreg
			popw
			reti
;************************************************************
