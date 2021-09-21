;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
/*
Comando
Ping			P			P

Firmware		F			F <ON_OFF> <VERSION DE FIRMWARE>

Test			T			Test de electrovalvulas
							T<PWM flujo><PWM O2><PWM aire><electro exha><test>
								PWM flujo	Valor de PWM para electrovalvula 0 (4 bytes)
								PWM O2		Valor de PWM para electrovalvula 1 (4 bytes)
								PWM aire	Valor de PWM para electrovalvula 2 (4 bytes)
								electr exha	Valor para la electrovalvula de exhalación
											'0' ó '1'
								test		'1'	Entra en modo de prueba
											'0' Sale del modo de prueba

Calibra 21%		O0			VERIFICA 21%
							O0<CODIGO DE FALLA>
								0	SENSOR OK
								1	SENSOR EN FALLA

Calibra 100%	O1			VERIFICA 100%
							O1<CODIGO DE FALLA>
								0	SENSOR OK
								1	SENSOR EN FALLA

Configuración	M			Comando para operación del respirador
							M<MODO><Tiempo i><Tiempo p><Tiempo e><Frec><>				
							 <PWM flujo><PWM O2><PWM aire><ON/OFF>
								MODO		Modo de operacion
											'0'	Modo CMV por presión
											'1' Modo CMV por volumen
											'2' Modo SIMV por presión
											'3' Modo SIMV por volumen
											'4' Modo PSV (definido, pero aun no tiene función)
											'5' Modo CPRV (definido, pero aun no tiene función)
											'6' Modo CPAP (definido, pero aun no tiene función)
								Tiempo i	Tiempo de inspiración (3 bytes x 0.1seg)
								Tiempo p	Tiempo pausa (3 bytes x 0.1seg)
								Tiempo e	Tiempo de exhalación (3 bytes x 0.1seg)
								Frec		Frecuencia respiratoria (3 bytes)
								PWM flujo	Valor de PWM para electrovalvula 0 (4 bytes)
								PWM O2		Valor de PWM para electrovalvula 1 (4 bytes)
								PWM aire	Valor de PWM para electrovalvula 2 (4 bytes)
								%O2				
								Pinspiración	3 bytes
								PEEP			3 bytes
								PS				3 bytes
								Pmax			3 bytes
								ON/OFF		'1'	Arranque del respirador
											'0' Detiene el respirador (regresa a STANDBY)
*/
;************************************************************************************
int_rx0_comandos:
			push	r16
			pushw
			pushx
			pushr	sreg
			inr		r25,udr_debug
			outr	buff_rx_debug,r25

			call	desactiva_alarma_ping_int_rx

			cpri	cont_rx0,0x00
			rbrne	compara_cont_1
;Decodifica el tipo de comando
			inr			r16,buff_rx_debug
			cpi			r16,'M'
			rbreq		comando_ok
			cpi			r16,'F'
			rbreq		ejecuta_F
			cpi			r16,'P'
			rbreq		comando_ok;ejecuta_P;
			cpi			r16,'O'
			rbreq		conf_O2
			cpi			r16,'T'
			rbreq		comando_ok
			cpi			r16,'Q'
			rbreq		ejecuta_Q
			cpi			r16,'U'
			rbreq		ejecuta_U
			cpi			r16,'C'
			rbreq		conf_mezcl
			cpi			r16,'A'
			rbreq		comando_ok
			jmp			salir_int_rx0
/**//////////////////////////////////////
APAG	:	outi		tmr_debug,0x00

			movr		rx_OFF		,buffer_rx0+0			;Rx_modo

			
			outi		cmd_pendiente,'A'
			jmp		salir_int_rx0


///*****************************************************
conf_O2:	call		desactiva_alarma_ping_int_rx
			outr		comando_rx0,r16
			outi		tmr_debug,200
			incr		cont_rx0
			outi		apuntador_buffer_rx0H,high(buffer_rx0)
			outi		apuntador_buffer_rx0L,low(buffer_rx0)
			jmp			salir_int_rx0
///*****************************************************
conf_mezcl:	call		desactiva_alarma_ping_int_rx
			outr		comando_rx0,r16
			outi		tmr_debug,200
			incr		cont_rx0
			outi		apuntador_buffer_rx0H,high(buffer_rx0)
			outi		apuntador_buffer_rx0L,low(buffer_rx0)
			jmp			salir_int_rx0
///*************************************


comando_ok:	call		desactiva_alarma_ping_int_rx
			outr		comando_rx0,r16
			outi		tmr_debug,200
			incr		cont_rx0
			outi		apuntador_buffer_rx0H,high(buffer_rx0)
			outi		apuntador_buffer_rx0L,low(buffer_rx0)
			jmp			salir_int_rx0

compara_cont_1:
		;	cpri	comando_rx0,'O'
		;	rbreq	tst_ejecuta_O
			inr			r16,buff_rx_debug	;En el caso de los comandos M y T, se verifica
			cpi			r16,0x0A			;cuando se rx el byte 0x0A
			rbreq		fin_rx_cadena
			cpi			r16,'0'				;Si el byte no corresponde a un valor ASCII
			rbrlo		tx_error_val_ascii_int_rx	;se desecha la trama
			cpi			r16,'9'+1
			rbrsh		tx_error_val_ascii_int_rx
;En caso de que el byte si sea un caracter entre '0' y '9', se guarda en el buffer
			inr		xh,apuntador_buffer_rx0H
			inr		xl,apuntador_buffer_rx0L
			st		x+,r16
			outr	apuntador_buffer_rx0H,xh
			outr	apuntador_buffer_rx0L,xl
			incr		cont_rx0
			jmp			salir_int_rx0

fin_rx_cadena:
			cpri	comando_rx0,'M'
			rbreq	tst_ejecuta_M
			cpri	comando_rx0,'C'
			rbreq	tst_ejecuta_C
			cpri	comando_rx0,'A'
			rbreq	APAG
			cpri	comando_rx0,'O'
			rbreq	tst_O2
			cpri	comando_rx0,'P'
			rbreq	ejecuta_P
			jmp		tst_ejecuta_T

salir_int_rx0:
			popr	sreg
			popx
			popw
			pop		r16
			reti
;******************************************* *****************************************

;************************************************************************************
int_tx0_respuesta:
			pushw
			pushx
			pushr	sreg
			cpri	cont_tx0,0x00
			brne	skip_restaura_usart

				inr			r25,udr_debug			;Borra posible int de Rx
				usart0		115200,8,n,1,rxtx,irxtx;Se habilita la int de Rx
	
			rjmp	salir_int_tx0

skip_restaura_usart:
			inr		xh,apuntador_buffer_tx0H
			inr		xl,apuntador_buffer_tx0L
			ld		r25,x+
			outr	apuntador_buffer_tx0H,xh
			outr	apuntador_buffer_tx0L,xl
			outr	udr_debug,r25
			decr	cont_tx0

salir_int_tx0:
			popr	sreg
			popx
			popw
			reti
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
;Seccion del comando U
;Este comando se emplea dentro del proceso de pruebas iniciales para borrar la bandera
;de envio perodico de trama U
ejecuta_U:	call		desactiva_alarma_ping_int_rx
			outi		envia_trama_U,'0'
			outi		tx_U,'0'
			outi		tmr_U,0
			//apagar timer 
			rjmp		salir_int_rx0
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
ejecuta_Q:	call	desactiva_alarma_ping_int_rx
			outi	cmd_pendiente,'Q'
			jmp		salir_int_rx0
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
;Sección del comando T
;Este comando solo se acepta si esta en STANDBY
tst_ejecuta_T:
			outi		tmr_debug,0x00
			;outi		B_PR_TST,'0'
			cpri		FASE,STANDBY
			breq		rx_bytes_tst
			cpri		FASE,TST_ELECTROVALVULAS
			rbrne		tx_error_rango_int_rx

rx_bytes_tst:
			movr		rx_PWM0H,buffer_rx0+0	;Valor de PWM para ventilar
			movr		rx_PWM0MH,buffer_rx0+1
			movr		rx_PWM0ML,buffer_rx0+2
			movr		rx_PWM0L,buffer_rx0+3

			movr		rx_PWM1_O2H,buffer_rx0+4		;Valor de PWM para ventilar
			movr		rx_PWM1_O2MH,buffer_rx0+5
			movr		rx_PWM1_O2ML,buffer_rx0+6
			movr		rx_PWM1_O2L,buffer_rx0+7

			movr		rx_PWM2_AireH,buffer_rx0+8		;Valor de PWM para ventilar
			movr		rx_PWM2_AireMH,buffer_rx0+9
			movr		rx_PWM2_AireML,buffer_rx0+10
			movr		rx_PWM2_AireL,buffer_rx0+11

			movr		rx_tst_electro_exp,buffer_rx0+12

			movr		rx_tst_run,buffer_rx0+13


			movr		B_TST_Run,rx_tst_electro_exp

			

			cpri				rx_tst_run,'1'
			breq				carga_tst
			cpri				rx_tst_run,'0'
			rbreq				finaliza_tst
			jmp					tx_error_rango_int_rx

carga_tst:	outi		FASE,PRE_TST_ELECTROVALVULAS
			outi		ON_OFF,'2'
			outi		B_TST_Run,'1'
			jmp			salir_int_rx0

finaliza_tst:
			FASE_STANDBY
			outi		B_TST_Run,'0'
;tx_respuesta_T:
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'T'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			movr	buffer_tx0+1,rx_PWM0H
			movr	buffer_tx0+2,rx_PWM0MH
			movr	buffer_tx0+3,rx_PWM0ML
			movr	buffer_tx0+4,rx_PWM0L
			movr	buffer_tx0+5,rx_PWM1_O2H
			movr	buffer_tx0+6,rx_PWM1_O2MH
			movr	buffer_tx0+7,rx_PWM1_O2ML
			movr	buffer_tx0+8,rx_PWM1_O2L
			movr	buffer_tx0+9,rx_PWM2_AireH
			movr	buffer_tx0+10,rx_PWM2_AireMH
			movr	buffer_tx0+11,rx_PWM2_AireML
			movr	buffer_tx0+12,rx_PWM2_AireL
			movr	buffer_tx0+13,rx_tst_electro_exp
			movr	buffer_tx0+14,rx_tst_run
			outi	buffer_tx0+15,0x0A
			outi	cont_tx0,15

			outi	udr_debug,'T'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx

			jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
;Sección del comando P
ejecuta_P:	call	desactiva_alarma_ping_int_rx
			outi		tmr_debug,0x00

			movr	rx_Ctrl_Vol,buffer_rx0+0
			movr	rx_Ctrl_FiO2,buffer_rx0+1
			outi	cmd_pendiente,'P'
					jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
ejecuta_F:	call		desactiva_alarma_ping_int_rx
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'F'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			movr	buffer_tx0+1,ON_OFF
			outi	buffer_tx0+2,version_firmwareH
			outi	buffer_tx0+3,version_firmwareP
			outi	buffer_tx0+4,version_firmwareM
			outi	buffer_tx0+5,version_firmwareP
			outi	buffer_tx0+6,version_firmwareL
			outi	buffer_tx0+7,0x0A
			outi	cont_tx0,7

			outi	udr_debug,'F'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx

			jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
;Sección del comando de calibración de O2
tst_ejecuta_O:
			outi		tmr_debug,0x00

			cpri		FASE,STANDBY
			rbrne		tx_error_rango_int_rx

			inr			r16,buff_rx_debug
			outr		rx_CAL_O2,r16

			inr					r16,rx_CAL_O2
			cpi					r16,'0'		;Si este byte es '0', indica que efectuara
			rbreq				calibra_21	;la calibración al 21%
			cpi					r16,'1'		;Si este byte es '1', indica que efectuara
			rbrne				tx_error_rango_int_rx	;la calibración al 100%

calibra_100:
			pre_FASE_O2_100
			jmp		salir_int_rx0

calibra_21:	
			pre_FASE_O2_21
			jmp		salir_int_rx0
;************************************************************************************
;************************************************************************************
;Sección del comando C
tst_ejecuta_C:
			outi		tmr_debug,0x00

			movr		rx_OPEEPMH		,buffer_rx0+0			;Rx_modo
			movr		rx_OPEEPML		,buffer_rx0+1		;Decenas
			movr		rx_OPEEPL		,buffer_rx0+2		;Unidades
			movr		rx_OPIPMH		,buffer_rx0+3		;Decimas de segundo
			movr		rx_OPIPML		,buffer_rx0+4		;Centesimas de segundo
			movr		rx_OPIPL		,buffer_rx0+5			;Espera los bytes del tiempo "pausa"
			movr		rx_O_CPEEPMH	,buffer_rx0+6
			movr		rx_O_CPEEPML	,buffer_rx0+7
			movr		rx_O_CPEEPL		,buffer_rx0+8
			movr		rx_O_CPIPMH		,buffer_rx0+9
			movr		rx_O_CPIPML		,buffer_rx0+10
			movr		rx_O_CPIPL		,buffer_rx0+11
			movr		rx_O_CPRESMH	,buffer_rx0+12
			movr		rx_O_CPRESML	,buffer_rx0+13			
			movr		rx_O_CPRESL		,buffer_rx0+14
			movr		rx_O_BATMH		,buffer_rx0+15
			movr		rx_O_BATML		,buffer_rx0+16
			movr		rx_O_BATL		,buffer_rx0+17
			movr		rx_O_GAN_BATMH	,buffer_rx0+18
			movr		rx_O_GAN_BATML	,buffer_rx0+19
			movr		rx_O_GAN_BATL	,buffer_rx0+20
			movr		rx_VV_off_MH	,buffer_rx0+21
			movr		rx_VV_off_ML	,buffer_rx0+22
			movr		rx_VV_off_L		,buffer_rx0+23			
			outi		cmd_pendiente,'C'
			jmp		salir_int_rx0
;************************************************************************************
tst_O2:
			outi		tmr_debug,0x00

			movr		rx_CAL_O2		,buffer_rx0+0			;Rx_modo
			outi		FASE,STANDBY
			
			

			inr					r16,rx_CAL_O2
			cpi					r16,'0'	
			rbreq				calibra_21
			cpi					r16,'1'	
			rbreq				calibra_100
		;	rbrne				tx_error_rango_int_rx
			;outi		cmd_pendiente,'O'
			jmp		salir_int_rx0
;************************************************************************************

;Sección del comando M
tst_ejecuta_M:
			outi		tmr_debug,0x00

			cpri				FASE,TST_ELECTROVALVULAS
			rbreq				tx_error_rango_int_rx
			cpri				FASE,PRE_TST_ELECTROVALVULAS
			rbreq				tx_error_rango_int_rx
			cpri				FASE,CALIBRA_O2_21
			rbreq				tx_error_rango_int_rx
			cpri				FASE,CALIBRA_O2_100
			rbreq				tx_error_rango_int_rx

			movr		rx_Modo_op		,buffer_rx0+0			;Rx_modo
			movr		rx_TIEMPO_INSH	,buffer_rx0+1		;Decenas
			movr		rx_TIEMPO_INSMH	,buffer_rx0+2		;Unidades
			movr		rx_TIEMPO_INSML	,buffer_rx0+3		;Decimas de segundo
			movr		rx_TIEMPO_INSL	,buffer_rx0+4		;Centesimas de segundo
			movr		rx_TIEMPOpausaH	,buffer_rx0+5			;Espera los bytes del tiempo "pausa"
			movr		rx_TIEMPOpausaMH,buffer_rx0+6
			movr		rx_TIEMPOpausaML,buffer_rx0+7
			movr		rx_TIEMPOpausaL	,buffer_rx0+8
			movr		rx_TIEMPO_EXPH	,buffer_rx0+9
			movr		rx_TIEMPO_EXPMH	,buffer_rx0+10
			movr		rx_TIEMPO_EXPML	,buffer_rx0+11
			movr		rx_TIEMPO_EXPL	,buffer_rx0+12
			movr		rx_FRH			,buffer_rx0+13			;Frecuencia respiratoria
			movr		rx_FRM			,buffer_rx0+14
			movr		rx_FRL			,buffer_rx0+15

			movr		rx_PWM0H		,buffer_rx0+16		;Valor de PWM para ventilar
			movr		rx_PWM0MH		,buffer_rx0+17
			movr		rx_PWM0ML		,buffer_rx0+18
			movr		rx_PWM0L		,buffer_rx0+19

			movr		rx_PWM1_O2H		,buffer_rx0+20		;Valor de PWM para ventilar
			movr		rx_PWM1_O2MH	,buffer_rx0+21
			movr		rx_PWM1_O2ML	,buffer_rx0+22
			movr		rx_PWM1_O2L		,buffer_rx0+23

			movr		rx_PWM2_AireH	,buffer_rx0+24		;Valor de PWM para ventilar
			movr		rx_PWM2_AireMH	,buffer_rx0+25
			movr		rx_PWM2_AireML	,buffer_rx0+26
			movr		rx_PWM2_AireL	,buffer_rx0+27

			movr		rx_O2H			,buffer_rx0+28		;Concentración de O2
			movr		rx_O2M			,buffer_rx0+29
			movr		rx_O2L			,buffer_rx0+30

			movr		rx_PinspiracionH,buffer_rx0+31
			movr		rx_PinspiracionM,buffer_rx0+32
			movr		rx_PinspiracionL,buffer_rx0+33
			movr		rx_PEEPH		,buffer_rx0+34
			movr		rx_PEEPM		,buffer_rx0+35
			movr		rx_PEEPL		,buffer_rx0+36
			movr		rx_PSH			,buffer_rx0+37
			movr		rx_PSM			,buffer_rx0+38
			movr		rx_PSL			,buffer_rx0+39
			movr		rx_PmaxH		,buffer_rx0+40
			movr		rx_PmaxM		,buffer_rx0+41
			movr		rx_PmaxL		,buffer_rx0+42

wait_On_Off:movr		rx_ON_OFF		,buffer_rx0+43

			outi		cmd_pendiente,'M'
			jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
tx_error_rango_int_rx:
			outi	cont_rx0,0x00	;Desecha los datos que se Rx

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'E'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'R'
			outi	buffer_tx0+2,0x0A
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)
			outi	cont_tx0,2

			outi	udr_debug,'E'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx

			jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
tx_error_val_ascii_int_rx:
			outi	cont_rx0,0x00	;Desecha los datos que se Rx

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'E'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'A'
			outi	buffer_tx0+2,0x0A
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)
			outi	cont_tx0,2

			outi	udr_debug,'E'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx

			jmp		salir_int_rx0
;************************************************************************************

;************************************************************************************
desactiva_alarma_ping_int_rx:
			outi		tmr_pingL,low(tiempo_ping_20s/10)		;Temporizador para ping
			outi		tmr_pingH,high(tiempo_ping_20s/10)		;0.1seg x tmr_ping
			outi		tmr_ping_run,0x01
			outi		tmp_led,10
			outi		tccr3b,0x08								;Desactiva la salida de alarma
			cbi			ALARMA
			ret
