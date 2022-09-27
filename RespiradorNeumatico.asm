/*
3.0
15 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
	VERSION PARA CONTROL DE ELECTROVALVULAS 

	Esta versión toma la lectura de un sensor de presión, para mejorar el control de las valvulas

	ESTA VERSION USA LA INTERRUPCION DE RX DE LA USART 0
*/

.include    "m2560def.inc"

;#define    simulacion

#define	version_firmwareH	'5'
#define	version_firmwareP	'.'
#define	version_firmwareM	'0'
;#define	version_firmwareLP	'.'
#define	version_firmwareL	'0'

.equ	fxtal		=	11059200		;Frecuencia del cristal

.equ	udr_debug	=	udr0
.equ	txc_debug	=	txc0
.equ	rxc_debug	=	rxc0
.equ	udre_debug	=	udre0
.equ	ucsra_debug	=	ucsr0a

.equ		Fpwm			=	20000
#define		Val_PWMmax		Fxtal/Fpwm
#define		FR_max			200
#define		T_max			6000			;60.00 segundos
#define		PWMmin			60
#define		PWMZERO			10	;353
#define		PWM_VV_Max		200



#define		PWMmax			551

.equ		Fpwm_exhalacion	=	1000/2
#define		Periodo_pwm_exhalacion	1/Fpwm_exhalacion
#define		Resolucion_pwm			Periodo_pwm_exhalacion/100
#define		Val_ocr2a				(Fxtal/8)*Resolucion_pwm

#define		Val_6p25p_escala_completa	((PWMmax-PWMmin)*0.0625)+PWMmin

#define		Val_12p5p_escala_completa	((PWMmax-PWMmin)*0.125)+PWMmin

#define		Val_time_outI2C				138	;0.0001/(1/(Fxtal/8))

#define		Pins_max		151
#define		PEEP_max		51
#define		PS_max			51
#define		P_max			17100		;120.00 cmH20
											;Valores para electrovalvulas 1
#define		Offset_NPA700			100;75

#define		limite_de_compensacion		130;130

#define		flujo_inicial_compensacion	80;100
;salto simulacion
			;jmp Primer_PEEP_D

            .org	0x0000
            jmp		inicio
			.org	0x0012
			jmp		int_PCINT4y5
			.org	0x002A
			jmp		timer_10ms
			.org	0x0032
			jmp		int_rx0_comandos
			.org	0x0036
			jmp		int_tx0_respuesta
            .org	0x0040
			jmp		timer_frecuencia
			.org	0x0048
			jmp		rx_trigger

			.org	0x0100
;**********************************************************
.include	"Tabla_incrementoParker.inc"
.include	"TablaO2.inc"
.include	"TablaPresion.inc"
.include	"TablaPresionNPA700.inc"

.include    "macroinstrucciones.inc"
.include	"definiciones.inc"
.include	"fp_lib.inc"
.include	"Calculos.inc"
.include	"Timers.inc"
.include	"Retroalimentacion.inc"
.include	"Trigger.inc"
.include	"Comandos.inc"
.include	"NPA700.inc"
;************************************************************

;************************************************************
inicio:     outiw       sph,spl,ramend



			
;			jmp		rx_ping_sv//SECCION_TST_ELECTROVALVULAS
;			jmp		int_PWM_exhalacion

            call		init_io
inicio2:
	
;************************************************************
;salto simulacion
;call	PIP_Baja_P
;jmp	actualizar_Pres_PEEP

;jmp	inicio2



/********************************************////

			call	tx_listo

wait_rx:	call	rd_NPA700
			call	control_FiO2
			call	control_Parker_in
			;sbi		led_run
			cpri	FASE,CALIBRA_O2_100
			rbreq	SECCION_CAL_O2_100
			cpri	FASE,CALIBRA_O2_21
			rbreq	SECCION_CAL_O2_21
			cpri	FASE,PRE_TST_ELECTROVALVULAS
			rbreq	SECCION_TST_ELECTROVALVULAS
			cpri	cmd_pendiente,'P'
			rbreq	tx_datos_P
			cpri	cmd_pendiente,'M'
			rbreq	verifica_M
			cpri	cmd_pendiente,'B'
			rbreq	modo_manual
			cpri	cmd_pendiente,'C'
			rbreq	calib_mezcl
			cpri	cmd_pendiente,'A'
			rbreq	tx_APAG
			cpri	cmd_pendiente,'X'
			rbreq	OFF
			cpri	cmd_pendiente,'Q'
			rbreq	tx_datos_Q
			cpri	tx_U,'1'
			rbreq	tx_cadenaU

;++++++++++++++++++++++++++++++++++
			inr			r17,alarma_sv
			cpi			r17,0x00
			breq		tst_ping_a
			call		activa_alarma_SV
			jmp			activa_alarma
tst_ping_a:	cpri		tmr_ping_run,0x02	;¿Se detecto el time out del ping con la
			rbreq		enviar_S			;Raspberry?
			inr			r16,bandera_tx_estado
			cpi			r16,0x01
			rbrne		wait_rx
			outi		bandera_tx_estado,0x00
			rjmp		wait_rx

enviar_S:	call		tx_S

;Seccion de espera de comandos, despues de enviar el ping
			call	rd_NPA700
			;call	control_FiO2

			cpri	FASE,CALIBRA_O2_100
			rbreq	SECCION_CAL_O2_100
			cpri	FASE,CALIBRA_O2_21
			rbreq	SECCION_CAL_O2_21
			cpri	FASE,PRE_TST_ELECTROVALVULAS
			rbreq	SECCION_TST_ELECTROVALVULAS
			cpri	cmd_pendiente,'P'
			rbreq	tx_datos_P
			cpri	cmd_pendiente,'M'
			rbreq	verifica_M
			cpri	cmd_pendiente,'C'
			rbreq	calib_mezcl																																																																																																																																																																																												
			cpri	cmd_pendiente,'A'
			rbreq	tx_APAG
			cpri	cmd_pendiente,'X'
			rbreq	OFF
			cpri	cmd_pendiente,'Q'
			rbreq	tx_datos_Q
			cpri	tx_U,'1'
			rbreq	tx_cadenaU

;++++++++++++++++++++++++++++++++++
			inr			r17,alarma_sv
			cpi			r17,0x00
			breq		tst_S			;Si alarma_sv != 0 -> alarma signos vitales
			call		activa_alarma_SV
tst_S:		cpi			r16,'S'			;Verifica si hay que activar la alarma por ping
			breq		ping_ok

;+++++	Activa alarma por Raspberry
			call		activa_alarma_Raspberry
			rjmp		activa_alarma
;************************************************************************************

;************************************************************************************
ping_ok:	inr			r16,estado_alarmas
			cpi			r16,'0'
			rbreq		desactiva_alarma
			cpi			r16,'1'
			rbreq		wait_rx
			cpi			r16,'2'
			rbreq		desactiva_alarma
			cpi			r16,'3'
			breq		carga_1d
carga_1d:	outi		estado_alarmas,'1'
			rjmp		wait_rx

;Activa alarma
activa_alarma:
			outi		alarma_sv,0x00
			outi		tmp_led,2
			outi		tccr3b,0x09								;Activa la salida de alarma
			sbi			ALARMA
			outi		estado_mute,0x00
			inr			r16,tmr_alarma_onL
			inr			r17,tmr_alarma_onH
			outr		tmr_alarmaL,r16
			outr		tmr_alarmaH,r17
			cli
			outi		tmr_pingL,low(tiempo_ping_20s/10)		;Temporizador para ping
			outi		tmr_pingH,high(tiempo_ping_20s/10)		;0.1seg x tmr_ping
			sei
			rjmp		wait_rx

;+++++	Activa alarma por SV
activa_alarma_SV:
			inr			r16,estado_alarmas
			cpi			r16,'0'
			breq		carga_1c		;Si es '0', asigna '1'
			cpi			r16,'1'
			rbreq		regresa	;Si ya estaba en '1'
			cpi			r16,'2'
			breq		carga_3c		;Si estaba en '2', asigna '3', alarma conjugada
			ret							;Si no es ninguna de las anteriores es '3', mantiene la alarma
carga_1c:	outi		estado_alarmas,'1'
			ret
carga_3c:	outi		estado_alarmas,'3'
regresa:	ret

;+++++	Activa alarma por Raspberry
activa_alarma_Raspberry:
			inr			r16,estado_alarmas
			cpi			r16,'0'
			breq		carga_2b		;Si es '0', asigna '2'
			cpi			r16,'1'
			breq		carga_3b		;Si es '1', alarma conjugada, debe de cambiarse a '3'
			ret							;Si no es ninguna de las anteriores es '3', mantiene la alarma
carga_2b:	outi		estado_alarmas,'2'
			ret
carga_3b:	outi		estado_alarmas,'3'
			ret

desactiva_alarma:
			outi		tmp_led,10
			outi		tccr3b,0x08								;Desactiva la salida de alarma
			cbi			ALARMA
			rjmp		wait_rx
;************************************************************************************

/*
;************************************************************************************
desactiva_alarma_ping:
			cli
			outi		tmr_pingL,low(tiempo_ping_10s/10)		;Temporizador para ping
			outi		tmr_pingH,high(tiempo_ping_10s/10)		;0.1seg x tmr_ping
			sei
			outi		tmr_ping_run,0x01
			outi		tmp_led,10
			outi		tccr3b,0x08								;Desactiva la salida de alarma
			cbi			ALARMA
			ret
;************************************************************************************
*/

;************************************************************************************
tx_datos_Q:	outi	cmd_pendiente,0x00
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'Q'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			call	convierte_cmH2OHEX
			movr	buffer_tx0+1,tx_PSI_4
			movr	buffer_tx0+2,tx_PSI_3
			movr	buffer_tx0+3,tx_PSI_2

			movr	buffer_tx0+4,estado_i2cNPA

			outi	buffer_tx0+5,0x0A
			outi	cont_tx0,5

			outi	udr_debug,'Q'	;Inicia la tx del primer byte de respuesta del
									;Time out
			jmp		wait_rx
;************************************************************************************

;************************************************************
tx_cadenaU:	
			outi	tx_U,'0'
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'U'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			call	convierte_cmH2OHEX
			movr	buffer_tx0+1,tx_PSI_4
			movr	buffer_tx0+2,tx_PSI_3
			movr	buffer_tx0+3,tx_PSI_2

			movr	buffer_tx0+4,estado_i2cNPA

			outi	buffer_tx0+5,0x0A
			outi	cont_tx0,5

			outi	udr_debug,'U'	;Inicia la tx del primer byte de respuesta del
									;Time out
			jmp		wait_rx
;************************************************************

;************************************************************
tx_S:		
			cpri	FASE,TST_ELECTROVALVULAS
			rbrne	envia_ping_int_tx
			ret

envia_ping_int_tx:
			cli
			cpri		tmr_ping_run,0x02	;Esta condición es para verificar que
			brne		cancela_tx_S		;no haya llegado un byte Rx, mientras
											;se estaba preparando la respuesta del
											;ping

			outi		tmr_ping_run,0x01
			outi	cont_rx0,0x00	;Desecha los datos que se Rx

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'S'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,0x0A
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)
			outi	cont_tx0,1
			outi	udr_debug,'S'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx
cancela_tx_S:
			sei
			ret

;************************************************************************************
;************************************************************************************
;************************************************************************************
SECCION_CAL_O2_100:
			
			;outi			CONT_RET,0x13
			;call	tiempo
			inr					r16,FiO2_nuevoL	;Se obtiene el valor HEX en % de O2
			inr					r17,FiO2_nuevoH

			outr				FiO2HEX_100L,r16
			outr				FiO2HEX_100H,r17

			cpi					r16,low(1010)	;Si la lectura del sensor de O2 genera
			cpci				r17,high(1010)	;un resultado > o = 125, indica un error
			brlo				tst_97			;en la operación del hardware
			outi				estado_100_O2,'0'
			rjmp				tx_resultado_O1

tst_97:		cpi					r16,low(1005)
			cpci				r17,high(1005)
			brlo				tst_93
			outi				estado_100_O2,'9'
			rjmp				tx_resultado_O1

tst_93:		cpi					r16,low(1000)
			cpci				r17,high(1000)
			brlo				tst_89
			outi				estado_100_O2,'8'
			rjmp				tx_resultado_O1

tst_89:		cpi					r16,low(995)
			cpci				r17,high(995)
			brlo				tst_85
			outi				estado_100_O2,'7'
			rjmp				tx_resultado_O1

tst_85:		cpi					r16,low(990)
			cpci				r17,high(990)
			brlo				tst_82
			outi				estado_100_O2,'6'
			rjmp				tx_resultado_O1

tst_82:		cpi					r16,low(905)
			cpci				r17,high(905)
			brlo				tst_79
			outi				estado_100_O2,'5'
			rjmp				tx_resultado_O1

tst_79:		cpi					r16,low(850)
			cpci				r17,high(850)
			brlo				tst_76
			outi				estado_100_O2,'4'
			rjmp				tx_resultado_O1

tst_76:		cpi					r16,low(805)
			cpci				r17,high(805)
			brlo				tst_75
			outi				estado_100_O2,'3'
			rjmp				tx_resultado_O1

tst_75:		cpi					r16,low(770)
			cpci				r17,high(770)
			brlo				error_O2_1
			outi				estado_100_O2,'2'
			rjmp				tx_resultado_O1

error_O2_1:	outi				estado_100_O2,'1'

tx_resultado_O1:
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'O'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'1'
			movr	buffer_tx0+2,estado_100_O2
								;tx_debug_reg	estado_100_O2,'0'	<--- ¿'0'?
			outi	buffer_tx0+3,0x0A
			outi	cont_tx0,3
			CLI
			FASE_STANDBY
			SEI

			outi	udr_debug,'O'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resto de la tx depende de la int de tx
			cpri	estado_100_O2,'1'
			rbreq	salir_rx
			inr 	r16,FiO2_nuevoL
			inr 	r17,FiO2_nuevoH
			stE2	dt_O2_100,r16
			stE2	dt_O2_100+1,r17 
			call	calculo_G_FIO2
			jmp			wait_rx

;++++++++++++++++++++++++++++++++++++++++++++++++++
SECCION_CAL_O2_O_O:
jmp			wait_rx
;++++++++++++++++++++++++++++++++++++++++++++++++++
;++++++++++++++++++++++++++++++++++++++++++++++++++
SECCION_CAL_O2_C:
			outi			CONT_RET,0x13
			call			tiempo
jmp			wait_rx
;++++++++++++++++++++++++++++++++++++++++++++++++++
;++++++++++++++++++++++++++++++++++++++++++++++++++
SECCION_CAL_O2_21:
			;outi			CONT_RET,0x13
			;call			tiempo
				

;			call				calcula_FiO2_HEX
			inr					r16,FiO2_nuevoL	;Se obtiene el valor HEX en % de O2
			inr					r17,FiO2_nuevoH

			outr				FiO2HEX_21L,r16
			outr				FiO2HEX_21H,r17

			cpi					r16,low(285)	;Si la lectura del sensor de O2 genera;
			cpci				r17,high(285);un resultado > o = 27, indica un error
			brlo				tst_21		;en la operación del hardware
			outi				estado_21_O2,'0'
			rjmp				tx_resultado_O0

tst_21:		cpi					r16,low(215)
			cpci				r17,high(215)
			brlo				tst_20
			outi				estado_21_O2,'9'
			rjmp				tx_resultado_O0

tst_20:		cpi					r16,low(205)
			cpci				r17,high(205)
			brlo				tst_19
			outi				estado_21_O2,'8'
			rjmp				tx_resultado_O0

tst_19:		cpi					r16,low(195)
			cpci				r17,high(195)
			brlo				tst_18
			outi				estado_21_O2,'7'
			rjmp				tx_resultado_O0

tst_18:		cpi					r16,low(185)
			cpci				r17,high(185)
			brlo				tst_17
			outi				estado_21_O2,'6'
			rjmp				tx_resultado_O0

tst_17:		cpi					r16,low(180)
			cpci				r17,high(180)
			brlo				tst_16
			outi				estado_21_O2,'4'
			rjmp				tx_resultado_O0

tst_16:		cpi					r16,low(177)
			cpci				r17,high(177)
			brlo				tst_15
			outi				estado_21_O2,'3'
			rjmp				tx_resultado_O0

tst_15:		cpi					r16,low(174)
			cpci				r17,high(174)
			brlo				error_O2_0
			outi				estado_21_O2,'2'
			rjmp				tx_resultado_O1

error_O2_0:	outi				estado_21_O2,'1'

tx_resultado_O0:
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'O'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'0'
			movr	buffer_tx0+2,estado_21_O2
								;tx_debug_reg	estado_21_O2,'0'	<--- ¿'0'?
			outi	buffer_tx0+3,0x0A
			outi	cont_tx0,3
			CLI
			FASE_STANDBY
			SEI

			outi	udr_debug,'O'	;Inicia la tx del primer byte de respuesta del
									;Time out
			////////////////////////////**************************************************
			cpri	estado_21_O2,'1'
			rbreq	salir_rx
			inr 	r16,FiO2_nuevoL
			inr 	r17,FiO2_nuevoH
			stE2	dt_O2_21,r16
			stE2	dt_O2_21+1,r17 
			;call calcula_02_21
							;El resro de la tx depende de la int de tx

salir_rx:	jmp			wait_rx
;************************************************************************************
;************************************************************************************
;************************************************************************************
tiempo:		;outi			CONT_RET,0x13
retardo:	call			desactiva_alarma_ping_int_rx
			call			tx_S
			delay_ms		3000
			inr				r25,CONT_RET
			dec				r25	
			outr			CONT_RET,r25	
			brne			retardo
			ret
;************************************************************************************
;************************************************************************************
;************************************************************************************
SECCION_TST_ELECTROVALVULAS:
;Copia el buffer rx hacia los registros para calcular y convertir los valores
			COPY_SRAM	PWM0L,rx_PWM0L,16
			inr			r16,rx_tst_electro_exp
			cpi			r16,'0'
			breq		skip_cp_1
			cpi			r16,'1'
			rbrne		tx_error_rango_rx
skip_cp_1:	outr		tst_electro_exp,r16
			call		calcula_PWM0
			call		calcula_PWM1_O2
			call		calcula_PWM2_Aire
			cp_limites	tmp_reg_PWM0H,tmp_reg_PWM0L,				1,	Val_PWMmax,tx_error_rango_rx
			cp_limites	tmp_reg_PWM1_O2H,tmp_reg_PWM1_O2L,			1,	Val_PWMmax,tx_error_rango_rx
			cp_limites	tmp_reg_PWM2_AireH,tmp_reg_PWM2_AireL,		1,	Val_PWMmax,tx_error_rango_rx

					cli

					outi	PWM_FLUJOH,high(PWMZERO);;*************modificacion
					outi	PWM_FLUJOL,low(PWMZERO);;
					
					;outiw		PWM_O2H,PWM_O2L,0x0000

					;outiw		PWM_AIREH,PWM_AIREL,0x0190





					inr			r16,tmp_reg_PWM1_O2L
					outr		reg_PWM1_O2L,r16
					inr			r16,tmp_reg_PWM1_O2H
					outr		reg_PWM1_O2H,r16



					inr			r16,tmp_reg_PWM2_AireL
					outr		reg_PWM2_AireL,r16
					inr			r16,tmp_reg_PWM2_AireH
					outr		reg_PWM2_AireH,r16
					
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL


			;OUTI	FASE,FASE_TST
					FASE_TST
					sei
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

			jmp		wait_rx
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
tx_datos_P:	outi	cmd_pendiente,0x00
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'P'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			movr	buffer_tx0+1,estado_electrovalvulas
					outi		estado_alarmas,'0'
			movr	buffer_tx0+2,estado_alarmas
			movr	buffer_tx0+3,alarmas_criticas
					outi		alarmas_criticas,'0'
;Sección para tx FiO2
;					cpri	FiO2_valido,'1'	;Si aun no hay una lectura valida, salta
;					brne	tx_FiO2			;a transmitir la lectura anterior
					call	calcula_FiO2_BCD
				
				movr	buffer_tx0+4,tx_FiO2H	//sin lectura de sensor
				movr	buffer_tx0+5,tx_FiO2M	//sin lectura de sensor
				movr	buffer_tx0+6,tx_FiO2L	//sin lectura de sensor
				//cambio_606001
				outi	B_PEEP_out,0x30
				//fin_CAMBIOS_606001
;Sección para tx estado de la alimentación y de la bateria
			sbis			pinc,0
			rjmp			carga_modo_bat
carga_modo_AC:
			outi			modo_alimentacion,modo_AC
			rjmp			tx_modo_alimentacion
carga_modo_bat:
			outi			modo_alimentacion,modo_Bateria
			rjmp			tx_modo_alimentacion
tx_modo_alimentacion:

		//	movr	buffer_tx0+7,modo_alimentacion
		//	movr	buffer_tx0+8,nivel_bat

;6 BYTES DE LA LECTURA DE PRESION DE ENTRADA DE AIRE Y O2 (EN PSI)
			call			calcula_PRESION_AIRE
			call			calcula_PRESION_O2
			movr	buffer_tx0+7,tx_presion_aireH
			movr	buffer_tx0+8,tx_presion_aireM
			movr	buffer_tx0+9,tx_presion_aireL
			movr	buffer_tx0+10,tx_presion_O2H
			movr	buffer_tx0+11,tx_presion_O2M
			movr	buffer_tx0+12,tx_presion_O2L


/*
;************cambio para eliminar alarmas y permitir ventilar********
			outi	buffer_tx0+7,0x30	;modo_alimentacion,31
			outi	buffer_tx0+8,0x30	;nivel_bat,35
			
			outi	buffer_tx0+9,0x30	;presion_aireH,37
			outi	buffer_tx0+10,0x30	;presion_aireM
			outi	buffer_tx0+11,0x30	;presion_aireL

			outi	buffer_tx0+12,0x30	;presion_O2H,37
			outi	buffer_tx0+13,0x30	;presion_O2M
			outi	buffer_tx0+14,0x30	;presion_O2L

;*****************fin cambio de eliminacion de alarmas***************
*/
;Lectura de la presión de aire del flujo principal
			call	convierte_cmH2OHEX
			movr	buffer_tx0+13,tx_PSI_4
			movr	buffer_tx0+14,tx_PSI_3
			movr	buffer_tx0+15,tx_PSI_2




			movr	buffer_tx0+16,estado_i2cNPA

			movr	buffer_tx0+17,rx_Ctrl_Vol
			movr	buffer_tx0+18,rx_Ctrl_FiO2
			
			outi	buffer_tx0+19,0x0A
					outi	bandera_tx_estado,0x00
					outi	cont_tx0,19

			outi	udr_debug,'P'	;Inicia la tx del primer byte de respuesta del
									;Time out
				jmp		wait_rx
;************************************************************************************
;************************************************************************************
;************************************************************************************
OFF:
			;outi		cmd_pendiente,0x00
			call		desactiva_alarma_ping_int_rx
							;Time out
										;El resto de la tx depende de la int de tx
			;outi		cmd_pendiente,'X'
			rjmp		OFF 
;************************************************************************************
tx_APAG:
			outi		cmd_pendiente,0x00
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			call		desactiva_alarma_ping_int_rx
							;Time out
;ecco
			
			
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi		buffer_tx0+0	,'A'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa


			outi		buffer_tx0+1	,0x32


;retorno
			outi		buffer_tx0+2	,0x0A
			outi		cont_tx0,2

			outi		udr_debug,'A'
													;El resto de la tx depende de la int de tx
			outi		cmd_pendiente,'X'
			rjmp		wait_rx
		
;************************************************************************************ calibracion mezclador 
calib_mezcl:
			outi		cmd_pendiente,0x00
			
			conv_A_H	temp_H,temp_L,				rx_flujo_O2_MH,rx_flujo_O2_ML,rx_flujo_O2_L
			;cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			movr		offset_PWM_FlujoH,temp_H
			movr		offset_PWM_FlujoL,temp_L

//CAMBIO_606001
			conv_A_H	temp_H,temp_L,				rx_O_OFSTFJMH,rx_O_OFSTFJML,rx_O_OFSTFJL
			movr		offset_Val_PrincH,temp_H
			movr		offset_Val_PrincL,temp_L	

//fin_cambios_606001

			conv_A_H	temp_H,temp_L,				rx_Cont_O2_MH,rx_Cont_O2_ML,rx_Cont_O2_L
			;cp_limites	temp_H,temp_L,				'n',	520,	tx_error_rango_rx
			movr		Set_Ctrl_FiO2H,temp_H
			movr		Set_Ctrl_FiO2L,temp_L	
			
					
			conv_A_H	temp_H,temp_L,				rx_desc_min_MH,rx_desc_min_ML,rx_desc_min_L
			;cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			movr		desc_min_H,temp_H
			movr		desc_min_L,temp_L

			conv_A_H	temp_H,temp_L,				rx_desc_max_MH,rx_desc_max_ML,rx_desc_max_L
			;cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			
			movr		desc_max_H,temp_H;PWM_O2_TST_H,temp_H
			movr		desc_max_L,temp_L;PWM_O2_TST_L,temp_L

			conv_A_H	temp_H,temp_L,				rx_PWM_EXH_MIN_MH,rx_PWM_EXH_MIN_ML,rx_PWM_EXH_MIN_L
			;cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			
			movr		PWM_EXH_IN_H,temp_H
			movr		PWM_EXH_IN_L,temp_L
			
			;movr		PWM_min_val_exha_H,temp_H
			;movr		PWM_min_val_exha_L,temp_L	
			
			
			conv_A_H	offset_batH,offset_batL,	rx_O_BATMH,rx_O_BATML,rx_O_BATL
			/*conv_A_H	temp_H,temp_L,				rx_O_BATMH,rx_O_BATML,rx_O_BATL
			movr		PWM_EXH_IN_H,temp_H
			movr		PWM_EXH_IN_L,temp_L
			*/
			conv_A_H	G_batH,G_batL,				rx_O_GAN_BATMH,rx_O_GAN_BATML,rx_O_GAN_BATL


			
;limites


;aSIGNAR VALORES

asigna_valores:


;ecco
			call		desactiva_alarma_ping_int_rx
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi		buffer_tx0+0	,'C'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa


			movr		buffer_tx0+1	,rx_flujo_O2_MH
			movr		buffer_tx0+2	,rx_flujo_O2_ML
			movr		buffer_tx0+3	,rx_flujo_O2_L
			movr		buffer_tx0+4	,rx_O_OFSTFJMH
			movr		buffer_tx0+5	,rx_O_OFSTFJML
			movr		buffer_tx0+6	,rx_O_OFSTFJL
			movr		buffer_tx0+7	,rx_Cont_O2_MH
			movr		buffer_tx0+8	,rx_Cont_O2_ML
			movr		buffer_tx0+9	,rx_Cont_O2_L
			movr		buffer_tx0+10	,rx_desc_min_MH
			movr		buffer_tx0+11	,rx_desc_min_ML
			movr		buffer_tx0+12	,rx_desc_min_L
			movr		buffer_tx0+13	,rx_desc_max_MH
			movr		buffer_tx0+14	,rx_desc_max_ML
			movr		buffer_tx0+15	,rx_desc_max_L
			movr		buffer_tx0+16	,rx_PWM_EXH_MIN_MH
			movr		buffer_tx0+17	,rx_PWM_EXH_MIN_ML
			movr		buffer_tx0+18	,rx_PWM_EXH_MIN_L
			movr		buffer_tx0+19	,rx_O_BATMH
			movr		buffer_tx0+20	,rx_O_BATML
			movr		buffer_tx0+21	,rx_O_BATL
			movr		buffer_tx0+22	,rx_O_GAN_BATMH
			movr		buffer_tx0+23	,rx_O_GAN_BATML
			movr		buffer_tx0+24	,rx_O_GAN_BATL
;retorno
			outi		buffer_tx0+25	,0x0A
			outi		cont_tx0,25

			outi		udr_debug,'C'	;Inicia la tx del primer byte de respuesta del
										;Time out
										;El resto de la tx depende de la int de tx

			jmp		wait_rx
		

;************************************************************************************
;************************************************************************************
modo_manual:

			conv_A_H	temp_H,temp_L,				rx_PWM_Aire_MH,rx_PWM_Aire_H,rx_PWM_Aire_L
			cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			movr		PWM_Aire_TST_H,temp_H
			movr		PWM_Aire_TST_L,temp_L
							
			conv_A_H	temp_H,temp_L,				rx_PWM_O2_MH,rx_PWM_O2_H,rx_PWM_O2_L
			cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			movr		PWM_O2_TST_H,temp_H
			movr		PWM_O2_TST_L,temp_L

			conv_A_H	temp_H,temp_L,				rx_PWM_Flujo_MH,rx_PWM_Flujo_H,rx_PWM_Flujo_L
			cp_limites	temp_H,temp_L,				'n',	520,tx_error_rango_rx
			movr		Vol_TST_H,temp_H
			movr		Vol_TST_L,temp_L
				
			;conv_A_H	temp_H,temp_L,				rx_Modo_trabajo_MH,rx_Modo_trabajo_H,rx_Modo_trabajo_L
			;movr		Vol_TST_H,temp_H
			;movr		Modo_TST,temp_L	
			
			conv_A_H	temp_H,Modo_TST,			rx_Modo_trabajo_MH,rx_Modo_trabajo_H,rx_Modo_trabajo_L
			
;limites


;aSIGNAR VALORES

			outi		cmd_pendiente,0x00

;ecco
			call		desactiva_alarma_ping_int_rx
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi		buffer_tx0+0	,'B'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa



			movr		buffer_tx0+1	,rx_PWM_Aire_MH
			movr		buffer_tx0+2	,rx_PWM_Aire_H
			movr		buffer_tx0+3	,rx_PWM_Aire_L
			movr		buffer_tx0+4	,rx_PWM_O2_MH
			movr		buffer_tx0+5	,rx_PWM_O2_H
			movr		buffer_tx0+6	,rx_PWM_O2_L
			movr		buffer_tx0+7	,rx_PWM_Flujo_MH
			movr		buffer_tx0+8	,rx_PWM_Flujo_H
			movr		buffer_tx0+9	,rx_PWM_Flujo_L
			movr		buffer_tx0+10	,rx_Modo_trabajo_MH;rx_O_BATMH;
			movr		buffer_tx0+11	,rx_Modo_trabajo_H;rx_O_BATML;
			movr		buffer_tx0+12	,rx_Modo_trabajo_L

			/*
			movr		buffer_tx0+10	,rx_PWM_VAL_EXH_MH
			movr		buffer_tx0+11	,rx_PWM_VAL_EXH_H
			movr		buffer_tx0+12	,rx_PWM_VAL_EXH_L

			movr		buffer_tx0+16	,rx_O_BATMH
			movr		buffer_tx0+17	,rx_O_BATML
			movr		buffer_tx0+18	,rx_O_BATL
			movr		buffer_tx0+19	,rx_O_GAN_BATMH
			movr		buffer_tx0+20	,rx_O_GAN_BATML
			movr		buffer_tx0+21	,rx_O_GAN_BATL
*/

;retorno
			outi		buffer_tx0+13	,0x0A
			outi		cont_tx0,13

			outi		udr_debug,'B'	;Inicia la tx del primer byte de respuesta del
										;Time out
										;El resto de la tx depende de la int de tx

			jmp		wait_rx
;************************************************************************************
;************************************************************************************
verifica_M:	;outi	cmd_pendiente,0x00
			inr					r16,rx_ON_OFF
			outr				ON_OFF,r16
			cpi					r16,'0'		;Si este byte es '0', indica que se debe de detener el motor
			rbreq				detener_ventilador
			cpi					r16,'1'		;Si este byte es '1' indica que se deben de aplicar los parametros para
			rbrne				tx_error_rango_rx	;arranque del motor, en caso de no ser asi, es error

			outi	datos_actualizados,'0'
			cpri		FASE,STANDBY
			breq		arranque_inicial
;			outi	stop_ventilador,0x01
;espera_fin_de_ciclo:
;			cpri	stop_ventilador,0x01
;			breq	wait_rx;espera_fin_de_ciclo
			cpri		b_act,'1'
			rbreq		wait_rx
arranque_inicial:
			outi		C_A_FiO2,0
			outi 		C_X_FiO2,0
			//CAMBIO_606001
			outi		B_PEEP_ok,0x30
			//fin_CAMBIO_606001
			outi		cmd_pendiente,0x00
			
			CLI

;Copia el buffer rx hacia los registros para calcular y convertir los valores
			COPY_SRAM	TIEMPO_INSL,rx_TIEMPO_INSL,44

			call		calcula_TIEMPO_inspiracion
			call		calcula_TIEMPO_pausa
			call		calcula_TIEMPO_exhalacion
			call		calcula_FR
			call		calcula_PWM0

			call		calcula_O2_rx

			call		calcula_PWM1_O2


			call		calcula_PWM2_Aire
			call		calcula_tiempo_total	;TI + Tpausa + TE

			call		calcula_PEEP		;Se cambia el orden de calculo, para PEEP
			call		calcula_Pinspiracion;y Pinspiracion, para poder sumarlas en la
									;subrutina de la segunda
			call		calcula_PS			;También se suma PEEP al valor rx de PS
			call		calcula_Pmax

			inr			r16,rx_Modo_op
			cpi			r16,P_CMV
			breq		modo_ok
			cpi			r16,V_CMV
			breq		modo_ok
			cpi			r16,P_SIMV
			breq		modo_ok
			cpi			r16,V_SIMV
			breq		modo_ok
			cpi			r16,P_CPAP
			breq		modo_ok
			cpi			r16,V_CPAP
			breq		modo_ok
			jmp			tx_error_rango_rx

;cp_limites	regH,regL,'n',Valor_max
;Si se especifica 'n', indica que solo hay limite maximo.
;En este caso si el valor de los registros (reg) es menor o igual a Valor_max,
;continua en la siguiente linea, en caso contrario salta a tx_error
;
;cp_limites	regH,regL,Valor_min,Valor_max
;En este caso si el valor de los registros (reg) es mayor o igual a Valor_min
; "Y" menor o igual a Valor_max, continua en la siguiente linea, en caso
;contrario salta a tx_error
modo_ok:	

			outi	timer_O2H,high(4000);(1500)
			outi	timer_O2L,low(4000);(1500)


			cp_limites	tmp_tmr_TIEMPO_INSH,tmp_tmr_TIEMPO_INSL,	1,	T_max,tx_error_rango_rx
			cp_limites	tmp_tmr_TIEMPOpausaH,tmp_tmr_TIEMPOpausaL,	'n',T_max,tx_error_rango_rx
			cp_limites	tmp_tmr_TIEMPO_EXPH,tmp_tmr_TIEMPO_EXPL,	1,	T_max,tx_error_rango_rx
			
			cp_limites	tmp_reg_FRH,tmp_reg_FRL,					1,	T_max,tx_error_rango_rx
			
			cp_limites	tmp_reg_PWM0H,tmp_reg_PWM0L,				1,	Val_PWMmax,tx_error_rango_rx
			cp_limites	tmp_reg_PWM1_O2H,tmp_reg_PWM1_O2L,			1,	Val_PWMmax,tx_error_rango_rx
			cp_limites	tmp_reg_PWM2_AireH,tmp_reg_PWM2_AireL,		1,	Val_PWMmax,tx_error_rango_rx

			cp_limites	tmp_reg_O2H,tmp_reg_O2L,				21,101,tx_error_rango_rx
/*
			cp_limites	tmp_reg_PEEPH,tmp_reg_PEEPL,				'n',PEEP_max,tx_error_rango_rx
			cp_limites	tmp_reg_PinspiracionH,tmp_reg_PinspiracionL,'n',Pins_max,tx_error_rango_rx	
			cp_limites	tmp_reg_PSH,tmp_reg_PSL,					'n',PS_max,tx_error_rango_rx
			cp_limites	tmp_reg_PmaxH,tmp_reg_PmaxL,			'n',P_max,tx_error_rango_rx
*/
			;call		calcula_presionesFP
			//CAMBIOS_606001
			outi 		C_A_FiO2,0
			outi 		C_x_FiO2,0
			//fin_CAMBIOS_606001


			CLI
			inr			xh,reg_O2H	;Toma % actual de O2
			inr			xl,reg_O2L
			inr			yh,tmp_reg_O2H	;Toma % reprogramado de O2
			inr			yl,tmp_reg_O2L
			cp			xl,yl		;Si el valor de % es igual al anterior, no se
			cpc			xh,yh		;modifica el PWM
			brne		skip_cambia_PWM_O2

;***************************************************************
;***************************************************************


;************************************************************
;************************************************************



			movr		reg_PWM2_AireH,	tmp_reg_PWM2_AireH
			movr		reg_PWM2_AireL,	tmp_reg_PWM2_AireL
			movr		reg_PWM1_O2H,	tmp_reg_PWM1_O2H
			movr		reg_PWM1_O2L,	tmp_reg_PWM1_O2L

skip_cambia_PWM_O2:
;Copia los 24 bytes de parametros "tmp" a los registros de trabajo
			COPY_SRAM	tmr_TIEMPO_INSL,tmp_tmr_TIEMPO_INSL,24

			movr	reg_PEEPH_100porciento,tmp_reg_PEEPH_100porciento
			movr	reg_PEEPL_100porciento,tmp_reg_PEEPL_100porciento




			call		calcula_presionesFP

			outi		ventana,'0'		;Este registro se usa para determinar la función
										;del trigger en modo SIMV (diferenciar entre
										;asistida y presión soporte
			inr			r16,tmp_tmr_ventanaSIMV_L
			inr			r17,tmp_tmr_ventanaSIMV_H
			outr		tmr_ventanaSIMV_L,r16
			outr		tmr_ventanaSIMV_H,r17

;			CLI

//CAMBIO_606001
;PWMmin.
;***********************************************************
			inr		XL,offset_Val_PrincL
			inr		XL,offset_Val_PrincL
			inr		XH,offset_Val_PrincH
			inr		ZL,reg_PWM0L
			inr		ZH,reg_PWM0H
			add		ZL,XL
			adc		ZH,XH
			outr	reg_PWM0H,ZH
			outr	reg_PWM0L,ZL
			cpi		ZL,low(480)
			cpci	ZH,high(480)
			brlo	flj_ins_max
			;outi	reg_PWM0H,high(480)
			;outi	reg_PWM0L,low(480)

;************************************************************
flj_ins_max:
//fin_cambios_606001
			inr					r16,rx_Modo_op
			outr				Modo_op,r16
			cpri				Modo_op,P_CPAP
			rbreq				inicia_CPAP
			cpri				Modo_op,V_CPAP
			rbreq				inicia_CPAP

			outi	CPAP_ACTIVO,'0'
			FASE_INSPIRACION

					outi	datos_actualizados,'1'

			SEI
			call		desactiva_alarma_ping_int_rx
			jmp			tx_respuesta_M

inicia_CPAP:
			outi	CPAP_ACTIVO,'1'
			FASE_CPAP_SIN_APNEA
			SEI
			call		desactiva_alarma_ping_int_rx
			jmp			tx_respuesta_M
;*********************************************

;*********************************************
detener_ventilador:
			outi	cmd_pendiente,0x00
			cpri		FASE,STANDBY
			rbreq		skip_stop_v

			CLI
			FASE_STANDBY
			SEI
skip_stop_v:call		desactiva_alarma_ping_int_rx

tx_respuesta_M:
			call		desactiva_alarma_ping_int_rx
			outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi		buffer_tx0+0	,'M'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			movr		buffer_tx0+1	,rx_Modo_op
			movr		buffer_tx0+2	,rx_TIEMPO_INSH
			movr		buffer_tx0+3	,rx_TIEMPO_INSMH
			movr		buffer_tx0+4	,rx_TIEMPO_INSML
			movr		buffer_tx0+5	,rx_TIEMPO_INSL
			movr		buffer_tx0+6	,rx_TIEMPOpausaH
			movr		buffer_tx0+7	,rx_TIEMPOpausaMH
			movr		buffer_tx0+8	,rx_TIEMPOpausaML
			movr		buffer_tx0+9	,rx_TIEMPOpausaL
			movr		buffer_tx0+10	,rx_TIEMPO_EXPH
			movr		buffer_tx0+11	,rx_TIEMPO_EXPMH
			movr		buffer_tx0+12	,rx_TIEMPO_EXPML
			movr		buffer_tx0+13	,rx_TIEMPO_EXPL
			movr		buffer_tx0+14	,rx_FRH
			movr		buffer_tx0+15	,rx_FRM
			movr		buffer_tx0+16	,rx_FRL

			movr		buffer_tx0+17	,rx_PWM0H
			movr		buffer_tx0+18	,rx_PWM0MH
			movr		buffer_tx0+19	,rx_PWM0ML
			movr		buffer_tx0+20	,rx_PWM0L

			movr		buffer_tx0+21	,rx_PWM1_O2H
			movr		buffer_tx0+22	,rx_PWM1_O2MH
			movr		buffer_tx0+23	,rx_PWM1_O2ML
			movr		buffer_tx0+24	,rx_PWM1_O2L

			movr		buffer_tx0+25	,rx_PWM2_AireH
			movr		buffer_tx0+26	,rx_PWM2_AireMH
			movr		buffer_tx0+27	,rx_PWM2_AireML
			movr		buffer_tx0+28	,rx_PWM2_AireL

			movr		buffer_tx0+29	,rx_O2H
			movr		buffer_tx0+30	,rx_O2M
			movr		buffer_tx0+31	,rx_O2L

			movr		buffer_tx0+32	,rx_PinspiracionH
			movr		buffer_tx0+33	,rx_PinspiracionM
			movr		buffer_tx0+34	,rx_PinspiracionL
			movr		buffer_tx0+35	,rx_PEEPH
			movr		buffer_tx0+36	,rx_PEEPM
			movr		buffer_tx0+37	,rx_PEEPL
			movr		buffer_tx0+38	,rx_PSH
			movr		buffer_tx0+39	,rx_PSM
			movr		buffer_tx0+40	,rx_PSL
			movr		buffer_tx0+41	,rx_PmaxH
			movr		buffer_tx0+42	,rx_PmaxM
			movr		buffer_tx0+43	,rx_PmaxL

			movr		buffer_tx0+44	,rx_ON_OFF

			outi		buffer_tx0+45	,0x0A
			outi		cont_tx0,45

			outi		udr_debug,'M'	;Inicia la tx del primer byte de respuesta del
										;Time out
										;El resto de la tx depende de la int de tx
			jmp		wait_rx
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
tx_error_rango_rx:
			outi	cmd_pendiente,0x00
			sei
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

			jmp		wait_rx
;************************************************************************************
;************************************************************************************
;************************************************************************************

;************************************************************************************
;************************************************************************************
;************************************************************************************
tx_listo:	outi	cont_rx0,0x00	;Desecha los datos que se Rx
			outi	apuntador_buffer_tx0H,high(buffer_tx0+1)
			outi	apuntador_buffer_tx0L,low(buffer_tx0+1)

					usart0		115200,8,n,1,rxtx,itx;Se deshabilita la int de Rx

			outi	buffer_tx0+0,'L'	;Escritura dummy (en realidad se tx el primer byte
										;de forma directa
			outi	buffer_tx0+1,'i'
			outi	buffer_tx0+2,'s'
			outi	buffer_tx0+3,'t'
			outi	buffer_tx0+4,'o'
			outi	buffer_tx0+5,0x0A
			outi	cont_tx0,5

			outi	udr_debug,'L'	;Inicia la tx del primer byte de respuesta del
									;Time out
							;El resro de la tx depende de la int de tx
			ret
;************************************************************

;************************************************************
control_FiO2:
			call	calcula_FiO2_HEX
;************************************************************
;****************modo manual*********************************			
			inr		YL,Modo_TST
			cpi		YL,0x02
			rbreq	FiO2_manual
;************************************************************
;************************************************************
			inr		ZL,reg_O2L		;compara si es 21% de oxigeno (hex)
			inr		ZH,reg_O2H
			cpi		ZL,0x15
			cpci	ZH,0x00
			rbreq	fiO2_21
;************************************************************
;************************************************************
			inr		ZL,reg_O2L		;compara si es 100% de oxigeno (hex)
			inr		ZH,reg_O2H
			cpi		ZL,0x64
			cpci	ZH,0x00
			rbreq	fiO2_100
;************************************************************
;************************************************************
			cpri	rx_Ctrl_FiO2,'1'
			rbreq	Falla_gases
;************************************************************
;************************************************************
			cpri	habilita_control_O2,'0'
			rbreq	retorno_control_O2

;************************************************************
;************************************************************
			cpri	B_FIO2_ANTL,'1'
			rbreq	SKIP_FiO2_read
			outi	B_FIO2_ANTL,'1'
			movr	FIO2_ANTL,FiO2HEX_L	
			movr	FIO2_ANTH,FiO2HEX_H	
SKIP_FiO2_read:

			inr		xl,FiO2HEX_L	;Lectura del sensor, el valor ya esta
			inr		xh,FiO2HEX_H	;convertido de ADC a % de Oxigeno en Hex
			inr		zl,reg_O2L		;Valor programado para FiO2 (hex)
			inr		zh,reg_O2H
			sbiw	ZL,5
			cp		xl,zl
			cpc		xh,zh
			;rbreq	Fio2_Ok
			rbrlo	aumentar_fio2;cp_o2_menos_5
			inr		xl,FiO2HEX_L	;Lectura del sensor, el valor ya esta
			inr		xh,FiO2HEX_H	;convertido de ADC a % de Oxigeno en Hex
			inr		zl,reg_O2L		;Valor programado para FiO2 (hex)
			inr		zh,reg_O2H
			sbiw	ZL,1
			cp		xl,zl
			cpc		xh,zh
			rbrlo	aumentar_fio2_1
			inr		xl,FiO2HEX_L	;Lectura del sensor, el valor ya esta
			inr		xh,FiO2HEX_H	;convertido de ADC a % de Oxigeno en Hex
			inr		zl,reg_O2L		;Valor programado para FiO2 (hex)
			inr		zh,reg_O2H
			adiw	ZL,6
			cp		xl,zl
			cpc		xh,zh
			rbrsh	disminuir_fio2
			inr		xl,FiO2HEX_L	;Lectura del sensor, el valor ya esta
			inr		xh,FiO2HEX_H	;convertido de ADC a % de Oxigeno en Hex
			inr		zl,reg_O2L		;Valor programado para FiO2 (hex)
			inr		zh,reg_O2H
			adiw	ZL,2
			cp		xl,zl
			cpc		xh,zh
			brsh	disminuir_fio2_1
			jmp		Fio2_Ok
			;rbrsh	disminuir_fio2

;Sensor > %O2 programado
;************************************************************
;****************disminuir_fio2_1****************************
disminuir_fio2_1:
			outi 	C_A_FiO2,0
			outi 	C_X_FiO2,0
;*****************disminuir FiO2*****************************
disminuir_fio2:
			inr		xl,FiO2HEX_L	
			inr		xh,FiO2HEX_H
			inr		zl,FIO2_ANTL	
			inr		zh,FIO2_ANTH
			sbiw	ZL,3
			cp		xl,zl
			cpc		xh,zh
			rbrlo	lim_desc_FiO2 
			////restalr O2
			jmp		restar_FiO2

lim_desc_FiO2:
			inr		xl,FiO2HEX_L	
			inr		xh,FiO2HEX_H
			inr		zl,FIO2_ANTL	
			inr		zh,FIO2_ANTH
			sbiw	ZL,6
			cp		xl,zl
			cpc		xh,zh
			rbrsh	rst_cont;act_FiO2
			////sumar O2	
			jmp		sumar_Fio2
;************************************************************
;****************aumentar_fio2_1*****************************
aumentar_fio2_1:
			outi 	C_A_FiO2,0
			outi 	C_X_FiO2,0
;**********************aumentar FiO2*************************
aumentar_fio2:
			inr		xl,FiO2HEX_L	
			inr		xh,FiO2HEX_H
			inr		zl,FIO2_ANTL	
			inr		zh,FIO2_ANTH
			adiw	ZL,4
			cp		xl,zl
			cpc		xh,zh
			rbrsh	lim_inc_FiO2
			
			jmp		sumar_FiO2

lim_inc_FiO2:
			inr		xl,FiO2HEX_L	
			inr		xh,FiO2HEX_H
			inr		zl,FIO2_ANTL	
			inr		zh,FIO2_ANTH
			sbiw	ZL,6
			cp		xl,zl
			cpc		xh,zh
			rbrlo	rst_cont;act_FiO2	
			jmp		restar_Fio2
;************************************************************
;*********************funciones generales********************
rst_cont:;act_FiO2
			////sumar O2
			outi 	C_X_FiO2,0
			outi 	C_A_FiO2,0	
			jmp		act_FiO2
;************************************************************
suma_max:	;outi	C_A_FiO2,0x32
			movr	C_A_FiO2,Set_Ctrl_FiO2L
			jmp		skip_suma_FiO2
;************************************************************
sumar_Fio2:
			outi 	C_X_FiO2,0
			ldi		ZH,0x00
			inr		ZL,C_A_FiO2

			inr		XH,Set_Ctrl_FiO2H
			inr		XL,Set_Ctrl_FiO2L
			
			inc		ZL
			inc		ZL
			;inc		ZL
			;add		XL,ZL
			;cpi		ZL,50
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	suma_max;skip_suma_FiO2;
			outr	C_A_FiO2,ZL
skip_suma_FiO2:
			inr		xh,reg_PWM1_O2H
			inr		xl,reg_PWM1_O2L
			ldi		ZH,0x00
			inr		ZL,C_A_FiO2
			;adiw	XL,1///checar
			add		XL,ZL
			adc		XH,ZH
			cpi		xl,low(480)
			cpci	xh,high(480)
			rbrsh	aumentar_fio2_aire////restar PWM del aire
			outr	reg_PWM1_O2H,xh
			outr	reg_PWM1_O2L,xl
			outr	tmp_reg_PWM1_O2H,xh
			outr	tmp_reg_PWM1_O2L,xl
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			rjmp	act_FiO2
aumentar_fio2_aire:		////restar PWM del aire
			call	fiO2_oxigeno_maximo
			inr		xh,reg_PWM2_AireH
			inr		xl,reg_PWM2_AireL
			ldi		ZH,0x00
			inr		ZL,C_A_FiO2
			;sbiw	XL,10///checar
			sub		XL,ZL
			sbc		XH,ZH
			cpi		xl,low(30)
			cpci	xh,high(30)
			rbrlt	fiO2_aire_min;act_FiO2
			
			outr	reg_PWM2_AireL,xl
			outr	reg_PWM2_AireH,xh
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
			jmp		act_FiO2
;************************************************************
;************************************************************
resta_max:	;outi	C_A_FiO2,0x32
			movr	C_A_FiO2,Set_Ctrl_FiO2L
			jmp		skip_resta_FiO2
;************************************************************
restar_FiO2:
			outi 	C_A_FiO2,0
			ldi		ZH,0x00
			inr		ZL,C_X_FiO2

			inr		XH,Set_Ctrl_FiO2H
			inr		XL,Set_Ctrl_FiO2L
			
			inc		ZL
			inc		ZL
			;inc		ZL
			;add		XL,ZL
			;cpi		ZL,50
			cp		ZL,XL
			cpc		ZH,XH
			rbrsh	resta_max;skip_resta_FiO2;
			outr	C_X_FiO2,ZL
skip_resta_FiO2: 
			inr		xh,reg_PWM2_AireH
			inr		xl,reg_PWM2_AireL
			;adiw	XL,1
			ldi		ZH,0x00
			inr		ZL,C_X_FiO2
			;adiw	XL,1///checar
			add		XL,ZL
			adc		XH,ZH
			cpi		xl,low(480)
			cpci	xh,high(480)
			rbrsh	disminuir_fio2_O2
			outr	reg_PWM2_AireL,xl
			outr	reg_PWM2_AireH,xh
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
			jmp		act_FiO2

disminuir_fio2_O2:
			call	fiO2_aire_maximo
			inr		xh,reg_PWM1_O2H
			inr		xl,reg_PWM1_O2L
			ldi		ZH,0x00
			inr		ZL,C_X_FiO2
			;sbiw	XL,1///checar
			sub		XL,ZL
			sbc		XH,ZH
			cpi		xl,low(30)
			cpci	xh,high(30)
			rbrlt	fiO2_oxigeno_min;act_FiO2
			outr	reg_PWM1_O2H,xh
			outr	reg_PWM1_O2L,xl
			outr	tmp_reg_PWM1_O2H,xh
			outr	tmp_reg_PWM1_O2L,xl
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			rjmp	act_FiO2
;************************************************************
;************************************************************
act_FiO2:
			movr	FIO2_ANTL,FiO2HEX_L	
			movr	FIO2_ANTH,FiO2HEX_H	
			jmp		salir_control_O2 /////////////cambiar 
;*******************final nuevo control*****************************
	
fiO2_aire_min:
		outi	reg_PWM2_AireH,0x00
		outi	reg_PWM2_AireL,0x00;28
		ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
		rjmp	act_FiO2;salir_control_O2
;************************************************************
;************************************************************
fiO2_aire_maximo:
		outi	reg_PWM2_AireH,0x01
		outi	reg_PWM2_AireL,0xE0
		ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
		ret
		;rjmp	salir_control_O2
;************************************************************
;************************************************************
fiO2_oxigeno_min:
		outi	reg_PWM1_O2H,0x00
		outi	reg_PWM1_O2L,0x00;28
		ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
		rjmp	act_FiO2;salir_control_O2
;************************************************************
;************************************************************
fiO2_oxigeno_maximo:
		outi	reg_PWM1_O2H,0x01
		outi	reg_PWM1_O2L,0xE0
		ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
		ret
		rjmp	salir_control_O2

;************************************************************
;************************************************************
fiO2_21:
		outi	PWM_AIREH,0x01
		outi	PWM_AIREL,0xE0
		outi	PWM_O2H,0x00
		outi	PWM_O2L,0x00
		outi		habilita_control_O2,'1'
		rjmp		retorno_control_O2;salir_control_O2
;************************************************************
;************************************************************
fiO2_100:
		outi	PWM_AIREH,0x00
		outi	PWM_AIREL,0x00
		outi	PWM_O2H,0x01
		outi	PWM_O2L,0xE0
		outi		habilita_control_O2,'1'
		rjmp		retorno_control_O2;salir_control_O2
;************************************************************
;************************************************************
falla_gases:
		outi	PWM_AIREH,0x01
		outi	PWM_AIREL,0xE0
		outi	PWM_O2H,0x00
		outi	PWM_O2L,0x00
		outi		habilita_control_O2,'1'
		rjmp		retorno_control_O2;salir_control_O2
;************************************************************
;************************************************************
Fio2_Ok:
			outi		B_FIO2_BAJO,'0'				
			outi		B_FIO2_ALTO,'0'
			outi		C_A_FiO2,0
			outi		C_X_FiO2,0
			rjmp	salir_control_O2
;************************************************************
;************************************************************


;************************************************************
;************************************************************
salir_control_O2:
			cli
			;sbi		led_run
//606001 se puede eliminar una variable
			outi	TMR_FiO2_O2H,0x00
			outi	TMR_FiO2_O2L,0xF0
			movr	timer_O2H,TMR_FiO2_O2H
			movr	timer_O2L,TMR_FiO2_O2L
			outi	habilita_control_O2,'0'
			sei
//----------------ANTERIOR A CAMBIO_606001
/*
salir_control_O2:
			cli
			
			;outi	timer_O2H,high(100);(1500)
			;outi	timer_O2L,low(100);(1500)
			inr		XH,TMR_FiO2_O2H
			inr		XL,TMR_FiO2_O2L
			inr		ZH,TMR_FiO2_O2H
			inr		ZL,TMR_FiO2_O2L
			add		XL,ZL
			adc		XH,ZH
			outr	timer_O2H,XH
			outr	timer_O2L,XL
			;movr	timer_O2H,TMR_FiO2_O2H
			;movr	timer_O2L,TMR_FiO2_O2L
			outi	habilita_control_O2,'0'
			sei
*/
retorno_control_O2:

			ret
;************************************************************
FiO2_manual:

			movr	reg_PWM2_AireH,PWM_Aire_TST_H;PWM_AIREH
			movr	reg_PWM2_AireL,PWM_Aire_TST_L;PWM_AIREL
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
			
			movr	reg_PWM1_O2H,PWM_O2_TST_H;PWM_Aire_TST_H;
			movr	reg_PWM1_O2L,PWM_O2_TST_L;PWM_Aire_TST_L;
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			

			cli
			outi	timer_O2H,high(140);(1500)
			outi	timer_O2L,low(140);(1500)
			outi	habilita_control_O2,'0'
			sei

			ret
;************************************************************
;************************************************************
control_Parker_in:
			;ret
			cpri	B_Ctrl_Parker,'0'
			rbreq	sin_gases
;			inr		XL,portl
;			sbrs	XL,6	
;			jmp		sin_gases
;************************************************************
;****************modo manual*********************************			
			inr		YL,Modo_TST
			cpi		YL,0x02
			rbreq	FiO2_manual
;************************************************************
;************************************************************
			inr		ZL,reg_O2L		;compara si es 21% de oxigeno (hex)
			inr		ZH,reg_O2H
			cpi		ZL,0x15
			cpci	ZH,0x00
			rbreq	fiO2_21
;************************************************************
;************************************************************
			inr		ZL,reg_O2L		;compara si es 100% de oxigeno (hex)
			inr		ZH,reg_O2H
			cpi		ZL,0x64
			cpci	ZH,0x00
			rbreq	fiO2_100
;************************************************************
;************************************************************
			cpri	rx_Ctrl_FiO2,'1'
			rbreq	Falla_gases
;************************************************************
;************************************************************
			ASIGNA_PWM_AIRE						reg_PWM2_AireH,reg_PWM2_AireL
			ASIGNA_PWM_O2						reg_PWM1_O2H,reg_PWM1_O2L
			ret
;************************************************************
;************************************************************
sin_gases:
			outi	PWM_AIREH,0x00
			outi	PWM_AIREL,0x00
			outi	PWM_O2H,0x00
			outi	PWM_O2L,0x00
			ret

;************************************************************
;************************************************************
			.eseg
			.org	0x0100
dt_O2_21:	.dw		0x00D3
			 ;.eseg
			;.org	0x0110
dt_O2_100:	.dw		0x03FF
			;.eseg
			;.org	0x0120
dt_O2_G:	.dw		0x000A
			;.eseg
			;.org	0x0130
dt_O2_G_FP:	.db		0x9A, 0x99, 0xF9, 0x40		





