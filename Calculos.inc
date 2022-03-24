;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;************************************************************
k1p05:		.db	0x66, 0x66, 0x86, 0x3F
k0p95:		.db	0x33, 0x33, 0x73, 0x3F

calcula_presionesFP:
					call	CLR_HEXADECIMAL
					inr		r16,reg_PinspiracionL
					inr		r17,reg_PinspiracionH
					outr	HEX_L,r16
					outr	HEX_ML,r17
					HTOD
					inr		r16,DEC_MH
					outr	DEC_H,r16
					inr		r16,DEC_M
					outr	DEC_MH,r16
					inr		r16,DEC_ML
					outr	DEC_M,r16
					inr		r16,DEC_L
					outr	DEC_ML,r16
							outi	DEC_L,0x00
					DTOH
					inr		r16,HEX_L
					inr		r17,HEX_ML
					outr	reg_Pinspiracionx100L,r16
					outr	reg_Pinspiracionx100H,r17


					call	CLR_HEXADECIMAL
					inr		r16,reg_PSL
					inr		r17,reg_PSH
					outr	HEX_L,r16
					outr	HEX_ML,r17
					HTOD
					inr		r16,DEC_MH
					outr	DEC_H,r16
					inr		r16,DEC_M
					outr	DEC_MH,r16
					inr		r16,DEC_ML
					outr	DEC_M,r16
					inr		r16,DEC_L
					outr	DEC_ML,r16
							outi	DEC_L,0x00
					DTOH
					inr		r16,HEX_L
					inr		r17,HEX_ML
					outr	reg_PSx100L,r16
					outr	reg_PSx100H,r17




			call	CLR_HEXADECIMAL
			inr		r16,reg_PinspiracionL
			inr		r17,reg_PinspiracionH
			outr	HEX_L,r16
			outr	HEX_ML,r17
			NORMHEXTOA1
			outr	reg_PinspiracionLFP,A1_L
			outr	reg_PinspiracionMLFP,A1_ML
			outr	reg_PinspiracionMHFP,A1_MH
			outr	reg_PinspiracionHFP,A1_H

			FMULT10
			FMULT10
			outr	reg_PinspiracionLFPx100,A1_L
			outr	reg_PinspiracionMLFPx100,A1_ML
			outr	reg_PinspiracionMHFPx100,A1_MH
			outr	reg_PinspiracionHFPx100,A1_H

			call	CLR_HEXADECIMAL
			inr		r16,reg_PEEPL
			inr		r17,reg_PEEPH
			outr	HEX_L,r16
			outr	HEX_ML,r17
			NORMHEXTOA1
			outr	reg_PEEPLFP,A1_L
			outr	reg_PEEPMLFP,A1_ML
			outr	reg_PEEPMHFP,A1_MH
			outr	reg_PEEPHFP,A1_H

			call	CLR_HEXADECIMAL
			inr		r16,reg_PSL
			inr		r17,reg_PSH
			outr	HEX_L,r16
			outr	HEX_ML,r17
			NORMHEXTOA1
			outr	reg_PSLFP,A1_L
			outr	reg_PSMLFP,A1_ML
			outr	reg_PSMHFP,A1_MH
			outr	reg_PSHFP,A1_H
			FMULT10
			FMULT10
			outr	reg_PSLFPx100,A1_L
			outr	reg_PSMLFPx100,A1_ML
			outr	reg_PSMHFPx100,A1_MH
			outr	reg_PSHFPx100,A1_H

			call	CLR_HEXADECIMAL
			inr		r16,reg_PmaxL
			inr		r17,reg_PmaxH
			outr	HEX_L,r16
			outr	HEX_ML,r17
			NORMHEXTOA1
			outr	reg_PmaxLFP,A1_L
			outr	reg_PmaxMLFP,A1_ML
			outr	reg_PmaxMHFP,A1_MH
			outr	reg_PmaxHFP,A1_H
			FMULT10
			FMULT10
			outr	reg_PmaxLFPx100,A1_L
			outr	reg_PmaxMLFPx100,A1_ML
			outr	reg_PmaxMHFPx100,A1_MH
			outr	reg_PmaxHFPx100,A1_H
			ret
;************************************************************

;************************************************************
convierte_cmH2OHEX:
			call	CLR_HEXADECIMAL
			inr		r25,Presion_cmH2OH
			outr	HEX_ML,r25
			inr		r25,Presion_cmH2OL
			outr	HEX_L,r25
			HTOD

			inr		r17,DEC_M
			andi	r17,0x0F
			subi	r17,-0x30
			outr	tx_PSI_4,r17

			inr		r17,DEC_ML
			andi	r17,0xF0
			swap	r17
			subi	r17,-0x30
			outr	tx_PSI_3,r17

			inr		r17,DEC_ML
			andi	r17,0x0F
			subi	r17,-0x30
			outr	tx_PSI_2,r17

			inr		r17,DEC_L
			andi	r17,0xF0
			swap	r17
			subi	r17,-0x30
			outr	tx_PSI_1,r17

			inr		r17,DEC_L
			andi	r17,0x0F
			subi	r17,-0x30
			outr	tx_PSI_0,r17
			ret
;************************************************************

;************************************************************
calcula_Pinspiracion:
CLT
			call	CLR_DECIMAL
			inr		r16,PinspiracionH
			subi	r16,0x30
			outr	DEC_ML,r16
			inr		r17,PinspiracionM
			subi	r17,0x30
			swap	r17
			inr		r16,PinspiracionL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			cli
			inr		r16,HEX_L
			inr		r17,tmp_reg_PEEPL
			add		r16,r17
			outr	tmp_reg_PinspiracionL,r16
			inr		r16,HEX_ML
			inr		r17,tmp_reg_PEEPH
			adc		r16,r17
			outr	tmp_reg_PinspiracionH,r16

;Calcula el 50% de la diferencia entre el PEEP y PIP, multiplicado por 100
			CLT
			call	CLR_DECIMAL
			inr		r16,PinspiracionH
			subi	r16,0x30
			outr	DEC_ML,r16
			inr		r17,PinspiracionM
			subi	r17,0x30
			swap	r17
			inr		r16,PinspiracionL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			inr		r24,HEX_L		;Se divide entre 2 Pinspiracion
			inr		r25,HEX_ML
			lsr		r25
			ror		r24

			lsr		r25
			ror		r24

			inr		r16,reg_PEEPL
			add		r16,r24
			inr		r17,reg_PEEPH
			adc		r17,r25

			call	CLR_HEXADECIMAL
			outr	HEX_ML,r17
			outr	HEX_L,r16
			HTOD

			inr		r16,DEC_MH
			outr	DEC_H,r16
			inr		r16,DEC_M
			outr	DEC_MH,r16
			inr		r16,DEC_ML
			outr	DEC_M,r16
			inr		r16,DEC_L
			outr	DEC_ML,r16
			outi	DEC_L,0x00
			DTOH

			inr		r16,HEX_L
			inr		r17,HEX_ML
			cli
			outr	reg_Dif50p_presionL,r16
			outr	reg_Dif50p_presionH,r17
			sei

			ret
;************************************************************

;************************************************************
calcula_PEEP:
			CLT
			call	CLR_DECIMAL
			inr		r16,PEEPH
			subi	r16,0x30
			outr	DEC_ML,r16
			inr		r17,PEEPM
			subi	r17,0x30
			swap	r17
			inr		r16,PEEPL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			inr		r17,HEX_ML
			inr		r16,HEX_L
			outr	tmp_reg_PEEPH,r17	;Se guarda el valor del PEEP en Hex
			outr	tmp_reg_PEEPL,r16

			call	CLR_DECIMAL
			inr		r17,PEEPH
			subi	r17,0x30
			swap	r17
			inr		r16,PEEPM
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			inr		r16,PEEPL
			subi	r16,0x30
			swap	r16
			outr	DEC_L,r16
			DTOH
			

			call	CLR_DECIMAL
			inr		r16,PEEPH
			subi	r16,0x30
			outr	DEC_M,r16
			inr		r17,PEEPM
			subi	r17,0x30
			swap	r17
			inr		r16,PEEPL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			DTOH
			inr		r17,HEX_ML
			inr		r16,HEX_L
			outr	tmp_reg_PEEPH_100porciento,r17
			outr	tmp_reg_PEEPL_100porciento,r16

			
			ret
;************************************************************

;************************************************************
calcula_PS:
			CLT
			call	CLR_DECIMAL
			inr		r16,PSH
			subi	r16,0x30
			outr	DEC_ML,r16
			inr		r17,PSM
			subi	r17,0x30
			swap	r17
			inr		r16,PSL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			cli
;			inr		r17,HEX_ML
;			outr	reg_PSH,r17
;			inr		r16,HEX_L
;			outr	reg_PSL,r16
			inr		r16,HEX_L
			inr		r17,tmp_reg_PEEPL
			add		r16,r17
			outr	tmp_reg_PSL,r16
			inr		r16,HEX_ML
			inr		r17,tmp_reg_PEEPH
			adc		r16,r17
			outr	tmp_reg_PSH,r16
			sei
			ret
;************************************************************

;************************************************************
calcula_Pmax:
			CLT
			call	CLR_DECIMAL
			inr		r16,PmaxH
			subi	r16,0x30
			outr	DEC_M,r16
			inr		r17,PmaxM
			subi	r17,0x30
			swap	r17
			inr		r16,PmaxL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			DTOH
			inr		r16,HEX_ML
			outr	tmp_reg_PmaxH,r16
			inr		r16,HEX_L
			outr	tmp_reg_PmaxL,r16
			ret
;************************************************************

;************************************************************
;Sensor MPX5700
;
;			ADC
;			---- - 0.04
;			1023
;		P=(-------------) = KPa
;			0.0012858
;
;			ADC
;			---- - 0.04
;			1023
;		P=(-------------) (0.145038) = PSI
;			0.0012858
;
calcula_PRESION_AIRE:
			cli
			//outi	presion_hex_aireH,0x02
			//outi	presion_hex_aireL,0x80
			inr		r17,presion_hex_aireH
			inr		r16,presion_hex_aireL
			sei

			ldiw	zh,zl,(TABLA_CONVERSION_PRESION*2)
			lsl		r16
			rol		r17
			add		zl,r16
			adc		zh,r17
			lpm		r17,z+
			lpm		r16,z

			andi	r17,0x0F
			subi	r17,-0x30
			outr	tx_presion_aireH,r17

			mov		r17,r16
			andi	r17,0xF0
			swap	r17
			subi	r17,-0x30
			outr	tx_presion_aireM,r17

			andi	r16,0x0F
			subi	r16,-0x30
			outr	tx_presion_aireL,r16
			ret
;************************************************************

;************************************************************
calcula_PRESION_O2:
			cli
			//outi	presion_hex_O2H,0x02
			//outi	presion_hex_O2L,0x80
			inr		r17,presion_hex_O2H
			inr		r16,presion_hex_O2L
			sei

			ldiw	zh,zl,(TABLA_CONVERSION_PRESION*2)
			lsl		r16
			rol		r17
			add		zl,r16
			adc		zh,r17
			lpm		r17,z+
			lpm		r16,z

			andi	r17,0x0F
			subi	r17,-0x30
			outr	tx_presion_O2H,r17

			mov		r17,r16
			andi	r17,0xF0
			swap	r17
			subi	r17,-0x30
			outr	tx_presion_O2M,r17

			andi	r16,0x0F
			subi	r16,-0x30
			outr	tx_presion_O2L,r16
			ret
;************************************************************

;************************************************************
calcula_O2_rx:
					CLT
			call	CLR_DECIMAL

			inr		r16,O2H
			subi	r16,0x30
			outr	DEC_ML,r16

			inr		r17,O2M
			subi	r17,0x30
			swap	r17
			inr		r16,O2L
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16

			DTOH
			inr		r16,HEX_ML
			outr	tmp_reg_O2H,r16
			inr		r16,HEX_L
			outr	tmp_reg_O2L,r16
			ret
;************************************************************

;************************************************************
calcula_TIEMPO_inspiracion:
					CLT
			call	CLR_DECIMAL

			inr		r17,TIEMPO_INSH
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPO_INSMH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			inr		r17,TIEMPO_INSML
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPO_INSL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16

			DTOH
			inr		r17,HEX_ML
			inr		r16,HEX_L
			outr	tmp_tmr_TIEMPO_INSH,r17
			outr	tmp_tmr_TIEMPO_INSL,r16

			lsr		r17
			ror		r16
			outr	TIEMPO_INSH_50p,r17
			outr	TIEMPO_INSL_50p,r16
			ret
;************************************************************

;************************************************************
calcula_TIEMPO_pausa:
					CLT
			call	CLR_DECIMAL

			inr		r17,TIEMPOpausaH
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPOpausaMH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16

			inr		r17,TIEMPOpausaML
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPOpausaL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH

			inr		r16,HEX_ML
			outr	tmp_tmr_TIEMPOpausaH,r16
			inr		r16,HEX_L
			outr	tmp_tmr_TIEMPOpausaL,r16
			ret
;************************************************************

;************************************************************
calcula_TIEMPO_exhalacion:
					CLT
			call	CLR_DECIMAL

			inr		r17,TIEMPO_EXPH
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPO_EXPMH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16

			inr		r17,TIEMPO_EXPML
			subi	r17,0x30
			swap	r17
			inr		r16,TIEMPO_EXPL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH

			inr		r16,HEX_ML
			outr	tmp_tmr_TIEMPO_EXPH,r16
			inr		r16,HEX_L
			outr	tmp_tmr_TIEMPO_EXPL,r16
			ret
;************************************************************

;************************************************************
calcula_tiempo_total:
					CLT
			inr		r16,tmp_tmr_TIEMPO_INSL
			inr		r17,tmp_tmr_TIEMPO_INSH
			inr		r18,tmp_tmr_TIEMPOpausaL
			inr		r19,tmp_tmr_TIEMPOpausaH
			inr		r20,tmp_tmr_TIEMPO_EXPL
			inr		r21,tmp_tmr_TIEMPO_EXPH
			add		r16,r18
			adc		r17,r19
			add		r16,r20
			adc		r17,r21
			outr	tmp_tmr_ventanaSIMV_L,r16
			outr	tmp_tmr_ventanaSIMV_H,r17
			ret
;************************************************************

;************************************************************
calcula_FR:			CLT
			call	CLR_DECIMAL
			inr		r16,FRH
			subi	r16,0x30
			outr	DEC_ML,r16
			inr		r17,FRM
			subi	r17,0x30
			swap	r17
			inr		r16,FRL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH

			inr		r16,HEX_ML
			outr	tmp_reg_FRH,r16
			inr		r16,HEX_L
			outr	tmp_reg_FRL,r16
			ret
;************************************************************

;************************************************************
calcula_PWM0:
					CLT
			call	CLR_DECIMAL
			inr		r17,PWM0H
			subi	r17,0x30
			swap	r17
			inr		r16,PWM0MH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			inr		r17,PWM0ML
			subi	r17,0x30
			swap	r17
			inr		r16,PWM0L
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			inr		r17,HEX_ML
			outr	tmp_reg_PWM0H,r17
			inr		r16,HEX_L
			outr	tmp_reg_PWM0L,r16

;Calcula el 50% del valor del PWM que se recibio, considerando el offset de
;PWMmin.
			inr		r16,tmp_reg_PWM0L
			inr		r17,tmp_reg_PWM0H
			ldiw	xh,xl,PWMmin
			sub		r16,xl	;Al valor de PWM, se le resta el valor minimo.
			sbc		r17,xh	;PWMmin de acuerdo con la caracterización de las
							;electrovalvulas.
			brpl	divide_2a
			ldi		r16,0x00
			ldi		r17,0x00
			jmp		carga_xa

divide_2a:	lsr		r17		;Se divide entre dos el resultado
			ror		r16

carga_xa:	inr		xl,tmp_reg_PWM0L	;Se toma nuevamente el valor original
			inr		xh,tmp_reg_PWM0H	;del PWM.
			sub		xl,r16
			sbc		xh,r17
			outr	reg_PWM0H_50p,xh
			outr	reg_PWM0L_50p,xl

;Calcula el 25% del valor del PWM que se recibio, considerando el offset de
;PWMmin.
			inr		r16,reg_PWM0L_50p
			inr		r17,reg_PWM0H_50p
			ldiw	xh,xl,PWMmin
			sub		r16,xl	;Al valor de 50% de PWM, se le resta el valor minimo.
			sbc		r17,xh	;PWMmin de acuerdo con la caracterización de las
							;electrovalvulas.
			brpl	divide_2b
			ldi		r16,0x00
			ldi		r17,0x00
			jmp		carga_xb

divide_2b:	lsr		r17		;Se divide entre dos el resultado
			ror		r16

carga_xb:	inr		xl,reg_PWM0L_50p	;Se toma el valor de PWM al 50%
			inr		xh,reg_PWM0H_50p	;del PWM.
			sub		xl,r16
			sbc		xh,r17
			outr	reg_PWM0H_25p,xh
			outr	reg_PWM0L_25p,xl

;Calcula el 12.5% del valor del PWM que se recibio, considerando el offset de
;PWMmin.
			inr		r16,reg_PWM0L_25p
			inr		r17,reg_PWM0H_25p
			ldiw	xh,xl,PWMmin
			sub		r16,xl	;Al valor de 25% de PWM, se le resta el valor minimo.
			sbc		r17,xh	;PWMmin de acuerdo con la caracterización de las
							;electrovalvulas.
			brpl	divide_2c
			ldi		r16,0x00
			ldi		r17,0x00
			jmp		carga_xc

divide_2c:	lsr		r17		;Se divide entre cuatro el resultado
			ror		r16

carga_xc:	inr		xl,reg_PWM0L_25p	;Se toma el valor de PWM al 25%
			inr		xh,reg_PWM0H_25p	;del PWM.
			sub		xl,r16
			sbc		xh,r17
			outr	reg_PWM0H_12p5p,xh
			outr	reg_PWM0L_12p5p,xl

;Calcula el 6.25% del valor del PWM que se recibio, considerando el offset de
;PWMmin.
			inr		r16,reg_PWM0L_12p5p
			inr		r17,reg_PWM0H_12p5p
			ldiw	xh,xl,PWMmin
			sub		r16,xl	;Al valor de PWM, se le resta el valor minimo.
			sbc		r17,xh	;PWMmin de acuerdo con la caracterización de las
							;electrovalvulas.
			brpl	divide_2d
			ldi		r16,0x00
			ldi		r17,0x00
			jmp		carga_xd

divide_2d:	lsr		r17		;Se divide entre cuatro el resultado
			ror		r16

carga_xd:	inr		xl,reg_PWM0L_12p5p	;Se toma el valor de PWM al 12.5%
			inr		xh,reg_PWM0H_12p5p	;del PWM.
			sub		xl,r16
			sbc		xh,r17
			outr	reg_PWM0H_6p25p,xh
			outr	reg_PWM0L_6p25p,xl
			ret
;************************************************************

;************************************************************
calcula_PWM1_O2:
					CLT
			call	CLR_DECIMAL
			inr		r17,PWM1_O2H
			subi	r17,0x30
			swap	r17
			inr		r16,PWM1_O2MH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			inr		r17,PWM1_O2ML
			subi	r17,0x30
			swap	r17
			inr		r16,PWM1_O2L
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			inr		r16,HEX_ML
			outr	tmp_reg_PWM1_O2H,r16
			inr		r16,HEX_L
			outr	tmp_reg_PWM1_O2L,r16
			ret
;************************************************************

;************************************************************
calcula_PWM2_Aire:
					CLT
			call	CLR_DECIMAL
			inr		r17,PWM2_AireH
			subi	r17,0x30
			swap	r17
			inr		r16,PWM2_AireMH
			subi	r16,0x30
			or		r16,r17
			outr	DEC_ML,r16
			inr		r17,PWM2_AireML
			subi	r17,0x30
			swap	r17
			inr		r16,PWM2_AireL
			subi	r16,0x30
			or		r16,r17
			outr	DEC_L,r16
			DTOH
			inr		r16,HEX_ML
			outr	tmp_reg_PWM2_AireH,r16
			inr		r16,HEX_L
			outr	tmp_reg_PWM2_AireL,r16
			ret
;************************************************************

;************************************************************
;Sección para FiO2
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
calcula_FiO2_BCD:
			inr		xl,FiO2HEX_L
			inr		xh,FiO2HEX_H
			ldiw	zh,zl,(TABLA_CONVERSION_O2_BCD*2)
			lsl		xl
			rol		xh
			add		zl,xl
			adc		zh,xh
			lpm		xh,z+
			lpm		xl,z

			andi	xh,0x0F
			subi	xh,-0x30
			outr	tx_FiO2H,xh
			mov		xh,xl
			andi	xh,0xF0
			swap	xh
			subi	xh,-0x30
			outr	tx_FiO2M,xh
			andi	xl,0x0F
			subi	xl,-0x30
			outr	tx_FiO2L,xl
			ret

N_FiO2_HX:	cli
			inr		XH,FiO2_nuevoH;lde2	XH,dt_O2_100+1
			inr		XL,FiO2_nuevoL;lde2	XL,dt_O2_100
			outi	FiO2_valido,'0'	;borra la bandera para indicar que ya se tomo
									;el valor de la nueva lectura de FiO2
			sei
			lde2	ZH,dt_O2_21+1
			lde2	ZL,dt_O2_21
			;sei
			cp		XL,ZL
			cpc		XH,ZH
			rbreq	FIO2_21_BCD
			rbrlo	FIO2_ER
			sub		xl,zl
			sbc		xh,zh
			outr	FIO2_NH,xh
			outr	FIO2_NL,xl
			call	cont_fio2
			ldi		xh,0x00
			inr		xl,C_FIO2
			;//////////////////////////
			subi	xl,low(-21)
			;ldiw	xh,xl,0x0015
			ret
FIO2_ER:	
			sub		zl,xl
			sbc		zh,xh
			outr	FIO2_NH,zh
			outr	FIO2_NL,zl
			call	cont_fio2
			ldiw	xh,xl,0x0015
			ldi		zh,0x00
			inr		zl,C_FIO2
			cpi		zl,0x10
			brsh	OL_FIO2_ER
			sub		xl,zl
			ret
OL_FIO2_ER:
/************************pruebas simulacion*********************************/
;			ldiw	xh,xl,0x0032
;			ret 
/*********************************************************/
			ldiw	xh,xl,0x0005
			ret 	
FIO2_21_BCD:	
			ldiw	xh,xl,0x0015
			ret

cont_fio2:
			outi	C_FIO2,0X00	
			inr		XH,FIO2_NH;lde2	XH,dt_O2_100+1
			inr		XL,FIO2_NL;lde2	XL,dt_O2_100
			ldi		ZH,0x00;lde2	XH,dt_O2_100+1
			lde2	ZL,dt_O2_G
			cp		xl,zl
			cpc		xh,zh
			brsh	s_cont_fio2
			outi	C_FIO2,0X00
			ret

s_cont_fio2:inr		r16,C_FIO2
			cpi		r16,0x4F
			brsh	OL_CONT
			inc 	r16
			sub		xl,zl
			sbc		xh,zh
			outr	C_FIO2,r16
			cp		zl,xl
			cpc		zh,xh
			brlo	s_cont_fio2
			ret
OL_CONT:	
			outi	C_FIO2,0x50
			ret
;************************************************************

;************************************************************
calcula_FiO2_HEX:

			call	N_FiO2_HX
			//outi	FiO2HEX_H,0x00
			//outi	FiO2HEX_L,0x15
			outr	FiO2HEX_H,XH
			outr	FiO2HEX_L,XL
			ret
;************************************************************

;************************************************************

;************************************************************

;************************************************************
K100:	.db	0x00,0X00,0X9E,0X42

calculo_G_FIO2:

			lde2	XH,dt_O2_100+1
			lde2	XL,dt_O2_100
			lde2	ZH,dt_O2_21+1
			lde2	ZL,dt_O2_21
			sub		XL,ZL
			sbc		XH,ZH
			call	CLR_HEXADECIMAL
			outr	HEX_ML,XH
			outr	HEX_L,XL
			CLT
			NORMHEXTOA1

			ldiw	zh,zl,k100*2
			FLDKA2

			FDIV

			DENORMA1TOHEX
			
			inr		r16,HEX_L
			inr		r17,FHEX_H
			andi	r17,0xF0
			cpi		r17,0x80
			brlo	guarda_G
sumar_G:	inc		r16
guarda_G:	ldi		r17,0x00
			stE2	dt_O2_G,r16
			stE2	dt_O2_G+1,r17 
			ret
;************************************************************
	

