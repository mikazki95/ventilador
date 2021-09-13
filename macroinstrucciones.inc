;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;Macros de esta libreria:
;		stsi
;		jnb
;		jb
;		inr
;		outr
;		outi
;		outiw
;
;		cpri	dirección,constante
;		decr	dirección
;		incr	dirección
;		delay_us	valor_us
;		delay_ms	valor_ms
;		usart0	baudaje,bits,paridad,paro,sel_rxtx,sel_int_rxtx
;		usart1	baudaje,bits,paridad,paro,sel_rxtx,sel_int_rxtx
;		usart2	baudaje,bits,paridad,paro,sel_rxtx,sel_int_rxtx
;		usart3	baudaje,bits,paridad,paro,sel_rxtx,sel_int_rxtx
;				n		=	Sin paridad
;				e		=	Paridad par
;				o		=	Paridad impar
;				rx		=	Rx habilitada
;				tx		=	Tx habilitada
;				rxtx	=	RX y TX habilitadas
;				no_irxtx=	Int de Rx y Tx deshabilitadas
;				irx		=	Int de Rx habilitada
;				itx		=	Int de Tx habilitada
;				irxtx	=	Int de Rx y Tx habilitadas
.def		wl		=	r24
.def		wh		=	r25

.macro		stsi
			ldi		r25,@1
			sts		@0,r25
			.endm

.macro		jnb
			.if		@0>0x40
			lds		r25,@0
			sbrs	r25,@1
			rjmp	@2
			.elif	@0<0x40
			sbis	@0,@1
			rjmp	@2
			.endif
			.endm

.macro		jb
			.if		@0>0x40
			lds		r25,@0
			sbrc	r25,@1
			rjmp	@2
			.elif	@0<0x40
			sbic	@0,@1
			rjmp	@2
			.endif
			.endm

.macro		inr
			.if		@1>0x40
			lds		@0,@1
			.elif	@1<0x40
			in		@0,@1
			.endif
			.endm

.macro		outr
			.if		@0>0x40
			sts		@0,@1
			.elif	@0<0x40
			out		@0,@1
			.endif
			.endm

.macro		outi
			ldi		r25,@1
			.if		@0>0x40
			sts		@0,r25
			.elif	@0<0x40
			out		@0,r25
			.endif
			.endm

.macro		conv_A_H
			inr		r25,@2
			inr		r26,@3
			inr		r27,@4
			subi	r25,0x30
			//swap	r25
			subi	r26,0x30
			swap	r26
			subi	r27,0x30
			or		r26,r27
			outr	DEC_L,R26
			outr	DEC_ML,R25
			outi	DEC_M,0X00
			outi	DEC_MH,0X00
			outi	DEC_H,0X00
			DTOH
			inr		r25,HEX_ML
			inr		r26,HEX_L
			outr	@0,r25
			outr	@1,r26
				
			.endm

.macro		outiw
			ldi		r24,low(@2)
			ldi		r25,high(@2)
			.if		@0>0x40
			sts		@0,r25
			sts		@1,r24
			.elif	@0<0x40
			out		@0,r25
			out		@1,r24
			.endif
			.endm

.macro		incw
			adiw	wl,@0
			.endm

.macro		incx
			adiw	xl,@0
			.endm

.macro		incy
			adiw	yl,@0
			.endm

.macro		incz
			adiw	zl,@0
			.endm

.macro		decw
			sbiw	wl,@0
			.endm

.macro		decx
			sbiw	xl,@0
			.endm

.macro		decy
			sbiw	yl,@0
			.endm

.macro		decz
			sbiw	zl,@0
			.endm

.macro		ldiw
			ldi		@1,low(@2)
			ldi		@0,high(@2)
			.endm

.macro		pushw
			push	r24
			push	r25
			.endm

.macro		popw
			pop		r25
			pop		r24
			.endm

.macro		pushx
			push	r26
			push	r27
			.endm

.macro		popx
			pop		r27
			pop		r26
			.endm

.macro		pushy
			push	r28
			push	r29
			.endm

.macro		popy
			pop		r29
			pop		r28
			.endm

.macro		pushz
			push	r30
			push	r31
			.endm

.macro		popz
			pop		r31
			pop		r30
			.endm

.macro		ste2
EEPROM_write:
			sbic	EECR,EEPE
			rjmp	EEPROM_write
			push	@1
			ldi		@1,high(@0)
			out		EEARH,@1
			ldi		@1,low(@0)
			out		EEARL,@1
			pop		@1
			out		EEDR,@1
			cli
			sbi		EECR,EEMPE
			sbi		EECR,EEPE
			sei
			.endm

.macro		lde2
EEPROM_read:
			sbic	EECR,EEPE
			rjmp	EEPROM_read
			ldi		@0,high(@1)
			out		EEARH,@0
			ldi		@0,low(@1)
			out		EEARL,@0
			sbi		EECR,EERE
			in		@0,EEDR
			.endm

.macro		cijne
			cpi		@0,@1
			breq	skip
			rjmp	@2
skip:
			.endm

.macro		cijeq
			cpi		@0,@1
			brne	skip
			rjmp	@2
skip:
			.endm

.macro		cpci
			ldi		r25,@1
			cpc		@0,r25
			.endm

.macro		rbreq
			brne	salto
			jmp	@0
salto:
			.endm

.macro		rbrne
			breq	salto
			jmp	@0
salto:
			.endm

.macro		rbrlo
			brsh	salto
			jmp	@0
salto:
			.endm

.macro		rbrsh
			brlo	salto
			jmp	@0
salto:
			.endm

.macro		rbrlt
			brge	salto
			jmp	@0
salto:
			.endm


.macro		rbrts
			brtc	salto
			jmp	@0
salto:
			.endm

.macro		rbrtc
			brts	salto
			jmp	@0
salto:
			.endm


.macro		rbrge
			brlt	salto
			jmp	@0
salto:
			.endm



;*********************************************************************************

;*********************************************************************************
.macro		cpri
			.if		@0>0x40
			lds		r25,@0
			cpi		r25,@1
			.elif	@0<0x40
			in		r25,@0
			cpi		r25,@1
			.endif
			.endm

.macro		decr
			.if		@0>0x40
			lds		r25,@0
			dec		r25
			sts		@0,r25
			.elif	@0<0x40
			in		r25,@0
			dec		r25
			out		@0,r25
			.endif
			.endm

.macro		incr
			.if		@0>0x40
			lds		r25,@0
			inc		r25
			sts		@0,r25
			.elif	@0<0x40
			in		r25,@0
			inc		r25
			out		@0,r25
			.endif
			.endm

.macro		movr
			.if		@1>0x40
			lds		r25,@1
			.elif	@1<0x40
			in		r25,@1
			.endif
			.if		@0>0x40
			sts		@0,r25
			.elif	@0<0x40
			out		@0,r25
			.endif
			.endm

.macro		pushr
			.if		@0>0x40
			lds		r25,@0
			push	r25
			.elif	@0<0x40
			in		r25,@0
			push	r25
			.endif
			.endm

.macro		popr
			.if		@0>0x40
			pop		r25
			sts		@0,r25
			.elif	@0<0x40
			pop		r25
			out		@0,r25
			.endif
			.endm

;Parametros del macro "FLASH_SRAM"
;@0		Dirección destino (SRAM)
;@1		Dirección fuente (flash)
;@2		Número de bytes
.macro		FLASH_SRAM
			push	xh
			push	xl
			push	yh
			push	yl
			push	zh
			push	zl
			push	r25
			ldiw	yh,yl,@0
			ldiw	zh,zl,@1*2
			ldiw	xh,xl,@2
next_copy:	outi	rampz,0x00
			lpm		r25,z+
			st		y+,r25
			sbiw	xl,1
			brne	next_copy
			pop		r25
			pop		zl
			pop		zh
			pop		yl
			pop		yh
			pop		xl
			pop		xh
.endm

;Parametros del macro "EEPROM_SRAM"
;@0		Dirección destino (SRAM)
;@1		Dirección fuente (EEPROM)
;@2		Número de bytes
.macro		EEPROM_SRAM
			push	xh
			push	xl
			push	yh
			push	yl
			push	zh
			push	zl
			push	r25
			ldiw	yh,yl,@0
			ldiw	zh,zl,@1
			ldiw	xh,xl,@2
wait_EEPROM:sbic	EECR,EEPE
			rjmp	wait_EEPROM
			out		EEARH,zh
			out		EEARL,zl
			adiw	zl,1
			sbi		EECR,EERE
			in		r25,EEDR
			st		y+,r25
			sbiw	xl,1
			brne	wait_EEPROM
			pop		r25
			pop		zl
			pop		zh
			pop		yl
			pop		yh
			pop		xl
			pop		xh
.endm

;Parametros del macro "COPY_SRAM"
;@0		Dirección destino (SRAM)
;@1		Dirección fuente (SRAM)
;@2		Número de bytes
.macro		COPY_SRAM
			push	xh
			push	xl
			push	yh
			push	yl
			push	zh
			push	zl
			push	r25
			ldiw	yh,yl,@0
			ldiw	zh,zl,@1
			ldiw	xh,xl,@2
next_copy:	ld		r25,z+
			st		y+,r25
			sbiw	xl,1
			brne	next_copy
			pop		r25
			pop		zl
			pop		zh
			pop		yl
			pop		yh
			pop		xl
			pop		xh
.endm

;Parametros del macro "FILL_SRAM"
;@0		Dirección destino (SRAM)
;@1		Dato
;@2		Número de bytes
.macro		FILL_SRAM
			push	xh
			push	xl
			push	yh
			push	yl
			push	r25
			ldiw	yh,yl,@0
			ldi		r25,@1
			ldiw	xh,xl,@2
next_copy:	st		y+,r25
			sbiw	xl,1
			brne	next_copy
			pop		r25
			pop		yl
			pop		yh
			pop		xl
			pop		xh
.endm
;**********************************************************

;**********************************************************
;Subrutinas ajustadas para cristales de cuarzo entre 1MHz y 20MHz 
.macro		delay_us
			.if		(((@0/4)*(fxtal/1000000))-2)>65536
				.error	"Valor de tiempo fuera de rango para la Fxtal definida"
			.else
			.endif
			push	xh
			push	xl
			ldi		xl,low (((@0/4)*(fxtal/1000000))-2)
			ldi		xh,high(((@0/4)*(fxtal/1000000))-2)
delay_microseg:
			sbiw	xl,1
			brne	delay_microseg
			nop
			pop		xl
			pop		xh
.endm

.macro		delay_ms
;			#ifdef	simulacion
;				rjmp	salto
;			#endif
			push	xh
			push	xl
			ldi		xl,low(@0)
			ldi		xh,high(@0)
ms:			delay_us	1000
			sbiw	xl,1
			brne	ms
			pop		xl
			pop		xh
;salto:		nop
.endm
;**********************************************************

;************************************************************
;			cjne	reg,0x00,salir_rx
;					reg	n	  e1
;
;	Registro (localidad de memoria)		= reg
;	Valor con el que se compara 'reg'	= n
;	Si reg no es igual a n, salta hacia	= e1
.macro		cjne
			inr			r25,@0
			cpi			r25,@1			;Número de byte = @0?
			rbrne		@2
.endm
;************************************************************

;************************************************************
;			cjeq	reg,0x00,salir_rx
;					reg	n	  e1
;
;	Registro (localidad de memoria)		= reg
;	Valor con el que se compara 'reg'	= n
;	Si reg es igual a n, salta hacia	= e1
.macro		cjeq
			inr			r25,@0
			cpi			r25,@1			;Número de byte = @0?
			rbreq		@2
.endm
;************************************************************

;************************************************************
.macro		setb
			inr			r25,@0
			.if			@1 == 0
				sbr		r25,0b00000001
			.elif		@1 == 1
				sbr		r25,0b00000010
			.elif		@1 == 2
				sbr		r25,0b00000100
			.elif		@1 == 3
				sbr		r25,0b00001000
			.elif		@1 == 4
				sbr		r25,0b00010000
			.elif		@1 == 5
				sbr		r25,0b00100000
			.elif		@1 == 6
				sbr		r25,0b01000000
			.elif		@1 == 7
				sbr		r25,0b10000000
			.endif
			outr		@0,r25
.endm
;************************************************************

;************************************************************
.macro		clrb
			inr			r25,@0
			.if			@1 == 0
				cbr		r25,0b00000001
			.elif		@1 == 1
				cbr		r25,0b00000010
			.elif		@1 == 2
				cbr		r25,0b00000100
			.elif		@1 == 3
				cbr		r25,0b00001000
			.elif		@1 == 4
				cbr		r25,0b00010000
			.elif		@1 == 5
				cbr		r25,0b00100000
			.elif		@1 == 6
				cbr		r25,0b01000000
			.elif		@1 == 7
				cbr		r25,0b10000000
			.endif
			outr		@0,r25
.endm
;************************************************************

;**********************************************************
;Macros de configuración para la USART, considerando que
;fxtal sea una frecuencia multiplo de 1.8432MHz
.equ	n		=	0x00
.equ	e		=	0x20
.equ	o		=	0x30
.equ	rx		=	0x10
.equ	tx		=	0x08
.equ	rxtx	=	0x18
.equ	no_irxtx=	0x00
.equ	irx		=	0x80
.equ	itx		=	0x40
.equ	irxtx	=	0xC0

.macro	usart0
	.if		fxtal==1843200
	.elif	fxtal==3686400
	.elif	fxtal==7372800
	.elif	fxtal==9216000
	.elif	fxtal==11059200
	.elif	fxtal==14745600
	.elif	fxtal==18432000
	.else
			.error	"fxtal no valida para la macro usart"
	.endif

	.if		@1==5
			ldi		r24,0x00
			ldi		r25,0x00
	.elif	@1==6
			ldi		r24,0x00
			ldi		r25,0x02
	.elif	@1==7
			ldi		r24,0x00
			ldi		r25,0x04
	.elif	@1==8
			ldi		r24,0x00
			ldi		r25,0x06
	.elif	@1==9
			ldi		r24,0x04
			ldi		r25,0x06
	.elif	@1<5
			.error	"El número de bits debe de ser entre 5 y 9"
	.elif	@1>9
			.error	"El número de bits debe de ser entre 5 y 9"
	.endif

	.if		@2==n
			ori		r25,n
	.elif	@2==e
			ori		r25,e
	.elif	@2==o
			ori		r25,o
	.else
			.error	"Error en paridad seleccionada"
	.endif

	.if		@3==1
			ori		r25,0x00
	.elif	@3==2
			ori		r25,0x08
	.else	
			.error	"Número de bits de paro incorrectos"
	.endif
	outr	ucsr0c,r25

	.if		@4==rx
			ori		r24,rx
			cbi		ddre,0
			sbi		porte,0
	.elif	@4==tx
			ori		r24,tx
			sbi		ddre,1
			sbi		porte,1
	.elif	@4==rxtx
			ori		r24,rxtx
			cbi		ddre,0
			sbi		porte,0
			sbi		ddre,1
			sbi		porte,1
	.else
			.error	"No se selecciono rx y/o tx"
	.endif

	.if		@5==no_irxtx
			ori		r24,no_irxtx
	.elif	@5==irx
			ori		r24,irx
	.elif	@5==itx
			ori		r24,itx
	.elif	@5==irxtx
			ori		r24,irxtx
	.else
			.error	"No configuro la(s) interrupcion(es) de rx y/o tx"
	.endif
	outr	ucsr0b,r24

		.set		UBRR_U2X0	=	(fxtal/(16*@0))-1
		.set		BAUD_U2X0	=	fxtal/(16*(UBRR_U2X0+1))
		.set		ERROR_U2X0	=	ABS((((BAUD_U2X0*1000)/@0)-1000));*100
		.if			ERROR_U2X0 > 20
			.set		UBRR_U2X1	=	(fxtal/(8*@0))-1
			.set		BAUD_U2X1	=	fxtal/(8*(UBRR_U2X1+1))
			.set		ERROR_U2X1	=	ABS((((BAUD_U2X1*1000)/@0)-1000));*100
			.if			ERROR_U2X1 > 20
				.error	"Baudaje no disponible"
			.else
				outi		ucsr0a,0x02
				outiw		ubrr0h,ubrr0l,UBRR_U2X1
			.endif
		.else
			outi		ucsr0a,0x00
			outiw		ubrr0h,ubrr0l,UBRR_U2X0
		.endif
.endm
;**********************************************************
.macro	usart1
	.if		@1==5
			ldi		r24,0x00
			ldi		r25,0x00
	.elif	@1==6
			ldi		r24,0x00
			ldi		r25,0x02
	.elif	@1==7
			ldi		r24,0x00
			ldi		r25,0x04
	.elif	@1==8
			ldi		r24,0x00
			ldi		r25,0x06
	.elif	@1==9
			ldi		r24,0x04
			ldi		r25,0x06
	.elif	@1<5
			.error	"El número de bits debe de ser entre 5 y 9"
	.elif	@1>9
			.error	"El número de bits debe de ser entre 5 y 9"
	.endif

	.if		@2==n
			ori		r25,n
	.elif	@2==e
			ori		r25,e
	.elif	@2==o
			ori		r25,o
	.else
			.error	"Error en paridad seleccionada"
	.endif

	.if		@3==1
			ori		r25,0x00
	.elif	@3==2
			ori		r25,0x08
	.else	
			.error	"Número de bits de paro incorrectos"
	.endif
	outr	ucsr1c,r25

	.if		@4==rx
			ori		r24,rx
			cbi		ddrd,2
			sbi		portd,2
	.elif	@4==tx
			ori		r24,tx
			sbi		ddrd,3
			sbi		portd,3
	.elif	@4==rxtx
			ori		r24,rxtx
			cbi		ddrd,2
			sbi		portd,2
			sbi		ddrd,3
			sbi		portd,3
	.else
			.error	"No se selecciono rx y/o tx"
	.endif

	.if		@5==no_irxtx
			ori		r24,no_irxtx
	.elif	@5==irx
			ori		r24,irx
	.elif	@5==itx
			ori		r24,itx
	.elif	@5==irxtx
			ori		r24,irxtx
	.else
			.error	"No configuro la(s) interrupcion(es) de rx y/o tx"
	.endif
	outr	ucsr1b,r24

		.set		UBRR_U2X0	=	(fxtal/(16*@0))-1
		.set		BAUD_U2X0	=	fxtal/(16*(UBRR_U2X0+1))
		.set		ERROR_U2X0	=	ABS((((BAUD_U2X0*1000)/@0)-1000));*100
		.if			ERROR_U2X0 > 20
			.set		UBRR_U2X1	=	(fxtal/(8*@0))-1
			.set		BAUD_U2X1	=	fxtal/(8*(UBRR_U2X1+1))
			.set		ERROR_U2X1	=	ABS((((BAUD_U2X1*1000)/@0)-1000));*100
			.if			ERROR_U2X1 > 20
				.error	"Baudaje no disponible"
			.else
				outi		ucsr1a,0x02
				outiw		ubrr1h,ubrr1l,UBRR_U2X1
			.endif
		.else
			outi		ucsr1a,0x00
			outiw		ubrr1h,ubrr1l,UBRR_U2X0
		.endif
.endm
;**********************************************************
.macro	usart2
	.if		@1==5
			ldi		r24,0x00
			ldi		r25,0x00
	.elif	@1==6
			ldi		r24,0x00
			ldi		r25,0x02
	.elif	@1==7
			ldi		r24,0x00
			ldi		r25,0x04
	.elif	@1==8
			ldi		r24,0x00
			ldi		r25,0x06
	.elif	@1==9
			ldi		r24,0x04
			ldi		r25,0x06
	.elif	@1<5
			.error	"El número de bits debe de ser entre 5 y 9"
	.elif	@1>9
			.error	"El número de bits debe de ser entre 5 y 9"
	.endif

	.if		@2==n
			ori		r25,n
	.elif	@2==e
			ori		r25,e
	.elif	@2==o
			ori		r25,o
	.else
			.error	"Error en paridad seleccionada"
	.endif

	.if		@3==1
			ori		r25,0x00
	.elif	@3==2
			ori		r25,0x08
	.else	
			.error	"Número de bits de paro incorrectos"
	.endif
	outr	ucsr2c,r25

	.if		@4==rx
			ori		r24,rx
			inr		r25,ddrh
			andi	r25,0b11111110
			outr	ddrh,r25
			inr		r25,porth
			ori		r25,0b00000001
			outr	porth,r25
	.elif	@4==tx
			ori		r24,tx
			inr		r25,ddrh
			ori		r25,0b00000010
			outr	ddrh,r25
			inr		r25,porth
			ori		r25,0b00000010
			outr	porth,r25
	.elif	@4==rxtx
			ori		r24,rxtx
			inr		r25,ddrh
			andi	r25,0b11111110
			ori		r25,0b00000010
			outr	ddrh,r25
			inr		r25,porth
			ori		r25,0b00000011
			outr	porth,r25
	.else
			.error	"No se selecciono rx y/o tx"
	.endif

	.if		@5==no_irxtx
			ori		r24,no_irxtx
	.elif	@5==irx
			ori		r24,irx
	.elif	@5==itx
			ori		r24,itx
	.elif	@5==irxtx
			ori		r24,irxtx
	.else
			.error	"No configuro la(s) interrupcion(es) de rx y/o tx"
	.endif
	outr	ucsr2b,r24

		.set		UBRR_U2X0	=	(fxtal/(16*@0))-1
		.set		BAUD_U2X0	=	fxtal/(16*(UBRR_U2X0+1))
		.set		ERROR_U2X0	=	ABS((((BAUD_U2X0*1000)/@0)-1000));*100
		.if			ERROR_U2X0 > 20
			.set		UBRR_U2X1	=	(fxtal/(8*@0))-1
			.set		BAUD_U2X1	=	fxtal/(8*(UBRR_U2X1+1))
			.set		ERROR_U2X1	=	ABS((((BAUD_U2X1*1000)/@0)-1000));*100
			.if			ERROR_U2X1 > 20
				.error	"Baudaje no disponible"
			.else
				outi		ucsr2a,0x02
				outiw		ubrr2h,ubrr2l,UBRR_U2X1
			.endif
		.else
			outi		ucsr2a,0x00
			outiw		ubrr2h,ubrr2l,UBRR_U2X0
		.endif
.endm
;**********************************************************
.macro	usart3
	.if		@1==5
			ldi		r24,0x00
			ldi		r25,0x00
	.elif	@1==6
			ldi		r24,0x00
			ldi		r25,0x02
	.elif	@1==7
			ldi		r24,0x00
			ldi		r25,0x04
	.elif	@1==8
			ldi		r24,0x00
			ldi		r25,0x06
	.elif	@1==9
			ldi		r24,0x04
			ldi		r25,0x06
	.elif	@1<5
			.error	"El número de bits debe de ser entre 5 y 9"
	.elif	@1>9
			.error	"El número de bits debe de ser entre 5 y 9"
	.endif

	.if		@2==n
			ori		r25,n
	.elif	@2==e
			ori		r25,e
	.elif	@2==o
			ori		r25,o
	.else
			.error	"Error en paridad seleccionada"
	.endif

	.if		@3==1
			ori		r25,0x00
	.elif	@3==2
			ori		r25,0x08
	.else	
			.error	"Número de bits de paro incorrectos"
	.endif
	outr	ucsr3c,r25

	.if		@4==rx
			ori		r24,rx
			inr		r25,ddrj
			andi	r25,0b11111110
			outr	ddrj,r25
			inr		r25,portj
			ori		r25,0b00000001
			outr	portj,r25
	.elif	@4==tx
			ori		r24,tx
			inr		r25,ddrj
			ori		r25,0b00000010
			outr	ddrj,r25
			inr		r25,portj
			ori		r25,0b00000010
			outr	portj,r25
	.elif	@4==rxtx
			ori		r24,rxtx
			inr		r25,ddrj
			andi	r25,0b11111110
			ori		r25,0b00000010
			outr	ddrj,r25
			inr		r25,portj
			ori		r25,0b00000011
			outr	portj,r25
	.else
			.error	"No se selecciono rx y/o tx"
	.endif

	.if		@5==no_irxtx
			ori		r24,no_irxtx
	.elif	@5==irx
			ori		r24,irx
	.elif	@5==itx
			ori		r24,itx
	.elif	@5==irxtx
			ori		r24,irxtx
	.else
			.error	"No configuro la(s) interrupcion(es) de rx y/o tx"
	.endif
	outr	ucsr3b,r24

		.set		UBRR_U2X0	=	(fxtal/(16*@0))-1
		.set		BAUD_U2X0	=	fxtal/(16*(UBRR_U2X0+1))
		.set		ERROR_U2X0	=	ABS((((BAUD_U2X0*1000)/@0)-1000));*100
		.if			ERROR_U2X0 > 20
			.set		UBRR_U2X1	=	(fxtal/(8*@0))-1
			.set		BAUD_U2X1	=	fxtal/(8*(UBRR_U2X1+1))
			.set		ERROR_U2X1	=	ABS((((BAUD_U2X1*1000)/@0)-1000));*100
			.if			ERROR_U2X1 > 20
				.error	"Baudaje no disponible"
			.else
				outi		ucsr3a,0x02
				outiw		ubrr3h,ubrr3l,UBRR_U2X1
			.endif
		.else
			outi		ucsr3a,0x00
			outiw		ubrr3h,ubrr3l,UBRR_U2X0
		.endif
.endm
