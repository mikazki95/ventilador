;3.0
;14 de Octubre de 2020 - CMV SIMV CPAP PRESION DE ENTRADA
;**********************************************************************************************
;* Libreria de macroinstrucciones y subrutinas de punto flotante
;**********************************************************************************************
;* Macros de operaciones aritmeticas
;* ACC1=ACC1-(Z)		FSUBM
;* ACC1=ACC1-ACC2		FSUB
;* ACC1=ACC1+0.5		FADDP5
;* ACC1=ACC1+(Z)		FADDM
;* ACC1=ACC1+ACC2		FADD
;* ACC1=-ACC1			FNEG
;* ACC1=1/(Z)			FINVM
;* ACC1=1/ACC1			FINV
;* ACC1=ACC1/(Z)		FDIVM
;* ACC1=ACC1/10			FDIV10
;* ACC1=ACC1/ACC2		FDIV
;* ACC1=ACC1*(Z)		FMULTM
;* ACC1=ACC1*10			FMULT10
;* ACC1=ACC1*ACC2		FMULT
;* ACC1=(Z)*(Z)			FSQRM
;* ACC1=ACC1*ACC1		FSQR
;* ACC1=Raiz cudrada de (Z)	FSQRTM
;* ACC1=Raiz cudrada de ACC1	FSQRT
;**********************************************************************************************
;* Macros para movimiento de datos
;* ACC1=(Z)			FLDA1
;* ACC2=(Z)			FLDA2
;* (Z)=ACC1			FSTA1
;* (Z)=ACC2			FSTA2
;* ACC1<=ACC2			FMOVA1A2
;* ACC2<=ACC1			FMOVA2A1
;* Intercambia Acumuladores	FSWAP
;* ACC1=Constante encontrada por Z en la memoria Flash	FLDKA1
;* ACC2=Constante encontrada por Z en la memoria Flash	FLDKA2
;* Guarda ACC1 en el stack virtual			FPUSHA1
;* Guarda ACC2 en el stack virtual			FPUSHA2
;* Recupera ACC1 del stack virtual			FPOPA1
;* Recupera ACC2 del stack virtual			FPOPA2
;**********************************************************************************************
;* Macroinstrucciones para salvar y recuperar registros usados en las rutinas
;* de punto flotante (R10 a R25). 16 bytes de stack+bytes reuqridos para la
;* aplicación.
;*				FSAVEREG
;*				FRESTOREREG
;**********************************************************************************************
;* Macros de comparacion de datos en F.P.
;* El registro "TEMP" regresa con el resultado de la comparacion.
;* TEMP = $00 El dato 1 es = al dato 2
;* TEMP = $01 El dato 1 es > el dato 2
;* TEMP = $FF El dato 1 es < el dato 2
;* Compara ACC1 con el dato en F.P. encontrado por Z en la memoria Flash (ACC1=dato 1)	FCPFLASH
;* Compara ACC1 con el dato en F.P. encontrado por Z en la memoria RAM (ACC1=dato 1)	FCPM
;* Compara el dato encontrado por Y en la memoria RAM con el dato encontrado por Z
;* en la memoria RAM (Y apunta al dato 1)						FCPRAM
;* Compara ACC1 con el ACC2 (ACC1=dato1)						FCP
;**********************************************************************************************
;Macroinstrucciones para preparar limpiar registros decimales y hexadecimales 
;*	CLR_DEC
;*	CLR_HEX
;**********************************************************************************************
;* Macroinstrucciones para normalizar numeros.
;*	NORMHEX
;*	NORMDEC
;*	NORMHEXTOA1
;*	NORMHEXTOA2
;*	NORMDECTOA1
;*	NORMDECTOA2
;**********************************************************************************************
;* Macroinstrucciones para denormalizar numeros.
;*	DENORMHEX
;*	DENORMDEC
;*	DENORMA1TOHEX
;*	DENORMA2TOHEX
;*	DENORMA1TODEC
;*	DENORMA2TODEC
;**********************************************************************************************
;Macros para realizar conversiones entre sistemas numericos
;Convierte el contenido de las localidades 60-63 a decimal.
;*	DTOH
;Convierte el contenido de las localidades 64-68 a hexadecimal.
;*	HTOD
;**********************************************************************************************
;Macroinstrucciones para conversion entre sistemas (fraccion).
;Convierte el contenido de las localidades        a decimal.
;*	FDTOH
;Convierte el contenido de las localidades a hexadecimal.
;*	FHTOD
;**********************************************************************************************
;Definiciones usadas:
;
;Valor no normalizado:
;  HEX_H HEX_MH HEXML HEX_L . FHEX_H FHEX_MH FHEX_ML FHEX_L
;		   Punto no normalizado.
;
;Registros empleados para normalizar y denormalizar:
;       HEX7 HEX6 HEX5 HEX4 . HEX3 HEX2 HEX1 HEX0
;		   Punto no normalizado.
;
;Registros con el contenido normalizado:
;	HEX3		Signo+7 bits del exponente
;	HEX2		1 bit del exponente + 7 bits de mantisa
;	HEX1:HEX0	16 bits de mantisa
;
;El signo del numero se debera de encontrar en la bandera T antes de normalizar y también se
;encontrara en esta bandera al final de la denormalización.
;
;Si al denormaliar el numero es muy grande (no se puede representar la parte entera en 32
;bits) C=1, en caso contrario C=0
;
;****************************************************************************
;Registros y definiciones para el manejo de enteros hex y dec
.equ	HEX_L 	=  reg_lib
.equ	HEX_ML	=  reg_lib+1
.equ	HEX_MH	=  reg_lib+2
.equ	HEX_H 	=  reg_lib+3
.equ	DEC_L   =  reg_lib+4
.equ	DEC_ML  =  reg_lib+5
.equ	DEC_M   =  reg_lib+6
.equ	DEC_MH  =  reg_lib+7
.equ	DEC_H   =  reg_lib+8
;****************************************************************************
;Registros y definiciones para el manejo de fracciones hex y dec
.equ		FHEX_H    = reg_lib+9
.equ		FHEX_MH   = reg_lib+10
.equ		FHEX_ML   = reg_lib+11
.equ		FHEX_L    = reg_lib+12
.equ		FDEC_15   = reg_lib+13
.equ		FDEC_14   = reg_lib+14
.equ		FDEC_13   = reg_lib+15
.equ		FDEC_12   = reg_lib+16
.equ		FDEC_11   = reg_lib+17
.equ		FDEC_10   = reg_lib+18
.equ		FDEC_9    = reg_lib+19
.equ		FDEC_8    = reg_lib+20
.equ		FDEC_7    = reg_lib+21
.equ		FDEC_6    = reg_lib+22
.equ		FDEC_5    = reg_lib+23
.equ		FDEC_4    = reg_lib+24
.equ		FDEC_3    = reg_lib+25
.equ		FDEC_2    = reg_lib+26
.equ		FDEC_1	  = reg_lib+27
.equ		FDEC_0	  = reg_lib+28
;****************************************************************************
;Registros y definiciones especificos de las subrutinas y macros de normalización y denormalización
.equ	HEX7	= reg_lib+5		;Entero Hex
.equ	HEX6	= reg_lib+6
.equ	HEX5	= reg_lib+7
.equ	HEX4	= reg_lib+8
.equ	HEX3	= reg_lib+9	;Fraccion Hex
.equ	HEX2	= reg_lib+10
.equ	HEX1	= reg_lib+11
.equ	HEX0	= reg_lib+12
.def	EXPONENTE=	R20

;Registros especificos de punto flotante
;****************************************************************************
; NOTA: Todas las funciones con memoria: por ejemplo; FADDMEM
; emplean al puntero Z para especificar la direcció del dato en FP de 32 bits
; que se encuentra en SRAM
.def     QUOT		=	r10
.def     QUOTM		=	r11
.def     QUOTH		=	r12
.def     A1_L		=	r13           ; Accumulador 1
.def     A1_ML		=	r14
.def     A1_MH		=	r15
.def     A1_H		=	r16
.def     SIGN1		=	r17
.def     A2_L		=	r18           ; Accumulador 2
.def     A2_ML		=	r19
.def     A2_MH		=	r20
.def     A2_H		=	r21
.def     SIGN2		=	r22
.def     A1_LT		=	r23
.def     COUNTFP32	=	r24
.def     TEMPFP32	=	r25
.equ     MATHTMP	=	reg_lib+29	;8 bytes reservados para FP
.equ     MATHTMPb	=	reg_lib+30
.equ     MATHTMPc	=	reg_lib+31
.equ     MATHTMPd	=	reg_lib+32
.equ     MATHTMPe	=	reg_lib+33
.equ     MATHTMPf	=	reg_lib+34
.equ     MATHTMPg	=	reg_lib+35
.equ     MATHTMPh	=	reg_lib+36

;**********************************************************************************************
;* Macroinstrucciones y subrutinas de punto flotante
;**********************************************************************************************

;**********************************************************************************************
;* Macros de operaciones aritmeticas
;* ACC1=ACC1-(Z)
.MACRO   FSUBM
         RCALL FSUBMEM
.ENDM

;* ACC1=ACC1-ACC2
.MACRO   FSUB
         RCALL FSUBS
.ENDM

;* ACC1=ACC1+0.5
.MACRO   FADDP5
         RCALL FADP5
.ENDM

;* ACC1=ACC1+(Z)
.MACRO   FADDM
         RCALL FADDMEM
.ENDM

;* ACC1=ACC1+ACC2
.MACRO   FADD
         RCALL FSUMA
.ENDM

;* ACC1=-ACC1
.MACRO   FNEG
         RCALL FLTNEG
.ENDM

;* ACC1=1/(Z)
.MACRO   FINVM
         RCALL F1OVERMEM
.ENDM

;* ACC1=1/ACC1
.MACRO   FINV
         RCALL F1OVERX
.ENDM

;* ACC1=ACC1/(Z)
.MACRO   FDIVM
         RCALL FDIVMEM
.ENDM

;* ACC1=ACC1/10
.MACRO   FDIV10
         RCALL FDIV_10
.ENDM

;* ACC1=ACC1/ACC2
.MACRO   FDIV
         CALL F_DIV
.ENDM

;* ACC1=ACC1*(Z)
.MACRO   FMULTM
         RCALL FMULMEM
.ENDM

;* ACC1=ACC1*10
.MACRO   FMULT10
         RCALL FMUL_10
.ENDM

;* ACC1=ACC1*ACC2
.MACRO   FMULT
         RCALL F_MUL
.ENDM

;* ACC1=(Z)*(Z)
.MACRO   FSQRM
         RCALL FSQRMEM
.ENDM

;* ACC1=ACC1*ACC1
.MACRO   FSQR
         RCALL F_SQR
.ENDM

;* ACC1=Raiz cudrada de (Z)
.MACRO   FSQRTM
         RCALL FSQRTMEM
.ENDM

;* ACC1=Raiz cudrada de ACC1
.MACRO   FSQRT
         RCALL F_SQRT
.ENDM
;**********************************************************************************************

;**********************************************************************************************
;* Macros para movimiento de datos

;* ACC1=(Z)
.MACRO   FLDA1
         RCALL FSRAMA1
.ENDM

;* ACC2=(Z)
.MACRO   FLDA2
         RCALL FSRAMA2
.ENDM

;* (Z)=ACC1
.MACRO   FSTA1
         RCALL FSRAMA_1
.ENDM

;* (Z)=ACC2
.MACRO   FSTA2
         RCALL FSRAMA_2
.ENDM

;* ACC1<=ACC2
.MACRO   FMOVA1A2
         RCALL AC2TOAC1
.ENDM

;* ACC2<=ACC1
.MACRO   FMOVA2A1
         RCALL AC1TOAC2
.ENDM

;* Intercambia Acumuladores
.MACRO   FSWAP
         CALL SWAPACC
.ENDM

;* ACC1=Constante encontrada por Z en la memoria Flash
.MACRO   FLDKA1
         CALL KTOAC1
.ENDM

;* ACC2=Constante encontrada por Z en la memoria Flash
.MACRO   FLDKA2
         CALL KTOAC2
.ENDM

;* Guarda ACC1 en el stack virtual
.MACRO   FPUSHA1
         CALL PUSH1
.ENDM

;* Guarda ACC2 en el stack virtual
.MACRO   FPUSHA2
         CALL PUSH2
.ENDM

;* Recupera ACC1 del stack virtual
.MACRO   FPOPA1
         CALL POP1
.ENDM

;* Recupera ACC2 del stack virtual
.MACRO   FPOPA2
         CALL POP2
.ENDM
;**********************************************************************************************

;****************************************************************************
;* Macroinstrucciones para salvar y recuperar registros usados en las rutinas
;* de punto flotante (R10 a R25). 16 bytes de stack+bytes reuqridos para la
;* aplicación.
.MACRO	FSAVEREG
	push	R10
	push	R11
	push	R12
	push	R13
	push	R14
	push	R15
	push	R16
	push	R17
	push	R18
	push	R19
	push	R20
	push	R21
	push	R22
	push	R23
	push	R24
	push	R25
	push	R30
	push	R31
.ENDM

.MACRO	FRESTOREREG
	pop	R31
	pop	R30
	pop	R25
	pop	R24
	pop	R23
	pop	R22
	pop	R21
	pop	R20
	pop	R19
	pop	R18
	pop	R17
	pop	R16
	pop	R15
	pop	R14
	pop	R13
	pop	R12
	pop	R11
	pop	R10
.ENDM
;****************************************************************************

;**********************************************************************************************
;* Macros de comparacion de datos en F.P.
;* El registro "TEMP" regresa con el resultado de la comparacion.
;* TEMP = $00 El dato 1 es = al dato 2
;* TEMP = $01 El dato 1 es > el dato 2
;* TEMP = $FF El dato 1 es < el dato 2

;* Compara ACC1 con el dato en F.P. encontrado por Z en la memoria Flash (ACC1=dato 1)
.MACRO   FCPFLASH
         RCALL FLTCPFLASH
.ENDM

;* Compara ACC1 con el dato en F.P. encontrado por Z en la memoria RAM (ACC1=dato 1)
.MACRO   FCPM
         RCALL FLTCPMEM
.ENDM

;* Compara el dato encontrado por Y en la memoria RAM con el dato encontrado por Z en la
;* memoria RAM (Y apunta al dato 1)
.MACRO   FCPRAM
         RCALL FLTCP2
.ENDM

;* Compara ACC1 con el ACC2 (ACC1=dato1)
.MACRO   FCP
         CALL FLTCP
.ENDM
;**********************************************************************************************

;**********************************************************************************************
;Macroinstrucciones para preparar limpiar registros decimales y hexadecimales 
.macro		CLR_DEC
		call	CLR_DECIMAL
.endm

.macro		CLR_HEX
		call	CLR_HEXADECIMAL
.endm
;**********************************************************************************************

;**********************************************************************************************
;* Macroinstrucciones para normalizar numeros.
.MACRO	NORMHEX
	CALL NORM_HEX
.ENDM

.MACRO	NORMDEC
	CALL DECTOHEX
	CALL FDECTOHEX
	CALL NORM_HEX
.ENDM

.MACRO	NORMHEXTOA1
	CALL NORM_HEX
	LDS R13,HEX0
	LDS R14,HEX1
	LDS R15,HEX2
	LDS R16,HEX3
.ENDM

.MACRO	NORMHEXTOA2
	CALL NORM_HEX
	LDS R18,HEX0
	LDS R19,HEX1
	LDS R20,HEX2
	LDS R21,HEX3
.ENDM

.MACRO	NORMDECTOA1
	CALL DECTOHEX
	CALL FDECTOHEX
	CALL NORM_HEX
	LDS R13,HEX0
	LDS R14,HEX1
	LDS R15,HEX2
	LDS R16,HEX3
.ENDM

.MACRO	NORMDECTOA2
	CALL DECTOHEX
	CALL FDECTOHEX
	CALL NORM_HEX
	LDS R18,HEX0
	LDS R19,HEX1
	LDS R20,HEX2
	LDS R21,HEX3
.ENDM
;****************************************************************************

;****************************************************************************
;* Macroinstrucciones para denormalizar numeros.
.MACRO	DENORMHEX
	RCALL DENORM_HEX
.ENDM

.MACRO	DENORMDEC
	RCALL DENORM_HEX
	RCALL HEXTODEC
	RCALL FHEXTODEC
.ENDM

.MACRO	DENORMA1TOHEX
	STS HEX0,R13
	STS HEX1,R14
	STS HEX2,R15
	STS HEX3,R16
	CALL DENORM_HEX
.ENDM

.MACRO	DENORMA2TOHEX
	STS HEX0,R18
	STS HEX1,R19
	STS HEX2,R20
	STS HEX3,R21
	RCALL DENORM_HEX
.ENDM

.MACRO	DENORMA1TODEC
	STS HEX0,R13
	STS HEX1,R14
	STS HEX2,R15
	STS HEX3,R16
	CALL DENORM_HEX
	CALL HEXTODEC
	CALL FHEXTODEC
.ENDM

.MACRO	DENORMA2TODEC
	STS HEX0,R18
	STS HEX1,R19
	STS HEX2,R20
	STS HEX3,R21
	CALL DENORM_HEX
	CALL HEXTODEC
	CALL FHEXTODEC
.ENDM
;****************************************************************************

;****************************************************************************
;Macros para realizar conversiones entre sistemas numericos
;****************************************************************************
;Macroinstrucciones para conversion entre sistemas (enteros).
;Convierte el contenido de las localidades 60-63 a decimal.
.MACRO	DTOH
	CALL DECTOHEX
.ENDM

;Convierte el contenido de las localidades 64-68 a hexadecimal.
.MACRO	HTOD
	CALL HEXTODEC
.ENDM
;****************************************************************************

;****************************************************************************
;Macroinstrucciones para conversion entre sistemas (fraccion).
;Convierte el contenido de las localidades        a decimal.
.MACRO	FDTOH
	CALL FDECTOHEX
.ENDM

;Convierte el contenido de las localidades a hexadecimal.
.MACRO	FHTOD
	CALL FHEXTODEC
.ENDM
;****************************************************************************

;****************************************************************************
;Subrutina de normalizacion de numeros en punto flotante
;Registros entregados por las subrutinas de conversion:
;  HEX_H HEX_MH HEXML HEX_L . FHEX_H FHEX_MH FHEX_ML FHEX_L
;		   Punto no normalizado.
;Registros empleados:
;       HEX7 HEX6 HEX5 HEX4 . HEX3 HEX2 HEX1 HEX0
;		   Punto no normalizado.
;
;Resultado normalizado:
;	HEX3		Signo+7 bits del exponente
;	HEX2		1 bit del exponente + 7 bits de mantisa
;	HEX1:HEX0	16 bits de mantisa
;
;El signo del numero se debera de encontrar en la bandera T
;.equ	HEX7	= $0065		;Entero Hex
;.equ	HEX6	= $0066
;.equ	HEX5	= $0067
;.equ	HEX4	= $0068
;.equ	HEX3	= $0069		;Fraccion Hex
;.equ	HEX2	= $006A
;.equ	HEX1	= $006B
;.equ	HEX0	= $006C
;.def	EXPONENTE=R20
.equ	BITS_HEX = $40         		;Numero de bits hex.

NORM_HEX:	PUSH R16
		PUSH R17
		PUSH R18
		PUSH R19
		PUSH ZL
		PUSH ZH
		PUSH EXPONENTE		
		LDS R16,HEX_L
		STS HEX4,R16
		LDS R16,HEX_ML
		STS HEX5,R16
		LDS R16,HEX_MH
		STS HEX6,R16
		LDS R16,HEX_H
		STS HEX7,R16
		LDI EXPONENTE,$9E	;Exponente mas alto
		LDI R18,BITS_HEX
NORM_HEX1:	LDI	ZL,LOW(HEX0)
			LDI	ZH,HIGH(HEX0)
		LDI R17,$08		;Numero de registros
		CLC
		IN R19,SREG	;Lectura dummy                    
NORM_HEX2:	OUT SREG,R19
		LD R16,Z
                ROL R16
                ST Z,R16
		IN R19,SREG
                SBIW ZL,1
                DEC R17
		BRNE NORM_HEX2
		OUT SREG,R19
		BRCS NORM_HEX3
		DEC EXPONENTE
		DEC R18
		BRNE NORM_HEX1
		RJMP END_NORM_HEX			;Resultado = 0.0
NORM_HEX3:	BRTS NORM_HEX4
		CLC
		RJMP NORM_HEX5
NORM_HEX4:	SEC
NORM_HEX5:	ROR EXPONENTE
		STS HEX3,EXPONENTE
		LDS R16,HEX7
		ROR R16
		STS HEX2,R16
		LDS R16,HEX6
		ROR R16
		STS HEX1,R16
		LDS R16,HEX5
		ROR R16
		STS HEX0,R16
END_NORM_HEX:	POP EXPONENTE
		POP ZH
		POP ZL
		POP R19
		POP R18
		POP R17
		POP R16
		RET
;****************************************************************************

;****************************************************************************
;Subrutina de denormalizacion de numeros en punto flotante
;Numero de entrada normalizado:
;	HEX3		Signo+7 bits del exponente
;	HEX2		1 bit del exponente + 7 bits de mantisa
;	HEX1:HEX0	16 bits de mantisa
;
;Resultado no normalizado:
;  HEX_H HEX_MH HEXML HEX_L . FHEX_H FHEX_MH FHEX_ML FHEX_L
;		   Punto no normalizado.
;
;El signo del numero se encontrara en la bandera T
;Si el numero es muy grande (no se puede representar la parte entera en 32
;bits) C=1, en caso contrario C=0

DENORM_HEX:	PUSH R16
		PUSH R17
		PUSH R19
		PUSH ZL
		PUSH ZH
		PUSH EXPONENTE		
		LDS R16,HEX2
		ROL R16
		STS HEX2,R16
		LDS EXPONENTE,HEX3	;Obtencion del exp.
		ROL EXPONENTE
		BRCS DENORM_HEX1	;Asigna el signo
		CLT
		RJMP DENORM_HEX2
DENORM_HEX1:	SET
DENORM_HEX2:	SEC
		LDS R16,HEX2		;Ajusta la mantisa
		ROR R16
		STS HEX2,R16
		CPI EXPONENTE,$5F	;Verifica que el exp se encuentre
		BRSH DENORM_HEX8	;en el rango de $5F a $9E
		RJMP DENORM_HEX3
DENORM_HEX8:	CPI EXPONENTE,$9F
		BRLO DENORM_HEX4
		SEC			;C=1 error de desbordamiento
		RJMP END_DENORM_HEX
DENORM_HEX4:	LDS R16,HEX2		;Valor valido (prepara los registros
		STS HEX7,R16		;para la denormalizacion)
		LDS R16,HEX1
		STS HEX6,R16
		LDS R16,HEX0
		STS HEX5,R16
		LDI	R25,0X00
		STS HEX4,R25
		STS HEX3,R25
		STS HEX2,R25
		STS HEX1,R25
		STS HEX0,R25
DENORM_HEX6:	LDI	ZL,LOW(HEX7)
				LDI	ZH,HIGH(HEX7)
		LDI R17,$08
		CLC
		IN R19,SREG	;Lectura dummy
DENORM_HEX5:	CPI EXPONENTE,$9E
		BREQ DENORM_HEX10
		OUT SREG,R19
		LD R16,Z
		ROR R16
		ST Z+,R16
		IN R19,SREG
		DEC R17
		BRNE DENORM_HEX5
		INC EXPONENTE
		RJMP DENORM_HEX6
DENORM_HEX10:	LDS R16,HEX7
		STS HEX_H,R16
		LDS R16,HEX6
		STS HEX_MH,R16
		LDS R16,HEX5
		STS HEX_ML,R16
		LDS R16,HEX4
		STS HEX_L,R16
		RJMP DENORM_HEX9
DENORM_HEX3:	LDI R17,$08		;Valor Hex = 0
		LDI		ZL,LOW(HEX_L)
		LDI		ZH,HIGH(HEX_L)
		CLR R19
DENORM_HEX7:	ST Z+,R19
		DEC R17
		BRNE DENORM_HEX7
DENORM_HEX9:	CLC
END_DENORM_HEX: POP EXPONENTE
		POP ZH
		POP ZL
		POP R19
		POP R17
		POP R16
		RET
;****************************************************************************

;****************************************************************************
;ESTE PROGRAMA CONVIERTE DATOS DE 10 DIGITOS DECIMALES (QUE NO EXCEDA A
;4,294,967,295) A HEXADECIMAL DE 32 BITS.
;.equ	HEX_L 	=  $0060
;.equ	HEX_ML	=  $0061
;.equ	HEX_MH	=  $0062
;.equ	HEX_H 	=  $0063  
;.equ	DEC_L   =  $0064
;.equ	DEC_ML  =  $0065
;.equ	DEC_M   =  $0066
;.equ	DEC_MH  =  $0067
;.equ	DEC_H   =  $0068
.equ	BITS    =  $20          ;NUMERO DE BITS.
.equ	N_REG   =  $09          ;NUMERO DE REGISTROS (TOTAL).
.equ	N_REGDEC=  $05          ;NUMERO DE REGISTROS DECIMALES.

DECTOHEX:           PUSH R16
		    PUSH R17
		    PUSH R18
		    PUSH R19
		    PUSH ZL
		    PUSH ZH
			LDI	R25,0X00
		    STS HEX_L,R25
            STS HEX_ML,R25
            STS HEX_MH,R25
            STS HEX_H,R25
                    LDI R18,BITS
DH1:                LDI	ZL,LOW(DEC_H)
					LDI	ZH,HIGH(DEC_H)
                    LDI R17,N_REG
		    CLC
		    IN R19,SREG	;Lectura dummy                    
DH2:		    OUT SREG,R19
		    LD R16,Z
                    ROR R16
                    ST Z,R16
		    IN R19,SREG
                    SBIW ZL,1
                    DEC R17
		    BRNE DH2
                    DEC R18
		    BRNE DH3
		    POP ZH
		    POP ZL
		    POP R19
		    POP	R18
		    POP R17
		    POP R16
                    RET
DH3:                LDI	ZL,LOW(DEC_L)
					LDI ZH,HIGH(DEC_L)
                    LDI R17,N_REGDEC
DH4:                LD R16,Z
                    ANDI R16,$0F
                    CPI R16,$05
		    BRLO DH5
                    SUBI R16,$03
DH5:                MOV R19,R16
                    LD R16,Z
                    ANDI R16,$F0
		    CPI R16,$50
		    BRLO DH6
                    SUBI R16,$30
DH6:                ADD R16,R19
                    ST Z+,R16
                    DEC R17
		    BRNE DH4
                    RJMP DH1
;****************************************************************************

;****************************************************************************
;ESTE PROGRAMA CONVIERTE DATOS DE 10 DIGITOS DECIMALES (QUE NO EXCEDA A
;4,294,967,295) A HEXADECIMAL DE 32 BITS.
HEXTODEC:           PUSH R16
		    PUSH R17
		    PUSH R18
		    PUSH R19
		    PUSH ZL
		    PUSH ZH
			LDI	R25,0X00
		    STS DEC_L,R25
            STS DEC_ML,R25
            STS DEC_M,R25
            STS DEC_MH,R25
		    STS DEC_H,R25
                    LDI R18,BITS
HD1:                LDI	ZL,LOW(HEX_L)
					LDI	ZH,HIGH(HEX_L)
                    LDI R17,N_REG
		    CLC
		    IN R19,SREG	;Lectura dummy
HD2:		    OUT SREG,R19
	            LD R16,Z
                    ROL R16
                    ST Z,R16
		    IN R19,SREG
                    ADIW ZL,1
                    DEC R17
		    BRNE HD2
                    DEC R18
		    BRNE HD3
		    POP ZH
		    POP ZL
		    POP R19
		    POP	R18
		    POP R17
		    POP R16
                    RET
HD3:                LDI	ZL,LOW(DEC_L)
					LDI	ZH,HIGH(DEC_L)
                    LDI R17,N_REGDEC
HD4:                LD R16,Z
                    ANDI R16,$0F
                    CPI R16,$05
		    BRLO HD5
                    SUBI R16,$FD		;= ADDI A,$03
HD5:                MOV R19,R16
                    LD R16,Z
                    ANDI R16,$F0
		    CPI R16,$50
		    BRLO HD6
                    SUBI R16,$D0		;= ADDI R16,$30
HD6:                ADD R16,R19
                    ST Z,R16
                    ADIW ZL,1
                    DEC R17
		    BRNE HD4
                    RJMP HD1
;****************************************************************************

;****************************************************************************
;ESTE PROGRAMA CONVIERTE DATOS DE 32 DIGITOS DECIMALES FRACCIONARIOS
;REPRESENTADOS EN BCD, A HEXADECIMAL DE 32 BITS.
;.equ		FHEX_H    = $0069
;.equ		FHEX_MH   = $006A
;.equ		FHEX_ML   = $006B
;.equ		FHEX_L    = $006C
;.equ		FDEC_15   = $006D
;.equ		FDEC_14   = $006E
;.equ		FDEC_13   = $006F
;.equ		FDEC_12   = $0070
;.equ		FDEC_11   = $0071
;.equ		FDEC_10   = $0072
;.equ		FDEC_9    = $0073
;.equ		FDEC_8    = $0074
;.equ		FDEC_7    = $0075
;.equ		FDEC_6    = $0076
;.equ		FDEC_5    = $0077
;.equ		FDEC_4    = $0078
;.equ		FDEC_3    = $0079
;.equ		FDEC_2    = $007A
;.equ		FDEC_1	  = $007B
;.equ		FDEC_0	  = $007C
.equ		BITSF     = $20	 ;Numero de bits de la conversion
.equ		N_FREG    = $14  ;NUMERO DE REGISTROS (TOTAL).
.equ		N_REGFHEX = $4   ;NUMERO DE REGISTROS HEXADECIMALES.
.equ		N_REGFDEC = $10  ;NUMERO DE REGISTROS DECIMALES.

FDECTOHEX:      PUSH R16
		PUSH R17
		PUSH R18
		PUSH R19
		PUSH ZL
		PUSH ZH
		LDI	ZL,LOW(FHEX_H)
		LDI	ZH,HIGH(FHEX_H)	;Borra los registros FDEC
		LDI R18,N_REGFHEX
		CLR R19
FPDH11:         ST Z+,R19
                DEC R18
		BRNE FPDH11
		LDI R18,BITSF
FPDH3:        	LDI	ZL,LOW(FDEC_15)
				LDI	ZH,HIGH(FDEC_15)
                LDI R17,N_REGFDEC
FPDH4:          LD R16,Z
                ANDI R16,$F0
		CPI R16,$50
		BRLO FPDH7
	        SUBI R16,$D0		;ADDI R16,#30H
FPDH7:          MOV R19,R16
                LD R16,Z
                ANDI R16,$0F
		CPI R16,$05
		BRLO FPDH10
	        SUBI R16,$FD		;ADDI R16,#03H
FPDH10:         OR R16,R19
                ST Z+,R16
		DEC R17
		BRNE FPDH4
FPDH1:		LDI	ZL,LOW(FDEC_0)
			LDI	ZH,HIGH(FDEC_0)
                LDI R17,N_FREG
                CLC
		IN R19,SREG	;Lectura dummy
FPDH2:          OUT SREG,R19
		LD R16,Z
                ROL R16
                ST Z,R16
		IN R19,SREG
                SBIW ZL,1
                DEC R17
		BRNE FPDH2
		DEC R18
		BRNE FPDH3
		POP ZH
		POP ZL
		POP R19
		POP R18
		POP R17
		POP R16
		RET
;****************************************************************************

;****************************************************************************
;ESTE PROGRAMA CONVIERTE DATOS DE 32 BITS HEXADECIMALES QUE REPRESENTEN FRACCION, A
;DECIMAL REPRESENTADOS EN BCD.
FHEXTODEC:	PUSH R16
		PUSH R17
		PUSH R18
		PUSH R19
		PUSH ZL
		PUSH ZH
		LDI	ZL,LOW(FDEC_15)
		LDI	ZH,HIGH(FDEC_15)	;Borra los registros FDEC
		LDI R18,N_REGFDEC
		CLR R19
FPHD11:         ST Z+,R19
                DEC R18
		BRNE FPHD11
                LDI R18,BITSF
FPHD1:		LDI	ZL,LOW(FHEX_H)
			LDI	ZH,HIGH(FHEX_H)
                LDI R17,N_FREG
                CLC
		IN R19,SREG	;Lectura dummy
FPHD2:          OUT SREG,R19
		LD R16,Z
                ROR R16
                ST Z+,R16
		IN R19,SREG
                DEC R17
		BRNE FPHD2
FPHD3:          LDI	ZL,LOW(FDEC_15)
				LDI	ZH,HIGH(FDEC_15)
                LDI R17,N_REGFDEC
FPHD4:          LD R16,Z
                ANDI R16,$F0
		CPI R16,$50
		BRLO FPHD7
	        SUBI R16,$30
FPHD7:          MOV R19,R16
                LD R16,Z
                ANDI R16,$0F
		CPI R16,$05
		BRLO FPHD10
	        SUBI R16,$03
FPHD10:         OR R16,R19
                ST Z+,R16
                DEC R17
		BRNE FPHD4
                DEC R18
		BRNE FPHD1
		POP ZH
		POP ZL
		POP R19
		POP R18
		POP R17
		POP R16
                RET
;****************************************************************************

;****************************************************************************
; NOTA: Todas las funciones con memoria: por ejemplo; FADDMEM
; emplean al puntero Z para especificar la direcció del dato en FP de 32 bits
; que se encuentra en SRAM
;.def     QUOT =r10
;.def     QUOTM =r11
;.def     QUOTH =r12
;.def     A1_L =r13           ; Accumulador 1
;.def     A1_ML =r14
;.def     A1_MH =r15
;.def     A1_H =r16
;.def     SIGN1 =r17
;.def     A2_L =r18           ; Accumulador 2
;.def     A2_ML =r19
;.def     A2_MH =r20
;.def     A2_H =r21
;.def     SIGN2 =r22
;.def     A1_LT =r23
;.def     COUNTFP32 =r24
;.def     TEMPFP32 =r25
;.equ     MATHTMP=$0080

;**************************************************************************
;** ACC1 - (Z) **
FSUBMEM: rcall FSRAMA2
;** ACC1 - ACC2 **
FSUBS:   rcall UNPACK
         cpi A2_H,0x80
         breq LOOP_FP
         ldi TEMPFP32,0x80
         eor sign2,TEMPFP32
         rjmp FADD1
FSUNDER: brmi LOOP_FP
FRET2:   rcall SWAPACC
LOOP_FP: rjmp REPACK
;** ACC1 + 0.5f **
FADp5:   ldi A2_H,0x3f
         clr A2_L
         clr A2_ML
         clr A2_MH
         rjmp FSUMA
;** ACC1 + (Z) **
FADDMEM: rcall FSRAMA2
;** ACC1 + ACC2 **
FSUMA:   rcall UNPACK
         cpi A2_H,0x80
         breq LOOP_FP
FADD1:   cpi A1_H,0x80
         breq FRET2
FADD2:   mov TEMPFP32,A1_H
         sub TEMPFP32,A2_H
         brvs LOOP_FP
         brpl FADD3
         rcall SWAPACC
         rjmp FADD2
FADD3:   cpi TEMPFP32,24
         brlo FADD3a
         clr A2_L
         clr A2_ML
         clr A2_MH
FADD3a:  cpi TEMPFP32,8
         brlo FADD3b
         mov A2_L,A2_ML
         mov A2_ML,A2_MH
         clr A2_MH
         subi TEMPFP32,8
         rjmp FADD3a
FADD3b:  tst TEMPFP32
         breq FADD3d
FADD3c:  lsr A2_MH
         ror A2_ML
         ror A2_L
         dec TEMPFP32
         brne FADD3c
FADD3d:  mov TEMPFP32,sign1
         eor TEMPFP32,sign2
         brmi FADD4
         rcall UADD
         brcc LOOP_FP
         ror A1_MH
         ror A1_ML
         ror A1_L
         subi A1_H,-1         ; Add +1
         brvc LOOP_FP
         rjmp MAXRES
FADD4:   rcall USUB
         breq FCLR
         brcc FADD5
         rcall NEGMANT
FADD5:   tst A1_MH
         brmi LOOP_FP
         lsl A1_L
         rol A1_ML
         rol A1_MH
         subi A1_H,1
         brvc FADD5
         rjmp MAXRES
FCLR:    rjmp MINRES
;
UADD:    add A1_L,A2_L
         adc A1_ML,A2_ML
         adc A1_MH,A2_MH
         ret
;
USUB:    sub A1_L,A2_L
         sbc A1_ML,A2_ML
         sbc A1_MH,A2_MH
         ret
;
;** ACC1 = -ACC1 ** Calcula el inverso aditivo
FLTNEG:  rcall UNPACK
         com sign1
         rjmp REPACK
;
NEGMANT: ldi TEMPFP32,-1
         sub A1_L,TEMPFP32
         sbc A1_ML,TEMPFP32
         sbc A1_MH,TEMPFP32
         ret
;
;** A1_L <<= 1 **
RLA1_L: lsl A1_L
         rol A1_ML
         rol A1_MH
         rol A1_Lt
         ret
;
UDIVMANT:clr quot
         clr quotm
         clr quoth
         ldi COUNTFP32,24
UDIVLP:  cp A1_L,A2_L
         cpc A1_ML,A2_ML
         cpc A1_MH,A2_MH
         brlo UDIV1
         sub A1_L,A2_L
         sbc A1_ML,A2_ML
         sbc A1_MH,A2_MH
         sec
         rjmp UDIV2
UDIV1:   clc
UDIV2:   rol quot
         rol quotm
         rol quoth
         lsl A1_L
         rol A1_ML
         rol A1_MH
         dec COUNTFP32
         brne UDIVLP
         push A1_L
         mov A1_L,quot
         mov A1_ML,quotm
         mov A1_MH,quoth
         pop TEMPFP32
         lsr TEMPFP32
         brcc udivx
         inc A1_L
         brne udivx
         inc A1_ML
         brne udivx
         inc A1_MH
udivx:   
         ret
;
;** ACC1 = 1/(Z) **
F1OVERMEM:rcall FSRAMA1
;** ACC1 = 1/ACC1 **
F1OVERX: rcall AC1TOAC2
         clr A1_L
         clr A1_ML
         ldi TEMPFP32,0x80
         mov A1_MH,TEMPFP32
         ldi A1_H,0x3f        ; ieee 1.0f
         rjmp F_DIV
;
;** Acc1 / (Z) **
FDIVMEM: rcall FSRAMA2
         rjmp F_DIV
;** ACC1 /= 10.0f */
FDIV_10: clr A2_L
         clr A2_ML
         ldi A2_MH,0x20
         ldi A2_H,0x41        ;Put 10.0 -> Acc2
;** ACC1 /= ACC2 **
F_DIV:   tst A2_H
         breq FDIVZ
         tst A1_H
         breq MINRES
FDIVa:   rcall UNPACK
         breq MINRES
         eor sign1,sign2
         sec
         sbc A1_H,A2_H
         brvs MAXRES
         lsr A2_MH
         ror A2_ML
         ror A2_L
         lsr A1_MH
         ror A1_ML
         ror A1_L
         rcall UDIVMANT
         tst A1_MH
         brmi FDIVX
         lsl A1_L
         rol A1_ML
         rol A1_MH
         subi A1_H,1
         brvs MAXRES
FDIVX:   rjmp REPACK
FDIVZ:   rjmp MINRES
MAXRES:  ldi TEMPFP32,0x7f
         mov A1_H,TEMPFP32
         or A1_MH,TEMPFP32
         ldi TEMPFP32,0xff
         mov A1_L,TEMPFP32
         mov A1_ML,TEMPFP32
         ret
MINRES:  clr A1_L
         clr A1_ML
         clr A1_MH
         clr A1_H
         clr sign1
         ret
;
;** ACC1 *= *r30 **
FMULMEM: rcall FSRAMA2        ; Acc1 *= *r30
         rjmp F_MUL
;** ACC1 *= 10.0f **
FMUL_10: ldi A2_H,0x41        ; IEEE 10.0f
         ldi A2_MH,0x20
         clr A2_ML
         clr A2_L
;** ACC1 *= ACC2 **
F_MUL:   rcall UNPACK
         breq MINRES
         cpi A2_H,0x80
         breq MINRES
         eor sign1,sign2
         sec
         adc A1_H,A2_H
         brvs MAXRES          ;FMULMAX
         rcall UMUL
         tst A1_MH
         brmi FXMULX
         lsl quoth
         rol A1_L
         rol A1_ML
         rol A1_MH
         rjmp REPACK
FXMULX:  subi A1_H,-1
         brvs MAXRES          ;FMULMAX
         rjmp REPACK
;
;** Multiplicacion sin signo de 24 bits
UMUL:    push TEMPFP32
         push COUNTFP32
         clr quot
         clr quotm
         clr quoth
         clr TEMPFP32
         ldi COUNTFP32,24
UMULLP:  lsl quot
         rol quotm
         rol quoth
         rol A1_L
         rol A1_ML
         rol A1_MH
         brcc UMUL1
         add quot,A2_L
         adc quotm,A2_ML
         adc quoth,A2_MH
         adc A1_L,TEMPFP32
         adc A1_ML,TEMPFP32
         adc A1_MH,TEMPFP32
UMUL1:   dec COUNTFP32
         brne UMULLP
         pop COUNTFP32
         pop TEMPFP32
         ret
;
;** ACC1 = (*r30) * (*r30) **
FSQRMEM: rcall FSRAMA1
;** ACC1 *= ACC1 **
F_SQR:   rcall AC1TOAC2
         rjmp F_MUL
;
sqrthalf:
         clr A2_L
         clr A2_ML
         clr A2_MH
         ldi A2_H,0x40
         rjmp FDIVa
SQERR:   rjmp MINRES
;** ACC1 = sqrt(*r30) **
FSQRTMEM:rcall FSRAMA1
;** ACC1 = sqrt(ACC1) **
F_SQRT:  tst A1_H
         breq SQERR
         brmi SQERR
         mov r5,A1_L
         mov r6,A1_ML
         mov r7,A1_MH
         mov r8,A1_H
         rcall sqrthalf
         rcall AC1TOAC2
         ldi TEMPFP32,5
SQRTLP:  push TEMPFP32
         rcall PUSH2
         mov A1_L,r5
         mov A1_ML,r6
         mov A1_MH,r7
         mov A1_H,r8
         rcall F_DIV
         rcall POP2
         rcall FSUMA
         rcall sqrthalf
         rcall AC1TOAC2
         pop TEMPFP32
         dec TEMPFP32
         brne SQRTLP
         ret
;
;** Compara ACC1 == ACC2 **
;** Regresa TEMPFP32 = 0,1,or -1 (Z,N flags set) **
FLTCPFLASH:rcall KTOAC2
         rjmp FLTCP
;** compara dos flotantes en sram **
FLTCP2:  push zl
         push zh
         mov zl,yl
         mov zh,yh
         rcall FSRAMA1
         pop zh
         pop zl
FLTCPMEM:rcall FSRAMA2
FLTCP:   rcall FCMP
         tst TEMPFP32
         ret

FCMP:    tst A1_H
         brmi FCMP1
         tst A2_H
         brmi A1GTA2
         cp A1_H,A2_H
         brlo A1LTA2
         brne A1GTA2
         cp A1_L,A2_L
         cpc A1_ML,A2_ML
         cpc A1_MH,A2_MH
         brlo A1LTA2
         breq A1EQA2
A1GTA2:  ldi TEMPFP32,1           ; ac1 > ac2
         ret
A1LTA2:  ldi TEMPFP32,0xff        ; ac1 < ac2
         ret
A1EQA2:  clr TEMPFP32             ; ac1 = ac2
         ret
;** acc1 -> '-' **
FCMP1:   tst A2_H             ; Si acc1 es '-' & acc2 is '+'
         brpl A1LTA2          ; entonces acc1 is < acc2
         cp A2_H,A1_H         ; si ambos son '-', es A1_H más negativo?
         brlo A1GTA2
         brne A1LTA2
         cp A2_L,A1_L
         cpc A2_ML,A1_ML
         cpc A2_MH,A1_MH
         brlo A1GTA2
         breq A1EQA2          ; return acc1 == acc2
         rjmp A1LTA2

;** convierte un entero a FP **
ITOFMEM: rcall FSRAMA1
         mov A1_Lt,A1_H
ITOF:    mov TEMPFP32,A1_L
         or TEMPFP32,A1_ML
         or TEMPFP32,A1_MH
         or TEMPFP32,A1_Lt
         brne ITOF1
         rjmp MINRES
ITOF1:   tst A1_Lt
         brpl ITOF2
         rcall NEGMANT
ITOF2:   ldi A1_H,30
         tst A1_Lt
ITOF3:   brmi ITOF4
         dec A1_H
         lsl A1_L
         rol A1_ML
         rol A1_MH
         rol A1_Lt
         rjmp ITOF3
ITOF4:   mov A1_L,A1_ML
         mov A1_ML,A1_MH
         mov A1_MH,A1_Lt
         rjmp REPACK

;********************************************************************
;** Copia SRAM Flotante -> Acc1 **
FSRAMA1: ld A1_L,z+
         ld A1_ML,z+
         ld A1_MH,z+
         ld A1_H,z+
         ret

;** Copia SRAM Flotante -> Acc2 **
FSRAMA2: ld A2_L,z+
         ld A2_ML,z+
         ld A2_MH,z+
         ld A2_H,z+
         ret

;** Copia ACC1 a SRAM
FSRAMA_1:st z+,A1_L
         st z+,A1_ML
         st z+,A1_MH
         st z+,A1_H
         ret
;
;** Copia ACC2 a SRAM
FSRAMA_2:st z+,A2_L
         st z+,A2_ML
         st z+,A2_MH
         st z+,A2_H
         ret
;
;** Copia ACC1 al ACC2 **
AC1TOAC2:
         mov A2_H,A1_H
         mov A2_L,A1_L
         mov A2_ML,A1_ML
         mov A2_MH,A1_MH
         mov sign2,sign1
         ret

AC2TOAC1:
         mov A1_H,A2_H
         mov A1_L,A2_L
         mov A1_ML,A2_ML
         mov A1_MH,A2_MH
         mov sign1,sign2
         ret
;
;** ACC1 <-> ACC2 **
SWAPACC: push A1_L
         push A1_ML
         push A1_MH
         push A1_H
         push sign1
         mov A1_L,A2_L
         mov A1_ML,A2_ML
         mov A1_MH,A2_MH
         mov A1_H,A2_H
         mov sign1,sign2
         pop sign2
         pop A2_H
         pop A2_MH
         pop A2_ML
         pop A2_L
         ret
;
;** Toma un byte de la memoria de programa e incrementa el apuntador **
_lpmbyte:outi	rampz,0x00
		 lpm
         adiw zl,1
         ret
;
;** Lee una constante de la Flash -> ACC1
KTOAC1:  rcall _lpmbyte
         mov A1_L,r0
         rcall _lpmbyte
         mov A1_ML,r0
         rcall _lpmbyte
         mov A1_MH,r0
         rcall _lpmbyte
         mov A1_H,r0
         ret
;
;** Lee una constante de la Flash -> ACC2
KTOAC2:  rcall _lpmbyte
         mov A2_L,r0
         rcall _lpmbyte
         mov A2_ML,r0
         rcall _lpmbyte
         mov A2_MH,r0
         rcall _lpmbyte
         mov A2_H,r0
         ret
;
;** Guarda Acc1 en sram **
PUSH1:   sts MATHTMP,A1_L
         sts MATHTMP+1,A1_ML
         sts MATHTMP+2,A1_MH
         sts MATHTMP+3,A1_H
         ret
;
;** Recupera el Acc1 de la sram **
POP1:    lds A1_L,MATHTMP
         lds A1_ML,MATHTMP+1
         lds A1_MH,MATHTMP+2
         lds A1_H,MATHTMP+3
         ret
;
;** Guarda Acc2 en sram **
PUSH2:   sts MATHTMP+4,A2_L
         sts MATHTMP+5,A2_ML
         sts MATHTMP+6,A2_MH
         sts MATHTMP+7,A2_H
         ret
;
;** Recupera el Acc2 de la sram **
POP2:    lds A2_L,MATHTMP+4
         lds A2_ML,MATHTMP+5
         lds A2_MH,MATHTMP+6
         lds A2_H,MATHTMP+7
         ret
;
;********************************************************************
;** IEEE convertions **
;** Convierte ACC1 & ACC2 del formato IEEE al formato de trabajo **
UNPACK:  push TEMPFP32
         mov sign1,A1_H
         ldi TEMPFP32,0x80
         lsl A1_MH
         rol A1_H
         eor A1_H,TEMPFP32
         lsl TEMPFP32
         ror A1_MH           ;Restaura el bit oculto '1'
         andi sign1,0x80
;** Desempaqueta Acc2 **
         mov sign2,A2_H
         ldi TEMPFP32,0x80
         lsl A2_MH
         rol A2_H
         eor A2_H,TEMPFP32
         lsl TEMPFP32
         ror A2_MH
         andi sign2,0x80
         pop TEMPFP32
         cpi A1_H,0x80        ; return w/ACC1=0 ?
         ret
;
;** Convierte Acc1 del formato de trabajo a IEEE **
REPACK:  push TEMPFP32
         lsl A1_MH 
         ldi TEMPFP32,0x80
         eor TEMPFP32,A1_H
         lsl sign1
         ror TEMPFP32
         ror A1_MH
         mov A1_H,TEMPFP32
         pop TEMPFP32
         ret

; Tabla de constantes de ejemplo

I1E6:    .db 0x40,0x42,0x0f,0 ; 1,000,000
I1E5:    .db 0xa0,0x86,0x01,0 ; 100,000
I1E4:    .db 0x10,0x27,0x00,0 ; 10,000
I1E3:    .db 0xe8,0x03,0x00,0 ; 1,000
I1E2:    .db 0x64,0x00,0x00,0 ; 100
I1E1:    .db 0x0a,0x00,0x00,0 ; 10
;
;** IEEE format **
PI:      .db 0xdb,0x0f,0x49,0x40 ; 3.1415927f
Kp1:     .db 0xCD,0xCC,0xCC,0x3D ; 0.1f
Kp5:     .db 0x00,0x00,0x00,0x3F ; 0.5f
K1:      .db 0x00,0x00,0x80,0x3F ; 1.0f
K10:     .db 0x00,0x00,0x20,0x41 ;10.0f
K1E7:    .db 0x80,0x96,0x18,0x4b
K1E8:    .db 0x20,0xbc,0xbe,0x4c

;************************************************************
CLR_DECIMAL:
			OUTI	DEC_H,0x00
			OUTI	DEC_MH,0x00
			OUTI	DEC_M,0x00
			OUTI	DEC_ML,0x00
			OUTI	DEC_L,0x00
			OUTI	FDEC_15,0x00
			OUTI	FDEC_14,0x00
			OUTI	FDEC_13,0x00
			OUTI	FDEC_12,0x00
			OUTI	FDEC_11,0x00
			OUTI	FDEC_10,0x00
			OUTI	FDEC_9,0x00
			OUTI	FDEC_8,0x00
			OUTI	FDEC_7,0x00
			OUTI	FDEC_6,0x00
			OUTI	FDEC_5,0x00
			OUTI	FDEC_4,0x00
			OUTI	FDEC_3,0x00
			OUTI	FDEC_2,0x00
			OUTI	FDEC_1,0x00
			OUTI	FDEC_0,0x00
			RET

;************************************************************
CLR_HEXADECIMAL:
			OUTI	HEX_H,0x00
			OUTI	HEX_MH,0x00
			OUTI	HEX_ML,0x00
			OUTI	HEX_L,0x00
			OUTI	FHEX_H,0x00
			OUTI	FHEX_MH,0x00
			OUTI	FHEX_ML,0x00
			OUTI	FHEX_L,0x00
			RET
;************************************************************
