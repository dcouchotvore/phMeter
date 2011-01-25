

; I/O space definitions


	.EQU	ADCL	= 0x04
	.EQU	ADCH	= 0x05
	.EQU	ADCSRA	= 0x06
	.EQU	ADCONV	= 6
	.EQU	ADMUX	= 0x07

	.EQU	PINB	= 0x16
	.EQU	DDRB	= 0x17
	.EQU	PORTB	= 0x18

	.EQU	EECR	= 0x1C
	.EQU	EEDR	= 0x1D
	.EQU	EEARL	= 0x1E
	.EQU	EEARH	= 0x0F
	.EQU	EERE	= 0
	.EQU	EEWE	= 1
	.EQU	EEMWE	= 2
	.EQU	EERIE	= 4

	.EQU	OCR1AL	= 0x2A
	.EQU	OCR1AH	= 0x2B
	.EQU	TCCR1B	= 0x2E
	.EQU	TCCR1A	= 0x2F

	.EQU	MCUCR	= 0x35

	.EQU	TIMSK	= 0x39
	.EQU	OCIE1A  = 4

	.EQU	SPL		= 0x3D
	.EQU	SPH		= 0x3E

; Register definitions

	.DEF	PSTATUS 	= R7				; Program status bits
	.EQU	PS_KEYCAP	= 0					; Keypad capture
	.EQU	PS_CAL		= 1					; Calibrating		@@@ this is redundant
	.EQU	PS_MV		= 2					; Display raw millivolts
	.EQU	PS_MANTEMP	= 3					; Using manually set temperature 
	.EQU	PS_DEGF		= 4					; Displey temperature in Fahrenheit

	.DEF	OP_MODE		= R6				; Operating mode
	.EQU	MODE_MASK	= 0x1F				; Lower 5 bits are measurement type: mV, pH, etc.
	.EQU	CAL_MASK	= 0xE0				; Upper 3 bits are calibration mode
	.EQU	CAL_PWR		= 0x20
	.EQU	CAL_NONE	= 0x00
	.EQU	CAL_ISE		= 0x20
	.EQU	CAL_TEMP	= 0x40
	.EQU	CAL_TMODE	= 0x50

	.DEF	XL			= R26
	.DEF	XH			= R27
	.DEF	YL			= R28
	.DEF	YH			= R29
	.DEF	ZL			= R30
	.DEF	ZH			= R31

; Key definitions
	.EQU	KEY_NONE	= 0
	.EQU	KEY_MODE	= 1
	.EQU	KEY_CAL		= 2
	.EQU	KEY_DOWN	= 3
	.EQU	KEY_UP		= 4
	.EQU	KEY_ENTER	= 5
