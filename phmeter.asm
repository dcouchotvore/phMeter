

			.INCLUDE "m8535def.inc"


//			.DEVICE		ATMEGA8353
			.CSEGSIZE	16


			.DSEG
			.ORG		0x60
DataStart:

			.CSEG

			.ORG			0


			.EQU		MOSI	= 6				; SPI Interface
			.EQU		SCK		= 7

			.EQU		CSREG = PINC			; Put Chip selects on port C
			.EQU		A2DBUSY = 6				; Put A/D busy on pin 6			;
			.EQU		CS_A2D	= 0b00000001	; A/D chip select bit, active low
			.EQU		CS_D2A	= 0b00000010	; D/A chip select bit, active low
			.EQU		CS_RTC	= 0b10000100	; Real-time clock chip select bit, active high


			; RTC Register read addresses; write addresses are 0x80 more

			.EQU		RTC_SEC		= 0x00
			.EQU		RTC_MIN		= 0x01
			.EQU		RTC_HOURS	= 0x02
			.EQU		RTC_DAY		= 0x03
			.EQU		RTC_DATE	= 0x04
			.EQU		RTC_MONTH	= 0x05
			.EQU		RTC_YEAR	= 0x06
			.EQU		RTC_CTRL	= 0x0F
			.EQU		RTC_STATUS	= 0x10

			.EQU		ASC_SO		= 0x0E
			.EQU		ASC_SI		= 0x0F

;=================================================================================================
; Interrupt vectors
;
; All must be one-word op code
			
			RJMP	Initialize					; Reset
			RJMP	ProcessLoop					; Ext INT 0
			RETI								; Ext INT 1
			RETI								; TC2 Compare Match
			RETI								; TC2 Overflow
			RETI								; TC1 Capture Event
			RETI								; TC1 Compare Match A
			RETI								; TC1 Compare Match B
			RETI								; TC1 Overflow
			RETI								; TCO Overflow
			RETI								; USART Rx Complete
			RETI								; USART Data Register Empty
			RETI								; USART Tx Complete
			RETI								; A/D Complete	
			RETI								; EEPROM Ready
			RETI								; Analog Comparator
			RETI								; TWI
			RETI								; Ext INT 2
			RETI								; TC0 Compare Match
			RETI								; Store Program Memory Ready
			
			.ORG	22							; End of interrupt vectors
			
DoNothing:
			RETI

;=================================================================================================
; Initialize processor and perpherials


Initialize:
			LDI		R16, HIGH(RamEnd)			; Set stack pointer to top of RAM
			OUT		SPH, R16
			LDI		R16, LOW(RamEnd)
			OUT		SPL, R16

			IN		R16, MCUCSR					; If caused by watchdog timer, skip warmup sequence
			SBRS	R16, WDRF
			RCALL	PowerUp

			SUB		R16, R16
			OUT		DDRA, R16					; Set port A to watch for buttons pressed
			LDI		R17, 0x3F
			OUT		PORTA, R17					; Enable pullup resisters for six pins
			
			; The SPI talks to the A/D, the D/A and the real-time clock.

			LDI		R17,(1<<MOSI)|(1<<SCK)		; Set MOSI and SCK output, all others input
			OUT		DDRA, R17
			LDI		R17,(1<<SPE)|(1<<MSTR)|(1<<SPR0); Enable SPI, Master, set clock rate fck/16
			OUT		SPCR, R17

			LDI		R17, RTC_CTRL
			LDI		R16, 0b10000000				; Enable RTC oscillator
			RCALL	RTCWrite
			LDI		R17, RTC_HOURS
			RCALL	RTCRead
			ORI		R16, 0b01000000				; Make sure clock is in 24-hour mode.  No option for 12-hour
			RCALL	RTCWrite
			

			LDI		R17, 0xF					; Initialize the watchdog timer for about 2 seconds
			OUT		WDTCR, R16

			IN		R16, MCUCSR					; If caused by watchdog timer, skip A/D cal sequence...
			SBRS	R16, WDRF
			RCALL	CalibrateA2D

			SUB		R17, R17					; ... and zeroing recorder output
			SUB		R16, R16
			RCALL	DACWrite

IdleReady:
			SUB		R16, R16
			OUT		MCUCR, R16					; Set sleep mode to IDLE

			STS		LastA2DMode, R16			; Start A/D running.  Take a temperature measurement
			RCALL	ExchangeA2D					; so the first signal measurement has the temperature.

IdleLoop:			
			SEI									; Enable interrupts and wait for something to happen
			SLEEP
			RJMP	IdleLoop

;-------------------------------------------------------------------------------------------------
; The warmup sequence

PowerUp:
			CALL	LCDClear


			; Clear RAM

			LDI		XH, HIGH(DataStart)
			LDI		XL, LOW(DataStart)
			LDI		YH, HIGH(DataEnd-DataStart)
			LDI		YL, LOW(DataEnd-DataStart)
			SUB		R16, R16
ClearRAMLoop:
			ST		X+, R16
			SBIW	YH:YL, 1
			BRNE	ClearRAMLoop

			LDI		R20, HIGH(FilterConstants)	; Save filter constants to RAM from EEPROM
			LDI		R19, LOW(FilterConstants)
			LDI		YH, HIGH(SignalFilter+FilterConstant)
			LDI		YL, LOW(SignalFilter+FilterConstant)
			RCALL	ReadEEPROM
			ST		Y,	R18
			LDI		YH, HIGH(TemperatureFilter+FilterConstant)
			LDI		YL, LOW(TemperatureFilter+FilterConstant)
			RCALL	ReadEEPROM
			ST		Y,	R18
			LDI		YH, HIGH(CalibrationFilter+FilterConstant)
			LDI		YL, LOW(CalibrationFilter+FilterConstant)
			RCALL	ReadEEPROM
			ST		Y,	R18


			LDI		R16, 1						; Default to pH mode
			RCALL	ModeChangeSave
			RET

;-------------------------------------------------------------------------------------------------
; Exchange the 16 bits in R3:R2 with the SPI peripheral selected by bit mask in R16

SPIInterface16:
			RCALL	EnableSPIDevice
			OUT		SPDR, R3					; Start transmission of data (r16)
SPI16Wait1:
			SBIS	SPSR, SPIF					; Wait for transmission complete
			RJMP	SPI16Wait1
			IN		R3, SPDR
			OUT		SPDR, R2
SPI16Wait2:
			SBIS	SPSR, SPIF					; Wait for transmission complete
			RJMP	SPI16Wait2
			IN		R2, SPDR
			RCALL	DisableSPIDevice			; Disable the chip 
			RET

;-------------------------------------------------------------------------------------------------
; Enable/disable SPI chip select.  Lower few bits of R16 select which chip select.  Bit 7 is 0 for active
; low and 1 for active high.  Bit 7 of output port will toggle

EnableSPIDevice:
			IN		R1, CSREG
			TST		R16
			BRGE	EnableSPIActiveLow
			OR		R1, R16
			RJMP	EnableSPIOut
EnableSPIActiveLow:
			COM		R16
			AND		R1, R16
			COM		R16
EnableSPIOut:
			OUT		CSREG, R1					; Set the chip select
			RET

DisableSPIDevice:
			IN		R1, CSREG
			TST		R16
			BRGE	DisableSPIActiveLow
			COM		R16
			AND		R1, R16
			COM		R16
			RJMP	EnableSPIOut
DisableSPIActiveLow:
			OR		R1, R16
DisableSPIOut:
			OUT		CSREG, R1					; Set the chip select
			RET

;-------------------------------------------------------------------------------------------------
; Perform A2D calibration.  Absolute accuracy is not an issue except for MV because we have our
; own cal tables.  Repeatability, however, is.  Calibration improves temperature drift.

CalibrateA2D:
			LDI		R19, 0b10000110
			LDI		R18, 0b00001100				; Offset correction
			RCALL	A2DWithWait
			LDI		R18, 0b00001000				; Gain correction
			RCALL	A2DWithWait
			LDI		R18, 0b00000100				; Null offset
			RCALL	A2DWithWait
			RET

;-------------------------------------------------------------------------------------------------
; Execute an A/D process with 16-bit data in R19:R18, waiting for the results.  Intended for A/D cal
; sequence that occurs with interrupts disabled.  Does not return result.

A2DWithWait:
			MOV		R3, R19
			MOV		R2, R18
			RCALL	SPIInterface16
A2DWithWaitLoop:
			SBIS	CSREG, A2DBUSY
			RJMP	A2DWithWaitLoop
			RET

;-------------------------------------------------------------------------------------------------
; Read/Write R16 from/to RTC register specified by R17, retaining R17

RTCWrite:
			ORI		R17, 0x80					; Convert to write address
			MOV		R3, R17
			ANDI	R17, 0x7F
			MOV		R2, R16
			LDI		R16, CS_RTC					; RTC chip select
			RCALL	SPIInterface16
			RET

RTCRead:
			MOV		R3, R17
			MOV		R2, R16
			LDI		R16, CS_RTC					; RTC chip select
			RCALL	SPIInterface16
			MOV		R16, R2
			RET

;-------------------------------------------------------------------------------------------------
; Write R17:R16 to DAC

DACWrite:
			MOV		R3, R17
			MOV		R2, R16
			LDI		R16, CS_D2A
			RCALL	SPIInterface16
			RET

;=================================================================================================
; Signal Processing

;-------------------------------------------------------------------------------------------------
; Data for signal processing

			.DSEG

			; Define offsets into filter structure

			.EQU	FilterInput  	= 0
			.EQU	FilterConstant  = 2		; Must be a power of two for division by shift
			.EQU	FilterSpare		= 3
			.EQU	FilterAccum		= 4
			.EQU	FilterFloat		= 8
			.EQU	FilterSize		= 12

SignalFilter: 	
			.BYTE	FilterSize
SignalFilterEnd:

TemperatureFilter:
			.BYTE	FilterSize
TemperatureFilterEnd:

CalibrationFilter:
			.BYTE	FilterSize
CalibrationFilterEnd:


; RAM-based cal table for current ion

CalTable:	.BYTE	24

; RAM-based math constants, variables

DegF:		.DD		0
ITemp:		.DD		0

; Operational variables

LastA2DMode:
			.DB		0
RawCalSignal:
			.DD		0
TempComp:
			.DD		0
ReportInterval:
			.DB		0

			.CSEG

;-------------------------------------------------------------------------------------------------
; Execute a cycle of the process loop
; Have to complete this in about 64ms

ProcessLoop:

			; Get Last A/D measurement and put it the right place.  Start next one.
			
			RCALL	ExchangeA2D					; Get Last A/D measurement; start next one
			LDI		R16, LastA2DMode			; Put results into the correct filter
			TST		R16
			BRNE	ProcessSaveTemperature
			RCALL	SignalProcessor
			LDI		R16, 1						; Waiting for temperature reading
			RJMP	ProcessSaveState

ProcessSaveTemperature:
			LDI		ZH, HIGH(TemperatureFilter)
			LDI		ZL, LOW(TemperatureFilter)
			RCALL	ProcessSignal
			SUB		R16, R16					; Waiting for signal reading

ProcessSaveState:
			STS		LastA2DMode, R16

			RCALL	ParseUserInput				; Now to everything else
			RCALL	UpdateDisplay
			RETI

;-------------------------------------------------------------------------------------------------
; A single cycle of the signal processing

SignalProcessor:
			LDI		ZH, HIGH(SignalFilter)		; Run the signal filter
			LDI		ZL, LOW(SignalFilter)
			RCALL	ProcessSignal
 
			LDS		R20, OpMode
			TST		R20							; If in mV mode, skip cal and TC. 
			BREQ	PrepareMV					; ??? Might want to add mV cal tables, but need references

			LDS		R20, CalMode				; Are we in calibration mode?
			TST		R20
			BREQ	CheckTC	
			PUSH	ZH
			PUSH	ZL
			LDI		ZH, HIGH(CalibrationFilter)	; The calibration filter runs off the ouput
			LDI		ZH, LOW(CalibrationFilter)	; of the signal filter, and so is a 2-pole
			STD		Z+FilterInput+0, R16		; filter.  
			STD		Z+FilterInput+1, R17
			RCALL	RunFilter
			ADIW	ZH:ZL, FilterFloat			; Keep copy of uncorrected value for insertion
			LDI		YH, HIGH(RawCalSignal)		; into cal tables
			LDI		YL, LOW(RawCalSignal)
			RCALL	Move4ZtoY
			POP		ZL
			POP		ZH

CheckTC:
			LDS		R16, ProcessStatus
			SBRS	R16, PS_MANTEMP
			RJMP	ApplyTC

			PUSH	ZH							; If not using manual temperature compensation,
			PUSH	ZL							; move output of temperature filter to TempComp
			LDI		ZH, HIGH(TemperatureFilter)
			LDI		ZL, LOW(TemperatureFilter)
			LDI		YH, HIGH(TempComp)
			LDI		YL, LOW(TempComp)
			RCALL	Move4ZtoY
			POP		ZL
			POP		ZH

ApplyTC:
			RCALL	ConvertToNominal
			RJMP	DisplayOutput

PrepareMV:
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, HIGH(SignalFilter+FilterFloat)
			RCALL	FSRAMA1
			LDI		ZH, HIGH(CA2DMax)
			LDI		ZL,	LOW(CA2DMax)
			RCALL	FMULMEM
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, HIGH(SignalFilter+FilterFloat)
			RCALL	MSave1ToMem

DisplayOutput:
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, HIGH(SignalFilter+FilterFloat)
			LDI		YH, HIGH(ReadoutBuf)
			LDI		YL, LOW(ReadoutBuf)
			RCALL	Write3Point2

			LDI		ZH, HIGH(TemperatureFilter+FilterFloat)
			LDI		ZL, LOW(TemperatureFilter+FilterFloat)
			LDs		R16, ProcessStatus
			BST		R16, PS_DEGF				; Convert to Fahrenheit if requested.
			BRTC	DisplayTemperature			; I'm not sure why I bothered with this
			RCALL	FSRAMA1
			LDI		ZH, HIGH(C5over9)
			LDI		ZL, LOW(C5over9)
			RCALL	FMULMEM
			LDI		ZH, HIGH(C32)
			LDI		ZH, LOW(C32)
			RCALL	FADDMEM
			LDI		ZH, HIGH(DegF)
			LDI		ZL, LOW(DegF)
			RCALL	MSave1ToMem			

DisplayTemperature:
			LDI		YH, HIGH(LineBuffer+TempOffset)
			LDI		YH, LOW(LineBuffer+TempOffset)
			RCALL	Write3Point2

			RCALL	RecorderOutput

			RET

;-------------------------------------------------------------------------------------------------
; Process a given input (signal or temperature) based on filter at ZH:ZL and  
; cal table at XH:ZL

ProcessSignal:
			RCALL	RunFilter
			RCALL	ConvertSignalToFloat
			RCALL	FitCalibration
			RET

;-------------------------------------------------------------------------------------------------
; Run n-measurement filter indicated by Z register  By convention, Z points to a filter structure

RunFilter:
			MOV		YH, ZH						; X -> new filter entry
			MOV		YL, ZL
			LDD		R16, Z+FilterConstant
			MOV		R0, R16
			ADIW	ZH:ZL, FilterAccum			; Accumulator carries filter average times filter constant n
			LD		R16, Z						; We want new signal + average *(n-1)
			LD		R20, Z+
			LD		R17, Z						; acc = avg * n so avg*(n-1) = acc/n * (n-1) = acc - acc/n
			LD		R21, Z+						;
			LD		R18, Z
			LD		R22, Z+
			LD		R19, Z
			LD		R23, Z+

			SUB		R1, R1
			DEC		R1
RunFilterFindN:									; Get N in R1
			INC		R1
			LSR		R0
			BRCC	RunFilterFindN

RunFilterDivByN:								; Div Accumulator value in R23::R20 by N
			LSR		R23		
			ROR		R22
			ROR		R21
			ROR		R20
			DEC		R1
			BRNE	RunFilterDivByN

			SUB		R16, R20					; Subtract acc/n from acc
			SBC		R17, R21
			SBC		R18, R22
			SBC		R19, R23

			MOV		ZH, YH						; Add new measurement to accumulator
			MOV		ZL, YL
			ADIW	ZH:ZL, FilterInput
			LD		R20, Z+
			ADD		R16, R20
			LD		R20, Z+
			ADC		R17, R20
			SUB		R20, R20
			ADC		R18, R20
			ADC		R19, R20

			MOV		ZH, YH						; Save new result
			MOV		ZL, YL
			ADIW	ZH:ZL, FilterAccum
			ST		Z+, R16
			ST		Z+, R17
			ST		Z+, R18
			ST		Z+, R19
			MOV		ZH, YH
			MOV		ZL, YL
			RET

;-------------------------------------------------------------------------------------------------
; Convert the fixed value in A/D bits in the Filter Structure @Z to floating point in Z->FilterFloat


ConvertSignalToFloat:
			MOV		YH, ZH
			MOV		YL, ZL
			LDD		R21, Z+FilterConstant		; R21 = filter constant
			SUB		R22, R22					; R22 = 0
			LDD		R16, Z+0					; Get accumulator
			LDD		R17, Z+1
			LDD		R18, Z+2
			LDD		R19, Z+3
ConvertSignalScale:								; Divide it by filter constant
			LSR		R21
			BRCS	ConvertSignalScaled
			LSR		R19
			ROR		R18
			ROR		R17
			ROR		R16
			RJMP	ConvertSignalScale
ConvertSignalScaled:
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(Itemp)
			STD		Z+0, R16					; Save it to ITemp
			STD		Z+1, R17
			STD		Z+2, R18
			STD		Z+3, R20
			RCALL	ITOFMEM						; Convert to float
			MOV		ZH, YH
			MOV		ZL, YL
			ADIW	ZH:ZL, FilterAccum
			RCALL	MSave1ToMem					; Save float value.
			MOV		ZH, YH
			MOV		ZL, YL
			RET

;-------------------------------------------------------------------------------------------------
; Temperature compensation routines.  Act on result value in filter structure @Z
/*
ConvertToStandardized:
			PUSH	ZH
			PUSH	ZL
			LDI		ZH, HIGH(TemperatureFilter+FilterFloat)
			LDI		ZL, LOW(TemperatureFIlter+FilterFloat)
			RCALL	FSRAMA1						; Measured Temperature
			LDI		ZH, HIGH(C273)
			LDI		ZL, LOW(C273)
			RCALL	FADDROM						; Convert to absolute
			LDI		ZH, HIGH(C298)
			LDI		ZL, LOW(C298)
			RCALL	FDIVROM						; Divide by std temp absolute
			POP		ZL
			POP		ZH
			PUSH	ZH
			PUSH	ZL
			ADIW	ZH:ZL, FilterFloat
			RCALL	FMULMEM						; Multiply by measured value
			POP		ZL
			POP		ZL
			ADIW	ZH:ZL, FilterFloat
			RCALL	MSave1ToMem
			RET
*/
ConvertToNominal:
			PUSH	ZH
			PUSH	ZL
			LDI		ZH, HIGH(TemperatureFilter+FilterFloat)
			LDI		ZL, LOW(TemperatureFIlter+FilterFloat)
			RCALL	FSRAMA1						; Measured Temperature
			LDI		ZH, HIGH(C273)
			LDI		ZL, LOW(C273)
			RCALL	FADDMEM						; Convert to absolute
			POP		ZL
			POP		ZH
			PUSH	ZH
			PUSH	ZL
			ADIW	ZH:ZL, FilterFloat
			RCALL	MSave1ToMem					; Invert
			RCALL	F1OVERMEM
			LDI		ZH, HIGH(C298)
			LDI		ZL, LOW(C298)
			RCALL	FMULMEM						; Mult by std absolute
			POP		ZL
			POP		ZH
			PUSH	ZH
			PUSH	ZL
			ADIW	ZH:ZL, FilterFloat
			RCALL	FMULMEM						; Multiply by standardized value
			POP		ZL
			POP		ZL
			ADIW	ZH:ZL, FilterFloat
			RCALL	MSave1ToMem
			RET

;-------------------------------------------------------------------------------------------------
; Calibrate signal in specified structure against calibration table @X

FitCalibration:
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, LOW(SignalFilter+FilterFloat)
			RCALL	FSRAMA1
			MOV		ZH, XH
			MOV		ZL, XL
			ADIW	Z, 8
			RCALL	FLTCPMEM					; Determine which leg of cal table to use
			BRGE	StartFit
			SBIW	Z, 8

StartFit:
			ADIW	Z, 8
			RCALL	FSRAMA1
			SBIW	Z, 8
			RCALL	FSUBMEM						; x1-x0
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	MSave1ToMem
			MOV		ZH, XH
			MOV		ZL, XL
			ADIW	Z, 12
			RCALL	FSRAMA1
			SBIW	Z, 8
			RCALL	FSUBMEM						; y1-y0
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	FDIVMEM						; (y1-y0)/(x1-x0)
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	MSave1ToMem
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, LOW(SignalFilter+FilterFloat)
			RCALL	FSRAMA1
			MOV		ZH, XH
			MOV		ZL, XL
			RCALL	FSUBMEM						; x-x0
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	FMULMEM						; (x-x0)(y1-y0)/(x1-x0)
			MOV		ZH, XH
			MOV		ZL, XL
			ADIW	Z, 4
			RCALL	FMULMEM						; y=y0(x-x0)(y1-y0)/(x1-x0)
			LDI		ZH, HIGH(SignalFilter+FilterFloat)
			LDI		ZL, HIGH(SignalFilter+FilterFloat)
			RCALL	MSave1ToMem
			RET

;-------------------------------------------------------------------------------------------------
; Get the results of the last A/D conversion in R3:R2 and start the the next one, chosing the 
; other port from last time.  LastA2DMode is 0 for signal, 1 for temp, corresponding to A/D channel

ExchangeA2D:
			PUSH	R16
			LDI		R16, LastA2DMode
			INC		R16
			CPI		R16, 2
			BRLO	ExchangeA2DInstr
			SUB		R16, R16
ExchangeA2DInstr:
			LSL		R16
			LSL		R16
			LSL		R16
			LSL		R16
			MOV		R2, R16
			LDI		R16, 0b10000110
			MOV		R3, R16
			LDI		R16, CS_A2D
			RCALL	SPIInterface16
			POP		R16
			RET

;=================================================================================================
; Process keypresses and dispatch accordingly

;-------------------------------------------------------------------------------------------------
; Input and state variables

				.DSEG
LastScan:		.DB		0
CapturedKey:	.DB		0
ProcessStatus:	.DB		0
				.EQU	PS_KEYCAP	= 0			; Keypad capture
				.EQU	PS_MV		= 2			; Display raw millivolts
				.EQU	PS_MANTEMP	= 3			; Using manually set temperature 
				.EQU	PS_DEGF		= 4			; Displey temperature in Fahrenheit

OpMode:			.DB		0
CalMode:		.DB		0
CalPoint:		.DB		1						; Which calibration point we are setting							
InitCal:		.DB		1						; Calibration needs to be initialized


; Key definitions
				.EQU	KEY_NONE	= 0
				.EQU	KEY_MODE	= 1
				.EQU	KEY_CAL		= 2
				.EQU	KEY_DOWN	= 3
				.EQU	KEY_UP		= 4
				.EQU	KEY_ENTER	= 5

				.CSEG

KeyDispatchTable:
				.DW		ModeChange
				.DW		CalModeChange
				.DW		ParseKeyDone
				.DW		ParseKeyDone
				.DW		ParseKeyDone

CalDispatchTable:
				.DW		ParseKeyDone			; Nothing
				.DW		CalISEpH				; Calibrate current ion
				.DW		CalTemp					; Calibrate temperature probe
				.DW		SetTempMode				; Set Temperature mode
				.DW		SetTempDisplay			; Set temperature display mode C/F
				.DW		SetReportInterval		; Set report interval
				.DW		SetAnalogOut			; Set analog output mode
				.DW		SetClock				; Set clock
				.DW		ParseKeyDone			; Set baud rate
CalDispatchTableEnd:

				.EQU	NumCalModes	= (CalDispatchTableEnd-CalDispatchTable)/2

;-------------------------------------------------------------------------------------------------
; Parse the keys

ParseUserInput:
			IN		R16, PINB
			NEG		R16
			LDI		R17, 6
			SUB		R18, R18
			STS		CapturedKey, R18			; No key captured yet
ParseKeyNextBit:
			INC		R18
			LSR		R16							; Take the first bit found as the pressed key
			BRCS	ParseKeyFound
			DEC		R17
			BRNE	ParseKeyNextBit
			STS		LastScan, R17				; If none found, save nothing and get out.
			RJMP	ParseKeyDone

ParseKeyFound:
			LDS		R19, LastScan				; Must match last scan for 25ms debounce time
			STS		LastScan, R18
			CP		R19, R18
			BRNE	ParseKeyDone
			LDI		R16, ProcessStatus
			BST		R16, PS_KEYCAP				; If key capture is set, save it
			BRTC	ParseKeyDispatch
			STS		CapturedKey, R18

			LDS		R16, CalMode				; If none-zero cal mode, dispatch to correct routine
			BREQ	ParseKeyDone
			LSR		R16
			LSR		R16
			LSR		R16
			LSR		R16
			LDI		ZH, HIGH(CalDispatchTable)
			LDI		ZL, LOW(CalDispatchTable)
			SUB		R17, R17
			ADD		ZL, R16
			ADC		ZH, R17
			IJMP

ParseKeyDispatch:
			DEC		R18							; Generate offset into jump table
			LSL		R18
			LDI		ZH, HIGH(KeyDispatchTable)
			LDI		ZL, LOW(KeyDispatchTable)
			SUB		R17, R17
			ADD		ZL, R18
			ADC		ZH,	R17
			LSL		ZL
			ROL		ZH
			LPM		XL, Z+
			LPM		XH, Z+
			MOV		ZH, XH
			MOV		ZL, XL
			IJMP

ParseKeyDone:
			RET

;-------------------------------------------------------------------------------------------------
; The mode button has been pressed

ModeChange:
			LDS		R16, OpMode					; Increment mode.  
			INC		R16
			CPI		R16, NumCalEntries
			BRLO	ModeChangeSave
			SUB		R16, R16

ModeChangeSave:
			STS		OpMode, R16
			LDI		R16, ModeOffset
			LDI		R17, ModeLength
			RCALL	ClearDisplayBuffer			; Clear line 1
			LDI		ZH, HIGH(Str_Mode)
			LDI		ZL,	LOW(Str_Mode)
			RCALL	WriteCString				; Write "Mode:" to first line
			LSL		R16
			LSL		R16
			SUB		R17, R17
			LDI		ZH, HIGH(ModeTableStart)	; Point Z to string defining mode
			LDI		ZL, LOW(ModeTableStart)
			ADD		ZL, R16
			ADC		ZH, R17
			LSL		ZL
			ROL		ZH
			LPM		R16, Z+
			LPM		R17, Z+
			MOV		ZH, R16
			MOV		ZL, R17
			INC		R16
			RCALL	WriteCString				; Write mode to display

			LDI		ZH, HIGH(CalTable)			; Load corresponding cal table from EEPROM to RAM
			LDI		ZL, LOW(CalTable)
			LDI		R21, 6						; Six entries to copy
			LDI		R16, OpMode
			LDI		R17, 6						; Six entries per cal table
			MUL		R16, R17					; R16 indexes into EEPROM table
			MOV		R16, R0
LoadCalTableLoop:
			RCALL	LoadCalibrationValue
			INC		R16
			DEC		R21
			BRNE	LoadCalTableLoop			
			RET

;-------------------------------------------------------------------------------------------------
; The calibration mode button has been pressed

CalModeChange:
			LDS		R16, CalMode				; Increment mode.
			INC		R16
			CPI		R16, NumCalModes
			BRLO	CalModeChangeSave
			SUB		R16, R16
CalModeChangeSave:
			STS		CalMode, R16
			TST		R16							; If active calibration mode, capture keys
			BREQ	CalModeChangeDisplay		; And flag to initialize cal code
			LDS		R16, ProcessStatus
			ORI		R16, 1<<PS_KEYCAP
			STS		ProcessStatus, R16
			LDI		R16, 1						; Default to middle calibration point
			STS		CalPoint, R16				; Main keyboard loop will dispatch to proper routine
			STS		InitCal, R16				; Flag to initialize calibration process
CalModeChangeDisplay:
			LDI		R16, FuncOffset				; Clear the function line
			LDI		R17, FuncLength
			RCALL	ClearDisplayBuffer

			LDS		R16, CalMode				; Update the button legends
			TST		R16
			BREQ	CalModeChangeDone
			LSR		R16
			LSR		R16
			LSR		R16
			LSR		R16
			LDI		YH, HIGH(CalDispatchTable)
			LDI		YL, LOW(CalDispatchTable)
			SUB		R17, R17
			ADD		YL, R16
			ADC		YH, R17
			LDD		ZL, Y+0
			LDD		ZH, Y+1
			SBIW	ZH:ZL, 6					; Legend table lies 6 bytes below entry point
			LDI		R17, 3						; Update three button legends...
			LDI		R16, 2						; ... starting at offset 2
CalModeChangeUpdateLegend:
			RCALL	WriteButtonLegend
			ADIW	ZH:ZL, 2
			INC		R16
			DEC		R17
			BRNE	CalModeChangeUpdateLegend
CalModeChangeDone:
			RET

;-------------------------------------------------------------------------------------------------
; Calibration tables maintained in EEPROM

			.ESEG

CalTableStart:
TempCalTable:
			.DD		0xC57FF000,	0xC2C80000		; -4095 -> -100 degrees not avail, but here for table symmetry
			.DD		0x00000000,	0x00000000		; 0 -> 0 degrees C
			.DD		0x457FF000,	0x42C80000		; 4095 -> 100 degrees C
pHCalTable:
			.DD		0x44723999,	0x40800000		; 968.9 -> 	pH 4.0  = .2366V
			.DD		0x44D40666,	0x40E00000		; 1696.2 -> pH 7.0  = .4141V
			.DD		0x45178CCC,	0x41200000		; 2424.8 -> pH 10.0 = .5920V

			; ISE cal tables go here

CalTableEnd:

			.EQU	NumCalEntries = (CalTableEnd-CalTableStart)/(phCalTable-TempCalTable)

FilterConstants:
SignalFilterConstants:	.DB		16
TempFilterConstant:		.DB		128
CalFilterConstant:		.DB		32

;-------------------------------------------------------------------------------------------------
; Mode specifiers  and constants stored in program memory

				.CSEG

ModeTableStart:
				.DW		Str_MV, 0, 0, 0
ModeTableEntryMark:
				.DW		Str_pH, Str_4, Str_7, Str_10
ModeTableEnd:

Str_MV:			.DB		"mV ", 0
Str_pH:			.DB		"pH ", 0

Str_4:			.DB		"4.0", 0
Str_7:			.DB		"7.0", 0
Str_10:			.DB		"10.0", 0, 0
Str_0C:			.DB		"0.0°C", 0
Str_100C:		.DB		"100.0°C", 0


Str_Mode:		.DB		"Mode: "				; Terminated by following null string
Str_Null:		.DB		0, 0
Str_Cal:		.DB		"Calibrate ", 0, 0
Str_TCMode:		.DB		"TC Mode: ", 0
Str_Auto:		.DB		"Auto ", 0
Str_Manual:		.DB		"Manual ", 0
Str_Left:		.DB		0x80, 0					; Left arrow
Str_Right:		.DB		0x81, 0					; Right arrow
Str_Up:			.DB		0x82, 0					; Up arrow
Str_Down:		.DB		0x83, 0					; Down arrow
Str_Sel:		.DB		"Sel", 0
Str_Save:		.DB		"Save", 0, 0
Str_DegC:		.DB		"°C", 0, 0
Str_DegF:		.DB		"°F", 0, 0
Str_Off:		.DB		"Off", 0

ConstantsStart:
C273:			.DD		0x43889333				; 273.15
C298:			.DD		0x43951333				; 298.15
C5over9:		.DD		0x3F0E38E3				; 5/9
CA2DMax:		.DD		0x4083126F				; 4.096				A/D scaling
C0:				.DD		0x00000000				; 0
C1:				.DD		0x3F800000				; 1.0
Cp1:			.DD		0x3F000000				; 0.1
Cpm1:			.DD		0xBF000000				; -0.1
C10:			.DD		0x41200000				; 10.0
C32:			.DD		0x42000000				; 32.0
ConstantsEnd:

;-------------------------------------------------------------------------------------------------
; Common way out of most cal routines

CalRestoreMode:
			LDS		R16, ProcessStatus			; Turn off  key capture
			ANDI	R16, !(1<<PS_KEYCAP)
			STS		ProcessStatus, R16
			SUB		R16, R16					; Reset cal mode
			STS		CalMode, R16
			RCALL	ModeChangeSave				; Reinitialize for normal operation
			LDI		R16, 1
			STS		InitCal, R16				; Init cal on next calibration
			LDI		R16, FuncOffset				; Clear the function line
			LDI		R17, FuncLength
			RCALL	ClearDisplayBuffer

CalDone:		
			RET

;-------------------------------------------------------------------------------------------------
; Return T bit set if calibration init required

CheckCalInit:
			SUB		R17, R17
			LDS		R19, InitCal				; Clear calibration init flag, but if set before,
			STS		InitCal, R17				; do the initialization stuff
			BST		R19, 0						; Set test bit to value of LSB
			RET

;-------------------------------------------------------------------------------------------------
; Dispatch to the proper destination point specified by the captured key and the jump table @X,
; or to CalDone if there is no captured key pending.  If CapturedKey is invalid, will jump to destruction.

DispatchToCalKeyDestination:
			LDS		R16, CapturedKey
			SUB		R17, R17
			STS		CapturedKey, R17
			TST		R16
			BREQ	CalDone
			DEC		R16
			LSR		R16
			LSR		R16
			LSR		R16
			LSR		R16
			SUB		R17, R17
			ADD		ZL, R16
			ADC		ZH, R17
			IJMP

;-------------------------------------------------------------------------------------------------
; Calibrate current pH/ISE selection

CalISEpHJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		CalISEpHDown				; Button 2 goes to the next lower cal point.
			.DW		CalISEpHUp					; Button 3 goes to the next higher cal point
			.DW		CalISEpHSaveCal				; Button 4 saves the cal point

CalISELegendTable:
			.DW		Str_Left
			.DW		Str_Right
			.DW		Str_Save

CalISEpH:
			LDI		XH, HIGH(CalTableStart)		; Initialize X to proper cal table
			LDI		XL, LOW(CalTableStart)		; Remember ph is mode 1
			LDS		R16, OpMode
			LDI		R17, pHCalTable-CalTableStart
			MUL		R16, R17
			ADD		XL, R0
			ADC		XH, R1

			RCALL	CheckCalInit
			BRTC	CalISEpHCheckKey

CalISEpHSelect:			
			LDI		YH, HIGH(ModeTableStart)	; Point Y to mode table entry
			LDI		YL, LOW(ModeTableStart)
			LDS		R16, OpMode
			LDI		R17, ModeTableEntryMark-ModeTableStart
			MUL		R16, R17
			ADD		YL, R16
			ADC		YH, R17
			RCALL	InitCalibration

CalISEpHCheckKey:
			LDI		ZH, HIGH(CalISEpHJumpTable)
			LDI		ZL, LOW(CalISEpHJumpTable)
			RJMP	DispatchToCalKeyDestination

CalISEpHSaveCal:
			LDS		R16, OpMode					; Save this calibration point
			LDI		R17, 12						; Size of cal table entry (3 points)
			MUL		R16, R17					; R16 is index to base of correct cal table
			LDS		R17, CalPoint
			LSL		R17
			LSL		R17
			ADD		R16, R17					; R16 indexes correct entry
			LDI		XH, HIGH(RawCalSignal)
			LDI		XL, LOW(RawCalSignal)
			CALL	SaveCalibrationValue		; Save in cal table
			RJMP	CalDone

CalISEpHDown:									; Go to next lower cal point
			LDS		R16, CalPoint
			DEC		R16
			BRCC	CalISEphPoint
			SUB		R16, R16
			RJMP	CalISEphPoint
CalISEpHUp:										; Go to next higher cal point
			LDS		R16, CalPoint
			INC		R16
			CPI		R16, 3
			BRLO	CalISEphPoint
			LDI		R16, 2
CalISEphPoint:
			STS		CalPoint, R16				; Either way, save new value and re-initialize
			RJMP	CalISEpHSelect

;-------------------------------------------------------------------------------------------------
; Calibrate temperature probe

CalTempJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		CalTempDown					; Button 2 goes to the next lower cal point.
			.DW		CalTempUp					; Button 3 goes to the next higher cal point
			.DW		CalTempSave					; Button 4 saves the cal point

CalTempLegendTable:
			.DW		Str_Left
			.DW		Str_Right
			.DW		Str_Save
CalTemp:
			LDI		XH, HIGH(CalTableStart)		; Initialize X to proper cal table
			LDI		XL, LOW(CalTableStart)		; Remember Temperature is equiv. to mode 0
			RCALL	CheckCalInit
			BRTC	CalTempCheckKey

CalTempSelect:
			LDI		ZH, HIGH(Str_Cal)			; First line to say something like 
			LDI		ZH, LOW(Str_Cal)			; "Calibrate 0°C"
			SUB		R16, R16
			RCALL	WriteCString				; "Calibrate"
			INC		R16							; Space
			LDI		ZH, HIGH(Str_100C)
			LDI		ZL, LOW(Str_100C)
			LDS		R17, CalPoint
			CPI		R17, 2
			BREQ	CalTempShowPoint
			LDI		ZH, HIGH(Str_0C)
			LDI		ZL, LOW(Str_0C)
CalTempShowPoint:
			RCALL	WriteCString				; Calibration point

CalTempCheckKey:
			LDI		XH, HIGH(CalTempJumpTable)
			LDI		XL, LOW(CalTempJumpTable)
			RJMP	DispatchToCalKeyDestination

CalTempSave:
			LDI		R16, 2						; Assume 0 deg point
			LDS		R17, CalPoint
			CPI		R17, 1
			BREQ	CalTempSave2
			LDI		R16, 4						; Else, 100 deg point
CalTempSave2:
			LDI		XH, HIGH(TemperatureFilter+FilterFloat)
			LDI		XL, LOW(TemperatureFilter+FilterFloat)
			CALL	SaveCalibrationValue		; Save in cal table
			RJMP	CalDone

CalTempDown:									; Go to next lower cal point
			LDS		R16, CalPoint
			DEC		R16
			BRNE	CalTempPoint
			LDI		R16, 1
			RJMP	CalTempPoint
CalTempUp:										; Go to next higher cal point
			LDS		R16, CalPoint
			INC		R16
			CPI		R16, 3
			BRLO	CalTempPoint
			LDI		R16, 2
CalTempPoint:
			STS		CalPoint, R16				; Either way, save new value and re-initialize
			RJMP	CalTempSelect

;-------------------------------------------------------------------------------------------------
; Initialize calibration for selected pH/ISE

InitCalibration:
			LDI		ZH, HIGH(Str_Cal)			; First line to say something like 
			LDI		ZH, LOW(Str_Cal)			; "Calibrate pH 4.0" depending on ion and cal point
			SUB		R16, R16
			RCALL	WriteCString				; "Calibrate"
			INC		R16							; Space
			LDD		ZL, Y+0
			LDD		ZH, Y+0
			RCALL	WriteCString				; Ion name
			INC		R16							; Space
			LDS		R17, CalPoint
			INC		R17
			LSL		R17
			SUB		R18, R18
			ADD		ZL, R17
			ADC		ZH, R18
			RCALL	WriteCString				; Calibration point
			RET

;-------------------------------------------------------------------------------------------------
; Set manual or automatic temperature compensation, and if manual, the temperature

SetTempModeJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		SetTempSwitchMode			; Button 2 changes mode
			.DW		SetTempDown					; Button 3 decreases temperature and sets to manual
			.DW		SetTempUp					; Button 4 increases temperature and sets to manual

SetTempModeLegendTable:
			.DW		Str_Sel
			.DW		Str_Down
			.DW		Str_Up

SetTempMode:
			RCALL	CheckCalInit
			BRTC	SetTempModeCheckKey

			SUB		R16, R16
			STS		CalPoint, R16				; 0 -> manual auto not chosen
SetTempModeStart:
			SUB		R16, R16
			LDI		ZH, HIGH(Str_TCMode)
			LDI		ZL, LOW(Str_TCMode)			; "TC Mode: "
			RCALL	WriteCString
			LDI		ZH, HIGH(Str_Auto)
			LDI		ZL, LOW(Str_Auto)
			LDS		R20, ProcessStatus
			BST		R20, PS_MANTEMP
			BRTC	SetTempModeDisplay
			LDI		ZH, HIGH(Str_Manual)
			LDI		ZL, LOW(Str_Manual)
SetTempModeDisplay:
			RCALL	WriteCString

SetTempModeCheckKey:
			LDI		XH, HIGH(SetTempModeJumpTable)
			LDI		XL, LOW(SetTempModeJumpTable)
			RJMP	DispatchToCalKeyDestination

SetTempDown:
			LDI		XH, HIGH(Cpm1)
			LDI		XL, LOW(Cpm1)
			RJMP	SetTempAdj

SetTempUp:
			LDI		XH, HIGH(Cp1)
			LDI		XL, LOW(Cp1)
SetTempAdj:
			LDI		ZH, HIGH(TempComp)
			LDI		ZL, LOW(TempComp)
			RCALL	FSRAMA1
			MOV		ZH, XH
			MOV		ZL, XL
			RCALL	FADDROM
			RJMP	CalDone

SetTempSwitchMode:
			LDI		R19, 1<<PS_MANTEMP
			EOR		R20, R19
			STS		ProcessStatus, R20
			RJMP	SetTempModeStart

;-------------------------------------------------------------------------------------------------
; Set temperature to read in C or F

SetTempDisplayJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		SetDisplayC					; Button 2 sets to °C
			.DW		SetDisplayF					; Button 3 sets to °F
			.DW		CalDone						; Button 4 does nothing

SetTempDisplayLegendTable:
			.DW		Str_DegC
			.DW		Str_DegF
			.DW		Str_Null

SetTempDisplay:
			LDI		XH, HIGH(SetTempDisplayJumpTable)
			LDI		XL, LOW(SetTempDisplayJumpTable)
			RJMP	DispatchToCalKeyDestination

SetDisplayC:
			SUB		R16, R16
			STS		DegF, R16
			RJMP	CalDone

SetDisplayF:
			LDI		R16, 1
			STS		DegF, R16
			RJMP	CalDone

;-------------------------------------------------------------------------------------------------
; Set report interval. 0 = off.

SetReportIntervalJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		SetIntervalDown				; Button 2 decreases the interval
			.DW		SetIntervalUp				; Button 3 increases the interval
			.DW		SetIntervalOff				; Button 4 Turns interval off

SetReportIntervalLegendTable:
			.DW		Str_Down
			.DW		Str_Up
			.DW		Str_Off

SetReportInterval:
			LDI		XH, HIGH(SetReportIntervalJumpTable)
			LDI		XL, LOW(SetReportIntervalJumpTable)
			RJMP	DispatchToCalKeyDestination

SetIntervalDown:
			LDS		R16, ReportInterval
			TST		R16							; Can't go below zero
			BREQ	SetIntervalDone
			LDI		R17, 1						; Ten or lower, decrement by one
			CPI		R16, 11		
			BRLO	SetIntervalDown2
			LDI		R17, 5						; else decrement by 5
SetIntervalDown2:
			SUB		R16, R17
			STS		ReportInterval, R16
			RJMP	CalDone

SetIntervalUp:
			LDS		R16, ReportInterval
			TST		R16							; Can't go below zero
			BREQ	SetIntervalDone
			LDI		R17, 5						; Ten or higher, increment by 5
			CPI		R16, 10		
			BRGE	SetIntervalUp2
			LDI		R17, 1						; else increment by 1
SetIntervalUp2:
			SUB		R16, R17
			STS		ReportInterval, R16
			RJMP	CalDone

SetIntervalOff:
			SUB		R16, R16
			STS		ReportInterval, R16
SetIntervalDone:
			RJMP	CalDone

;-------------------------------------------------------------------------------------------------
; Set analog out format
;  0 = off, 1 = proportional, 2 = diff to set point, 3 = toggle high crossing set point upward, 5 = toggle low crossing set point upward

Str_AOut:	.DB		"AOut:", 0
Str_Prop:	.DB		"Proportional", 0, 0
Str_Diff:	.DB		"Diff", 0, 0
Str_Pos:	.DB		"Pos", 0
Str_Neg:	.DB		"Neg", 0

SetAnalogOutString:

SetAnalogOutJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		SetAnalogSelect				; Button 2 selects the analog output mode
			.DW		SetAnalogNext				; Button 3 goes to next set digit
			.DW		SetAnalogIncr				; Button 4 increments chosen value

SetAnalogOutLegendTable:
			.DW		Str_Sel
			.DW		Str_Right
			.DW		Str_Up

SetAnalogOut:		
			RCALL	CheckCalInit
			BRTC	SetAnalogOutCheckKey

SetAnalogShowInit:
			LDI		R16, FuncOffset				; Show what mode the output is in
			LDI		R17, FuncLength
			RCALL	ClearDisplayBuffer
			LDI		ZH, HIGH(Str_AOut)
			LDI		ZL, LOW(Str_AOut)
			RCALL	WriteCString
			LDI		R16, FuncOffset+6
			LDS		R17, AOutMode
			LDI		ZH, HIGH(Str_Prop)
			LDI		ZL, LOW(Str_Prop)
			TST		R17
			BREQ	SetAnalogShowMode
			LDI		ZH, HIGH(Str_Diff)
			LDI		ZL, LOW(Str_Diff)
			DEC		R17
			BREQ	SetAnalogShowMode
			LDI		ZH, HIGH(Str_Pos)
			LDI		ZL, LOW(Str_Pos)
			DEC		R17
			BREQ	SetAnalogShowMode
			LDI		ZH, HIGH(Str_Neg)
			LDI		ZL, LOW(Str_Neg)
SetAnalogShowMode:
			RCALL	WriteCString

SetAnalogCheckMode:
			LDS		R18, AOutMode
			TST		R18
			BREQ	SetAnalogOutCheckKey

SetAnalogShowSetPoint:


SetAnalogOutCheckKey:


;-------------------------------------------------------------------------------------------------
; Set clock

			.DSEG
ClkSetDigit:.DB		0							; Index int H:M:S D:M:YYYY

CSDataTable:									; BCD
CSHours:	.DB		0
CSMinutes:	.DB		0
CSSeconds:	.DB		0
CSDay:		.DB		0
CSMonth:	.DB		0
CSYear10:	.DB		0
CSYear1:	.DB		0

			.CSEG

SetClockRangeTable:
			.DB		0, 23						; Hours
			.DB		0, 59						; Minutes
			.DB		0, 59						; Seconds
			.DB		1, 31						; Day of month
			.DB		1, 12						; Month
			.DB		1, 2						; 1000 years
			.DB		0, 9						; 100 years
			.DB		0, 9						; 10 years
			.DB		0, 9						; years

SetClockJumpTable:
			.DW		CalRestoreMode				; The "Mode" key aborts cal and puts us back where we were
			.DW		CalDone						; The "Cal" key does nothing right now
			.DW		SetClockPrev				; Button 2 goes to previous item
			.DW		SetClockNext				; Button 3 goes to next item
			.DW		SetClockIncr				; Button 4 increments chosen value

SetClockLegendTable:
			.DW		Str_Left
			.DW		Str_Right
			.DW		Str_Up

SetClock:
			RCALL	CheckCalInit
			BRTC	SetClockCheckKey

SetClockReadRegisters:
			LDI		ZH, HIGH(CSDataTable)		; read in clock registers and fill the memory slots
			LDI		ZL, LOW(CSDataTable)
			LDI		R17, RTC_HOURS
			RCALL	RTCRead
			ST		Z+, R16
			LDI		R17, RTC_MIN
			RCALL	RTCRead
			ST		Z+, R16
			LDI		R17, RTC_SEC
			RCALL	RTCRead
			ST		Z+, R16
			LDI		R17, RTC_DAY
			RCALL	RTCRead
			ST		Z+, R16
			LDI		R17, RTC_YEAR
			RCALL	RTCRead
			MOV		R17, R16
			ANDI	R16, 0x0F
			SWAP	R16
			ANDI	R17, 0x05
			ST		Z+, R17
			ST		Z+, R16

SetClockFormatDisplay:
			LDI		ZH, HIGH(CSDataTable)		; Format the memory slots for editing.  Use SO/SI to mark selected text in reverse video.
			LDI		ZL, LOW(CSDataTable)
			LDI		YH, HIGH(LineBuffer+FuncOffset)
			LDI		YL, LOW(LineBuffer+FuncOffset)
			LDI		R18, ':'
			LDI		R20, ' '
			LDS		R18, ClkSetDigit
			LD		R16, Z+						; Hours
			RCALL	SetClockFormat2Digits
			ST		Y+, R18
			LD		R16, Z+						; Minutes
			RCALL	SetClockFormat2Digits
			ST		Y+, R19
			LD		R16, Z+						; Seconds
			RCALL	SetClockFormat2Digits
			ST		Y+, R20
			ST		Y+, R20
			LDI		R19, '/'
			LD		R16, Z+						; Day
			RCALL	SetClockFormat2Digits
			ST		Y+, R19
			LD		R16, Z+
			RCALL	SetClockFormat2Digits
			ST		Y+, R19
			LD		R16, Z+						; Decades
			RCALL	SetClockFormat1Digit
			LD		R16, Z+
			RCALL	SetClockFormat1Digit
								;
SetClockCheckKey:
			LDI		XH, HIGH(SetClockJumpTable)
			LDI		XL, LOW(SetClockJumpTable)
			RJMP	DispatchToCalKeyDestination

SetClockPrev:
			LDS		R16, ClkSetDigit
			DEC		R16
			BRCS	SetClockDone
			SUB		R16, R16
			RJMP	SetClockNew

SetClocknext:
			LDS		R16, ClkSetDigit
			CPI		R16, 8
			BRGE	SetClockDone
			INC		R16
SetClockNew:
			STS		ClkSetDigit, R16
			RJMP	SetClockFormatDisplay

SetClockIncr:
			LDS		R16, ClkSetDigit			; Increment current selection.  If a sixth button
			LDI		ZH, HIGH(SetClockRangeTable); will fill, also add a decrement button
			LDI		ZL, LOW(SetClockRangeTable)
			LSL		R16
			SUB		R17, R17
			ADD		XL, R16
			ADC		XH, R17						; Point to proper range table.
			LSR		R16
			LDI		YH, HIGH(CSDataTable)
			LDI		YL, LOW(CSDataTable)
			ADD		YL, R16
			ADC		YH, R17						; Point to right data entry
			LDD		R16, Y+0
			LDI		R17, 1
			RCALL	BCDADD
			LDD		R17, Z+1
			CP		R16, R17
			BRCC	SetClockSave
			LD		R16, X

SetClockSave:
			LDI		R18, ClkSetDigit
			LDI		R17, RTC_HOURS
			TST		R18
			BREQ	SetClockWrite
			LDI		R17, RTC_MIN
			DEC		R18
			BREQ	SetClockWrite
			LDI		R17, RTC_SEC
			DEC		R18
			BREQ	SetClockWrite
			LDI		R17, RTC_DAY
			DEC		R18
			BREQ	SetClockWrite
			LDI		R17, RTC_MONTH
			DEC		R18
			BRNE	SetClockYear
SetClockWrite:
			RCALL	RTCWrite
			RJMP	SetClockFormatDisplay
SetClockYear:
			LDI		R16, CSYear1
			LDI		R17, CSYear10
			SWAP	R17
			ADD		R16, R17
			LDI		R17, RTC_YEAR
			RJMP	SetClockWrite

SetClockDone:
			RJMP	CalDone

; For the following, R16 is the value,  Y is buffer dest, R18 is data index.  When it goes to zero, highlight.  Save R19, 20

SetClockFormat2Digits:
			TST		R18
			BRNE	SetClockFormat2DigitsWrite
			LDI		R22, ASC_SO
			ST		Y+, R22
SetClockFormat2DigitsWrite:
			MOV		R21, R16
			RCALL	WriteDigitY
			MOV		R16, R20
			SWAP	R16
			RCALL	WriteDigitY
			TST		R18
			BRNE	SetClockFormat2DigitsComplete
			LDI		R22, ASC_SO
			ST		Y+, R22
SetClockFormat2DigitsComplete:
			DEC		R18
			RET

SetClockFormat1Digit:
			TST		R18
			BRNE	SetClockFormat1DigitWrite
			LDI		R22, ASC_SI
			ST		Y+, R22
SetClockFormat1DigitWrite:
			RCALL	WriteDigitY
			TST		R18
			BRNE	SetClockFormat1DigitComplete
			LDI		R22, ASC_SI
			ST		Y+, R22
SetClockFormat1DigitComplete:
			DEC		R18
			RET

;-------------------------------------------------------------------------------------------------
; Save calibration value, X->source, R16 = destination index.

SaveCalibrationValue:
			LDI		R17, 4
			MOV		R19, R16					; Generate EEPROM address
			SUB		R20, R20
			LSL		R19
			ROL		R20
			LSL		R19
			ROL		R20
			SUB		R0, R0
SaveCalValLoop:
			LD		R18, X+
			RCALL	WriteEEPROM
			INC		R19
			ADC		R20, R0
			DEC		R17
			BRNE	SaveCalValLoop
			RET

			; Write byte in R18 to EEPROM address R20:R19

WriteEEPROM:					
			SBIC	EECR, EEWE					; Wait for completion of previous write
			RJMP	WriteEEPROM
			OUT		EEARH, R20
			OUT		EEARL, R19
			OUT		EEDR, R18
			SBI		EECR, EEMWE					; Write logical one to EEMWE
			SBI		EECR, EEWE					; Start eeprom write by setting EEWE
			RET

;-------------------------------------------------------------------------------------------------
; Load calibration value, R16 = source index. Z->destination

LoadCalibrationValue:
			LDI		R17, 4						; Move 4 bytes (one float)
			MOV		R19, R16					; Generate EEPROM address
			SUB		R20, R20
			LSL		R19
			ROL		R20
			LSL		R19
			ROL		R20
			SUB		R0, R0
LoadCalValLoop:
			RCALL	ReadEEPROM
			ST		Z+, R18
			INC		R19
			ADC		R20, R0
			DEC		R17
			BRNE	LoadCalValLoop
			RET

			; Read byte in EEPROM address R20:R19 to R18

ReadEEPROM:					
			SBIC	EECR, EEWE					; Wait for completion of any previous write
			RJMP	ReadEEPROM
			OUT		EEARL, R19
			OUT		EEARH, R20
			SBI		EECR, EERE					; Start eeprom read by setting EERE
			IN		R18, EEDR
			CBI		EECR, EERE
			RET

;=================================================================================================
; Output information to the real world

;-------------------------------------------------------------------------------------------------
; Format floating to string as 3.2.  Minus sign may take first position before decimal
; Z->source, Y->SRAM destination

Write3Point2:
			PUSH	YH
			PUSH	YL
			RCALL	FSRAMA1
			BST		expnt1, 7
			BRTC	WritePositive

			LDI		R16, '-'					; Minus sign for negative numbers
			ST		Y+, R16	
			DEC		R1							; That leaves two slots for digits
			LDI		R25, 0x80
			EOR		expnt1, R25					; Make positive

WritePositive:
			SUB		R2, R2						; Count the number of divisions by 10 to bring number below one
CountIntegerDigits:
			LDI		ZH, HIGH(C1)
			LDI		ZL, LOW(C1)
			RCALL	FLTCPMEM
			BRLT	IntegerDigitCountKnown
			LDI		ZH, HIGH(C10)
			LDI		ZL, LOW(C10)
			RCALL	FDIVMEM
			RJMP	CountIntegerDigits

IntegerDigitCountKnown:
			LDI		R20, '0'
			MOV		R4, R20
			LDI		R20, 255
			MOV		R5, R20						; Digits right of decimal
			SUB		R1, R2						; If more digits than will fit, overflow
			BRLT	WriteOverflow
IntegerDigitCountOK:
			BREQ	ExtractNextDigit
WriteLeadingSpace:
			LDI		R20, ' '					; R1 has number of leading spaces
			ST		Y+, R20
			DEC		R1
			BRNE	WriteLeadingSpace
			
ExtractNextDigit:								; Successively multipy by 10 to get peel off digits
			AND		R2, R2						; R2 is number of positions before decimal point
			BRNE	ExtractNextDigit2
			LDI		R20, '.'
			ST		Y+, R20
			LDI		R20, 2						; 2 decimal digits
			MOV		R5, R20

ExtractNextDigit2:
			LDI		ZH, HIGH(C10)
			LDI		ZL, LOW(C10)
			RCALL	FMULMEM
			RCALL	AC1TOAC2					; Save in AC2
			RCALL	FTOI						; Convert to integer; fractional part discarded
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	MSave1ToMem					; Save integer part
			ADD		R16, R4						; Convert to ascii.  0 <= R16 <=9
			ST		Y+, R17
			RCALL	ITOFMEM						; Integer to float
			LDI		ZH, HIGH(ITemp)
			LDI		ZL, LOW(ITemp)
			RCALL	MSave1ToMem
			RCALL	SWAPACC
			RCALL	FSUBMEM						; Leftmost digit stripped off

			DEC		R5
			BREQ	CleanUpFormat
			DEC		R2
			RJMP	ExtractNextDigit

CleanUpFormat:									; If there is no digit before the decimal,
			POP		YL							; put a zero there
			POP		YH
			ADIW	YH:YL, 2
			LD		R16, Y
			CPI		R16, ' '
			BRNE	WriteFormatDone
			ST		Y, R4
			RJMP	WriteFormatDone

WriteOverflow:
			LDI		R16, 5
			LDI		R17, '*'
WriteOverflowLoop:
			ST		Y+, R17
			DEC		R16
			BRNE	WriteOverflowLoop
			POP		YL
			POP		YL
			
WriteFormatDone:
			RET

;-------------------------------------------------------------------------------------------------
; Analog output for recorder

			.DSEG

AOutMode:	.DB		0
AOutNullPt:	.DD		0

			.CSEG

RecorderOutput:

			RET

;-------------------------------------------------------------------------------------------------
; Transfer RAM buffers to physical display

UpdateDisplay:
			RET

;-------------------------------------------------------------------------------------------------
; Clear the display from R16 offset for R17 characters

ClearDisplayBuffer:
			TST		R17
			BREQ	ClearDisplayDone
			LDI		YH, HIGH(LineBuffer)
			LDI		YL, LOW(LineBuffer)
			SUB		R18, R18
			ADD		YL, R16
			ADC		YH, R18
			LDI		R18, ' '
ClearDisplayChar:
			ST		Y+, R18
			DEC		R17
			BRNE	ClearDisplayChar
ClearDisplayDone:
			RET

;***************************************************************************
;*
;* "BCDadd" - 2-digit packed BCD addition
;*
;* This subroutine adds the two unsigned 2-digit BCD numbers
;* "BCD1" and "BCD2". The result is returned in "BCD1", and the overflow
;* carry in "BCD2".
;*  
;* Number of words :19
;* Number of cycles :17/20 (Min/Max)
;* Low registers used :None
;* High registers used  :3 (BCD1,BCD2,tmpadd)
;*
;***************************************************************************


BCDadd:
			LDI		R18,6							; Value to be added later
			ADD		R16, R17						; Add the numbers binary
			CLR		R17								; Clear BCD carry
			BRCC	add_0 							; If carry not clear
			LDI		R17, 1 							; Set BCD carry
add_0:		BRHS	add_1							; If half carry not set
			ADD		R16, R18						; Add 6 to LSD
			BRHS	add_2 							; If half carry not set (LSD <= 9)
			SUBI	R16, 6							; Restore value
			RJMP	add_2							; else
add_1:		ADD		R16, R18						; Add 6 to LSD
add_2:		SWAP	R18
			ADD		R16, R18						; Add 6 to MSD
			BRCS	add_4							; If carry not set (MSD <= 9)
			SBRS	R17, 0 							; If previous carry not set
			SUBI	R16, $60						; restore value
add_3:		RET
add_4: 		LDI		R17, 1 							; Set BCD carry
			RET



;***************************************************************************
;*
;* "BCDsub" - 2-digit packed BCD subtraction
;*
;* This subroutine subtracts the two unsigned 2-digit BCD numbers
;* "BCDa" and "BCDb" (BCDa - BCDb). The result is returned in "BCDa", and
;* the underflow carry in "BCDb".
;*  
;* Number of words :13
;* Number of cycles :12/17 (Min/Max)
;* Low registers used :None
;* High registers used  :2 (BCDa,BCDb)
;*
;***************************************************************************
/*
;***** Subroutine Register Variables

.def BCDa =r16 ;BCD input value #1
.def BCDb =r17 ;BCD input value #2

;***** Code

BCDsub:
sub BCDa,BCDb;subtract the numbers binary
clr BCDb
brcc sub_0 ;if carry not clear
ldi BCDb,1 ;    store carry in BCDB1, bit 0
sub_0: brhc sub_1 ;if half carry not clear
subi BCDa,$06;    LSD = LSD - 6
sub_1: sbrs BCDb,0 ;if previous carry not set
ret  ;    return
subi BCDa,$60;subtract 6 from MSD
ldi BCDb,1 ;set underflow carry
brcc sub_2 ;if carry not clear
ldi BCDb,1 ;    clear underflow carry
sub_2: ret  
*/

;------------------------------------------------------------------------------------------------
; Covert BCD digit in R16 to ASCII and write to @Y

WriteDigitY:
			ANDI	R16, 0x0F					; Mask out high digit if any
			LDI		R23, '0'
			ADD		R16, R23
			ST		Y+, R16
			RET

;------------------------------------------------------------------------------------------------
; Move 4 bytes from Z to Y

Move4ZtoY:
			LDI		R17, 4
Move4ZtoYLoop:
			LD		R0, Z+
			ST		Y+, R0
			DEC		R17
			BRNE	Move4ZtoYLoop
			RET

;-------------------------------------------------------------------------------------------------
; Write a cstring in Program Memory at Z to display memory offset R16.  We trust the line is not too long

WriteCString:
			LSL		ZL						; Convert word addressing to byte addressing
			ROL		ZH
			LDI		YH, HIGH(LineBuffer)
			LDI		YL, LOW(LineBuffer)
			SUB		R17, R17
			ADD		YL, R16
			ADC		YH, R17
WriteCStringChar:
			LPM		R17, Z+
			TST		R17
			BREQ	WriteCStringDone
			ST		Y+, R17
			INC		R16						; Save index for sequential text writes.
			RJMP	WriteCStringChar
WriteCStringDone:
			RET

;-------------------------------------------------------------------------------------------------
; Write a button legend @Z to button index R16.  Preserve R16, R17, and ZH:ZL

WriteButtonLegend:
			MOV		R2, R16					; Saves stack space over push/pop
			MOV		R3, R17
			MOV		R4, ZL
			MOV		R5, ZH
			LDI		R17, 5
			MUL		R16, R17
			LDI		YH, HIGH(LegendBuf)
			LDI		YL, LOW(LegendBuf)
			ADD		YL, R0
			ADC		YL, R1
			LDI		R17, 5					; Legend width.
WriteButtonLegendLoop:
			LD		R6, Z+
			TST		R6
			BREQ	WriteButtonLegendEnd
			ST		Y+, R6					; Copy each character
			DEC		R17
			BRNE	WriteButtonLegendLoop
WriteButtonLegendEnd:
			TST		R17						; If we have written less than 5 chars,
			BREQ	WriteButtonLegendDone	; pad rest with spaces
			LDI		R16, ' '
WriteButtonLegendPad:
			ST		Y+, R16
			DEC		R17
			BRNE	WriteButtonLegendPad
WriteButtonLegendDone:
			MOV		ZH, R5
			MOV		ZL, R4
			MOV		R17, R3
			MOV		R16, R2
			RET

			.DSEG

LineBuffer:	.BYTE		52					; Top 2 lines of display
ReadoutBuf:	.BYTE		8					; Large font readout
LegendBuf:	.BYTE		25					; Buffer for button legends at bottom

			; Define offsets into buffers

			.EQU		ModeOffset = 0;
			.EQU		TempOffset = 12;
			.EQU		TimeOffset = 20
			.EQU		FuncOffset = 26

			.EQU		ModeLength = TempOffset-ModeOffset
			.EQU		TempLength = TimeOffset-TempOffset
			.EQU		TimeLength = 5
			.EQU		FuncLength = 25

			.CSEG
;=================================================================================================
; Math helper routines. Things that should have been in JT's math library

; Save Acc1 to memory

MSave1ToMem:
			STD		Z+0, MANT1
			STD		Z+1, MANT1M
			STD		Z+2, MANT1H
			STD		Z+3, EXPNT1
			RET

FADDROM:
FMULROM:	
FDIVROM:	RET



			.INCLUDE "jtmath.asm"			; Floating-point math library

			.INCLUDE "fontlarge.asm"		; 24-pixel font for numeric output

			.INCLUDE "LCDAGM1264F.asm"		; Driver for AGM1264F 128x64 LCD display

			.DSEG
DataEnd:



