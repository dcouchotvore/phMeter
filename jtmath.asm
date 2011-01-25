; ** FAST IEEE FP Routines for Atmel AVR RISC **
; ** 1998,1999 by Jack Tidwell <jackt@igalaxy.net> **
; ** These routines may be used for personal and educational
; **      purposes, and are free to download. **
; ** Work is still in progress (smaller & FASTER!) **
; ** Trig funcs comming soon **
; ** I would like feedback. **

; ** Most problems fixed, converting to Decimal seems to be
;    the only one left?
; ** Added some 'C' support **
; ** Work on preservation of r9 to r25 is needed **

; NOTE: All funcs w/xxxMEM: i.e; FDIVMEM:
; enter w/r30,r31 as a pointer to 32bit SRAM FPvalue.
; You still have to preserv r9-r25
; It's a start toward Rons' SmallC.

//.include "8535def.inc"; (Any AVR except 1200)

.def QUOT =r9; partial product scratch area
.def QUOTM =r10
.def QUOTH =r11

.def DEXP =r12; Decimal Exp for 'ftoa(), ecvt()' funcs

.def MANT1 =r13; Accumulator 1
.def MANT1m =r14
.def MANT1h =r15
.def EXPNT1 =r16
.def SIGN1 =r17

.def MANT2 =r18; Accumulator 2
.def MANT2m =r19
.def MANT2h =r20
.def EXPNT2 =r21
.def SIGN2 =r22

.def MANT1T =r23; used to extend 24bit to 32bit ops
.def count =r24; GP loop counter
.def temp =r25; scratch 'ram'?

.dseg

DECBUFF: .byte 10; some place to hold Decimal Digits

MATHTMP: .byte 8; Accumulator 'Holding' registers.

.cseg

;
;**************************************************************************
;**** FP Math Routines: ****
; ** Very similar to _ecvt(), (still a little buggy?) **
FLTDEC:
		RCALL	Push1
		PUSH	sign1
		PUSH	EXPNT1 								; FP to 7 digit dec # +exp
		TST		EXPNT1
		BRPL	FLTD1
		RCALL	FLTNEG
FLTD1:
		LDI		temp,0x7f
		SUB		temp,EXPNT1
		MOV		DEXP,temp
FLTD2:
		LDI		ZL, LOW(k1E8 *2)
		LDI		ZH, HIGH(k1E8 *2)
rcall KTOAC2
rcall FLTCP
brpl FLTD5
ldi zl,low(k10*2)
ldi zh,high(k10*2)
rcall KTOAC2
rcall FMULT
inc DEXP
rjmp FLTD2
FLTD5: pop temp; orig EXPNT1
mov count,temp
neg temp
add temp,DEXP
subi temp,0x7f
mov DEXP,temp
andi count,0x80
rcall FADDp5
pop sign1
ITOD: rcall FTOI
ldi yl,low(DECBUFF)
ldi yh,high(DECBUFF)
tst sign1
brpl ITOD1
ldi temp,'-'
st y+,temp
rcall FLTNEG
ITOD1: ldi zl,low(I1E6*2); decimal 'lookup' table
ldi zh,high(I1E6*2)
ldi count,6
ITODLP: push count
rcall KTOAC2
clr temp
ITOD2: cp mant1,mant2
cpc mant1m,mant2m
cpc mant1h,mant2h
brlo ITOD3
sub mant1,mant2
sbc mant1m,mant2m
sbc mant1h,mant2h
inc temp
brne ITOD2
ITOD3: ori temp,'0'; convert to Ascii char
st y+,temp ; store the dig in the SRAM Buffer, and post-inc 'y' reg
pop count
dec count ; have we processed all tables yet?
brne ITODLP
mov temp,mant1
ori temp,'0'
st y+,temp
; * count = 0 here, use it to add ending 'NULL' char. *
st y+,count
; * DEXP has dp position -1 *
st y+,DEXP
rjmp POP1 ; restore ac1
;
FTOIMEM: rcall FSRAMA1
FTOI: rcall UNPACK
brne FTOI1
rjmp MINRES
FTOI1: ldi temp,22
sub temp,EXPNT1
brpl FTOI2
rjmp MAXRES
FTOI2: cpi temp,24
brlo FTOI3
rjmp MINRES
FTOI3: cpi temp,8; use fast byte-move, if possible
brlo FTOI4
mov mant1,mant1m
mov mant1m,mant1h
mov mant1h,quot
clr quot
subi temp,8
rjmp FTOI3
FTOI4: tst temp
breq FTOI6
FTOI5: lsr quot
ror mant1h
ror mant1m
ror mant1
dec temp
brne FTOI5
FTOI6:
; tst sign1
; brpl FTOIX
; rjmp NEGMANT
FTOIX: ret
;
;** ACC1 -= *r30 **
FSUBMEM: rcall FSRAMA2; Acc1 -= *r30
;** ACC1 -= ACC2 **
FSUB: rcall UNPACK
cpi EXPNT2,0x80
breq FSADX
ldi temp,0x80
eor sign2,temp
rjmp FADD1
FSUNDER: brmi FSADX
FRET2: rcall SWAPACC
FSADX: rjmp REPACK
;** ACC1 += 0.5f **
FADDp5: ldi EXPNT2,0x3f
clr mant2
clr mant2m
clr mant2h
rjmp FADD
FADDMEM: rcall FSRAMA2; Acc1 += *r30
;** ACC1 += ACC2 **
FADD: rcall UNPACK
cpi EXPNT2,0x80
breq FSADX
FADD1: cpi EXPNT1,0x80
breq FRET2
FADD2: mov temp,EXPNT1
sub temp,EXPNT2
brvs FSADX
brpl FADD3
rcall SWAPACC
rjmp FADD2
FADD3: cpi temp,24; are we more than 24 bits diff?
brlo FADD3a; no, we can add it.
clr mant2; yes, acc2 is to small
clr mant2m
clr mant2h
FADD3a: cpi temp,8
brlo FADD3b
mov mant2,mant2m
mov mant2m,mant2h
clr mant2h
subi temp,8
rjmp FADD3a
FADD3b: tst temp
breq FADD3d
FADD3c: lsr mant2h
ror mant2m
ror mant2
dec temp
brne FADD3c
FADD3d: mov temp,sign1
eor temp,sign2
brmi FADD4
rcall UADD
brcc FSADX
ror mant1h
ror mant1m
ror mant1
subi EXPNT1,-1; Add +1
brvc FSADX
rjmp MAXRES
FADD4: rcall USUB
breq FCLR
brcc FADD5
rcall NEGMANT
FADD5: tst mant1h; normalize
brmi FSADX
lsl mant1
rol mant1m
rol mant1h
subi EXPNT1,1
brvc FADD5
rjmp MAXRES
FCLR: rjmp MINRES
;
UADD: add mant1,mant2
adc mant1m,mant2m
adc mant1h,mant2h
ret
;
USUB: sub mant1,mant2
sbc mant1m,mant2m
sbc mant1h,mant2h
ret
;
;** ACC1 = -ACC1 **
FLTNEG: rcall UNPACK
com sign1
rjmp REPACK
;
NEGMANT: ldi temp,-1
sub mant1,temp
sbc mant1m,temp
sbc mant1h,temp
ret
;
;** MANT1 <<= 1 **
RLMANT1: lsl mant1
rol mant1m
rol mant1h
rol mant1t
ret
;
;** Straight 24bit Unsigned ACC1 / ACC2 **
UDIVMANT: clr quot
clr quotm
clr quoth
ldi count,24
UDIVLP: cp mant1,mant2
cpc mant1m,mant2m
cpc mant1h,mant2h
brlo UDIV1
sub mant1,mant2
sbc mant1m,mant2m
sbc mant1h,mant2h
sec
rjmp UDIV2
UDIV1: clc
UDIV2: rol quot
rol quotm
rol quoth
lsl mant1
rol mant1m
rol mant1h
dec count
brne UDIVLP
push mant1
mov mant1,quot
mov mant1m,quotm
mov mant1h,quoth
pop temp
lsr temp
brcc udivx
inc mant1
brne udivx
inc mant1m
brne udivx
inc mant1h
udivx:
ret
;
;** ACC1 = 1/*r30 **
F1OVERMEM: rcall FSRAMA1
;** ACC1 = 1/ACC1 **
F1OVERX: rcall AC1TOAC2
clr mant1
clr mant1m
ldi temp,0x80
mov mant1h,temp
ldi EXPNT1,0x3f; ieee 1.0f
rjmp FDIV ; do 1/x
;
;** Acc1 /= *r30 **
FDIVMEM: rcall FSRAMA2
rjmp FDIV
;** ACC1 /= 10.0f */
FDIV10: clr mant2 ; x/10.0f  could use fmul * 0.1f?
clr mant2m
ldi mant2h,0x20
ldi EXPNT2,0x41;Put 10.0 -> Acc2
;** ACC1 /= ACC2 **
FDIV: tst EXPNT2 ; -*test*-('") for x/0.0
breq FDIVZ
tst EXPNT1
breq MINRES ; 0.0f/x = return 0.0f
FDIVa: rcall UNPACK
breq MINRES
eor sign1,sign2; get result sign
sec
sbc EXPNT1,EXPNT2; sub exps.
brvs MAXRES ; overflow ?
lsr mant2h ; no, de-norm & div mants
ror mant2m
ror mant2
lsr mant1h
ror mant1m
ror mant1
rcall UDIVMANT
tst mant1h ; need norm?
brmi FDIVX ; no, exit
lsl mant1
rol mant1m
rol mant1h
subi EXPNT1,1 ; --exp
brvs MAXRES
FDIVX: rjmp REPACK ; re-pack
FDIVZ: rjmp MINRES
MAXRES: ldi temp,0x7f
mov EXPNT1,temp
or mant1h,temp
ldi temp,0xff
mov mant1,temp
mov mant1m,temp
ret
MINRES: clr mant1;Result = 0.0f
clr mant1m
clr mant1h
clr EXPNT1
clr sign1
ret
;
;** ACC1 *= *r30 **
FMULMEM: rcall FSRAMA2 ; Acc1 *= *r30
rjmp FMULT
;** ACC1 *= 10.0f **
FMUL10: ldi EXPNT2,0x41; IEEE 10.0f
ldi mant2h,0x20
clr mant2m
clr mant2
;** ACC1 *= ACC2 **
FMULT: rcall UNPACK
breq MINRES
cpi EXPNT2,0x80
breq MINRES
eor sign1,sign2
sec
adc EXPNT1,EXPNT2
brvs MAXRES;FMULMAX
rcall UMUL
tst mant1h
brmi FMULX
lsl quoth
rol mant1
rol mant1m
rol mant1h
rjmp REPACK
FMULX: subi EXPNT1,-1
brvs MAXRES;FMULMAX
rjmp REPACK
;
;** 24bit unsigned Multiply
UMUL: push temp
push count
clr quot
clr quotm
clr quoth
clr temp
ldi count,24
UMULLP: lsl quot
rol quotm
rol quoth
rol mant1
rol mant1m
rol mant1h
brcc UMUL1
add quot,mant2
adc quotm,mant2m
adc quoth,mant2h
adc mant1,temp
adc mant1m,temp
adc mant1h,temp
UMUL1: dec count
brne UMULLP
pop count
pop temp
ret
;
;** ACC1 = (*r30) * (*r30) **
FSQRMEM: rcall FSRAMA1
;** ACC1 *= ACC1 **
FSQR: rcall AC1TOAC2
rjmp FMULT
;
sqrthalf:
clr mant2
clr mant2m
clr mant2h
ldi EXPNT2,0x40
rjmp FDIVa ; div by 2 to get our guess
SQERR: rjmp MINRES ; add your own error handler here.
;** ACC1 = sqrt(*r30) **
FSQRTMEM: rcall FSRAMA1
;** ACC1 = sqrt(ACC1) **
FSQRT: tst EXPNT1
breq SQERR ; trap sqrt(0) or sqrt(-x)!
brmi SQERR
mov r5,mant1; save org Num. user must preserve r5-r8!
mov r6,mant1m
mov r7,mant1h
mov r8,EXPNT1
rcall sqrthalf; get first guess
rcall AC1TOAC2; put it in Acc2
ldi temp,5 ; number of iterations
SQRTLP: push temp
rcall PUSH2 ; save 'guess' results
mov mant1,r5; restore orig Number
mov mant1m,r6
mov mant1h,r7
mov EXPNT1,r8
rcall FDIV
rcall POP2
rcall FADD
rcall sqrthalf
rcall AC1TOAC2
pop temp
dec temp
brne SQRTLP
ret
;
;** Compare ACC1 == ACC2 **
;** return w/temp = 0,1,or -1 (Z,N flags set) **
FLTCPFLASH: rcall KTOAC2
rjmp FLTCP
;** compare two sram floats **
FLTCP2: push zl; save r30,r31, (pointer to acc2 variable)
push zh
mov zl,yl
mov zh,yh
rcall FSRAMA1
pop zh
pop zl
FLTCPMEM: rcall FSRAMA2 ; compare Acc1 w/Memory
FLTCP: rcall FCMP ; do compare
tst temp ; set status reg accordingly
ret  ; return to caller
;
FCMP: tst EXPNT1 ; is acc1 pos?
brmi FCMP1 ; no. -*test*-('") acc2
tst EXPNT2 ; yes, is acc2 pos?
brmi A1GTA2 ; no, return Acc1 > Acc2
cp EXPNT1,EXPNT2; both are '+'
brlo A1LTA2
brne A1GTA2
cp mant1,mant2; exps are equal, are mants?
cpc mant1m,mant2m
cpc mant1h,mant2h
brlo A1LTA2
breq A1EQA2
A1GTA2: ldi temp,1 ; ac1 > ac2
ret
A1LTA2: ldi temp,0xff; ac1 < ac2
ret
A1EQA2: clr temp ; ac1 = ac2
ret
;** we're here, acc1 must be '-' **
FCMP1: tst EXPNT2 ; if acc1 is '-' & acc2 is '+'
brpl A1LTA2 ; then acc1 is < acc2
cp EXPNT2,EXPNT1; both are '-', is EXPNT1 more negative?
brlo A1GTA2
brne A1LTA2
cp mant2,mant1
cpc mant2m,mant1m
cpc mant2h,mant1h
brlo A1GTA2
breq A1EQA2 ; return acc1 == acc2
rjmp A1LTA2
;
;*********************************************************************
; * These routines are for future atof(), atoi(), atol(), etc.
;** Fast Multiply by 10 **
UMUL10: rcall AC1TOAC2
rcall RLMANT1
rcall RLMANT1
rcall UADD
sbci mant1t,-1; adci 0
rjmp RLMANT1
;
ADDDIG: rcall UMUL10
clr mant2m
clr mant2h
mov mant2,temp
andi mant2,0x0f
rcall UADD
sbci mant1t,-1; just add 'C'
ret
;*********************************************************************
;
;** convert an integer to FP **
ITOFMEM:
		RCALL	FSRAMA1 					; psuedo 32bit long?
		mov 	mant1t,EXPNT1
ITOF:	mov		temp,mant1
		or 		temp,mant1m
		or 		temp,mant1h
		or 		temp,mant1t
		brne 	ITOF1
		rjmp 	MINRES
ITOF1:
		tst		mant1t
		brpl 	ITOF2
		rcall 	NEGMANT
ITOF2: 	ldi 	EXPNT1,30
		tst 	mant1t
ITOF3: 	brmi 	ITOF4
		dec 	EXPNT1
		lsl 	mant1
		rol 	mant1m
		rol 	mant1h
		rol 	mant1t
		rjmp 	ITOF3
ITOF4: 	mov 	mant1,mant1m
		mov		mant1m,mant1h
		mov		mant1h,mant1t
		rjmp	REPACK
;
;********************************************************************
;** Memory Move funcs **
;** Copy SRAM Float to Acc1 **
FSRAMA1: ld mant1,z+
ld mant1m,z+
ld mant1h,z+
ld EXPNT1,z+
ret
;
;** Copy SRAM Float to Acc2 **
FSRAMA2: ld mant2,z+
ld mant2m,z+
ld mant2h,z+
ld EXPNT2,z+
ret
;
;
;** Copy ACC2 = ACC1 **
AC1TOAC2:
mov EXPNT2,EXPNT1
mov mant2,mant1
mov mant2m,mant1m
mov mant2h,mant1h
mov sign2,sign1
ret
;
;** ACC1 <-> ACC2 **
SWAPACC: push mant1
push mant1m
push mant1h
push EXPNT1
push sign1
mov mant1,mant2
mov mant1m,mant2m
mov mant1h,mant2h
mov EXPNT1,EXPNT2
mov sign1,sign2
pop sign2
pop EXPNT2
pop mant2h
pop mant2m
pop mant2
ret
;
;** Get 1 byte from program memory & inc pointer **
_lpmbyte: lpm
adiw zl,1
ret
;
;** Put a Flash const -> ACC1
KTOAC1: rcall _lpmbyte
mov mant1,r0
rcall _lpmbyte
mov mant1m,r0
rcall _lpmbyte
mov mant1h,r0
rcall _lpmbyte
mov EXPNT1,r0
ret
;
;** Put a Flash const -> ACC2
KTOAC2: rcall _lpmbyte
mov mant2,r0
rcall _lpmbyte
mov mant2m,r0
rcall _lpmbyte
mov mant2h,r0
rcall _lpmbyte
mov EXPNT2,r0
ret
;
;** 'Hold' Acc1 in sram **
PUSH1: sts MATHTMP,mant1
sts MATHTMP+1,mant1m
sts MATHTMP+2,mant1h
sts MATHTMP+3,EXPNT1
ret
;
;** 'Restore' Acc1 from sram **
POP1: lds mant1,MATHTMP
lds mant1m,MATHTMP+1
lds mant1h,MATHTMP+2
lds EXPNT1,MATHTMP+3
ret
;
;** 'Hold' Acc2 in sram **
PUSH2: sts MATHTMP+4,mant2
sts MATHTMP+5,mant2m
sts MATHTMP+6,mant2h
sts MATHTMP+7,EXPNT2
ret
;
;** 'Restore' Acc2 from sram **
POP2: lds mant2,MATHTMP+4
lds mant2m,MATHTMP+5
lds mant2h,MATHTMP+6
lds EXPNT2,MATHTMP+7
ret
;
;********************************************************************
;** IEEE convertions **
;** Convert ACC1 & ACC2 from IEEE to work format **
UNPACK: push temp
mov sign1,EXPNT1
ldi temp,0x80
lsl mant1h
rol EXPNT1
eor EXPNT1,temp; AVR has no 'eori'
lsl temp
ror mant1h ; restore Hidden '1' bit
andi sign1,0x80
;** Un-Pack Acc2 **
mov sign2,EXPNT2
ldi temp,0x80
lsl mant2h
rol EXPNT2
eor EXPNT2,temp
lsl temp
ror mant2h
andi sign2,0x80
pop temp
cpi EXPNT1,0x80; return w/ACC1=0 ?
ret
;
;** Convert Acc1 from work format to IEEE **
REPACK: push temp
lsl mant1h
ldi temp,0x80
eor temp,EXPNT1
lsl sign1
ror temp
ror mant1h
mov EXPNT1,temp
pop temp
ret
;
;** Constants used for decimal conversion **
I1E6: .db 0x40,0x42,0x0f,0;  1,000,000
I1E5: .db 0xa0,0x86,0x01,0;    100,000
I1E4: .db 0x10,0x27,0x00,0;     10,000
I1E3: .db 0xe8,0x03,0x00,0;      1,000
I1E2: .db 0x64,0x00,0x00,0;        100
I1E1: .db 0x0a,0x00,0x00,0;         10
;
;** IEEE format **
PI: .db 0xdb,0x0f,0x49,0x40; 3.1415927f
Kp1: .db 0xCD,0xCC,0xCC,0x3D; 0.1f
Kp5: .db 0x00,0x00,0x00,0x3F; 0.5f
K1: .db 0x00,0x00,0x80,0x3F; 1.0f
K10: .db 0x00,0x00,0x20,0x41;10.0f
;
K1E7: .db 0x80,0x96,0x18,0x4b
K1E8: .db 0x20,0xbc,0xbe,0x4c
K999999p9: .db 0xfe,0x23,0x74,0x49; 999999.9f
K9999999:  .db 0x7f,0x96,0x18,0x4b;9999999.0f
;


