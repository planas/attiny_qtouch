
hex/tiny20_qt.hex:     file format ihex


Disassembly of section .sec1:

00000000 <.sec1>:
   0:	c5 c2       	rjmp	.+1418   	;  0x58c
   2:	18 95       	reti
   4:	18 95       	reti
   6:	18 95       	reti
   8:	18 95       	reti
   a:	18 95       	reti
   c:	18 95       	reti
   e:	18 95       	reti
  10:	18 95       	reti
  12:	a2 c2       	rjmp	.+1348   	;  0x558
  14:	18 95       	reti
  16:	18 95       	reti
  18:	18 95       	reti
  1a:	18 95       	reti
  1c:	18 95       	reti
  1e:	18 95       	reti
  20:	18 95       	reti
  22:	22 27       	eor	r18, r18
  24:	20 a8       	sts	0x00, r18	;  0x800000
  26:	4a 93       	st	-Y, r20
  28:	1a 93       	st	-Y, r17
  2a:	aa 93       	st	-Y, r26
  2c:	ba 93       	st	-Y, r27
  2e:	9a 93       	st	-Y, r25
  30:	8a 93       	st	-Y, r24
  32:	0a 93       	st	-Y, r16
  34:	eb ea       	ldi	r30, 0xAB	; 171
  36:	f0 e0       	ldi	r31, 0x00	; 0
  38:	a7 e9       	ldi	r26, 0x97	; 151
  3a:	b0 e0       	ldi	r27, 0x00	; 0
  3c:	11 e0       	ldi	r17, 0x01	; 1
  3e:	11 23       	and	r17, r17
  40:	e1 f0       	breq	.+56     	;  0x7a
  42:	41 91       	ld	r20, Z+
  44:	8d 91       	ld	r24, X+
  46:	9d 91       	ld	r25, X+
  48:	97 fd       	sbrc	r25, 7
  4a:	80 e0       	ldi	r24, 0x00	; 0
  4c:	01 e0       	ldi	r16, 0x01	; 1
  4e:	32 17       	cp	r19, r18
  50:	79 f0       	breq	.+30     	;  0x70
  52:	10 17       	cp	r17, r16
  54:	69 f4       	brne	.+26     	;  0x70
  56:	4f 70       	andi	r20, 0x0F	; 15
  58:	44 23       	and	r20, r20
  5a:	39 f0       	breq	.+14     	;  0x6a
  5c:	80 30       	cpi	r24, 0x00	; 0
  5e:	11 f0       	breq	.+4      	;  0x64
  60:	2d d2       	rcall	.+1114   	;  0x4bc
  62:	36 d2       	rcall	.+1132   	;  0x4d0
  64:	68 17       	cp	r22, r24
  66:	08 f0       	brcs	.+2      	;  0x6a
  68:	03 c0       	rjmp	.+6      	;  0x70
  6a:	01 e0       	ldi	r16, 0x01	; 1
  6c:	00 a8       	sts	0x00, r16	;  0x800000
  6e:	05 c0       	rjmp	.+10     	;  0x7a
  70:	23 95       	inc	r18
  72:	00 27       	eor	r16, r16
  74:	00 a8       	sts	0x00, r16	;  0x800000
  76:	25 30       	cpi	r18, 0x05	; 5
  78:	21 f7       	brne	.-56     	;  0x42
  7a:	09 91       	ld	r16, Y+
  7c:	89 91       	ld	r24, Y+
  7e:	99 91       	ld	r25, Y+
  80:	b9 91       	ld	r27, Y+
  82:	a9 91       	ld	r26, Y+
  84:	19 91       	ld	r17, Y+
  86:	49 91       	ld	r20, Y+
  88:	08 95       	ret
  8a:	53 e0       	ldi	r21, 0x03	; 3
  8c:	22 27       	eor	r18, r18
  8e:	21 a8       	sts	0x01, r18	;  0x800001
  90:	8f e0       	ldi	r24, 0x0F	; 15
  92:	33 27       	eor	r19, r19
  94:	99 27       	eor	r25, r25
  96:	eb ea       	ldi	r30, 0xAB	; 171
  98:	f0 e0       	ldi	r31, 0x00	; 0
  9a:	51 93       	st	Z+, r21
  9c:	33 95       	inc	r19
  9e:	35 30       	cpi	r19, 0x05	; 5
  a0:	e1 f7       	brne	.-8      	;  0x9a
  a2:	33 27       	eor	r19, r19
  a4:	55 27       	eor	r21, r21
  a6:	e1 ea       	ldi	r30, 0xA1	; 161
  a8:	f0 e0       	ldi	r31, 0x00	; 0
  aa:	81 93       	st	Z+, r24
  ac:	91 93       	st	Z+, r25
  ae:	33 95       	inc	r19
  b0:	35 30       	cpi	r19, 0x05	; 5
  b2:	d9 f7       	brne	.-10     	;  0xaa
  b4:	08 95       	ret
  b6:	53 e0       	ldi	r21, 0x03	; 3
  b8:	8f e0       	ldi	r24, 0x0F	; 15
  ba:	08 95       	ret
  bc:	33 27       	eor	r19, r19
  be:	b0 e0       	ldi	r27, 0x00	; 0
  c0:	21 e0       	ldi	r18, 0x01	; 1
  c2:	eb ea       	ldi	r30, 0xAB	; 171
  c4:	f0 e0       	ldi	r31, 0x00	; 0
  c6:	a1 91       	ld	r26, Z+
  c8:	aa 23       	and	r26, r26
  ca:	09 f4       	brne	.+2      	;  0xce
  cc:	b2 2b       	or	r27, r18
  ce:	33 95       	inc	r19
  d0:	22 0f       	add	r18, r18
  d2:	35 30       	cpi	r19, 0x05	; 5
  d4:	c1 f7       	brne	.-16     	;  0xc6
  d6:	b0 ae       	sts	0x30, r27	;  0x800030
  d8:	08 95       	ret
  da:	4a 93       	st	-Y, r20
  dc:	5a 93       	st	-Y, r21
  de:	2a 93       	st	-Y, r18
  e0:	3a 93       	st	-Y, r19
  e2:	8a 93       	st	-Y, r24
  e4:	9a 93       	st	-Y, r25
  e6:	ea 93       	st	-Y, r30
  e8:	fa 93       	st	-Y, r31
  ea:	44 27       	eor	r20, r20
  ec:	42 a8       	sts	0x02, r20	;  0x800002
  ee:	cd df       	rcall	.-102    	;  0x8a
  f0:	f9 91       	ld	r31, Y+
  f2:	e9 91       	ld	r30, Y+
  f4:	99 91       	ld	r25, Y+
  f6:	89 91       	ld	r24, Y+
  f8:	39 91       	ld	r19, Y+
  fa:	29 91       	ld	r18, Y+
  fc:	59 91       	ld	r21, Y+
  fe:	49 91       	ld	r20, Y+
 100:	08 95       	ret
 102:	0a 93       	st	-Y, r16
 104:	1a 93       	st	-Y, r17
 106:	2a 93       	st	-Y, r18
 108:	3a 93       	st	-Y, r19
 10a:	4a 93       	st	-Y, r20
 10c:	5a 93       	st	-Y, r21
 10e:	6a 93       	st	-Y, r22
 110:	7a 93       	st	-Y, r23
 112:	8a 93       	st	-Y, r24
 114:	9a 93       	st	-Y, r25
 116:	aa 93       	st	-Y, r26
 118:	ba 93       	st	-Y, r27
 11a:	ea 93       	st	-Y, r30
 11c:	fa 93       	st	-Y, r31
 11e:	42 a0       	lds	r20, 0x02	;  0x800002
 120:	00 27       	eor	r16, r16
 122:	00 ae       	sts	0x30, r16	;  0x800030
 124:	a3 e8       	ldi	r26, 0x83	; 131
 126:	a2 af       	sts	0x72, r26	;  0x800072
 128:	a0 e0       	ldi	r26, 0x00	; 0
 12a:	a3 af       	sts	0x73, r26	;  0x800073
 12c:	ad e8       	ldi	r26, 0x8D	; 141
 12e:	a0 af       	sts	0x70, r26	;  0x800070
 130:	a0 e0       	ldi	r26, 0x00	; 0
 132:	a1 af       	sts	0x71, r26	;  0x800071
 134:	a1 ea       	ldi	r26, 0xA1	; 161
 136:	aa af       	sts	0x7a, r26	;  0x80007a
 138:	a0 e0       	ldi	r26, 0x00	; 0
 13a:	ab af       	sts	0x7b, r26	;  0x80007b
 13c:	a7 e9       	ldi	r26, 0x97	; 151
 13e:	a8 af       	sts	0x78, r26	;  0x800078
 140:	a0 e0       	ldi	r26, 0x00	; 0
 142:	a9 af       	sts	0x79, r26	;  0x800079
 144:	ab ea       	ldi	r26, 0xAB	; 171
 146:	a4 af       	sts	0x74, r26	;  0x800074
 148:	a0 e0       	ldi	r26, 0x00	; 0
 14a:	a5 af       	sts	0x75, r26	;  0x800075
 14c:	a1 eb       	ldi	r26, 0xB1	; 177
 14e:	a6 af       	sts	0x76, r26	;  0x800076
 150:	a0 e0       	ldi	r26, 0x00	; 0
 152:	a7 af       	sts	0x77, r26	;  0x800077
 154:	06 a6       	lds	r16, 0x36	;  0x800036
 156:	44 23       	and	r20, r20
 158:	09 f0       	breq	.+2      	;  0x15c
 15a:	4a 95       	dec	r20
 15c:	f8 94       	cli
 15e:	cf 93       	push	r28
 160:	df 93       	push	r29
 162:	e2 a7       	lds	r30, 0x72	;  0x800072
 164:	f3 a7       	lds	r31, 0x73	;  0x800073
 166:	c4 a7       	lds	r28, 0x74	;  0x800074
 168:	d5 a7       	lds	r29, 0x75	;  0x800075
 16a:	31 e0       	ldi	r19, 0x01	; 1
 16c:	59 91       	ld	r21, Y+
 16e:	cf 93       	push	r28
 170:	df 93       	push	r29
 172:	c1 91       	ld	r28, Z+
 174:	d0 81       	ld	r29, Z
 176:	b1 e0       	ldi	r27, 0x01	; 1
 178:	aa 27       	eor	r26, r26
 17a:	bb 0f       	add	r27, r27
 17c:	a3 95       	inc	r26
 17e:	a3 13       	cpse	r26, r19
 180:	fc cf       	rjmp	.-8      	;  0x17a
 182:	11 27       	eor	r17, r17
 184:	66 27       	eor	r22, r22
 186:	77 27       	eor	r23, r23
 188:	a8 e0       	ldi	r26, 0x08	; 8
 18a:	a0 bb       	out	0x10, r26	; 16
 18c:	a2 b1       	in	r26, 0x02	; 2
 18e:	ab 2b       	or	r26, r27
 190:	a2 b9       	out	0x02, r26	; 2
 192:	a1 b1       	in	r26, 0x01	; 1
 194:	ab 2b       	or	r26, r27
 196:	a1 b9       	out	0x01, r26	; 1
 198:	89 d1       	rcall	.+786    	;  0x4ac
 19a:	b0 95       	com	r27
 19c:	ab 23       	and	r26, r27
 19e:	a1 b9       	out	0x01, r26	; 1
 1a0:	30 bb       	out	0x10, r19	; 16
 1a2:	84 d1       	rcall	.+776    	;  0x4ac
 1a4:	a2 b3       	in	r26, 0x12	; 18
 1a6:	a0 64       	ori	r26, 0x40	; 64
 1a8:	a2 bb       	out	0x12, r26	; 18
 1aa:	a2 b3       	in	r26, 0x12	; 18
 1ac:	a4 ff       	sbrs	r26, 4
 1ae:	fd cf       	rjmp	.-6      	;  0x1aa
 1b0:	8e b1       	in	r24, 0x0e	; 14
 1b2:	9f b1       	in	r25, 0x0f	; 15
 1b4:	a8 e0       	ldi	r26, 0x08	; 8
 1b6:	a0 bb       	out	0x10, r26	; 16
 1b8:	a2 b1       	in	r26, 0x02	; 2
 1ba:	ab 23       	and	r26, r27
 1bc:	a2 b9       	out	0x02, r26	; 2
 1be:	b0 95       	com	r27
 1c0:	a1 b1       	in	r26, 0x01	; 1
 1c2:	ab 2b       	or	r26, r27
 1c4:	a1 b9       	out	0x01, r26	; 1
 1c6:	a2 b3       	in	r26, 0x12	; 18
 1c8:	a0 61       	ori	r26, 0x10	; 16
 1ca:	a2 bb       	out	0x12, r26	; 18
 1cc:	aa 27       	eor	r26, r26
 1ce:	a0 bb       	out	0x10, r26	; 16
 1d0:	b0 95       	com	r27
 1d2:	a1 b1       	in	r26, 0x01	; 1
 1d4:	ab 23       	and	r26, r27
 1d6:	a1 b9       	out	0x01, r26	; 1
 1d8:	30 bb       	out	0x10, r19	; 16
 1da:	68 d1       	rcall	.+720    	;  0x4ac
 1dc:	a2 b3       	in	r26, 0x12	; 18
 1de:	a0 64       	ori	r26, 0x40	; 64
 1e0:	a2 bb       	out	0x12, r26	; 18
 1e2:	a2 b3       	in	r26, 0x12	; 18
 1e4:	a4 ff       	sbrs	r26, 4
 1e6:	fd cf       	rjmp	.-6      	;  0x1e2
 1e8:	a2 b1       	in	r26, 0x02	; 2
 1ea:	ab 23       	and	r26, r27
 1ec:	a2 b9       	out	0x02, r26	; 2
 1ee:	b0 95       	com	r27
 1f0:	a1 b1       	in	r26, 0x01	; 1
 1f2:	ab 2b       	or	r26, r27
 1f4:	a1 b9       	out	0x01, r26	; 1
 1f6:	bf 93       	push	r27
 1f8:	2e b1       	in	r18, 0x0e	; 14
 1fa:	bf b1       	in	r27, 0x0f	; 15
 1fc:	a2 b3       	in	r26, 0x12	; 18
 1fe:	a0 61       	ori	r26, 0x10	; 16
 200:	a2 bb       	out	0x12, r26	; 18
 202:	81 50       	subi	r24, 0x01	; 1
 204:	9c 4f       	sbci	r25, 0xFC	; 252
 206:	82 1b       	sub	r24, r18
 208:	9b 0b       	sbc	r25, r27
 20a:	bf 91       	pop	r27
 20c:	88 94       	clc
 20e:	97 95       	ror	r25
 210:	87 95       	ror	r24
 212:	68 0f       	add	r22, r24
 214:	79 1f       	adc	r23, r25
 216:	13 95       	inc	r17
 218:	11 30       	cpi	r17, 0x01	; 1
 21a:	09 f0       	breq	.+2      	;  0x21e
 21c:	b5 cf       	rjmp	.-150    	;  0x188
 21e:	11 e0       	ldi	r17, 0x01	; 1
 220:	14 30       	cpi	r17, 0x04	; 4
 222:	30 f0       	brcs	.+12     	;  0x230
 224:	88 94       	clc
 226:	77 95       	ror	r23
 228:	67 95       	ror	r22
 22a:	16 95       	lsr	r17
 22c:	16 95       	lsr	r17
 22e:	f8 cf       	rjmp	.-16     	;  0x220
 230:	53 30       	cpi	r21, 0x03	; 3
 232:	19 f4       	brne	.+6      	;  0x23a
 234:	c6 2f       	mov	r28, r22
 236:	d7 2f       	mov	r29, r23
 238:	12 c0       	rjmp	.+36     	;  0x25e
 23a:	bd 2f       	mov	r27, r29
 23c:	2c 2f       	mov	r18, r28
 23e:	88 94       	clc
 240:	cc 1f       	adc	r28, r28
 242:	dd 1f       	adc	r29, r29
 244:	88 94       	clc
 246:	cc 1f       	adc	r28, r28
 248:	dd 1f       	adc	r29, r29
 24a:	c2 1b       	sub	r28, r18
 24c:	db 0b       	sbc	r29, r27
 24e:	c6 0f       	add	r28, r22
 250:	d7 1f       	adc	r29, r23
 252:	88 94       	clc
 254:	d7 95       	ror	r29
 256:	c7 95       	ror	r28
 258:	88 94       	clc
 25a:	d7 95       	ror	r29
 25c:	c7 95       	ror	r28
 25e:	d0 83       	st	Z, r29
 260:	c2 93       	st	-Z, r28
 262:	ee 5f       	subi	r30, 0xFE	; 254
 264:	df 91       	pop	r29
 266:	cf 91       	pop	r28
 268:	33 95       	inc	r19
 26a:	36 30       	cpi	r19, 0x06	; 6
 26c:	61 f0       	breq	.+24     	;  0x286
 26e:	bd 2f       	mov	r27, r29
 270:	ac 2f       	mov	r26, r28
 272:	df 91       	pop	r29
 274:	cf 91       	pop	r28
 276:	78 94       	sei
 278:	00 c0       	rjmp	.+0      	;  0x27a
 27a:	f8 94       	cli
 27c:	cf 93       	push	r28
 27e:	df 93       	push	r29
 280:	db 2f       	mov	r29, r27
 282:	ca 2f       	mov	r28, r26
 284:	73 cf       	rjmp	.-282    	;  0x16c
 286:	33 27       	eor	r19, r19
 288:	22 27       	eor	r18, r18
 28a:	21 a8       	sts	0x01, r18	;  0x800001
 28c:	df 91       	pop	r29
 28e:	cf 91       	pop	r28
 290:	78 94       	sei
 292:	e2 a7       	lds	r30, 0x72	;  0x800072
 294:	f3 a7       	lds	r31, 0x73	;  0x800073
 296:	a1 91       	ld	r26, Z+
 298:	ae af       	sts	0x7e, r26	;  0x80007e
 29a:	a0 81       	ld	r26, Z
 29c:	af af       	sts	0x7f, r26	;  0x80007f
 29e:	e0 a7       	lds	r30, 0x70	;  0x800070
 2a0:	f1 a7       	lds	r31, 0x71	;  0x800071
 2a2:	a1 91       	ld	r26, Z+
 2a4:	ac af       	sts	0x7c, r26	;  0x80007c
 2a6:	a0 81       	ld	r26, Z
 2a8:	ad af       	sts	0x7d, r26	;  0x80007d
 2aa:	ea a7       	lds	r30, 0x7a	;  0x80007a
 2ac:	fb a7       	lds	r31, 0x7b	;  0x80007b
 2ae:	81 91       	ld	r24, Z+
 2b0:	90 81       	ld	r25, Z
 2b2:	e6 a7       	lds	r30, 0x76	;  0x800076
 2b4:	f7 a7       	lds	r31, 0x77	;  0x800077
 2b6:	10 81       	ld	r17, Z
 2b8:	e4 a7       	lds	r30, 0x74	;  0x800074
 2ba:	f5 a7       	lds	r31, 0x75	;  0x800075
 2bc:	50 81       	ld	r21, Z
 2be:	aa 27       	eor	r26, r26
 2c0:	6c a7       	lds	r22, 0x7c	;  0x80007c
 2c2:	a3 95       	inc	r26
 2c4:	7d a7       	lds	r23, 0x7d	;  0x80007d
 2c6:	60 95       	com	r22
 2c8:	70 95       	com	r23
 2ca:	6f 5f       	subi	r22, 0xFF	; 255
 2cc:	7f 4f       	sbci	r23, 0xFF	; 255
 2ce:	ea 93       	st	-Y, r30
 2d0:	fa 93       	st	-Y, r31
 2d2:	ee a7       	lds	r30, 0x7e	;  0x80007e
 2d4:	ff a7       	lds	r31, 0x7f	;  0x80007f
 2d6:	6e 0f       	add	r22, r30
 2d8:	7f 1f       	adc	r23, r31
 2da:	f9 91       	ld	r31, Y+
 2dc:	e9 91       	ld	r30, Y+
 2de:	1a f0       	brmi	.+6      	;  0x2e6
 2e0:	77 23       	and	r23, r23
 2e2:	09 f0       	breq	.+2      	;  0x2e6
 2e4:	6f ef       	ldi	r22, 0xFF	; 255
 2e6:	50 30       	cpi	r21, 0x00	; 0
 2e8:	09 f4       	brne	.+2      	;  0x2ec
 2ea:	73 c0       	rjmp	.+230    	;  0x3d2
 2ec:	51 30       	cpi	r21, 0x01	; 1
 2ee:	29 f1       	breq	.+74     	;  0x33a
 2f0:	52 30       	cpi	r21, 0x02	; 2
 2f2:	09 f4       	brne	.+2      	;  0x2f6
 2f4:	58 c0       	rjmp	.+176    	;  0x3a6
 2f6:	8a 95       	dec	r24
 2f8:	20 e8       	ldi	r18, 0x80	; 128
 2fa:	21 a8       	sts	0x01, r18	;  0x800001
 2fc:	48 e2       	ldi	r20, 0x28	; 40
 2fe:	8a 30       	cpi	r24, 0x0A	; 10
 300:	98 f4       	brcc	.+38     	;  0x328
 302:	ac a7       	lds	r26, 0x7c	;  0x80007c
 304:	bd a7       	lds	r27, 0x7d	;  0x80007d
 306:	77 ff       	sbrs	r23, 7
 308:	03 c0       	rjmp	.+6      	;  0x310
 30a:	a1 50       	subi	r26, 0x01	; 1
 30c:	b0 40       	sbci	r27, 0x00	; 0
 30e:	06 c0       	rjmp	.+12     	;  0x31c
 310:	66 23       	and	r22, r22
 312:	11 f4       	brne	.+4      	;  0x318
 314:	77 23       	and	r23, r23
 316:	11 f0       	breq	.+4      	;  0x31c
 318:	af 5f       	subi	r26, 0xFF	; 255
 31a:	bf 4f       	sbci	r27, 0xFF	; 255
 31c:	ac af       	sts	0x7c, r26	;  0x80007c
 31e:	bd af       	sts	0x7d, r27	;  0x80007d
 320:	88 23       	and	r24, r24
 322:	09 f4       	brne	.+2      	;  0x326
 324:	74 c0       	rjmp	.+232    	;  0x40e
 326:	77 c0       	rjmp	.+238    	;  0x416
 328:	ea 93       	st	-Y, r30
 32a:	fa 93       	st	-Y, r31
 32c:	ee a7       	lds	r30, 0x7e	;  0x80007e
 32e:	ff a7       	lds	r31, 0x7f	;  0x80007f
 330:	ec af       	sts	0x7c, r30	;  0x80007c
 332:	fd af       	sts	0x7d, r31	;  0x80007d
 334:	f9 91       	ld	r31, Y+
 336:	e9 91       	ld	r30, Y+
 338:	6e c0       	rjmp	.+220    	;  0x416
 33a:	77 fd       	sbrc	r23, 7
 33c:	07 c0       	rjmp	.+14     	;  0x34c
 33e:	d2 d0       	rcall	.+420    	;  0x4e4
 340:	28 f0       	brcs	.+10     	;  0x34c
 342:	52 e0       	ldi	r21, 0x02	; 2
 344:	20 e8       	ldi	r18, 0x80	; 128
 346:	21 a8       	sts	0x01, r18	;  0x800001
 348:	11 e0       	ldi	r17, 0x01	; 1
 34a:	2c c0       	rjmp	.+88     	;  0x3a4
 34c:	00 23       	and	r16, r16
 34e:	51 f5       	brne	.+84     	;  0x3a4
 350:	77 ff       	sbrs	r23, 7
 352:	18 c0       	rjmp	.+48     	;  0x384
 354:	7f 3f       	cpi	r23, 0xFF	; 255
 356:	11 f4       	brne	.+4      	;  0x35c
 358:	cf d0       	rcall	.+414    	;  0x4f8
 35a:	34 f4       	brge	.+12     	;  0x368
 35c:	93 95       	inc	r25
 35e:	9b 30       	cpi	r25, 0x0B	; 11
 360:	08 f4       	brcc	.+2      	;  0x364
 362:	05 c0       	rjmp	.+10     	;  0x36e
 364:	a8 de       	rcall	.-688    	;  0xb6
 366:	57 c0       	rjmp	.+174    	;  0x416
 368:	44 23       	and	r20, r20
 36a:	e1 f4       	brne	.+56     	;  0x3a4
 36c:	99 27       	eor	r25, r25
 36e:	83 95       	inc	r24
 370:	8b 38       	cpi	r24, 0x8B	; 139
 372:	38 f0       	brcs	.+14     	;  0x382
 374:	80 e8       	ldi	r24, 0x80	; 128
 376:	ac a7       	lds	r26, 0x7c	;  0x80007c
 378:	bd a7       	lds	r27, 0x7d	;  0x80007d
 37a:	a1 50       	subi	r26, 0x01	; 1
 37c:	b0 40       	sbci	r27, 0x00	; 0
 37e:	ac af       	sts	0x7c, r26	;  0x80007c
 380:	bd af       	sts	0x7d, r27	;  0x80007d
 382:	ce cf       	rjmp	.-100    	;  0x320
 384:	44 23       	and	r20, r20
 386:	71 f4       	brne	.+28     	;  0x3a4
 388:	99 27       	eor	r25, r25
 38a:	69 17       	cp	r22, r25
 38c:	79 07       	cpc	r23, r25
 38e:	41 f2       	breq	.-112    	;  0x320
 390:	8a 95       	dec	r24
 392:	88 35       	cpi	r24, 0x58	; 88
 394:	38 f4       	brcc	.+14     	;  0x3a4
 396:	80 e8       	ldi	r24, 0x80	; 128
 398:	ac a7       	lds	r26, 0x7c	;  0x80007c
 39a:	bd a7       	lds	r27, 0x7d	;  0x80007d
 39c:	af 5f       	subi	r26, 0xFF	; 255
 39e:	bf 4f       	sbci	r27, 0xFF	; 255
 3a0:	ac af       	sts	0x7c, r26	;  0x80007c
 3a2:	bd af       	sts	0x7d, r27	;  0x80007d
 3a4:	38 c0       	rjmp	.+112    	;  0x416
 3a6:	20 e8       	ldi	r18, 0x80	; 128
 3a8:	21 a8       	sts	0x01, r18	;  0x800001
 3aa:	48 e2       	ldi	r20, 0x28	; 40
 3ac:	77 fd       	sbrc	r23, 7
 3ae:	2f c0       	rjmp	.+94     	;  0x40e
 3b0:	99 d0       	rcall	.+306    	;  0x4e4
 3b2:	08 f4       	brcc	.+2      	;  0x3b6
 3b4:	2c c0       	rjmp	.+88     	;  0x40e
 3b6:	13 95       	inc	r17
 3b8:	34 de       	rcall	.-920    	;  0x22
 3ba:	a0 a0       	lds	r26, 0x00	;  0x800000
 3bc:	a0 fd       	sbrc	r26, 0
 3be:	11 27       	eor	r17, r17
 3c0:	14 30       	cpi	r17, 0x04	; 4
 3c2:	28 f0       	brcs	.+10     	;  0x3ce
 3c4:	14 e0       	ldi	r17, 0x04	; 4
 3c6:	50 e0       	ldi	r21, 0x00	; 0
 3c8:	88 27       	eor	r24, r24
 3ca:	99 27       	eor	r25, r25
 3cc:	24 c0       	rjmp	.+72     	;  0x416
 3ce:	44 27       	eor	r20, r20
 3d0:	22 c0       	rjmp	.+68     	;  0x416
 3d2:	48 e2       	ldi	r20, 0x28	; 40
 3d4:	77 fd       	sbrc	r23, 7
 3d6:	1b c0       	rjmp	.+54     	;  0x40e
 3d8:	8a d0       	rcall	.+276    	;  0x4ee
 3da:	38 f4       	brcc	.+14     	;  0x3ea
 3dc:	20 e8       	ldi	r18, 0x80	; 128
 3de:	21 a8       	sts	0x01, r18	;  0x800001
 3e0:	11 23       	and	r17, r17
 3e2:	a9 f0       	breq	.+42     	;  0x40e
 3e4:	1a 95       	dec	r17
 3e6:	99 f0       	breq	.+38     	;  0x40e
 3e8:	03 c0       	rjmp	.+6      	;  0x3f0
 3ea:	14 30       	cpi	r17, 0x04	; 4
 3ec:	08 f4       	brcc	.+2      	;  0x3f0
 3ee:	13 95       	inc	r17
 3f0:	00 23       	and	r16, r16
 3f2:	61 f4       	brne	.+24     	;  0x40c
 3f4:	ac e2       	ldi	r26, 0x2C	; 44
 3f6:	aa 23       	and	r26, r26
 3f8:	11 f4       	brne	.+4      	;  0x3fe
 3fa:	a1 30       	cpi	r26, 0x01	; 1
 3fc:	39 f0       	breq	.+14     	;  0x40c
 3fe:	8f 5f       	subi	r24, 0xFF	; 255
 400:	9f 4f       	sbci	r25, 0xFF	; 255
 402:	a1 e0       	ldi	r26, 0x01	; 1
 404:	8c 32       	cpi	r24, 0x2C	; 44
 406:	9a 07       	cpc	r25, r26
 408:	08 f0       	brcs	.+2      	;  0x40c
 40a:	55 de       	rcall	.-854    	;  0xb6
 40c:	04 c0       	rjmp	.+8      	;  0x416
 40e:	51 e0       	ldi	r21, 0x01	; 1
 410:	11 27       	eor	r17, r17
 412:	80 e8       	ldi	r24, 0x80	; 128
 414:	99 27       	eor	r25, r25
 416:	e2 a7       	lds	r30, 0x72	;  0x800072
 418:	f3 a7       	lds	r31, 0x73	;  0x800073
 41a:	aa 93       	st	-Y, r26
 41c:	ba 93       	st	-Y, r27
 41e:	ae a7       	lds	r26, 0x7e	;  0x80007e
 420:	bf a7       	lds	r27, 0x7f	;  0x80007f
 422:	a1 93       	st	Z+, r26
 424:	b1 93       	st	Z+, r27
 426:	b9 91       	ld	r27, Y+
 428:	a9 91       	ld	r26, Y+
 42a:	e2 af       	sts	0x72, r30	;  0x800072
 42c:	f3 af       	sts	0x73, r31	;  0x800073
 42e:	e0 a7       	lds	r30, 0x70	;  0x800070
 430:	f1 a7       	lds	r31, 0x71	;  0x800071
 432:	ac a7       	lds	r26, 0x7c	;  0x80007c
 434:	a1 93       	st	Z+, r26
 436:	ad a7       	lds	r26, 0x7d	;  0x80007d
 438:	a1 93       	st	Z+, r26
 43a:	e0 af       	sts	0x70, r30	;  0x800070
 43c:	f1 af       	sts	0x71, r31	;  0x800071
 43e:	ea a7       	lds	r30, 0x7a	;  0x80007a
 440:	fb a7       	lds	r31, 0x7b	;  0x80007b
 442:	81 93       	st	Z+, r24
 444:	91 93       	st	Z+, r25
 446:	ea af       	sts	0x7a, r30	;  0x80007a
 448:	fb af       	sts	0x7b, r31	;  0x80007b
 44a:	e6 a7       	lds	r30, 0x76	;  0x800076
 44c:	f7 a7       	lds	r31, 0x77	;  0x800077
 44e:	11 93       	st	Z+, r17
 450:	e6 af       	sts	0x76, r30	;  0x800076
 452:	f7 af       	sts	0x77, r31	;  0x800077
 454:	e4 a7       	lds	r30, 0x74	;  0x800074
 456:	f5 a7       	lds	r31, 0x75	;  0x800075
 458:	51 93       	st	Z+, r21
 45a:	e4 af       	sts	0x74, r30	;  0x800074
 45c:	f5 af       	sts	0x75, r31	;  0x800075
 45e:	e8 a7       	lds	r30, 0x78	;  0x800078
 460:	f9 a7       	lds	r31, 0x79	;  0x800079
 462:	61 93       	st	Z+, r22
 464:	71 93       	st	Z+, r23
 466:	e8 af       	sts	0x78, r30	;  0x800078
 468:	f9 af       	sts	0x79, r31	;  0x800079
 46a:	33 95       	inc	r19
 46c:	35 30       	cpi	r19, 0x05	; 5
 46e:	09 f0       	breq	.+2      	;  0x472
 470:	10 cf       	rjmp	.-480    	;  0x292
 472:	21 a0       	lds	r18, 0x01	;  0x800001
 474:	22 23       	and	r18, r18
 476:	09 f0       	breq	.+2      	;  0x47a
 478:	53 ce       	rjmp	.-858    	;  0x120
 47a:	20 de       	rcall	.-960    	;  0xbc
 47c:	42 a8       	sts	0x02, r20	;  0x800002
 47e:	f9 91       	ld	r31, Y+
 480:	e9 91       	ld	r30, Y+
 482:	b9 91       	ld	r27, Y+
 484:	a9 91       	ld	r26, Y+
 486:	99 91       	ld	r25, Y+
 488:	89 91       	ld	r24, Y+
 48a:	79 91       	ld	r23, Y+
 48c:	69 91       	ld	r22, Y+
 48e:	59 91       	ld	r21, Y+
 490:	49 91       	ld	r20, Y+
 492:	39 91       	ld	r19, Y+
 494:	29 91       	ld	r18, Y+
 496:	19 91       	ld	r17, Y+
 498:	09 91       	ld	r16, Y+
 49a:	08 95       	ret
 49c:	ea 93       	st	-Y, r30
 49e:	fa 93       	st	-Y, r31
 4a0:	4a 93       	st	-Y, r20
 4a2:	08 95       	ret
 4a4:	49 91       	ld	r20, Y+
 4a6:	f9 91       	ld	r31, Y+
 4a8:	e9 91       	ld	r30, Y+
 4aa:	08 95       	ret
 4ac:	af 93       	push	r26
 4ae:	a1 e0       	ldi	r26, 0x01	; 1
 4b0:	aa 95       	dec	r26
 4b2:	12 f0       	brmi	.+4      	;  0x4b8
 4b4:	a0 30       	cpi	r26, 0x00	; 0
 4b6:	e1 f7       	brne	.-8      	;  0x4b0
 4b8:	af 91       	pop	r26
 4ba:	08 95       	ret
 4bc:	ef df       	rcall	.-34     	;  0x49c
 4be:	eb ee       	ldi	r30, 0xEB	; 235
 4c0:	f7 e4       	ldi	r31, 0x47	; 71
 4c2:	e2 0f       	add	r30, r18
 4c4:	40 81       	ld	r20, Z
 4c6:	84 1b       	sub	r24, r20
 4c8:	ed df       	rcall	.-38     	;  0x4a4
 4ca:	87 fd       	sbrc	r24, 7
 4cc:	80 e0       	ldi	r24, 0x00	; 0
 4ce:	08 95       	ret
 4d0:	e5 df       	rcall	.-54     	;  0x49c
 4d2:	eb ee       	ldi	r30, 0xEB	; 235
 4d4:	f7 e4       	ldi	r31, 0x47	; 71
 4d6:	e3 0f       	add	r30, r19
 4d8:	40 81       	ld	r20, Z
 4da:	64 1b       	sub	r22, r20
 4dc:	e3 df       	rcall	.-58     	;  0x4a4
 4de:	67 fd       	sbrc	r22, 7
 4e0:	60 e0       	ldi	r22, 0x00	; 0
 4e2:	08 95       	ret
 4e4:	db df       	rcall	.-74     	;  0x49c
 4e6:	eb ee       	ldi	r30, 0xEB	; 235
 4e8:	f7 e4       	ldi	r31, 0x47	; 71
 4ea:	0b d0       	rcall	.+22     	;  0x502
 4ec:	08 95       	ret
 4ee:	d6 df       	rcall	.-84     	;  0x49c
 4f0:	e0 ef       	ldi	r30, 0xF0	; 240
 4f2:	f7 e4       	ldi	r31, 0x47	; 71
 4f4:	06 d0       	rcall	.+12     	;  0x502
 4f6:	08 95       	ret
 4f8:	d1 df       	rcall	.-94     	;  0x49c
 4fa:	e5 ef       	ldi	r30, 0xF5	; 245
 4fc:	f7 e4       	ldi	r31, 0x47	; 71
 4fe:	01 d0       	rcall	.+2      	;  0x502
 500:	08 95       	ret
 502:	e3 0f       	add	r30, r19
 504:	40 81       	ld	r20, Z
 506:	64 17       	cp	r22, r20
 508:	cd df       	rcall	.-102    	;  0x4a4
 50a:	08 95       	ret
 50c:	00 e0       	ldi	r16, 0x00	; 0
 50e:	03 b9       	out	0x03, r16	; 3
 510:	03 e0       	ldi	r16, 0x03	; 3
 512:	08 b9       	out	0x08, r16	; 8
 514:	08 ed       	ldi	r16, 0xD8	; 216
 516:	0c bf       	out	0x3c, r16	; 60
 518:	01 e0       	ldi	r16, 0x01	; 1
 51a:	06 bf       	out	0x36, r16	; 54
 51c:	0f ef       	ldi	r16, 0xFF	; 255
 51e:	01 b9       	out	0x01, r16	; 1
 520:	0f e0       	ldi	r16, 0x0F	; 15
 522:	05 b9       	out	0x05, r16	; 5
 524:	10 9a       	sbi	0x02, 0	; 2
 526:	05 e8       	ldi	r16, 0x85	; 133
 528:	02 bb       	out	0x12, r16	; 18
 52a:	00 e0       	ldi	r16, 0x00	; 0
 52c:	01 bb       	out	0x11, r16	; 17
 52e:	08 95       	ret
 530:	ed df       	rcall	.-38     	;  0x50c
 532:	02 e0       	ldi	r16, 0x02	; 2
 534:	09 bb       	out	0x19, r16	; 25
 536:	00 e5       	ldi	r16, 0x50	; 80
 538:	06 bb       	out	0x16, r16	; 22
 53a:	02 e0       	ldi	r16, 0x02	; 2
 53c:	06 bd       	out	0x26, r16	; 38
 53e:	05 e0       	ldi	r16, 0x05	; 5
 540:	08 bb       	out	0x18, r16	; 24
 542:	78 94       	sei
 544:	ca dd       	rcall	.-1132   	;  0xda
 546:	e7 eb       	ldi	r30, 0xB7	; 183
 548:	f0 e0       	ldi	r31, 0x00	; 0
 54a:	00 81       	ld	r16, Z
 54c:	01 30       	cpi	r16, 0x01	; 1
 54e:	d9 f7       	brne	.-10     	;  0x546
 550:	00 e0       	ldi	r16, 0x00	; 0
 552:	00 83       	st	Z, r16
 554:	d6 dd       	rcall	.-1108   	;  0x102
 556:	f7 cf       	rjmp	.-18     	;  0x546
 558:	fa 93       	st	-Y, r31
 55a:	ea 93       	st	-Y, r30
 55c:	1a 93       	st	-Y, r17
 55e:	0a 93       	st	-Y, r16
 560:	1f b7       	in	r17, 0x3f	; 63
 562:	e7 eb       	ldi	r30, 0xB7	; 183
 564:	f0 e0       	ldi	r31, 0x00	; 0
 566:	01 e0       	ldi	r16, 0x01	; 1
 568:	00 83       	st	Z, r16
 56a:	e6 eb       	ldi	r30, 0xB6	; 182
 56c:	f0 e0       	ldi	r31, 0x00	; 0
 56e:	00 81       	ld	r16, Z
 570:	0c 5e       	subi	r16, 0xEC	; 236
 572:	00 83       	st	Z, r16
 574:	00 81       	ld	r16, Z
 576:	04 36       	cpi	r16, 0x64	; 100
 578:	10 f0       	brcs	.+4      	;  0x57e
 57a:	00 e0       	ldi	r16, 0x00	; 0
 57c:	00 83       	st	Z, r16
 57e:	1f bf       	out	0x3f, r17	; 63
 580:	09 91       	ld	r16, Y+
 582:	19 91       	ld	r17, Y+
 584:	e9 91       	ld	r30, Y+
 586:	f9 91       	ld	r31, Y+
 588:	18 95       	reti
 58a:	3d c0       	rjmp	.+122    	;  0x606
 58c:	0f e6       	ldi	r16, 0x6F	; 111
 58e:	0d bf       	out	0x3d, r16	; 61
 590:	00 e0       	ldi	r16, 0x00	; 0
 592:	0e bf       	out	0x3e, r16	; 62
 594:	cc e5       	ldi	r28, 0x5C	; 92
 596:	d0 e0       	ldi	r29, 0x00	; 0
 598:	07 d0       	rcall	.+14     	;  0x5a8
 59a:	00 23       	and	r16, r16
 59c:	09 f0       	breq	.+2      	;  0x5a0
 59e:	12 d0       	rcall	.+36     	;  0x5c4
 5a0:	c7 df       	rcall	.-114    	;  0x530
 5a2:	01 d0       	rcall	.+2      	;  0x5a6
 5a4:	f2 cf       	rjmp	.-28     	;  0x58a
 5a6:	f1 cf       	rjmp	.-30     	;  0x58a
 5a8:	01 e0       	ldi	r16, 0x01	; 1
 5aa:	08 95       	ret
 5ac:	20 e0       	ldi	r18, 0x00	; 0
 5ae:	2d 93       	st	X+, r18
 5b0:	01 50       	subi	r16, 0x01	; 1
 5b2:	10 40       	sbci	r17, 0x00	; 0
 5b4:	e1 f7       	brne	.-8      	;  0x5ae
 5b6:	08 95       	ret
 5b8:	21 91       	ld	r18, Z+
 5ba:	2d 93       	st	X+, r18
 5bc:	01 50       	subi	r16, 0x01	; 1
 5be:	10 40       	sbci	r17, 0x00	; 0
 5c0:	d9 f7       	brne	.-10     	;  0x5b8
 5c2:	08 95       	ret
 5c4:	4a ef       	ldi	r20, 0xFA	; 250
 5c6:	57 e4       	ldi	r21, 0x47	; 71
 5c8:	e4 2f       	mov	r30, r20
 5ca:	f5 2f       	mov	r31, r21
 5cc:	01 91       	ld	r16, Z+
 5ce:	10 81       	ld	r17, Z
 5d0:	e4 2f       	mov	r30, r20
 5d2:	f5 2f       	mov	r31, r21
 5d4:	ee 5f       	subi	r30, 0xFE	; 254
 5d6:	ff 4f       	sbci	r31, 0xFF	; 255
 5d8:	a1 91       	ld	r26, Z+
 5da:	b0 81       	ld	r27, Z
 5dc:	e4 2f       	mov	r30, r20
 5de:	f5 2f       	mov	r31, r21
 5e0:	ec 5f       	subi	r30, 0xFC	; 252
 5e2:	ff 4f       	sbci	r31, 0xFF	; 255
 5e4:	21 91       	ld	r18, Z+
 5e6:	30 81       	ld	r19, Z
 5e8:	62 2f       	mov	r22, r18
 5ea:	63 2b       	or	r22, r19
 5ec:	21 f0       	breq	.+8      	;  0x5f6
 5ee:	e2 2f       	mov	r30, r18
 5f0:	f3 2f       	mov	r31, r19
 5f2:	e2 df       	rcall	.-60     	;  0x5b8
 5f4:	01 c0       	rjmp	.+2      	;  0x5f8
 5f6:	da df       	rcall	.-76     	;  0x5ac
 5f8:	4a 5f       	subi	r20, 0xFA	; 250
 5fa:	5f 4f       	sbci	r21, 0xFF	; 255
 5fc:	18 e4       	ldi	r17, 0x48	; 72
 5fe:	40 30       	cpi	r20, 0x00	; 0
 600:	51 07       	cpc	r21, r17
 602:	10 f3       	brcs	.-60     	;  0x5c8
 604:	08 95       	ret
 606:	ff cf       	rjmp	.-2      	;  0x606
