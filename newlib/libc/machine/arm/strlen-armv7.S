/* Copyright (c) 2010-2011,2013 Linaro Limited
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

      * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

      * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

      * Neither the name of Linaro Limited nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Assumes:
   ARMv6T2, AArch32
 */

#include "arm_asm.h"

/* NOTE: This ifdef MUST match the ones in arm/strlen.c
   We fallback to the one in arm/strlen.c for size optimised or
   for older architectures. */
#if defined(_ISA_ARM_7) || defined(__ARM_ARCH_6T2__) && \
    !(defined (__OPTIMIZE_SIZE__) || defined (PREFER_SIZE_OVER_SPEED) || \
      (defined (__thumb__) && !defined (__thumb2__)))

	.macro def_fn f p2align=0
	.text
	.p2align \p2align
	.global \f
	.type \f, %function
\f:
	.endm

#ifdef __ARMEB__
#define S2LO		lsl
#define S2HI		lsr
#else
#define S2LO		lsr
#define S2HI		lsl
#endif

#ifndef __native_client__
	/* This code is best on Thumb.  */
	.thumb
#else /* ARM, not Thumb.  */
/* Using bne.w explicitly is desirable in Thumb mode because it helps
   align the following label without a nop.  In ARM mode there is no
   such difference.  */
.macro bne.w label
	bne \label
.endm

/* This clobbers the condition codes, which the real Thumb cbnz instruction
   does not do.  But it doesn't matter for any of the uses here.  */
.macro cbnz reg, label
	cmp \reg, #0
	bne \label
.endm
#endif
	.syntax unified

/* Parameters and result.  */
#define srcin		r0
#define result		r0

/* Internal variables.  */
#define src		r1
#define data1a		r2
#define data1b		r3
#define const_m1	r12
#define const_0		r4
#define tmp1		r4		/* Overlaps const_0  */
#define tmp2		r5

def_fn	strlen p2align=6
	SFI_BREG(srcin) \
	pld	[srcin, #0]
	strd	r4, r5, [sp, #-8]!
	bic	src, srcin, #7
	mvn	const_m1, #0
	ands	tmp1, srcin, #7		/* (8 - bytes) to alignment.  */
	SFI_BREG(src) \
	pld	[src, #32]
	bne.w	.Lmisaligned8
	mov	const_0, #0
	mov	result, #-8
.Lloop_aligned:
	/* Bytes 0-7.  */
	SFI_BREG(src) \
	ldrd	data1a, data1b, [src]
	SFI_BREG(src) \
	pld	[src, #64]
	add	result, result, #8
.Lstart_realigned:
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 8-15.  */
	SFI_BREG(src) \
	ldrd	data1a, data1b, [src, #8]
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 16-23.  */
	SFI_BREG(src) \
	ldrd	data1a, data1b, [src, #16]
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cbnz	data1b, .Lnull_found

	/* Bytes 24-31.  */
	SFI_BREG(src) \
	ldrd	data1a, data1b, [src, #24]
	add	src, src, #32
	uadd8	data1a, data1a, const_m1	/* Saturating GE<0:3> set.  */
	add	result, result, #8
	sel	data1a, const_0, const_m1	/* Select based on GE<0:3>.  */
	uadd8	data1b, data1b, const_m1
	sel	data1b, data1a, const_m1	/* Only used if d1a == 0.  */
	cmp	data1b, #0
	beq	.Lloop_aligned

.Lnull_found:
	cmp	data1a, #0
	itt	eq
	addeq	result, result, #4
	moveq	data1a, data1b
#ifndef __ARMEB__
	rev	data1a, data1a
#endif
	clz	data1a, data1a
	ldrd	r4, r5, [sp], #8
	add	result, result, data1a, lsr #3	/* Bits -> Bytes.  */
	RETURN

.Lmisaligned8:
	SFI_BREG(src) \
	ldrd	data1a, data1b, [src]
	and	tmp2, tmp1, #3
	rsb	result, tmp1, #0
	lsl	tmp2, tmp2, #3			/* Bytes -> bits.  */
	tst	tmp1, #4
	SFI_BREG(src) \
	pld	[src, #64]
	S2HI	tmp2, const_m1, tmp2
#ifdef __native_client__                /* ARM, not Thumb.  */
	mvn	tmp1, tmp2
	orr	data1a, data1a, tmp1
	itt	ne
	orrne	data1b, data1b, tmp1
#else
	orn	data1a, data1a, tmp2
	itt	ne
	ornne	data1b, data1b, tmp2
#endif
	movne	data1a, const_m1
	mov	const_0, #0
	b	.Lstart_realigned
	.size	strlen, . - strlen

#endif
