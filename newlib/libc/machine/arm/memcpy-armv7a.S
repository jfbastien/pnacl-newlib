/* Copyright (c) 2013, Linaro Limited
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

   This memcpy routine is optimised for Cortex-A15 cores and takes advantage
   of VFP or NEON when built with the appropriate flags.

   Assumptions:

    ARMv6 (ARMv7-a if using Neon)
    ARM state
    Unaligned accesses
    LDRD/STRD support unaligned word accesses

   If compiled with GCC, this file should be enclosed within following
   pre-processing check:
   if defined (__ARM_ARCH_7A__) && defined (__ARM_FEATURE_UNALIGNED)

 */

#include "arm_asm.h"

	.syntax unified
	/* This implementation requires ARM state.  */
	.arm

#if defined(__ARM_NEON__) || defined(__native_client__)

	.fpu	neon
	.arch	armv7-a
# define FRAME_SIZE	4
# define USE_VFP
# define USE_NEON

#elif !defined (__SOFTFP__)

	.arch	armv6
	.fpu	vfpv2
# define FRAME_SIZE	32
# define USE_VFP

#else
	.arch	armv6
# define FRAME_SIZE    32

#endif

/* Old versions of GAS incorrectly implement the NEON align semantics.  */
#ifdef BROKEN_ASM_NEON_ALIGN
#define ALIGN(addr, align) addr,:align
#else
#define ALIGN(addr, align) addr:align
#endif

#define PC_OFFSET	8	/* PC pipeline compensation.  */
#define INSN_SIZE	4

/* Call parameters.  */
#define dstin	r0
#define src	r1
#define count	r2

/* Locals.  */
#define tmp1	r3
#define dst	ip
#define tmp2	r8

/* These two macros both work by repeated invocation of the macro
   dispatch_step (not defined here).  That macro performs one "step",
   doing one load instruction and one store instruction to copy one
   "unit".  On entry, TMP1 contains the number of bytes to be copied,
   a multiple of the unit size.  The macro clobbers TMP1 in the
   process of doing a computed jump to the tail containing the
   appropriate number of steps.

   In dispatch_7_dword, dispatch_step is invoked seven times, with an
   argument that is 7 for the first and 1 for the last.  Units are
   double-words (8 bytes).  TMP1 is at most 56.

   In dispatch_15_word, dispatch_step is invoked fifteen times,
   with an argument that is 15 for the first and 1 for the last.
   Units are words (4 bytes).  TMP1 is at most 60.  */

#ifndef __native_client__
	.macro dispatch_7_dword
	rsb	tmp1, tmp1, #((7 * 8) - PC_OFFSET + INSN_SIZE)
	add	pc, pc, tmp1
	dispatch_step 7
	dispatch_step 6
	dispatch_step 5
	dispatch_step 4
	dispatch_step 3
	dispatch_step 2
	dispatch_step 1
	.purgem dispatch_step
	.endm

	.macro dispatch_15_word
	rsb	tmp1, tmp1, #((15 * 4) - PC_OFFSET/2 + INSN_SIZE/2)
	add	pc, pc, tmp1, lsl #1
	dispatch_step 15
	dispatch_step 14
	dispatch_step 13
	dispatch_step 12
	dispatch_step 11
	dispatch_step 10
	dispatch_step 9
	dispatch_step 8
	dispatch_step 7
	dispatch_step 6
	dispatch_step 5
	dispatch_step 4
	dispatch_step 3
	dispatch_step 2
	dispatch_step 1
	.purgem dispatch_step
	.endm
#else
	.macro dispatch_helper steps, log2_bytes_per_step
	/* TMP1 gets (max_bytes - bytes_to_copy), where max_bytes is
	   (STEPS << LOG2_BYTES_PER_STEP).
	   So this is (steps_to_skip << LOG2_BYTES_PER_STEP).
	   Then it needs further adjustment to compensate for the
	   distance between the PC value taken below (0f + PC_OFFSET)
	   and the first step's instructions (1f).  */
	rsb	tmp1, tmp1, #((\steps << \log2_bytes_per_step) \
			      + ((1f - PC_OFFSET - 0f) \
				 >> (4 - \log2_bytes_per_step)))
	/* Shifting down LOG2_BYTES_PER_STEP gives us the number of
	   steps to skip, then shifting up 4 gives us
	   the (byte) distance to add to the PC.  */
0:	add	tmp1, pc, tmp1, lsl #(4 - \log2_bytes_per_step)
	sfi_bx	tmp1
	.p2align 4
1:
	.endm

	.macro dispatch_7_dword
	dispatch_helper 7, 3
	.p2align 4
	dispatch_step 7
	.p2align 4
	dispatch_step 6
	.p2align 4
	dispatch_step 5
	.p2align 4
	dispatch_step 4
	.p2align 4
	dispatch_step 3
	.p2align 4
	dispatch_step 2
	.p2align 4
	dispatch_step 1
	.p2align 4
	.purgem dispatch_step
	.endm

	.macro dispatch_15_word
	dispatch_helper 15, 2
	dispatch_step 15
	.p2align 4
	dispatch_step 14
	.p2align 4
	dispatch_step 13
	.p2align 4
	dispatch_step 12
	.p2align 4
	dispatch_step 11
	.p2align 4
	dispatch_step 10
	.p2align 4
	dispatch_step 9
	.p2align 4
	dispatch_step 8
	.p2align 4
	dispatch_step 7
	.p2align 4
	dispatch_step 6
	.p2align 4
	dispatch_step 5
	.p2align 4
	dispatch_step 4
	.p2align 4
	dispatch_step 3
	.p2align 4
	dispatch_step 2
	.p2align 4
	dispatch_step 1
	.p2align 4
	.purgem dispatch_step
	.endm

#endif

#ifndef USE_NEON
/* For bulk copies using GP registers.  */
#define	A_l	r2		/* Call-clobbered.  */
#define	A_h	r3		/* Call-clobbered.  */
#define	B_l	r4
#define	B_h	r5
#define	C_l	r6
#define	C_h	r7
/* Don't use the pair r8,r9 because in some EABI variants r9 is reserved.  */
#define	D_l	r10
#define	D_h	r11
#endif

/* Number of lines ahead to pre-fetch data.  If you change this the code
   below will need adjustment to compensate.  */

#define prefetch_lines	5

#ifdef USE_VFP
	.macro	cpy_line_vfp vreg, base
	SFI_BREG(dst) \
	vstr	\vreg, [dst, #\base]
	SFI_BREG(src) \
	vldr	\vreg, [src, #\base]
	SFI_BREG(dst) \
	vstr	d0, [dst, #\base + 8]
	SFI_BREG(src) \
	vldr	d0, [src, #\base + 8]
	SFI_BREG(dst) \
	vstr	d1, [dst, #\base + 16]
	SFI_BREG(src) \
	vldr	d1, [src, #\base + 16]
	SFI_BREG(dst) \
	vstr	d2, [dst, #\base + 24]
	SFI_BREG(src) \
	vldr	d2, [src, #\base + 24]
	SFI_BREG(dst) \
	vstr	\vreg, [dst, #\base + 32]
	SFI_BREG(src) \
	vldr	\vreg, [src, #\base + prefetch_lines * 64 - 32]
	SFI_BREG(dst) \
	vstr	d0, [dst, #\base + 40]
	SFI_BREG(src) \
	vldr	d0, [src, #\base + 40]
	SFI_BREG(dst) \
	vstr	d1, [dst, #\base + 48]
	SFI_BREG(src) \
	vldr	d1, [src, #\base + 48]
	SFI_BREG(dst) \
	vstr	d2, [dst, #\base + 56]
	SFI_BREG(src) \
	vldr	d2, [src, #\base + 56]
	.endm

	.macro	cpy_tail_vfp vreg, base
	SFI_BREG(dst) \
	vstr	\vreg, [dst, #\base]
	SFI_BREG(src) \
	vldr	\vreg, [src, #\base]
	SFI_BREG(dst) \
	vstr	d0, [dst, #\base + 8]
	SFI_BREG(src) \
	vldr	d0, [src, #\base + 8]
	SFI_BREG(dst) \
	vstr	d1, [dst, #\base + 16]
	SFI_BREG(src) \
	vldr	d1, [src, #\base + 16]
	SFI_BREG(dst) \
	vstr	d2, [dst, #\base + 24]
	SFI_BREG(src) \
	vldr	d2, [src, #\base + 24]
	SFI_BREG(dst) \
	vstr	\vreg, [dst, #\base + 32]
	SFI_BREG(dst) \
	vstr	d0, [dst, #\base + 40]
	SFI_BREG(src) \
	vldr	d0, [src, #\base + 40]
	SFI_BREG(dst) \
	vstr	d1, [dst, #\base + 48]
	SFI_BREG(src) \
	vldr	d1, [src, #\base + 48]
	SFI_BREG(dst) \
	vstr	d2, [dst, #\base + 56]
	SFI_BREG(src) \
	vldr	d2, [src, #\base + 56]
	.endm
#endif

	.macro def_fn f p2align=0
	.text
	.p2align \p2align
	.global \f
	.type \f, %function
\f:
	.endm

def_fn memcpy p2align=6

	mov	dst, dstin	/* Preserve dstin, we need to return it.  */
	cmp	count, #64
	bge	.Lcpy_not_short
	/* Deal with small copies quickly by dropping straight into the
	   exit block.  */

.Ltail63unaligned:
#ifdef USE_NEON
	and	tmp1, count, #0x38
	.macro dispatch_step i
	SFI_BREG(src) \
	vld1.8	{d0}, [src]!
	SFI_BREG(dst) \
	vst1.8	{d0}, [dst]!
	.endm
	dispatch_7_dword

	tst	count, #4
	SFI_BREG(src) \
	ldrne	tmp1, [src], #4
	SFI_BREG(dst) \
	strne	tmp1, [dst], #4
#else
	/* Copy up to 15 full words of data.  May not be aligned.  */
	/* Cannot use VFP for unaligned data.  */
	and	tmp1, count, #0x3c
	add	dst, dst, tmp1
	add	src, src, tmp1
	/* Jump directly into the sequence below at the correct offset.  */
	.macro dispatch_step i
	SFI_BREG(src) \
	ldr	tmp1, [src, #-(\i * 4)]
	SFI_BREG(dst) \
	str	tmp1, [dst, #-(\i * 4)]
	.endm
	dispatch_15_word
#endif

	lsls	count, count, #31
	SFI_BREG(src) \
	ldrhcs	tmp1, [src], #2
	SFI_BREG(src) \
	ldrbne	src, [src]		/* Src is dead, use as a scratch.  */
	SFI_BREG(dst) \
	strhcs	tmp1, [dst], #2
	SFI_BREG(dst) \
	strbne	src, [dst]
	RETURN

.Lcpy_not_short:
	/* At least 64 bytes to copy, but don't know the alignment yet.  */
	str	tmp2, [sp, #-FRAME_SIZE]!
	and	tmp2, src, #7
	and	tmp1, dst, #7
	cmp	tmp1, tmp2
	bne	.Lcpy_notaligned

#ifdef USE_VFP
	/* Magic dust alert!  Force VFP on Cortex-A9.  Experiments show
	   that the FP pipeline is much better at streaming loads and
	   stores.  This is outside the critical loop.  */
	vmov.f32	s0, s0
#endif

	/* SRC and DST have the same mutual 32-bit alignment, but we may
	   still need to pre-copy some bytes to get to natural alignment.
	   We bring DST into full 64-bit alignment.  */
	lsls	tmp2, dst, #29
	beq	1f
	rsbs	tmp2, tmp2, #0
	sub	count, count, tmp2, lsr #29
	SFI_BREG(src) \
	ldrmi	tmp1, [src], #4
	SFI_BREG(dst) \
	strmi	tmp1, [dst], #4
	lsls	tmp2, tmp2, #2
	SFI_BREG(src) \
	ldrhcs	tmp1, [src], #2
	SFI_BREG(src) \
	ldrbne	tmp2, [src], #1
	SFI_BREG(dst) \
	strhcs	tmp1, [dst], #2
	SFI_BREG(dst) \
	strbne	tmp2, [dst], #1

1:
	subs	tmp2, count, #64	/* Use tmp2 for count.  */
	blt	.Ltail63aligned

	cmp	tmp2, #512
	bge	.Lcpy_body_long

.Lcpy_body_medium:			/* Count in tmp2.  */
#ifdef USE_VFP
1:
	SFI_BREG(src) \
	vldr	d0, [src, #0]
	subs	tmp2, tmp2, #64
	SFI_BREG(src) \
	vldr	d1, [src, #8]
	SFI_BREG(dst) \
	vstr	d0, [dst, #0]
	SFI_BREG(src) \
	vldr	d0, [src, #16]
	SFI_BREG(dst) \
	vstr	d1, [dst, #8]
	SFI_BREG(src) \
	vldr	d1, [src, #24]
	SFI_BREG(dst) \
	vstr	d0, [dst, #16]
	SFI_BREG(src) \
	vldr	d0, [src, #32]
	SFI_BREG(dst) \
	vstr	d1, [dst, #24]
	SFI_BREG(src) \
	vldr	d1, [src, #40]
	SFI_BREG(dst) \
	vstr	d0, [dst, #32]
	SFI_BREG(src) \
	vldr	d0, [src, #48]
	SFI_BREG(dst) \
	vstr	d1, [dst, #40]
	SFI_BREG(src) \
	vldr	d1, [src, #56]
	SFI_BREG(dst) \
	vstr	d0, [dst, #48]
	add	src, src, #64
	SFI_BREG(dst) \
	vstr	d1, [dst, #56]
	add	dst, dst, #64
	bge	1b
	tst	tmp2, #0x3f
	beq	.Ldone

.Ltail63aligned:			/* Count in tmp2.  */
	and	tmp1, tmp2, #0x38
	add	dst, dst, tmp1
	add	src, src, tmp1
	.macro dispatch_step i
	SFI_BREG(src) \
	vldr	d0, [src, #-(\i * 8)]
	SFI_BREG(dst) \
	vstr	d0, [dst, #-(\i * 8)]
	.endm
	dispatch_7_dword
#else
	sub	src, src, #8
	sub	dst, dst, #8
1:
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #8]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #8]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #16]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #16]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #24]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #24]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #32]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #32]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #40]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #40]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #48]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #48]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #56]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #56]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #64]!
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #64]!
	subs	tmp2, tmp2, #64
	bge	1b
	tst	tmp2, #0x3f
	bne	1f
	ldr	tmp2,[sp], #FRAME_SIZE
	RETURN
1:
	add	src, src, #8
	add	dst, dst, #8

.Ltail63aligned:			/* Count in tmp2.  */
	/* Copy up to 7 d-words of data.  Similar to Ltail63unaligned, but
	   we know that the src and dest are 32-bit aligned so we can use
	   LDRD/STRD to improve efficiency.  */
	/* TMP2 is now negative, but we don't care about that.  The bottom
	   six bits still tell us how many bytes are left to copy.  */

	and	tmp1, tmp2, #0x38
	add	dst, dst, tmp1
	add	src, src, tmp1
	.macro dispatch_step i
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #-(\i * 8)]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #-(\i * 8)]
	.endm
	dispatch_7_dword
#endif

	tst	tmp2, #4
	SFI_BREG(src) \
	ldrne	tmp1, [src], #4
	SFI_BREG(dst) \
	strne	tmp1, [dst], #4
	lsls	tmp2, tmp2, #31		/* Count (tmp2) now dead. */
	SFI_BREG(src) \
	ldrhcs	tmp1, [src], #2
	SFI_BREG(src) \
	ldrbne	tmp2, [src]
	SFI_BREG(dst) \
	strhcs	tmp1, [dst], #2
	SFI_BREG(dst) \
	strbne	tmp2, [dst]

.Ldone:
	ldr	tmp2, [sp], #FRAME_SIZE
	RETURN

.Lcpy_body_long:			/* Count in tmp2.  */

	/* Long copy.  We know that there's at least (prefetch_lines * 64)
	   bytes to go.  */
#ifdef USE_VFP
	/* Don't use PLD.  Instead, read some data in advance of the current
	   copy position into a register.  This should act like a PLD
	   operation but we won't have to repeat the transfer.  */

	SFI_BREG(src) \
	vldr	d3, [src, #0]
	SFI_BREG(src) \
	vldr	d4, [src, #64]
	SFI_BREG(src) \
	vldr	d5, [src, #128]
	SFI_BREG(src) \
	vldr	d6, [src, #192]
	SFI_BREG(src) \
	vldr	d7, [src, #256]

	SFI_BREG(src) \
	vldr	d0, [src, #8]
	SFI_BREG(src) \
	vldr	d1, [src, #16]
	SFI_BREG(src) \
	vldr	d2, [src, #24]
	add	src, src, #32

	subs	tmp2, tmp2, #prefetch_lines * 64 * 2
	blt	2f
1:
	cpy_line_vfp	d3, 0
	cpy_line_vfp	d4, 64
	cpy_line_vfp	d5, 128
	add	dst, dst, #3 * 64
	add	src, src, #3 * 64
	cpy_line_vfp	d6, 0
	cpy_line_vfp	d7, 64
	add	dst, dst, #2 * 64
	add	src, src, #2 * 64
	subs	tmp2, tmp2, #prefetch_lines * 64
	bge	1b

2:
	cpy_tail_vfp	d3, 0
	cpy_tail_vfp	d4, 64
	cpy_tail_vfp	d5, 128
	add	src, src, #3 * 64
	add	dst, dst, #3 * 64
	cpy_tail_vfp	d6, 0
	SFI_BREG(dst) \
	vstr	d7, [dst, #64]
	SFI_BREG(src) \
	vldr	d7, [src, #64]
	SFI_BREG(dst) \
	vstr	d0, [dst, #64 + 8]
	SFI_BREG(src) \
	vldr	d0, [src, #64 + 8]
	SFI_BREG(dst) \
	vstr	d1, [dst, #64 + 16]
	SFI_BREG(src) \
	vldr	d1, [src, #64 + 16]
	SFI_BREG(dst) \
	vstr	d2, [dst, #64 + 24]
	SFI_BREG(src) \
	vldr	d2, [src, #64 + 24]
	SFI_BREG(dst) \
	vstr	d7, [dst, #64 + 32]
	add	src, src, #96
	SFI_BREG(dst) \
	vstr	d0, [dst, #64 + 40]
	SFI_BREG(dst) \
	vstr	d1, [dst, #64 + 48]
	SFI_BREG(dst) \
	vstr	d2, [dst, #64 + 56]
	add	dst, dst, #128
	add	tmp2, tmp2, #prefetch_lines * 64
	b	.Lcpy_body_medium
#else
	/* Long copy.  Use an SMS style loop to maximize the I/O
	   bandwidth of the core.  We don't have enough spare registers
	   to synthesise prefetching, so use PLD operations.  */
	/* Pre-bias src and dst.  */
	sub	src, src, #8
	sub	dst, dst, #8
	SFI_BREG(src) \
	pld	[src, #8]
	SFI_BREG(src) \
	pld	[src, #72]
	subs	tmp2, tmp2, #64
	SFI_BREG(src) \
	pld	[src, #136]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #8]
	strd	B_l, B_h, [sp, #8]
	SFI_BREG(src) \
	ldrd	B_l, B_h, [src, #16]
	strd	C_l, C_h, [sp, #16]
	SFI_BREG(src) \
	ldrd	C_l, C_h, [src, #24]
	strd	D_l, D_h, [sp, #24]
	SFI_BREG(src) \
	pld	[src, #200]
	SFI_BREG(src) \
	ldrd	D_l, D_h, [src, #32]!
	b	1f
	.p2align	6
2:
	SFI_BREG(src) \
	pld	[src, #232]
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #40]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #40]
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #48]
	SFI_BREG(src) \
	ldrd	B_l, B_h, [src, #48]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #56]
	SFI_BREG(src) \
	ldrd	C_l, C_h, [src, #56]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #64]!
	SFI_BREG(src) \
	ldrd	D_l, D_h, [src, #64]!
	subs	tmp2, tmp2, #64
1:
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #8]
	SFI_BREG(src) \
	ldrd	A_l, A_h, [src, #8]
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #16]
	SFI_BREG(src) \
	ldrd	B_l, B_h, [src, #16]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #24]
	SFI_BREG(src) \
	ldrd	C_l, C_h, [src, #24]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #32]
	SFI_BREG(src) \
	ldrd	D_l, D_h, [src, #32]
	bcs	2b
	/* Save the remaining bytes and restore the callee-saved regs.  */
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #40]
	add	src, src, #40
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #48]
	ldrd	B_l, B_h, [sp, #8]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #56]
	ldrd	C_l, C_h, [sp, #16]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #64]
	ldrd	D_l, D_h, [sp, #24]
	add	dst, dst, #72
	tst	tmp2, #0x3f
	bne	.Ltail63aligned
	ldr	tmp2, [sp], #FRAME_SIZE
	RETURN
#endif

.Lcpy_notaligned:
	SFI_BREG(src) \
	pld	[src]
	SFI_BREG(src) \
	pld	[src, #64]
	/* There's at least 64 bytes to copy, but there is no mutual
	   alignment.  */
	/* Bring DST to 64-bit alignment.  */
	lsls	tmp2, dst, #29
	SFI_BREG(src) \
	pld	[src, #(2 * 64)]
	beq	1f
	rsbs	tmp2, tmp2, #0
	sub	count, count, tmp2, lsr #29
	SFI_BREG(src) \
	ldrmi	tmp1, [src], #4
	SFI_BREG(dst) \
	strmi	tmp1, [dst], #4
	lsls	tmp2, tmp2, #2
	SFI_BREG(src) \
	ldrbne	tmp1, [src], #1
	SFI_BREG(src) \
	ldrhcs	tmp2, [src], #2
	SFI_BREG(dst) \
	strbne	tmp1, [dst], #1
	SFI_BREG(dst) \
	strhcs	tmp2, [dst], #2
1:
	SFI_BREG(src) \
	pld	[src, #(3 * 64)]
	subs	count, count, #64
	ldrmi	tmp2, [sp], #FRAME_SIZE
	bmi	.Ltail63unaligned
	SFI_BREG(src) \
	pld	[src, #(4 * 64)]

#ifdef USE_NEON
	SFI_BREG(src) \
	vld1.8	{d0-d3}, [src]!
	SFI_BREG(src) \
	vld1.8	{d4-d7}, [src]!
	subs	count, count, #64
	bmi	2f
1:
	SFI_BREG(src) \
	pld	[src, #(4 * 64)]
	SFI_BREG(dst) \
	vst1.8	{d0-d3}, [ALIGN (dst, 64)]!
	SFI_BREG(src) \
	vld1.8	{d0-d3}, [src]!
	SFI_BREG(dst) \
	vst1.8	{d4-d7}, [ALIGN (dst, 64)]!
	SFI_BREG(src) \
	vld1.8	{d4-d7}, [src]!
	subs	count, count, #64
	bpl	1b
2:
	SFI_BREG(dst) \
	vst1.8	{d0-d3}, [ALIGN (dst, 64)]!
	SFI_BREG(dst) \
	vst1.8	{d4-d7}, [ALIGN (dst, 64)]!
	ands	count, count, #0x3f
#else
	/* Use an SMS style loop to maximize the I/O bandwidth.  */
	sub	src, src, #4
	sub	dst, dst, #8
	subs	tmp2, count, #64	/* Use tmp2 for count.  */
	SFI_BREG(src) \
	ldr	A_l, [src, #4]
	SFI_BREG(src) \
	ldr	A_h, [src, #8]
	strd	B_l, B_h, [sp, #8]
	SFI_BREG(src) \
	ldr	B_l, [src, #12]
	SFI_BREG(src) \
	ldr	B_h, [src, #16]
	strd	C_l, C_h, [sp, #16]
	SFI_BREG(src) \
	ldr	C_l, [src, #20]
	SFI_BREG(src) \
	ldr	C_h, [src, #24]
	strd	D_l, D_h, [sp, #24]
	SFI_BREG(src) \
	ldr	D_l, [src, #28]
	SFI_BREG(src) \
	ldr	D_h, [src, #32]!
	b	1f
	.p2align	6
2:
	SFI_BREG(src) \
	pld	[src, #(5 * 64) - (32 - 4)]
	strd	A_l, A_h, [dst, #40]
	SFI_BREG(src) \
	ldr	A_l, [src, #36]
	SFI_BREG(src) \
	ldr	A_h, [src, #40]
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #48]
	SFI_BREG(src) \
	ldr	B_l, [src, #44]
	SFI_BREG(src) \
	ldr	B_h, [src, #48]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #56]
	SFI_BREG(src) \
	ldr	C_l, [src, #52]
	SFI_BREG(src) \
	ldr	C_h, [src, #56]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #64]!
	SFI_BREG(src) \
	ldr	D_l, [src, #60]
	SFI_BREG(src) \
	ldr	D_h, [src, #64]!
	subs	tmp2, tmp2, #64
1:
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #8]
	SFI_BREG(src) \
	ldr	A_l, [src, #4]
	SFI_BREG(src) \
	ldr	A_h, [src, #8]
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #16]
	SFI_BREG(src) \
	ldr	B_l, [src, #12]
	SFI_BREG(src) \
	ldr	B_h, [src, #16]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #24]
	SFI_BREG(src) \
	ldr	C_l, [src, #20]
	SFI_BREG(src) \
	ldr	C_h, [src, #24]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #32]
	SFI_BREG(src) \
	ldr	D_l, [src, #28]
	SFI_BREG(src) \
	ldr	D_h, [src, #32]
	bcs	2b

	/* Save the remaining bytes and restore the callee-saved regs.  */
	SFI_BREG(dst) \
	strd	A_l, A_h, [dst, #40]
	add	src, src, #36
	SFI_BREG(dst) \
	strd	B_l, B_h, [dst, #48]
	ldrd	B_l, B_h, [sp, #8]
	SFI_BREG(dst) \
	strd	C_l, C_h, [dst, #56]
	ldrd	C_l, C_h, [sp, #16]
	SFI_BREG(dst) \
	strd	D_l, D_h, [dst, #64]
	ldrd	D_l, D_h, [sp, #24]
	add	dst, dst, #72
	ands	count, tmp2, #0x3f
#endif
	ldr	tmp2, [sp], #FRAME_SIZE
	bne	.Ltail63unaligned
	RETURN

	.size	memcpy, . - memcpy
