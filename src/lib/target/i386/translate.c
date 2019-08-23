/*
 *  i386 translation
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "lib/internal.h"
#include "lib/tcg/tcg.h"
#include "lib/tcg/tcg-op.h"
#include "lib/bitops.h"
#include "lib/target/i386/translate.h"

#define PREFIX_REPZ   0x01
#define PREFIX_REPNZ  0x02
#define PREFIX_LOCK   0x04
#define PREFIX_DATA   0x08
#define PREFIX_ADR    0x10
#define PREFIX_VEX    0x20

#ifdef TARGET_X86_64
#define CODE64(dc) ((dc)->code64)
#define REX_X(dc) ((dc)->rex_x)
#define REX_B(dc) ((dc)->rex_b)
#else
#define CODE64(dc) 0
#define REX_X(dc) 0
#define REX_B(dc) 0
#endif

#ifdef TARGET_X86_64
# define ctztl  ctz64
# define clztl  clz64
#else
# define ctztl  ctz32
# define clztl  clz32
#endif

 /* For a switch indexed by MODRM, match all memory operands for a given OP.  */
 //#define CASE_MODRM_MEM_OP(OP) \
 //    case (0 << 6) | (OP << 3) | 0 ... (0 << 6) | (OP << 3) | 7: \
 //    case (1 << 6) | (OP << 3) | 0 ... (1 << 6) | (OP << 3) | 7: \
 //    case (2 << 6) | (OP << 3) | 0 ... (2 << 6) | (OP << 3) | 7

 //#define CASE_MODRM_OP(OP) \
 //    case (0 << 6) | (OP << 3) | 0 ... (0 << 6) | (OP << 3) | 7: \
 //    case (1 << 6) | (OP << 3) | 0 ... (1 << 6) | (OP << 3) | 7: \
 //    case (2 << 6) | (OP << 3) | 0 ... (2 << 6) | (OP << 3) | 7: \
 //    case (3 << 6) | (OP << 3) | 0 ... (3 << 6) | (OP << 3) | 7

#define CASE_MODRM_MEM_OP(OP) \
	case (0 << 6) | (OP << 3) | 0: \
	case (0 << 6) | (OP << 3) | 1: \
	case (0 << 6) | (OP << 3) | 2: \
	case (0 << 6) | (OP << 3) | 3: \
	case (0 << 6) | (OP << 3) | 4: \
	case (0 << 6) | (OP << 3) | 5: \
	case (0 << 6) | (OP << 3) | 6: \
	case (0 << 6) | (OP << 3) | 7: \
	case (1 << 6) | (OP << 3) | 0: \
	case (1 << 6) | (OP << 3) | 1: \
	case (1 << 6) | (OP << 3) | 2: \
	case (1 << 6) | (OP << 3) | 3: \
	case (1 << 6) | (OP << 3) | 4: \
	case (1 << 6) | (OP << 3) | 5: \
	case (1 << 6) | (OP << 3) | 6: \
	case (1 << 6) | (OP << 3) | 7: \
	case (2 << 6) | (OP << 3) | 0: \
	case (2 << 6) | (OP << 3) | 1: \
	case (2 << 6) | (OP << 3) | 2: \
	case (2 << 6) | (OP << 3) | 3: \
	case (2 << 6) | (OP << 3) | 4: \
	case (2 << 6) | (OP << 3) | 5: \
	case (2 << 6) | (OP << 3) | 6: \
	case (2 << 6) | (OP << 3) | 7

#define CASE_MODRM_OP(OP) \
	case (0 << 6) | (OP << 3) | 0: \
	case (0 << 6) | (OP << 3) | 1: \
	case (0 << 6) | (OP << 3) | 2: \
	case (0 << 6) | (OP << 3) | 3: \
	case (0 << 6) | (OP << 3) | 4: \
	case (0 << 6) | (OP << 3) | 5: \
	case (0 << 6) | (OP << 3) | 6: \
	case (0 << 6) | (OP << 3) | 7: \
	case (1 << 6) | (OP << 3) | 0: \
	case (1 << 6) | (OP << 3) | 1: \
	case (1 << 6) | (OP << 3) | 2: \
	case (1 << 6) | (OP << 3) | 3: \
	case (1 << 6) | (OP << 3) | 4: \
	case (1 << 6) | (OP << 3) | 5: \
	case (1 << 6) | (OP << 3) | 6: \
	case (1 << 6) | (OP << 3) | 7: \
	case (2 << 6) | (OP << 3) | 0: \
	case (2 << 6) | (OP << 3) | 1: \
	case (2 << 6) | (OP << 3) | 2: \
	case (2 << 6) | (OP << 3) | 3: \
	case (2 << 6) | (OP << 3) | 4: \
	case (2 << 6) | (OP << 3) | 5: \
	case (2 << 6) | (OP << 3) | 6: \
	case (2 << 6) | (OP << 3) | 7: \
	case (3 << 6) | (OP << 3) | 0: \
	case (3 << 6) | (OP << 3) | 1: \
	case (3 << 6) | (OP << 3) | 2: \
	case (3 << 6) | (OP << 3) | 3: \
	case (3 << 6) | (OP << 3) | 4: \
	case (3 << 6) | (OP << 3) | 5: \
	case (3 << 6) | (OP << 3) | 6: \
	case (3 << 6) | (OP << 3) | 7

static void gen_eob(TCGContext * s, DisasContext *dc);
static void gen_jr(TCGContext * s, DisasContext *dc, TCGv dest);
static void gen_jmp(TCGContext * s, DisasContext *dc, target_ulong eip);
static void gen_jmp_tb(TCGContext * s, DisasContext *dc, target_ulong eip, int tb_num);
static void gen_op(TCGContext * s, DisasContext *dc1, int op, TCGMemOp ot, int d);

static void gen_io_start(TCGContext * s)
{
	TCGv_i32 tmp = tcg_const_i32(s, 1);

	tcg_gen_st_i32(s, tmp, s->tcg_env, -ENV_OFFSET + offsetof(CPUState, can_do_io));

	tcg_temp_free_i32(s, tmp);
}

static void gen_io_end(TCGContext * s)
{
	TCGv_i32 tmp = tcg_const_i32(s, 0);

	tcg_gen_st_i32(s, tmp, s->tcg_env, -ENV_OFFSET + offsetof(CPUState, can_do_io));

	tcg_temp_free_i32(s, tmp);
}

/* i386 arith/logic operations */
enum {
	OP_ADDL,
	OP_ORL,
	OP_ADCL,
	OP_SBBL,
	OP_ANDL,
	OP_SUBL,
	OP_XORL,
	OP_CMPL,
};

/* i386 shift ops */
enum {
	OP_ROL,
	OP_ROR,
	OP_RCL,
	OP_RCR,
	OP_SHL,
	OP_SHR,
	OP_SHL1, /* undocumented */
	OP_SAR = 7,
};

enum {
	JCC_O,
	JCC_B,
	JCC_Z,
	JCC_BE,
	JCC_S,
	JCC_P,
	JCC_L,
	JCC_LE,
};

enum {
	/* I386 int registers */
	OR_EAX,   /* MUST be even numbered */
	OR_ECX,
	OR_EDX,
	OR_EBX,
	OR_ESP,
	OR_EBP,
	OR_ESI,
	OR_EDI,

	OR_TMP0 = 16,    /* temporary operand register */
	OR_TMP1,
	OR_A0, /* temporary register used when doing address evaluation */
};

enum {
	USES_CC_DST = 1,
	USES_CC_SRC = 2,
	USES_CC_SRC2 = 4,
	USES_CC_SRCT = 8,
};

/* Bit set if the global variable is live after setting CC_OP to X.  */
static const uint8_t cc_op_live[CC_OP_NB] = {
	/*CC_OP_EFLAGS */ 0,
	/*CC_OP_MULB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_MULW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_MULL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_MULQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADDB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADDW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADDL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADDQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADCB*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_ADCW*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_ADCL*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_ADCQ*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_SUBB*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRCT,
	/*CC_OP_SUBW*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRCT,
	/*CC_OP_SUBL*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRCT,
	/*CC_OP_SUBQ*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRCT,
	/*CC_OP_SBBB*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_SBBW*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_SBBL*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_SBBQ*/ USES_CC_DST | USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_LOGICB*/ USES_CC_DST,
	/*CC_OP_LOGICW*/ USES_CC_DST,
	/*CC_OP_LOGICL*/ USES_CC_DST,
	/*CC_OP_LOGICQ*/ USES_CC_DST,
	/*CC_OP_INCB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_INCW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_INCL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_INCQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_DECB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_DECW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_DECL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_DECQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SHLB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SHLW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SHLL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SHLQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SARB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SARW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SARL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_SARQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_BMILGB*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_BMILGW*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_BMILGL*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_BMILGQ*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADCX*/ USES_CC_DST | USES_CC_SRC,
	/*CC_OP_ADOX*/ USES_CC_SRC | USES_CC_SRC2,
	/*CC_OP_ADCOX*/ USES_CC_DST | USES_CC_SRC2,
	/*CC_OP_CLR*/ 0,
	/*CC_OP_POPCNT*/ USES_CC_SRC
};

static void gen_compute_eflags(TCGContext * s, DisasContext *dc);

static void set_cc_op(TCGContext * s, DisasContext *dc, CCOp op)
{
	if (dc->cc_op != op)
	{
		// Discard CC computation that will no longer be used.
		int dead = cc_op_live[dc->cc_op] & ~cc_op_live[op];

		if (dead & USES_CC_DST) {
			tcg_gen_discard_tl(s, s->cpu_cc_dst);
		}

		if (dead & USES_CC_SRC) {
			tcg_gen_discard_tl(s, s->cpu_cc_src);
		}

		if (dead & USES_CC_SRC2) {
			tcg_gen_discard_tl(s, s->cpu_cc_src2);
		}

		if (dead & USES_CC_SRCT) {
			tcg_gen_discard_tl(s, s->cpu_cc_srcT);
		}

		dc->cc_op = op;
	}

	gen_compute_eflags(s, dc);
}

static void gen_update_cc_op(TCGContext * s, DisasContext *dc)
{

}

#ifdef TARGET_X86_64

#define NB_OP_SIZES 4

#else /* !TARGET_X86_64 */

#define NB_OP_SIZES 3

#endif /* !TARGET_X86_64 */

#if defined(HOST_WORDS_BIGENDIAN)
#define REG_B_OFFSET (sizeof(target_ulong) - 1)
#define REG_H_OFFSET (sizeof(target_ulong) - 2)
#define REG_W_OFFSET (sizeof(target_ulong) - 2)
#define REG_L_OFFSET (sizeof(target_ulong) - 4)
#define REG_LH_OFFSET (sizeof(target_ulong) - 8)
#else
#define REG_B_OFFSET 0
#define REG_H_OFFSET 1
#define REG_W_OFFSET 0
#define REG_L_OFFSET 0
#define REG_LH_OFFSET 4
#endif

/* In instruction encodings for byte register accesses the
 * register number usually indicates "low 8 bits of register N";
 * however there are some special cases where N 4..7 indicates
 * [AH, CH, DH, BH], ie "bits 15..8 of register N-4". Return
 * true for this special case, false otherwise.
 */
static inline bool byte_reg_is_xH(TCGContext * s, int reg)
{
	if (reg < 4) {
		return false;
	}
#ifdef TARGET_X86_64
	if (reg >= 8 || s->x86_64_hregs) {
		return false;
	}
#endif
	return true;
}

/* Select the size of a push/pop operation.  */
static inline TCGMemOp mo_pushpop(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	if (CODE64(dc)) {
		return ot == MO_16 ? MO_16 : MO_64;
	}
	else {
		return ot;
	}
}

/* Select the size of the stack pointer.  */
static inline TCGMemOp mo_stacksize(TCGContext * s, DisasContext *dc)
{
	return CODE64(dc) ? MO_64 : dc->ss32 ? MO_32 : MO_16;
}

/* Select only size 64 else 32.  Used for SSE operand sizes.  */
static inline TCGMemOp mo_64_32(TCGMemOp ot)
{
#ifdef TARGET_X86_64
	return ot == MO_64 ? MO_64 : MO_32;
#else
	return MO_32;
#endif
}

/* Select size 8 if lsb of B is clear, else OT.  Used for decoding
   byte vs word opcodes.  */
static inline TCGMemOp mo_b_d(int b, TCGMemOp ot)
{
	return b & 1 ? ot : MO_8;
}

/* Select size 8 if lsb of B is clear, else OT capped at 32.
   Used for decoding operand size of port opcodes.  */
static inline TCGMemOp mo_b_d32(int b, TCGMemOp ot)
{
	return b & 1 ? (ot == MO_16 ? MO_16 : MO_32) : MO_8;
}

static void gen_op_mov_reg_v(TCGContext * s, TCGMemOp ot, int reg, TCGv t0)
{
	switch (ot) {
	case MO_8:
		if (!byte_reg_is_xH(s, reg)) {
			tcg_gen_deposit_tl(s, s->cpu_regs[reg], s->cpu_regs[reg], t0, 0, 8);
		}
		else {
			tcg_gen_deposit_tl(s, s->cpu_regs[reg - 4], s->cpu_regs[reg - 4], t0, 8, 8);
		}
		break;
	case MO_16:
		tcg_gen_deposit_tl(s, s->cpu_regs[reg], s->cpu_regs[reg], t0, 0, 16);
		break;
	case MO_32:
		/* For x86_64, this sets the higher half of register to zero.
		   For i386, this is equivalent to a mov. */
		tcg_gen_ext32u_tl(s, s->cpu_regs[reg], t0);
		break;
#ifdef TARGET_X86_64
	case MO_64:
		tcg_gen_mov_tl(s, s->cpu_regs[reg], t0);
		break;
#endif
	default:
		tcg_abort();
	}
}

static inline void gen_op_mov_v_reg(TCGContext * s, TCGMemOp ot, TCGv t0, int reg)
{
	if (ot == MO_8 && byte_reg_is_xH(s, reg)) {
		tcg_gen_extract_tl(s, t0, s->cpu_regs[reg - 4], 8, 8);
	}
	else {
		tcg_gen_mov_tl(s, t0, s->cpu_regs[reg]);
	}
}

static void gen_add_A0_im(TCGContext * s, DisasContext *dc, int val)
{
	tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_A0, val);
	if (!CODE64(dc)) {
		tcg_gen_ext32u_tl(s, s->cpu_A0, s->cpu_A0);
	}
}

static inline void gen_op_jmp_v(TCGContext * s, TCGv dest)
{
	tcg_gen_st_tl(s, dest, s->cpu_env, offsetof(CPUX86State, eip));
}

static inline void gen_op_add_reg_im(TCGContext * s, TCGMemOp size, int reg, int32_t val)
{
	tcg_gen_addi_tl(s, s->cpu_tmp0, s->cpu_regs[reg], val);
	gen_op_mov_reg_v(s, size, reg, s->cpu_tmp0);
}

static inline void gen_op_add_reg_T0(TCGContext * s, TCGMemOp size, int reg)
{
	tcg_gen_add_tl(s, s->cpu_tmp0, s->cpu_regs[reg], s->cpu_T0);
	gen_op_mov_reg_v(s, size, reg, s->cpu_tmp0);
}

static inline void gen_op_ld_v(TCGContext * s, DisasContext *dc, int idx, TCGv t0, TCGv a0)
{
	tcg_gen_qemu_ld_tl(s, t0, a0, dc->mem_index, (TCGMemOp)(idx | MO_LE));
}

static inline void gen_op_st_v(TCGContext * s, DisasContext *dc, int idx, TCGv t0, TCGv a0)
{
	tcg_gen_qemu_st_tl(s, t0, a0, dc->mem_index, (TCGMemOp)(idx | MO_LE));
}

static inline void gen_op_st_rm_T0_A0(TCGContext * s, DisasContext *dc, int idx, int d)
{
	if (d == OR_TMP0) {
		gen_op_st_v(s, dc, idx, s->cpu_T0, s->cpu_A0);
	}
	else {
		gen_op_mov_reg_v(s, (TCGMemOp)idx, d, s->cpu_T0);
	}
}

static inline void gen_jmp_im(TCGContext * s, target_ulong pc)
{
	tcg_gen_movi_tl(s, s->cpu_tmp0, pc);
	gen_op_jmp_v(s, s->cpu_tmp0);
}

/* Compute SEG:REG into A0.  SEG is selected from the override segment
   (OVR_SEG) and the default segment (DEF_SEG).  OVR_SEG may be -1 to
   indicate no override.  */
static void gen_lea_v_seg(TCGContext * s, DisasContext *dc, TCGMemOp aflag, TCGv a0,
	int def_seg, int ovr_seg)
{
	switch (aflag) {
#ifdef TARGET_X86_64
	case MO_64:
		if (ovr_seg < 0) {
			tcg_gen_mov_tl(s, s->cpu_A0, a0);
			return;
		}
		break;
#endif
	case MO_32:
		/* 32 bit address */
		if (ovr_seg < 0 && dc->addseg) {
			ovr_seg = def_seg;
		}
		if (ovr_seg < 0) {
			tcg_gen_ext32u_tl(s, s->cpu_A0, a0);
			return;
		}
		break;
	case MO_16:
		/* 16 bit address */
		tcg_gen_ext16u_tl(s, s->cpu_A0, a0);
		a0 = s->cpu_A0;
		if (ovr_seg < 0) {
			if (dc->addseg) {
				ovr_seg = def_seg;
			}
			else {
				return;
			}
		}
		break;
	default:
		tcg_abort();
	}

	if (ovr_seg >= 0) {
		TCGv seg = s->cpu_seg_base[ovr_seg];

		if (aflag == MO_64) {
			tcg_gen_add_tl(s, s->cpu_A0, a0, seg);
		}
		else if (CODE64(dc)) {
			tcg_gen_ext32u_tl(s, s->cpu_A0, a0);
			tcg_gen_add_tl(s, s->cpu_A0, s->cpu_A0, seg);
		}
		else {
			tcg_gen_add_tl(s, s->cpu_A0, a0, seg);
			tcg_gen_ext32u_tl(s, s->cpu_A0, s->cpu_A0);
		}
	}
}

static inline void gen_string_movl_A0_ESI(TCGContext * s, DisasContext *dc)
{
	gen_lea_v_seg(s, dc, dc->aflag, s->cpu_regs[R_ESI], R_DS, dc->override);
}

static inline void gen_string_movl_A0_EDI(TCGContext * s, DisasContext *dc)
{
	gen_lea_v_seg(s, dc, dc->aflag, s->cpu_regs[R_EDI], R_ES, -1);
}

static inline void gen_op_movl_T0_Dshift(TCGContext * s, TCGMemOp ot)
{
	tcg_gen_ld32s_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, df));
	tcg_gen_shli_tl(s, s->cpu_T0, s->cpu_T0, ot);
};

static TCGv gen_ext_tl(TCGContext * s, TCGv dst, TCGv src, TCGMemOp size, bool sign)
{
	switch (size) {
	case MO_8:
		if (sign) {
			tcg_gen_ext8s_tl(s, dst, src);
		}
		else {
			tcg_gen_ext8u_tl(s, dst, src);
		}
		return dst;
	case MO_16:
		if (sign) {
			tcg_gen_ext16s_tl(s, dst, src);
		}
		else {
			tcg_gen_ext16u_tl(s, dst, src);
		}
		return dst;
#ifdef TARGET_X86_64
	case MO_32:
		if (sign) {
			tcg_gen_ext32s_tl(s, dst, src);
		}
		else {
			tcg_gen_ext32u_tl(s, dst, src);
		}
		return dst;
#endif
	default:
		return src;
	}
}

static void gen_extu(TCGContext * s, TCGMemOp ot, TCGv reg)
{
	gen_ext_tl(s, reg, reg, ot, false);
}

static void gen_exts(TCGContext *s, TCGMemOp ot, TCGv reg)
{
	gen_ext_tl(s, reg, reg, ot, true);
}

static inline void gen_op_jnz_ecx(TCGContext *s, TCGMemOp size, TCGLabel *label1)
{
	tcg_gen_mov_tl(s, s->cpu_tmp0, s->cpu_regs[R_ECX]);
	gen_extu(s, size, s->cpu_tmp0);
	tcg_gen_brcondi_tl(s, TCG_COND_NE, s->cpu_tmp0, 0, label1);
}

static inline void gen_op_jz_ecx(TCGContext *s, TCGMemOp size, TCGLabel *label1)
{
	tcg_gen_mov_tl(s, s->cpu_tmp0, s->cpu_regs[R_ECX]);
	gen_extu(s, size, s->cpu_tmp0);
	tcg_gen_brcondi_tl(s, TCG_COND_EQ, s->cpu_tmp0, 0, label1);
}

static void gen_helper_in_func(TCGContext *s, TCGMemOp ot, TCGv v, TCGv_i32 n)
{
	switch (ot) {
	case MO_8:
		gen_helper_inb(s, v, s->cpu_env, n);
		break;
	case MO_16:
		gen_helper_inw(s, v, s->cpu_env, n);
		break;
	case MO_32:
		gen_helper_inl(s, v, s->cpu_env, n);
		break;
	default:
		tcg_abort();
	}
}

static void gen_helper_out_func(TCGContext * s, TCGMemOp ot, TCGv_i32 v, TCGv_i32 n)
{
	switch (ot) {
	case MO_8:
		gen_helper_outb(s, s->cpu_env, v, n);
		break;
	case MO_16:
		gen_helper_outw(s, s->cpu_env, v, n);
		break;
	case MO_32:
		gen_helper_outl(s, s->cpu_env, v, n);
		break;
	default:
		tcg_abort();
	}
}

static void gen_check_io(TCGContext * s, DisasContext *dc, TCGMemOp ot, target_ulong cur_eip,
	uint32_t svm_flags)
{
	target_ulong next_eip;

	if (dc->pe && (dc->cpl > dc->iopl || dc->vm86)) {
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		switch (ot) {
		case MO_8:
			gen_helper_check_iob(s, s->cpu_env, s->cpu_tmp2_i32);
			break;
		case MO_16:
			gen_helper_check_iow(s, s->cpu_env, s->cpu_tmp2_i32);
			break;
		case MO_32:
			gen_helper_check_iol(s, s->cpu_env, s->cpu_tmp2_i32);
			break;
		default:
			tcg_abort();
		}
	}
	if (dc->flags & HF_SVMI_MASK) {
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, cur_eip);
		svm_flags |= (1 << (4 + ot));
		next_eip = dc->pc - dc->cs_base;
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		gen_helper_svm_check_io(s, s->cpu_env, s->cpu_tmp2_i32,
			tcg_const_i32(s, svm_flags),
			tcg_const_i32(s, next_eip - cur_eip));
	}
}

static inline void gen_movs(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_string_movl_A0_ESI(s, dc);
	gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	gen_string_movl_A0_EDI(s, dc);
	gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_ESI);
	gen_op_add_reg_T0(s, dc->aflag, R_EDI);
}

static void gen_op_update1_cc(TCGContext * s)
{
	tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
}

static void gen_op_update2_cc(TCGContext * s)
{
	tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T1);
	tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
}

static void gen_op_update3_cc(TCGContext * s, TCGv reg)
{
	tcg_gen_mov_tl(s, s->cpu_cc_src2, reg);
	tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T1);
	tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
}

static inline void gen_op_testl_T0_T1_cc(TCGContext * s)
{
	tcg_gen_and_tl(s, s->cpu_cc_dst, s->cpu_T0, s->cpu_T1);
}

static void gen_op_update_neg_cc(TCGContext * s)
{
	tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
	tcg_gen_neg_tl(s, s->cpu_cc_src, s->cpu_T0);
	tcg_gen_movi_tl(s, s->cpu_cc_srcT, 0);
}

static inline void gen_extu_bits(TCGContext * s, TCGv reg, unsigned bits)
{
	if (bits == 8)
		gen_extu(s, MO_8, reg);
	else if (bits == 16)
		gen_extu(s, MO_16, reg);
#ifdef TARGET_X86_64
	else if (bits == 32)
		gen_extu(s, MO_32, reg);
#endif
}

static void cc_compute_all(TCGContext * s, TCGv dst, TCGv src1, TCGv src2, int op)
{
	TCGv zero, tmp1, tmp2, tmp3;
	unsigned data_bits = 0;
	uint64_t sign_mask = 1;

	TCGV_UNUSED(zero);
	TCGV_UNUSED(tmp1);
	TCGV_UNUSED(tmp2);
	TCGV_UNUSED(tmp3);

	switch (op)
	{
	case CC_OP_CLR:
	{
		// return CC_Z | CC_P;
		zero = tcg_const_tl(s, 0);
		tmp1 = tcg_const_tl(s, 1);

		tcg_gen_mov_tl(s, s->cpu_eflags_c, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_p, tmp1);
		tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_z, tmp1);
		tcg_gen_mov_tl(s, s->cpu_eflags_s, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_o, zero);
		break;
	}
	case CC_OP_POPCNT:
	{
		// return src1 ? 0 : CC_Z;
		zero = tcg_const_tl(s, 0);
		tmp1 = tcg_const_tl(s, 1);

		tcg_gen_mov_tl(s, s->cpu_eflags_c, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_p, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
		tcg_gen_movcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, src1, zero, tmp1, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_s, zero);
		tcg_gen_mov_tl(s, s->cpu_eflags_o, zero);
		break;
	}
	case CC_OP_MULQ:
		data_bits += 32;
	case CC_OP_MULL:
		data_bits += 16;
	case CC_OP_MULW:
		data_bits += 8;
	case CC_OP_MULB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			// cf = (src1 != 0);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, src1, zero);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			// af = 0; // undefined
			tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			// of = cf * CC_O;
			tcg_gen_mov_tl(s, s->cpu_eflags_o, s->cpu_eflags_c);
			break;
		}
	case CC_OP_ADDQ:
		data_bits += 32;
	case CC_OP_ADDL:
		data_bits += 16;
	case CC_OP_ADDW:
		data_bits += 8;
	case CC_OP_ADDB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//DATA_TYPE src2 = dst - src1;
			tcg_gen_sub_tl(s, src2, dst, src1);
			gen_extu_bits(s, src2, data_bits);
			//cf = dst < src1;
			tcg_gen_setcond_tl(s, TCG_COND_LTU, s->cpu_eflags_c, dst, src1);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ src1 ^ src2) & CC_A;
			tcg_gen_xor_tl(s, tmp1, dst, src1);
			tcg_gen_xor_tl(s, tmp1, tmp1, src2);
			tcg_gen_andi_tl(s, tmp1, tmp1, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp1, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift((src1 ^ src2 ^ -1) & (src1 ^ dst), 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, src1, src2);
			tcg_gen_xori_tl(s, tmp1, tmp1, -1);
			tcg_gen_xor_tl(s, tmp2, src1, dst);
			tcg_gen_and_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_ADCQ:
		data_bits += 32;
	case CC_OP_ADCL:
		data_bits += 16;
	case CC_OP_ADCW:
		data_bits += 8;
	case CC_OP_ADCB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			tmp3 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			gen_extu_bits(s, src2, data_bits);
			//DATA_TYPE tmp3 = dst - src1 - src2;
			tcg_gen_sub_tl(s, tmp3, dst, src1);
			tcg_gen_sub_tl(s, tmp3, tmp3, src2);
			gen_extu_bits(s, tmp3, data_bits);
			//cf = (src2 ? dst <= src1 : dst < src1);
			tcg_gen_setcond_tl(s, TCG_COND_LEU, tmp1, dst, src1);
			tcg_gen_setcond_tl(s, TCG_COND_LTU, tmp2, dst, src1);
			tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, src2, zero, tmp1, tmp2);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ src1 ^ tmp3) & CC_A;
			tcg_gen_xor_tl(s, tmp1, dst, src1);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp3);
			tcg_gen_andi_tl(s, tmp1, tmp1, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp1, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift((src1 ^ tmp3 ^ -1) & (src1 ^ dst), 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, src1, tmp3);
			tcg_gen_xori_tl(s, tmp1, tmp1, -1);
			tcg_gen_xor_tl(s, tmp2, src1, dst);
			tcg_gen_and_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_SUBQ:
		data_bits += 32;
	case CC_OP_SUBL:
		data_bits += 16;
	case CC_OP_SUBW:
		data_bits += 8;
	case CC_OP_SUBB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			tmp3 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//DATA_TYPE tmp3 = dst + src1;
			tcg_gen_add_tl(s, tmp3, dst, src1);
			gen_extu_bits(s, tmp3, data_bits);
			//cf = tmp3 < src1;
			tcg_gen_setcond_tl(s, TCG_COND_LTU, s->cpu_eflags_c, tmp3, src1);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ tmp3 ^ src1) & CC_A;
			tcg_gen_xor_tl(s, tmp1, dst, tmp3);
			tcg_gen_xor_tl(s, tmp1, tmp1, src1);
			tcg_gen_andi_tl(s, tmp1, tmp1, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp1, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift((tmp3 ^ src1) & (tmp3 ^ dst), 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, tmp3, src1);
			tcg_gen_xor_tl(s, tmp2, tmp3, dst);
			tcg_gen_and_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_SBBQ:
		data_bits += 32;
	case CC_OP_SBBL:
		data_bits += 16;
	case CC_OP_SBBW:
		data_bits += 8;
	case CC_OP_SBBB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			tmp3 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			gen_extu_bits(s, src2, data_bits);
			//DATA_TYPE tmp3 = dst + src1 + src2;
			tcg_gen_add_tl(s, tmp3, dst, src1);
			tcg_gen_add_tl(s, tmp3, tmp3, src2);
			gen_extu_bits(s, tmp3, data_bits);
			//cf = (src2 ? tmp3 <= src1 : tmp3 < src1);
			tcg_gen_setcond_tl(s, TCG_COND_LEU, tmp1, tmp3, src1);
			tcg_gen_setcond_tl(s, TCG_COND_LTU, tmp2, tmp3, src1);
			tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, src2, zero, tmp1, tmp2);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ tmp3 ^ src1) & 0x10;
			tcg_gen_xor_tl(s, tmp1, dst, tmp3);
			tcg_gen_xor_tl(s, tmp1, tmp1, src1);
			tcg_gen_andi_tl(s, tmp1, tmp1, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp1, zero);
			//zf = (dst == 0) << 6;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & 0x80;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift((tmp3 ^ src1) & (tmp3 ^ dst), 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, tmp3, src1);
			tcg_gen_xor_tl(s, tmp2, tmp3, dst);
			tcg_gen_and_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_LOGICQ:
		data_bits += 32;
	case CC_OP_LOGICL:
		data_bits += 16;
	case CC_OP_LOGICW:
		data_bits += 8;
	case CC_OP_LOGICB:
		data_bits += 8;
		{
			// The OF and CF flags are cleared; the SF, ZF, and PF flags are set according to the result. The state of the AF flag is undefined.
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			//cf = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_c, zero);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_o, zero);
			break;
		}
	case CC_OP_INCQ:
		data_bits += 32;
		sign_mask <<= 32;
	case CC_OP_INCL:
		data_bits += 16;
		sign_mask <<= 16;
	case CC_OP_INCW:
		data_bits += 8;
		sign_mask <<= 8;
	case CC_OP_INCB:
		data_bits += 8;
		sign_mask <<= 7;
		{
			// The CF flag is not affected. The OF, SF, ZF, AF, and PF flags are set according to the result.
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			tmp3 = tcg_const_tl(s, 1);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//cf = src1;
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, src1, zero);
			//src1 = dst - 1;
			tcg_gen_sub_tl(s, src1, dst, tmp3);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ src1 ^ tmp3) & CC_A;
			tcg_gen_xor_tl(s, tmp2, dst, src1);
			tcg_gen_xor_tl(s, tmp2, tmp2, tmp3);
			tcg_gen_andi_tl(s, tmp2, tmp2, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp2, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = (dst == SIGN_MASK) * CC_O;
			tcg_gen_movi_tl(s, tmp2, sign_mask);
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_o, dst, tmp2);
			break;
		}
	case CC_OP_DECQ:
		data_bits += 32;
		sign_mask <<= 32;
	case CC_OP_DECL:
		data_bits += 16;
		sign_mask <<= 16;
	case CC_OP_DECW:
		data_bits += 8;
		sign_mask <<= 8;
	case CC_OP_DECB:
		data_bits += 8;
		sign_mask <<= 7;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			tmp3 = tcg_const_tl(s, 1);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//cf = src1;
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, src1, zero);
			//src1 = dst + 1;
			tcg_gen_add_tl(s, src1, dst, tmp3);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = (dst ^ src1 ^ tmp3) & CC_A;
			tcg_gen_xor_tl(s, tmp2, dst, src1);
			tcg_gen_xor_tl(s, tmp2, tmp2, tmp3);
			tcg_gen_andi_tl(s, tmp2, tmp2, CC_A);
			tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp2, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = (dst == SIGN_MASK) * CC_O;
			tcg_gen_movi_tl(s, tmp2, sign_mask - 1);
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_o, dst, tmp2);
			break;
		}
	case CC_OP_SHLQ:
		data_bits += 32;
	case CC_OP_SHLL:
		data_bits += 16;
	case CC_OP_SHLW:
		data_bits += 8;
	case CC_OP_SHLB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_temp_new(s);
			tmp2 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//cf = (src1 >> (DATA_BITS - 1)) & CC_C;
			tcg_gen_shri_tl(s, s->cpu_eflags_c, src1, data_bits - 1);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift(src1 ^ dst, 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, src1, dst);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_SARQ:
		data_bits += 32;
	case CC_OP_SARL:
		data_bits += 16;
	case CC_OP_SARW:
		data_bits += 8;
	case CC_OP_SARB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			tmp1 = tcg_const_tl(s, 1);
			tmp2 = tcg_temp_new(s);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//cf = src1 & 1;
			tcg_gen_and_tl(s, s->cpu_eflags_c, src1, tmp1);
			//pf = ((0x9669 >> (((uint8_t)dst ^ ((uint8_t)dst >> 4)) & 0xF)) & 1); (http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel)
			tcg_gen_ext8u_tl(s, tmp1, dst);
			tcg_gen_shri_tl(s, tmp2, tmp1, 4);
			tcg_gen_xor_tl(s, tmp1, tmp1, tmp2);
			tcg_gen_andi_tl(s, tmp1, tmp1, 0xF);
			tcg_gen_movi_tl(s, tmp2, 0x9669);
			tcg_gen_shr_tl(s, tmp1, tmp2, tmp1);
			tcg_gen_andi_tl(s, s->cpu_eflags_p, tmp1, 1);
			//af = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = lshift(src1 ^ dst, 12 - DATA_BITS) & CC_O;
			tcg_gen_xor_tl(s, tmp1, src1, dst);
			tcg_gen_shri_tl(s, s->cpu_eflags_o, tmp1, data_bits - 1);
			break;
		}
	case CC_OP_BMILGQ:
		data_bits += 32;
	case CC_OP_BMILGL:
		data_bits += 16;
	case CC_OP_BMILGW:
		data_bits += 8;
	case CC_OP_BMILGB:
		data_bits += 8;
		{
			zero = tcg_const_tl(s, 0);
			gen_extu_bits(s, dst, data_bits);
			gen_extu_bits(s, src1, data_bits);
			//cf = (src1 == 0);
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_c, src1, zero);
			//pf = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_p, zero);
			//af = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_a, zero);
			//zf = (dst == 0) * CC_Z;
			tcg_gen_setcond_tl(s, TCG_COND_EQ, s->cpu_eflags_z, dst, zero);
			//sf = lshift(dst, 8 - DATA_BITS) & CC_S;
			tcg_gen_shri_tl(s, s->cpu_eflags_s, dst, data_bits - 1);
			//of = 0;
			tcg_gen_mov_tl(s, s->cpu_eflags_o, zero);
			break;
		}
	case CC_OP_ADCX:
	{
		zero = tcg_const_tl(s, 0);
		//return (src1 & ~CC_C) | (dst * CC_C);
		tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, dst, zero);
		break;
	}
	case CC_OP_ADOX:
	{
		zero = tcg_const_tl(s, 0);
		//return (src1 & ~CC_O) | (src2 * CC_O);
		tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_o, src2, zero);
		break;
	}
	case CC_OP_ADCOX:
	{
		zero = tcg_const_tl(s, 0);
		//return (src1 & ~(CC_C | CC_O)) | (dst * CC_C) | (src2 * CC_O);
		tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, dst, zero);
		tcg_gen_setcond_tl(s, TCG_COND_NE, s->cpu_eflags_o, src2, zero);
		break;
	}
	default:
	{
		tcg_abort();
		break;
	}
	}
	switch (op) {
	case CC_OP_ADCX:
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		break;
	case CC_OP_ADOX:
		tcg_gen_st_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		break;
	case CC_OP_ADCOX:
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		tcg_gen_st_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		break;
	default:
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		tcg_gen_st_tl(s, s->cpu_eflags_p, s->cpu_env, offsetof(CPUX86State, pf));
		tcg_gen_st_tl(s, s->cpu_eflags_a, s->cpu_env, offsetof(CPUX86State, af));
		tcg_gen_st_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		tcg_gen_st_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		tcg_gen_st_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		break;
	}

	if (!TCGV_IS_UNUSED(zero))
		tcg_temp_free(s, zero);

	if (!TCGV_IS_UNUSED(tmp1))
		tcg_temp_free(s, tmp1);

	if (!TCGV_IS_UNUSED(tmp2))
		tcg_temp_free(s, tmp2);

	if (!TCGV_IS_UNUSED(tmp3))
		tcg_temp_free(s, tmp3);
}

/* compute all eflags to cc_src */
static void gen_compute_eflags(TCGContext * s, DisasContext *dc)
{
	TCGv zero;
	TCGv dst, src1, src2;
	int live, dead;

	if (dc->cc_op == CC_OP_EFLAGS) {
		return;
	}

	TCGV_UNUSED(zero);
	dst = s->cpu_cc_dst;
	src1 = s->cpu_cc_src;
	src2 = s->cpu_cc_src2;

	// Take care to not read values that are not live.
	live = cc_op_live[dc->cc_op] & ~USES_CC_SRCT;
	dead = live ^ (USES_CC_DST | USES_CC_SRC | USES_CC_SRC2);
	if (dead) {
		zero = tcg_const_tl(s, 0);
		if (dead & USES_CC_DST) {
			dst = zero;
		}
		if (dead & USES_CC_SRC) {
			src1 = zero;
		}
		if (dead & USES_CC_SRC2) {
			src2 = zero;
		}
	}

	cc_compute_all(s, dst, src1, src2, dc->cc_op);

	dc->cc_op = CC_OP_EFLAGS;

	if (dead) {
		tcg_temp_free(s, zero);
	}
}

typedef struct CCPrepare {
	TCGCond cond;
	TCGv reg;
	TCGv reg2;
	target_ulong imm;
	bool use_reg2;
} CCPrepare;

/* perform a conditional store into register 'reg' according to jump opcode
   value 'b'. In the fast case, T0 is guaranted not to be used. */
static CCPrepare gen_prepare_cc(TCGContext * s, DisasContext *dc, int b, TCGv reg)
{
	int inv, jcc_op;
	CCPrepare cc;

	inv = b & 1;
	jcc_op = (b >> 1) & 7;

	cc.use_reg2 = false;

	switch (dc->cc_op) {
	case CC_OP_SUBB:
	case CC_OP_SUBW:
	case CC_OP_SUBL:
	case CC_OP_SUBQ:
	{
		// We optimize relational operators for the cmp/jcc case.
		TCGv t0;
		TCGMemOp size = (TCGMemOp)(dc->cc_op - CC_OP_SUBB);
		switch (jcc_op) {
		case JCC_BE:
			tcg_gen_mov_tl(s, s->cpu_tmp4, s->cpu_cc_srcT);
			gen_extu(s, size, s->cpu_tmp4);
			t0 = gen_ext_tl(s, s->cpu_tmp0, s->cpu_cc_src, size, false);
			cc.cond = TCG_COND_LEU;
			cc.reg = s->cpu_tmp4;
			cc.reg2 = t0;
			cc.use_reg2 = true;
			return cc;
		case JCC_L:
			tcg_gen_mov_tl(s, s->cpu_tmp4, s->cpu_cc_srcT);
			gen_exts(s, size, s->cpu_tmp4);
			t0 = gen_ext_tl(s, s->cpu_tmp0, s->cpu_cc_src, size, true);
			cc.cond = TCG_COND_LT;
			cc.reg = s->cpu_tmp4;
			cc.reg2 = t0;
			cc.use_reg2 = true;
			return cc;
		case JCC_LE:
			tcg_gen_mov_tl(s, s->cpu_tmp4, s->cpu_cc_srcT);
			gen_exts(s, size, s->cpu_tmp4);
			t0 = gen_ext_tl(s, s->cpu_tmp0, s->cpu_cc_src, size, true);
			cc.cond = TCG_COND_LE;
			cc.reg = s->cpu_tmp4;
			cc.reg2 = t0;
			cc.use_reg2 = true;
			return cc;
		default:
			break;
		}
		break;
	}
	default:
	{
		break;
	}
	}

	switch (jcc_op) {
	case JCC_O: // of == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		cc.cond = TCG_COND_EQ;
		cc.reg = s->cpu_eflags_o;
		cc.imm = 1;
		break;
	case JCC_B: // cf == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		cc.cond = TCG_COND_EQ;
		cc.reg = s->cpu_eflags_c;
		cc.imm = 1;
		break;
	case JCC_Z: // zf == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		cc.cond = TCG_COND_EQ;
		cc.reg = s->cpu_eflags_z;
		cc.imm = 1;
		break;
	case JCC_BE: // cf == 1 || zf == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		tcg_gen_ld_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		tcg_gen_or_tl(s, reg, s->cpu_eflags_c, s->cpu_eflags_z);
		cc.cond = TCG_COND_EQ;
		cc.reg = reg;
		cc.imm = 1;
		break;
	case JCC_S: // sf == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		cc.cond = TCG_COND_EQ;
		cc.reg = s->cpu_eflags_s;
		cc.imm = 1;
		break;
	case JCC_P: // pf == 1
		tcg_gen_ld_tl(s, s->cpu_eflags_p, s->cpu_env, offsetof(CPUX86State, pf));
		cc.cond = TCG_COND_EQ;
		cc.reg = s->cpu_eflags_p;
		cc.imm = 1;
		break;
	case JCC_L: // sf != of
		tcg_gen_ld_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		tcg_gen_ld_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		cc.cond = TCG_COND_NE;
		cc.reg = s->cpu_eflags_s;
		cc.reg2 = s->cpu_eflags_o;
		cc.use_reg2 = true;
		break;
	case JCC_LE: // zf ==1 || sf != of
		tcg_gen_ld_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		tcg_gen_ld_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
		tcg_gen_ld_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		tcg_gen_xor_tl(s, reg, s->cpu_eflags_s, s->cpu_eflags_o);
		tcg_gen_or_tl(s, reg, reg, s->cpu_eflags_z);
		cc.cond = TCG_COND_EQ;
		cc.reg = reg;
		cc.imm = 1;
		break;
	default:
		tcg_abort();
		break;
	}

	if (inv) {
		cc.cond = tcg_invert_cond(cc.cond);
	}

	return cc;
}

static void gen_setcc1(TCGContext * s, DisasContext *dc, int b, TCGv reg)
{
	CCPrepare cc = gen_prepare_cc(s, dc, b, reg);

	if (cc.use_reg2) {
		tcg_gen_setcond_tl(s, cc.cond, reg, cc.reg, cc.reg2);
	}
	else {
		tcg_gen_setcondi_tl(s, cc.cond, reg, cc.reg, cc.imm);
	}
}

static inline void gen_compute_eflags_c(TCGContext * s, DisasContext *dc, TCGv reg)
{
	tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
	tcg_gen_mov_tl(s, reg, s->cpu_eflags_c);
}

/* generate a conditional jump to label 'l1' according to jump opcode
   value 'b'. In the fast case, T0 is guaranted not to be used. */
static inline void gen_jcc1_noeob(TCGContext * s, DisasContext *dc, int b, TCGLabel *l1)
{
	CCPrepare cc = gen_prepare_cc(s, dc, b, s->cpu_T0);

	if (cc.use_reg2) {
		tcg_gen_brcond_tl(s, cc.cond, cc.reg, cc.reg2, l1);
	}
	else {
		tcg_gen_brcondi_tl(s, cc.cond, cc.reg, cc.imm, l1);
	}
}

/* Generate a conditional jump to label 'l1' according to jump opcode
   value 'b'. In the fast case, T0 is guaranted not to be used.
   A translation block must end soon.  */
static inline void gen_jcc1(TCGContext * s, DisasContext *dc, int b, TCGLabel *l1)
{
	CCPrepare cc = gen_prepare_cc(s, dc, b, s->cpu_T0);

	gen_update_cc_op(s, dc);

	set_cc_op(s, dc, CC_OP_EFLAGS);

	if (cc.use_reg2) {
		tcg_gen_brcond_tl(s, cc.cond, cc.reg, cc.reg2, l1);
	}
	else {
		tcg_gen_brcondi_tl(s, cc.cond, cc.reg, cc.imm, l1);
	}
}

/* XXX: does not work with gdbstub "ice" single step - not a
   serious problem */
static TCGLabel *gen_jz_ecx_string(TCGContext * s, DisasContext *dc, target_ulong next_eip)
{
	TCGLabel *l1 = gen_new_label(s);
	TCGLabel *l2 = gen_new_label(s);
	gen_op_jnz_ecx(s, dc->aflag, l1);
	gen_set_label(s, l2);
	gen_jmp_tb(s, dc, next_eip, 1);
	gen_set_label(s, l1);
	return l2;
}

static inline void gen_stos(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_op_mov_v_reg(s, MO_32, s->cpu_T0, R_EAX);
	gen_string_movl_A0_EDI(s, dc);
	gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_EDI);
}

static inline void gen_lods(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_string_movl_A0_ESI(s, dc);
	gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	gen_op_mov_reg_v(s, ot, R_EAX, s->cpu_T0);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_ESI);
}

static inline void gen_scas(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_string_movl_A0_EDI(s, dc);
	gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
	gen_op(s, dc, OP_CMPL, ot, R_EAX);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_EDI);
}

static inline void gen_cmps(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_string_movl_A0_EDI(s, dc);
	gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
	gen_string_movl_A0_ESI(s, dc);
	gen_op(s, dc, OP_CMPL, ot, OR_TMP0);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_ESI);
	gen_op_add_reg_T0(s, dc->aflag, R_EDI);
}

static void gen_bpt_io(TCGContext * s, DisasContext *dc, TCGv_i32 t_port, int ot)
{
	if (dc->flags & HF_IOBPT_MASK) {
		TCGv_i32 t_size = tcg_const_i32(s, 1 << ot);
		TCGv t_next = tcg_const_tl(s, dc->pc - dc->cs_base);

		gen_helper_bpt_io(s, s->cpu_env, t_port, t_size, t_next);
		tcg_temp_free_i32(s, t_size);
		tcg_temp_free(s, t_next);
	}
}

static inline void gen_ins(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	if (dc->tb->cflags & CF_USE_ICOUNT) {
		gen_io_start(s);
	}
	gen_string_movl_A0_EDI(s, dc);
	/* Note: we must do this dummy write first to be restartable in
	   case of page fault. */
	tcg_gen_movi_tl(s, s->cpu_T0, 0);
	gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_EDX]);
	tcg_gen_andi_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, 0xffff);
	gen_helper_in_func(s, ot, s->cpu_T0, s->cpu_tmp2_i32);
	gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_EDI);
	gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
	if (dc->tb->cflags & CF_USE_ICOUNT) {
		gen_io_end(s);
	}
}

static inline void gen_outs(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	if (dc->tb->cflags & CF_USE_ICOUNT) {
		gen_io_start(s);
	}
	gen_string_movl_A0_ESI(s, dc);
	gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);

	tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_EDX]);
	tcg_gen_andi_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, 0xffff);
	tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_T0);
	gen_helper_out_func(s, ot, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
	gen_op_movl_T0_Dshift(s, ot);
	gen_op_add_reg_T0(s, dc->aflag, R_ESI);
	gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
	if (dc->tb->cflags & CF_USE_ICOUNT) {
		gen_io_end(s);
	}
}

/* same method as Valgrind : we generate jumps to current or next
   instruction */
#define GEN_REPZ(op)                                                          \
static inline void gen_repz_ ## op(TCGContext * s,DisasContext *dc, TCGMemOp ot,              \
                                 target_ulong cur_eip, target_ulong next_eip) \
{                                                                             \
    TCGLabel *l2;                                                             \
    gen_update_cc_op(s,dc);                                                      \
    l2 = gen_jz_ecx_string(s,dc, next_eip);                                      \
    gen_ ## op(s,dc, ot);                                                        \
    gen_op_add_reg_im(s,dc->aflag, R_ECX, -1);                                   \
    /* a loop would cause two single step exceptions if ECX = 1               \
       before rep string_insn */                                              \
    if (dc->repz_opt)                                                          \
        gen_op_jz_ecx(s,dc->aflag, l2);                                          \
    gen_jmp(s,dc, cur_eip);                                                      \
}

#define GEN_REPZ2(op)                                                         \
static inline void gen_repz_ ## op(TCGContext * s,DisasContext *dc, TCGMemOp ot,              \
                                   target_ulong cur_eip,                      \
                                   target_ulong next_eip,                     \
                                   int nz)                                    \
{                                                                             \
    TCGLabel *l2;                                                             \
    gen_update_cc_op(s,dc);                                                      \
    l2 = gen_jz_ecx_string(s,dc, next_eip);                                      \
    gen_ ## op(s,dc, ot);                                                        \
    gen_op_add_reg_im(s,dc->aflag, R_ECX, -1);                                   \
    gen_update_cc_op(s,dc);                                                      \
    gen_jcc1(s,dc, (JCC_Z << 1) | (nz ^ 1), l2);                                 \
    if (dc->repz_opt)                                                          \
        gen_op_jz_ecx(s,dc->aflag, l2);                                          \
    gen_jmp(s,dc, cur_eip);                                                      \
}

GEN_REPZ(movs)
GEN_REPZ(stos)
GEN_REPZ(lods)
GEN_REPZ(ins)
GEN_REPZ(outs)
GEN_REPZ2(scas)
GEN_REPZ2(cmps)

static void gen_helper_fp_arith_ST0_FT0(TCGContext * s, int op)
{
	switch (op) {
	case 0:
		gen_helper_fadd_ST0_FT0(s, s->cpu_env);
		break;
	case 1:
		gen_helper_fmul_ST0_FT0(s, s->cpu_env);
		break;
	case 2:
		gen_helper_fcom_ST0_FT0(s, s->cpu_env);
		break;
	case 3:
		gen_helper_fcom_ST0_FT0(s, s->cpu_env);
		break;
	case 4:
		gen_helper_fsub_ST0_FT0(s, s->cpu_env);
		break;
	case 5:
		gen_helper_fsubr_ST0_FT0(s, s->cpu_env);
		break;
	case 6:
		gen_helper_fdiv_ST0_FT0(s, s->cpu_env);
		break;
	case 7:
		gen_helper_fdivr_ST0_FT0(s, s->cpu_env);
		break;
	}
}

/* NOTE the exception in "r" op ordering */
static void gen_helper_fp_arith_STN_ST0(TCGContext * s, int op, int opreg)
{
	TCGv_i32 tmp = tcg_const_i32(s, opreg);
	switch (op) {
	case 0:
		gen_helper_fadd_STN_ST0(s, s->cpu_env, tmp);
		break;
	case 1:
		gen_helper_fmul_STN_ST0(s, s->cpu_env, tmp);
		break;
	case 4:
		gen_helper_fsubr_STN_ST0(s, s->cpu_env, tmp);
		break;
	case 5:
		gen_helper_fsub_STN_ST0(s, s->cpu_env, tmp);
		break;
	case 6:
		gen_helper_fdivr_STN_ST0(s, s->cpu_env, tmp);
		break;
	case 7:
		gen_helper_fdiv_STN_ST0(s, s->cpu_env, tmp);
		break;
	}
}

/* if d == OR_TMP0, it means memory operand (address in A0) */
static void gen_op(TCGContext * s, DisasContext *dc1, int op, TCGMemOp ot, int d)
{
	if (d != OR_TMP0) {
		gen_op_mov_v_reg(s, ot, s->cpu_T0, d);
	}
	else if (!(dc1->prefix & PREFIX_LOCK)) {
		gen_op_ld_v(s, dc1, ot, s->cpu_T0, s->cpu_A0);
	}
	switch (op) {
	case OP_ADCL:
		gen_compute_eflags_c(s, dc1, s->cpu_tmp4);
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_tmp4, s->cpu_T1);
			tcg_gen_atomic_add_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T0,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp4);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update3_cc(s, s->cpu_tmp4);
		set_cc_op(s, dc1, (CCOp)(CC_OP_ADCB + ot));
		break;
	case OP_SBBL:
		gen_compute_eflags_c(s, dc1, s->cpu_tmp4);
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T1, s->cpu_tmp4);
			tcg_gen_neg_tl(s, s->cpu_T0, s->cpu_T0);
			tcg_gen_atomic_add_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T0,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_sub_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			tcg_gen_sub_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp4);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update3_cc(s, s->cpu_tmp4);
		set_cc_op(s, dc1, (CCOp)(CC_OP_SBBB + ot));
		break;
	case OP_ADDL:
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_atomic_add_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T1,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update2_cc(s);
		set_cc_op(s, dc1, (CCOp)(CC_OP_ADDB + ot));
		break;
	case OP_SUBL:
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_neg_tl(s, s->cpu_T0, s->cpu_T1);
			tcg_gen_atomic_fetch_add_tl(s, s->cpu_cc_srcT, s->cpu_A0, s->cpu_T0,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
			tcg_gen_sub_tl(s, s->cpu_T0, s->cpu_cc_srcT, s->cpu_T1);
		}
		else {
			tcg_gen_mov_tl(s, s->cpu_cc_srcT, s->cpu_T0);
			tcg_gen_sub_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update2_cc(s);
		set_cc_op(s, dc1, (CCOp)(CC_OP_SUBB + ot));
		break;
	default:
	case OP_ANDL:
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_atomic_and_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T1,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_and_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update1_cc(s);
		set_cc_op(s, dc1, (CCOp)(CC_OP_LOGICB + ot));
		break;
	case OP_ORL:
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_atomic_or_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T1,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update1_cc(s);
		set_cc_op(s, dc1, (CCOp)(CC_OP_LOGICB + ot));
		break;
	case OP_XORL:
		if (dc1->prefix & PREFIX_LOCK) {
			tcg_gen_atomic_xor_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T1,
				dc1->mem_index, (TCGMemOp)(ot | MO_LE));
		}
		else {
			tcg_gen_xor_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_st_rm_T0_A0(s, dc1, ot, d);
		}
		gen_op_update1_cc(s);
		set_cc_op(s, dc1, (CCOp)(CC_OP_LOGICB + ot));
		break;
	case OP_CMPL:
		tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T1);
		tcg_gen_mov_tl(s, s->cpu_cc_srcT, s->cpu_T0);
		tcg_gen_sub_tl(s, s->cpu_cc_dst, s->cpu_T0, s->cpu_T1);
		set_cc_op(s, dc1, (CCOp)(CC_OP_SUBB + ot));
		break;
	}
}

/* if d == OR_TMP0, it means memory operand (address in A0) */
static void gen_inc(TCGContext * s, DisasContext *dc1, TCGMemOp ot, int d, int c)
{
	if (dc1->prefix & PREFIX_LOCK) {
		tcg_gen_movi_tl(s, s->cpu_T0, c > 0 ? 1 : -1);
		tcg_gen_atomic_add_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T0,
			dc1->mem_index, (TCGMemOp)(ot | MO_LE));
	}
	else {
		if (d != OR_TMP0) {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, d);
		}
		else {
			gen_op_ld_v(s, dc1, ot, s->cpu_T0, s->cpu_A0);
		}
		tcg_gen_addi_tl(s, s->cpu_T0, s->cpu_T0, (c > 0 ? 1 : -1));
		gen_op_st_rm_T0_A0(s, dc1, ot, d);
	}

	gen_compute_eflags_c(s, dc1, s->cpu_cc_src);
	tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
	set_cc_op(s, dc1, (CCOp)((c > 0 ? CC_OP_INCB : CC_OP_DECB) + ot));
}

static void gen_shift_flags(TCGContext * s, DisasContext *dc, TCGMemOp ot, TCGv result,
	TCGv shm1, TCGv count, bool is_right)
{
	tcg_gen_mov_tl(s, s->cpu_cc_dst, result);
	tcg_gen_mov_tl(s, s->cpu_cc_src, shm1);

	TCGLabel * l = gen_new_label(s);
	TCGv zero = tcg_const_tl(s, 0);

	tcg_gen_brcond_tl(s, TCG_COND_EQ, count, zero, l);

	set_cc_op(s, dc, (CCOp)((is_right ? CC_OP_SARB : CC_OP_SHLB) + ot));

	gen_set_label(s, l);

	tcg_temp_free(s, zero);
}

static void gen_shift_rm_T1(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1,
	int is_right, int is_arith)
{
	target_ulong mask = (ot == MO_64 ? 0x3f : 0x1f);

	/* load */
	if (op1 == OR_TMP0) {
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	}
	else {
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);
	}

	tcg_gen_andi_tl(s, s->cpu_T1, s->cpu_T1, mask);
	tcg_gen_subi_tl(s, s->cpu_tmp0, s->cpu_T1, 1);

	if (is_right) {
		if (is_arith) {
			gen_exts(s, ot, s->cpu_T0);
			tcg_gen_sar_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
			tcg_gen_sar_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
		}
		else {
			gen_extu(s, ot, s->cpu_T0);
			tcg_gen_shr_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
			tcg_gen_shr_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
		}
	}
	else {
		tcg_gen_shl_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
		tcg_gen_shl_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
	}

	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);

	gen_shift_flags(s, dc, ot, s->cpu_T0, s->cpu_tmp0, s->cpu_T1, is_right);
}

static void gen_shift_rm_im(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1, int op2,
	int is_right, int is_arith)
{
	int mask = (ot == MO_64 ? 0x3f : 0x1f);

	/* load */
	if (op1 == OR_TMP0)
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	else
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);

	op2 &= mask;
	if (op2 != 0) {
		if (is_right) {
			if (is_arith) {
				gen_exts(s, ot, s->cpu_T0);
				tcg_gen_sari_tl(s, s->cpu_tmp4, s->cpu_T0, op2 - 1);
				tcg_gen_sari_tl(s, s->cpu_T0, s->cpu_T0, op2);
			}
			else {
				gen_extu(s, ot, s->cpu_T0);
				tcg_gen_shri_tl(s, s->cpu_tmp4, s->cpu_T0, op2 - 1);
				tcg_gen_shri_tl(s, s->cpu_T0, s->cpu_T0, op2);
			}
		}
		else {
			tcg_gen_shli_tl(s, s->cpu_tmp4, s->cpu_T0, op2 - 1);
			tcg_gen_shli_tl(s, s->cpu_T0, s->cpu_T0, op2);
		}
	}

	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);

	/* update eflags if non zero shift */
	if (op2 != 0) {
		tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_tmp4);
		tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
		set_cc_op(s, dc, (CCOp)((is_right ? CC_OP_SARB : CC_OP_SHLB) + ot));
	}
}

static void gen_rot_rm_T1(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1, int is_right)
{
	target_ulong mask = (ot == MO_64 ? 0x3f : 0x1f);

	/* load */
	if (op1 == OR_TMP0) {
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	}
	else {
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);
	}

	tcg_gen_andi_tl(s, s->cpu_T1, s->cpu_T1, mask);

	switch (ot) {
	case MO_8:
		/* Replicate the 8-bit input so that a 32-bit rotate works.  */
		tcg_gen_ext8u_tl(s, s->cpu_T0, s->cpu_T0);
		tcg_gen_muli_tl(s, s->cpu_T0, s->cpu_T0, 0x01010101);
		goto do_long;
	case MO_16:
		/* Replicate the 16-bit input so that a 32-bit rotate works.  */
		tcg_gen_deposit_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T0, 16, 16);
		goto do_long;
	do_long:
#ifdef TARGET_X86_64
	case MO_32:
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_T1);
		if (is_right) {
			tcg_gen_rotr_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
		}
		else {
			tcg_gen_rotl_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
		}
		tcg_gen_extu_i32_tl(s, s->cpu_T0, s->cpu_tmp2_i32);
		break;
#endif
	default:
		if (is_right) {
			tcg_gen_rotr_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
		}
		else {
			tcg_gen_rotl_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
		}
		break;
	}

	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);

	/* The value that was "rotated out" is now present at the other end
	   of the word.  Compute C into CC_DST and O into CC_SRC2.  Note that
	   since we've computed the flags into CC_SRC, these variables are
	   currently dead.  */
	if (is_right) {
		tcg_gen_shri_tl(s, s->cpu_cc_src2, s->cpu_T0, mask - 1);
		tcg_gen_shri_tl(s, s->cpu_cc_dst, s->cpu_T0, mask);
		tcg_gen_andi_tl(s, s->cpu_cc_dst, s->cpu_cc_dst, 1);
	}
	else {
		tcg_gen_shri_tl(s, s->cpu_cc_src2, s->cpu_T0, mask);
		tcg_gen_andi_tl(s, s->cpu_cc_dst, s->cpu_T0, 1);
	}
	tcg_gen_andi_tl(s, s->cpu_cc_src2, s->cpu_cc_src2, 1);
	tcg_gen_xor_tl(s, s->cpu_cc_src2, s->cpu_cc_src2, s->cpu_cc_dst);

	TCGLabel * l = gen_new_label(s);
	TCGv zero = tcg_const_tl(s, 0);
	tcg_gen_brcond_tl(s, TCG_COND_EQ, s->cpu_T1, zero, l);

	set_cc_op(s, dc, CC_OP_ADCOX);

	gen_set_label(s, l);

	tcg_temp_free(s, zero);
}

static void gen_rot_rm_im(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1, int op2,
	int is_right)
{
	int mask = (ot == MO_64 ? 0x3f : 0x1f);
	int shift;

	/* load */
	if (op1 == OR_TMP0) {
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	}
	else {
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);
	}

	op2 &= mask;
	if (op2 != 0) {
		switch (ot) {
#ifdef TARGET_X86_64
		case MO_32:
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
			if (is_right) {
				tcg_gen_rotri_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, op2);
			}
			else {
				tcg_gen_rotli_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, op2);
			}
			tcg_gen_extu_i32_tl(s, s->cpu_T0, s->cpu_tmp2_i32);
			break;
#endif
		default:
			if (is_right) {
				tcg_gen_rotri_tl(s, s->cpu_T0, s->cpu_T0, op2);
			}
			else {
				tcg_gen_rotli_tl(s, s->cpu_T0, s->cpu_T0, op2);
			}
			break;
		case MO_8:
			mask = 7;
			goto do_shifts;
		case MO_16:
			mask = 15;
		do_shifts:
			shift = op2 & mask;
			if (is_right) {
				shift = mask + 1 - shift;
			}
			gen_extu(s, ot, s->cpu_T0);
			tcg_gen_shli_tl(s, s->cpu_tmp0, s->cpu_T0, shift);
			tcg_gen_shri_tl(s, s->cpu_T0, s->cpu_T0, mask + 1 - shift);
			tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp0);
			break;
		}
	}

	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);

	if (op2 != 0) {

		/* The value that was "rotated out" is now present at the other end
		   of the word.  Compute C into CC_DST and O into CC_SRC2.  Note that
		   since we've computed the flags into CC_SRC, these variables are
		   currently dead.  */
		if (is_right) {
			tcg_gen_shri_tl(s, s->cpu_cc_src2, s->cpu_T0, mask - 1);
			tcg_gen_shri_tl(s, s->cpu_cc_dst, s->cpu_T0, mask);
			tcg_gen_andi_tl(s, s->cpu_cc_dst, s->cpu_cc_dst, 1);
		}
		else {
			tcg_gen_shri_tl(s, s->cpu_cc_src2, s->cpu_T0, mask);
			tcg_gen_andi_tl(s, s->cpu_cc_dst, s->cpu_T0, 1);
		}
		tcg_gen_andi_tl(s, s->cpu_cc_src2, s->cpu_cc_src2, 1);
		tcg_gen_xor_tl(s, s->cpu_cc_src2, s->cpu_cc_src2, s->cpu_cc_dst);
		set_cc_op(s, dc, CC_OP_ADCOX);
	}
}

/* XXX: add faster immediate = 1 case */
static void gen_rotc_rm_T1(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1,
	int is_right)
{
	/* load */
	if (op1 == OR_TMP0)
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	else
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);

	if (is_right) {
		switch (ot) {
		case MO_8:
			gen_helper_rcrb(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
		case MO_16:
			gen_helper_rcrw(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
		case MO_32:
			gen_helper_rcrl(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
#ifdef TARGET_X86_64
		case MO_64:
			gen_helper_rcrq(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
#endif
		default:
			tcg_abort();
		}
	}
	else {
		switch (ot) {
		case MO_8:
			gen_helper_rclb(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
		case MO_16:
			gen_helper_rclw(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
		case MO_32:
			gen_helper_rcll(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
#ifdef TARGET_X86_64
		case MO_64:
			gen_helper_rclq(s, s->cpu_T0, s->cpu_env, s->cpu_T0, s->cpu_T1);
			break;
#endif
		default:
			tcg_abort();
		}
	}
	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);
}

/* XXX: add faster immediate case */
static void gen_shiftd_rm_T1(TCGContext * s, DisasContext *dc, TCGMemOp ot, int op1,
	bool is_right, TCGv count_in)
{
	target_ulong mask = (ot == MO_64 ? 63 : 31);
	TCGv count;

	/* load */
	if (op1 == OR_TMP0) {
		gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
	}
	else {
		gen_op_mov_v_reg(s, ot, s->cpu_T0, op1);
	}

	count = tcg_temp_new(s);
	tcg_gen_andi_tl(s, count, count_in, mask);

	switch (ot) {
	case MO_16:
		/* Note: we implement the Intel behaviour for shift count > 16.
		   This means "shrdw C, B, A" shifts A:B:A >> C.  Build the B:A
		   portion by constructing it as a 32-bit value.  */
		if (is_right) {
			tcg_gen_deposit_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_T1, 16, 16);
			tcg_gen_mov_tl(s, s->cpu_T1, s->cpu_T0);
			tcg_gen_mov_tl(s, s->cpu_T0, s->cpu_tmp0);
		}
		else {
			tcg_gen_deposit_tl(s, s->cpu_T1, s->cpu_T0, s->cpu_T1, 16, 16);
		}
		/* FALLTHRU */
#ifdef TARGET_X86_64
	case MO_32:
		/* Concatenate the two 32-bit values and use a 64-bit shift.  */
		tcg_gen_subi_tl(s, s->cpu_tmp0, count, 1);
		if (is_right) {
			tcg_gen_concat_tl_i64(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			tcg_gen_shr_i64(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
			tcg_gen_shr_i64(s, s->cpu_T0, s->cpu_T0, count);
		}
		else {
			tcg_gen_concat_tl_i64(s, s->cpu_T0, s->cpu_T1, s->cpu_T0);
			tcg_gen_shl_i64(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
			tcg_gen_shl_i64(s, s->cpu_T0, s->cpu_T0, count);
			tcg_gen_shri_i64(s, s->cpu_tmp0, s->cpu_tmp0, 32);
			tcg_gen_shri_i64(s, s->cpu_T0, s->cpu_T0, 32);
		}
		break;
#endif
	default:
		tcg_gen_subi_tl(s, s->cpu_tmp0, count, 1);
		if (is_right) {
			tcg_gen_shr_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);

			tcg_gen_subfi_tl(s, s->cpu_tmp4, mask + 1, count);
			tcg_gen_shr_tl(s, s->cpu_T0, s->cpu_T0, count);
			tcg_gen_shl_tl(s, s->cpu_T1, s->cpu_T1, s->cpu_tmp4);
		}
		else {
			tcg_gen_shl_tl(s, s->cpu_tmp0, s->cpu_T0, s->cpu_tmp0);
			if (ot == MO_16) {
				/* Only needed if count > 16, for Intel behaviour.  */
				tcg_gen_subfi_tl(s, s->cpu_tmp4, 33, count);
				tcg_gen_shr_tl(s, s->cpu_tmp4, s->cpu_T1, s->cpu_tmp4);
				tcg_gen_or_tl(s, s->cpu_tmp0, s->cpu_tmp0, s->cpu_tmp4);
			}

			tcg_gen_subfi_tl(s, s->cpu_tmp4, mask + 1, count);
			tcg_gen_shl_tl(s, s->cpu_T0, s->cpu_T0, count);
			tcg_gen_shr_tl(s, s->cpu_T1, s->cpu_T1, s->cpu_tmp4);
		}
		tcg_gen_movi_tl(s, s->cpu_tmp4, 0);
		tcg_gen_movcond_tl(s, TCG_COND_EQ, s->cpu_T1, count, s->cpu_tmp4,
			s->cpu_tmp4, s->cpu_T1);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
		break;
	}

	/* store */
	gen_op_st_rm_T0_A0(s, dc, ot, op1);

	gen_shift_flags(s, dc, ot, s->cpu_T0, s->cpu_tmp0, count, is_right);
	tcg_temp_free(s, count);
}

static void gen_shift(TCGContext * s, DisasContext *dc1, int op, TCGMemOp ot, int d, int ss)
{
	if (ss != OR_TMP1)
		gen_op_mov_v_reg(s, ot, s->cpu_T1, ss);
	switch (op) {
	case OP_ROL:
		gen_rot_rm_T1(s, dc1, ot, d, 0);
		break;
	case OP_ROR:
		gen_rot_rm_T1(s, dc1, ot, d, 1);
		break;
	case OP_SHL:
	case OP_SHL1:
		gen_shift_rm_T1(s, dc1, ot, d, 0, 0);
		break;
	case OP_SHR:
		gen_shift_rm_T1(s, dc1, ot, d, 1, 0);
		break;
	case OP_SAR:
		gen_shift_rm_T1(s, dc1, ot, d, 1, 1);
		break;
	case OP_RCL:
		gen_rotc_rm_T1(s, dc1, ot, d, 0);
		break;
	case OP_RCR:
		gen_rotc_rm_T1(s, dc1, ot, d, 1);
		break;
	}
}

static void gen_shifti(TCGContext * s, DisasContext *dc1, int op, TCGMemOp ot, int d, int c)
{
	switch (op) {
	case OP_ROL:
		gen_rot_rm_im(s, dc1, ot, d, c, 0);
		break;
	case OP_ROR:
		gen_rot_rm_im(s, dc1, ot, d, c, 1);
		break;
	case OP_SHL:
	case OP_SHL1:
		gen_shift_rm_im(s, dc1, ot, d, c, 0, 0);
		break;
	case OP_SHR:
		gen_shift_rm_im(s, dc1, ot, d, c, 1, 0);
		break;
	case OP_SAR:
		gen_shift_rm_im(s, dc1, ot, d, c, 1, 1);
		break;
	default:
		/* currently not optimized */
		tcg_gen_movi_tl(s, s->cpu_T1, c);
		gen_shift(s, dc1, op, ot, d, OR_TMP1);
		break;
	}
}

/* Decompose an address.  */

typedef struct AddressParts {
	int def_seg;
	int base;
	int index;
	int scale;
	target_long disp;
} AddressParts;

static AddressParts gen_lea_modrm_0(CPUX86State *env, DisasContext *dc,
	int modrm)
{
	int def_seg, base, index, scale, mod, rm;
	target_long disp;
	bool havesib;

	def_seg = R_DS;
	index = -1;
	scale = 0;
	disp = 0;

	mod = (modrm >> 6) & 3;
	rm = modrm & 7;
	base = rm | REX_B(dc);

	if (mod == 3) {
		/* Normally filtered out earlier, but including this path
		   simplifies multi-byte nop, as well as bndcl, bndcu, bndcn.  */
		goto done;
	}

	switch (dc->aflag) {
	case MO_64:
	case MO_32:
		havesib = 0;
		if (rm == 4) {
			int code = cpu_ldub_code(env, dc->pc++);
			scale = (code >> 6) & 3;
			index = ((code >> 3) & 7) | REX_X(dc);
			if (index == 4) {
				index = -1;  /* no index */
			}
			base = (code & 7) | REX_B(dc);
			havesib = 1;
		}

		switch (mod) {
		case 0:
			if ((base & 7) == 5) {
				base = -1;
				disp = (int32_t)cpu_ldul_code(env, dc->pc);
				dc->pc += 4;
				if (CODE64(dc) && !havesib) {
					base = -2;
					disp += dc->pc + dc->rip_offset;
				}
			}
			break;
		case 1:
			disp = (int8_t)cpu_ldub_code(env, dc->pc++);
			break;
		default:
		case 2:
			disp = (int32_t)cpu_ldul_code(env, dc->pc);
			dc->pc += 4;
			break;
		}

		/* For correct popl handling with esp.  */
		if (base == R_ESP && dc->popl_esp_hack) {
			disp += dc->popl_esp_hack;
		}
		if (base == R_EBP || base == R_ESP) {
			def_seg = R_SS;
		}
		break;

	case MO_16:
		if (mod == 0) {
			if (rm == 6) {
				base = -1;
				disp = cpu_lduw_code(env, dc->pc);
				dc->pc += 2;
				break;
			}
		}
		else if (mod == 1) {
			disp = (int8_t)cpu_ldub_code(env, dc->pc++);
		}
		else {
			disp = (int16_t)cpu_lduw_code(env, dc->pc);
			dc->pc += 2;
		}

		switch (rm) {
		case 0:
			base = R_EBX;
			index = R_ESI;
			break;
		case 1:
			base = R_EBX;
			index = R_EDI;
			break;
		case 2:
			base = R_EBP;
			index = R_ESI;
			def_seg = R_SS;
			break;
		case 3:
			base = R_EBP;
			index = R_EDI;
			def_seg = R_SS;
			break;
		case 4:
			base = R_ESI;
			break;
		case 5:
			base = R_EDI;
			break;
		case 6:
			base = R_EBP;
			def_seg = R_SS;
			break;
		default:
		case 7:
			base = R_EBX;
			break;
		}
		break;

	default:
		tcg_abort();
	}

done:
	AddressParts result = { def_seg, base, index, scale, disp };
	return result;
}

/* Compute the address, with a minimum number of TCG ops.  */
static TCGv gen_lea_modrm_1(TCGContext * s, AddressParts a)
{
	TCGv ea;

	TCGV_UNUSED(ea);
	if (a.index >= 0) {
		if (a.scale == 0) {
			ea = s->cpu_regs[a.index];
		}
		else {
			tcg_gen_shli_tl(s, s->cpu_A0, s->cpu_regs[a.index], a.scale);
			ea = s->cpu_A0;
		}
		if (a.base >= 0) {
			tcg_gen_add_tl(s, s->cpu_A0, ea, s->cpu_regs[a.base]);
			ea = s->cpu_A0;
		}
	}
	else if (a.base >= 0) {
		ea = s->cpu_regs[a.base];
	}
	if (TCGV_IS_UNUSED(ea)) {
		tcg_gen_movi_tl(s, s->cpu_A0, a.disp);
		ea = s->cpu_A0;
	}
	else if (a.disp != 0) {
		tcg_gen_addi_tl(s, s->cpu_A0, ea, a.disp);
		ea = s->cpu_A0;
	}

	return ea;
}

static void gen_lea_modrm(TCGContext * s, CPUX86State *env, DisasContext *dc, int modrm)
{
	AddressParts a = gen_lea_modrm_0(env, dc, modrm);
	TCGv ea = gen_lea_modrm_1(s, a);
	gen_lea_v_seg(s, dc, dc->aflag, ea, a.def_seg, dc->override);
}

static void gen_nop_modrm(TCGContext * s, CPUX86State *env, DisasContext *dc, int modrm)
{
	(void)gen_lea_modrm_0(env, dc, modrm);
}

/* Used for BNDCL, BNDCU, BNDCN.  */
static void gen_bndck(TCGContext * s, CPUX86State *env, DisasContext *dc, int modrm,
	TCGCond cond, TCGv_i64 bndv)
{
	TCGv ea = gen_lea_modrm_1(s, gen_lea_modrm_0(env, dc, modrm));

	tcg_gen_extu_tl_i64(s, s->cpu_tmp1_i64, ea);
	if (!CODE64(dc)) {
		tcg_gen_ext32u_i64(s, s->cpu_tmp1_i64, s->cpu_tmp1_i64);
	}
	tcg_gen_setcond_i64(s, cond, s->cpu_tmp1_i64, s->cpu_tmp1_i64, bndv);
	tcg_gen_extrl_i64_i32(s, s->cpu_tmp2_i32, s->cpu_tmp1_i64);
	gen_helper_bndck(s, s->cpu_env, s->cpu_tmp2_i32);
}

/* used for LEA and MOV AX, mem */
static void gen_add_A0_ds_seg(TCGContext * s, DisasContext *dc)
{
	gen_lea_v_seg(s, dc, dc->aflag, s->cpu_A0, R_DS, dc->override);
}

/* generate modrm memory load or store of 'reg'. TMP0 is used if reg ==
   OR_TMP0 */
static void gen_ldst_modrm(TCGContext * s, CPUX86State *env, DisasContext *dc, int modrm,
	TCGMemOp ot, int reg, int is_store)
{
	int mod, rm;

	mod = (modrm >> 6) & 3;
	rm = (modrm & 7) | REX_B(dc);
	if (mod == 3) {
		if (is_store) {
			if (reg != OR_TMP0)
				gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
			gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
			if (reg != OR_TMP0)
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
		}
	}
	else {
		gen_lea_modrm(s, env, dc, modrm);
		if (is_store) {
			if (reg != OR_TMP0)
				gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
			gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
		}
		else {
			gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			if (reg != OR_TMP0)
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
		}
	}
}

static inline uint32_t insn_get(TCGContext * s, CPUX86State *env, DisasContext *dc, TCGMemOp ot)
{
	uint32_t ret;

	switch (ot) {
	case MO_8:
		ret = cpu_ldub_code(env, dc->pc);
		dc->pc++;
		break;
	case MO_16:
		ret = cpu_lduw_code(env, dc->pc);
		dc->pc += 2;
		break;
	case MO_32:
#ifdef TARGET_X86_64
	case MO_64:
#endif
		ret = cpu_ldul_code(env, dc->pc);
		dc->pc += 4;
		break;
	default:
		tcg_abort();
	}
	return ret;
}

static inline int insn_const_size(TCGMemOp ot)
{
	if (ot <= MO_32) {
		return 1 << ot;
	}
	else {
		return 4;
	}
}

static inline bool use_goto_tb(TCGContext * s, DisasContext *dc, target_ulong pc)
{
#ifndef CONFIG_USER_ONLY
	return (pc & TARGET_PAGE_MASK) == (dc->tb->pc & TARGET_PAGE_MASK) ||
		(pc & TARGET_PAGE_MASK) == (dc->pc_start & TARGET_PAGE_MASK);
#else
	return true;
#endif
}

static inline void gen_goto_tb(TCGContext * s, DisasContext *dc, int tb_num, target_ulong eip)
{
	target_ulong pc = dc->cs_base + eip;

	if (use_goto_tb(s, dc, pc)) {
		/* jump to same page: we can use a direct jump */
		tcg_gen_goto_tb(s, tb_num);
		gen_jmp_im(s, eip);
		tcg_gen_exit_tb(s, (uintptr_t)dc->tb + tb_num);
	}
	else {
		/* jump to another page */
		gen_jmp_im(s, eip);
		gen_jr(s, dc, s->cpu_tmp0);
	}
}

static inline void gen_jcc(TCGContext * s, DisasContext *dc, int b,
	target_ulong val, target_ulong next_eip)
{
	TCGLabel *l1, *l2;

	if (dc->jmp_opt) {
		l1 = gen_new_label(s);
		gen_jcc1(s, dc, b, l1);

		gen_goto_tb(s, dc, 0, next_eip);

		gen_set_label(s, l1);
		gen_goto_tb(s, dc, 1, val);
		dc->is_jmp = DISAS_TB_JUMP;
	}
	else {
		l1 = gen_new_label(s);
		l2 = gen_new_label(s);
		gen_jcc1(s, dc, b, l1);

		gen_jmp_im(s, next_eip);
		tcg_gen_br(s, l2);

		gen_set_label(s, l1);
		gen_jmp_im(s, val);
		gen_set_label(s, l2);
		gen_eob(s, dc);
	}
}

static void gen_cmovcc1(TCGContext * s, CPUX86State *env, DisasContext *dc, TCGMemOp ot, int b,
	int modrm, int reg)
{
	CCPrepare cc;

	gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);

	cc = gen_prepare_cc(s, dc, b, s->cpu_T1);

	if (!cc.use_reg2) {
		cc.reg2 = tcg_const_tl(s, cc.imm);
	}

	tcg_gen_movcond_tl(s, cc.cond, s->cpu_T0, cc.reg, cc.reg2, s->cpu_T0, s->cpu_regs[reg]);

	gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);

	if (!cc.use_reg2) {
		tcg_temp_free(s, cc.reg2);
	}
}

static inline void gen_op_movl_T0_seg(TCGContext * s, int seg_reg)
{
	tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env,
		offsetof(CPUX86State, segs[seg_reg].selector));
}

static inline void gen_op_movl_seg_T0_vm(TCGContext * s, int seg_reg)
{
	tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_T0);
	tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env,
		offsetof(CPUX86State, segs[seg_reg].selector));
	tcg_gen_shli_tl(s, s->cpu_seg_base[seg_reg], s->cpu_T0, 4);
}

/* move T0 to seg_reg and compute if the CPU state may change. Never
   call this function with seg_reg == R_CS */
static void gen_movl_seg_T0(TCGContext * s, DisasContext *dc, int seg_reg)
{
	if (dc->pe && !dc->vm86) {
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		gen_helper_load_seg(s, s->cpu_env, tcg_const_i32(s, seg_reg), s->cpu_tmp2_i32);
		/* abort translation because the addseg value may change or
		   because ss32 may change. For R_SS, translation must always
		   stop as a special handling must be done to disable hardware
		   interrupts for the next instruction */
		if (seg_reg == R_SS || (dc->code32 && seg_reg < R_FS))
			dc->is_jmp = DISAS_TB_JUMP;
	}
	else {
		gen_op_movl_seg_T0_vm(s, seg_reg);
		if (seg_reg == R_SS)
			dc->is_jmp = DISAS_TB_JUMP;
	}
}

static inline int svm_is_rep(int prefixes)
{
	return ((prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) ? 8 : 0);
}

static inline void
gen_svm_check_intercept_param(TCGContext * s, DisasContext *dc, target_ulong pc_start,
	uint32_t type, uint64_t param)
{
	/* no SVM activated; fast case */
	if (likely(!(dc->flags & HF_SVMI_MASK)))
		return;
	gen_update_cc_op(s, dc);
	gen_jmp_im(s, pc_start - dc->cs_base);
	gen_helper_svm_check_intercept_param(s, s->cpu_env, tcg_const_i32(s, type),
		tcg_const_i64(s, param));
}

static inline void
gen_svm_check_intercept(TCGContext * s, DisasContext *dc, target_ulong pc_start, uint64_t type)
{
	gen_svm_check_intercept_param(s, dc, pc_start, type, 0);
}

static inline void gen_stack_update(TCGContext * s, DisasContext *dc, int addend)
{
	gen_op_add_reg_im(s, mo_stacksize(s, dc), R_ESP, addend);
}

/* Generate a push. It depends on ss32, addseg and dflag.  */
static void gen_push_v(TCGContext * s, DisasContext *dc, TCGv val)
{
	TCGMemOp d_ot = mo_pushpop(s, dc, dc->dflag);
	TCGMemOp a_ot = mo_stacksize(s, dc);
	int size = 1 << d_ot;
	TCGv new_esp = s->cpu_A0;

	tcg_gen_subi_tl(s, s->cpu_A0, s->cpu_regs[R_ESP], size);

	if (!CODE64(dc)) {
		if (dc->addseg) {
			new_esp = s->cpu_tmp4;
			tcg_gen_mov_tl(s, new_esp, s->cpu_A0);
		}
		gen_lea_v_seg(s, dc, a_ot, s->cpu_A0, R_SS, -1);
	}

	gen_op_st_v(s, dc, d_ot, val, s->cpu_A0);
	gen_op_mov_reg_v(s, a_ot, R_ESP, new_esp);
}

/* two step pop is necessary for precise exceptions */
static TCGMemOp gen_pop_T0(TCGContext * s, DisasContext *dc)
{
	TCGMemOp d_ot = mo_pushpop(s, dc, dc->dflag);

	gen_lea_v_seg(s, dc, mo_stacksize(s, dc), s->cpu_regs[R_ESP], R_SS, -1);
	gen_op_ld_v(s, dc, d_ot, s->cpu_T0, s->cpu_A0);

	return d_ot;
}

static inline void gen_pop_update(TCGContext * s, DisasContext *dc, TCGMemOp ot)
{
	gen_stack_update(s, dc, 1 << ot);
}

static inline void gen_stack_A0(TCGContext * s, DisasContext *dc)
{
	gen_lea_v_seg(s, dc, dc->ss32 ? MO_32 : MO_16, s->cpu_regs[R_ESP], R_SS, -1);
}

static void gen_pusha(TCGContext * s, DisasContext *dc)
{
	TCGMemOp s_ot = dc->ss32 ? MO_32 : MO_16;
	TCGMemOp d_ot = dc->dflag;
	int size = 1 << d_ot;
	int i;

	for (i = 0; i < 8; i++) {
		tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_regs[R_ESP], (i - 8) * size);
		gen_lea_v_seg(s, dc, s_ot, s->cpu_A0, R_SS, -1);
		gen_op_st_v(s, dc, d_ot, s->cpu_regs[7 - i], s->cpu_A0);
	}

	gen_stack_update(s, dc, -8 * size);
}

static void gen_popa(TCGContext * s, DisasContext *dc)
{
	TCGMemOp s_ot = dc->ss32 ? MO_32 : MO_16;
	TCGMemOp d_ot = dc->dflag;
	int size = 1 << d_ot;
	int i;

	for (i = 0; i < 8; i++) {
		/* ESP is not reloaded */
		if (7 - i == R_ESP) {
			continue;
		}
		tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_regs[R_ESP], i * size);
		gen_lea_v_seg(s, dc, s_ot, s->cpu_A0, R_SS, -1);
		gen_op_ld_v(s, dc, d_ot, s->cpu_T0, s->cpu_A0);
		gen_op_mov_reg_v(s, d_ot, 7 - i, s->cpu_T0);
	}

	gen_stack_update(s, dc, 8 * size);
}

static void gen_enter(TCGContext * s, DisasContext *dc, int esp_addend, int level)
{
	TCGMemOp d_ot = mo_pushpop(s, dc, dc->dflag);
	TCGMemOp a_ot = CODE64(dc) ? MO_64 : dc->ss32 ? MO_32 : MO_16;
	int size = 1 << d_ot;

	/* Push BP; compute FrameTemp into T1.  */
	tcg_gen_subi_tl(s, s->cpu_T1, s->cpu_regs[R_ESP], size);
	gen_lea_v_seg(s, dc, a_ot, s->cpu_T1, R_SS, -1);
	gen_op_st_v(s, dc, d_ot, s->cpu_regs[R_EBP], s->cpu_A0);

	level &= 31;
	if (level != 0) {
		int i;

		/* Copy level-1 pointers from the previous frame.  */
		for (i = 1; i < level; ++i) {
			tcg_gen_subi_tl(s, s->cpu_A0, s->cpu_regs[R_EBP], size * i);
			gen_lea_v_seg(s, dc, a_ot, s->cpu_A0, R_SS, -1);
			gen_op_ld_v(s, dc, d_ot, s->cpu_tmp0, s->cpu_A0);

			tcg_gen_subi_tl(s, s->cpu_A0, s->cpu_T1, size * i);
			gen_lea_v_seg(s, dc, a_ot, s->cpu_A0, R_SS, -1);
			gen_op_st_v(s, dc, d_ot, s->cpu_tmp0, s->cpu_A0);
		}

		/* Push the current FrameTemp as the last level.  */
		tcg_gen_subi_tl(s, s->cpu_A0, s->cpu_T1, size * level);
		gen_lea_v_seg(s, dc, a_ot, s->cpu_A0, R_SS, -1);
		gen_op_st_v(s, dc, d_ot, s->cpu_T1, s->cpu_A0);
	}

	/* Copy the FrameTemp value to EBP.  */
	gen_op_mov_reg_v(s, a_ot, R_EBP, s->cpu_T1);

	/* Compute the final value of ESP.  */
	tcg_gen_subi_tl(s, s->cpu_T1, s->cpu_T1, esp_addend + size * level);
	gen_op_mov_reg_v(s, a_ot, R_ESP, s->cpu_T1);
}

static void gen_leave(TCGContext * s, DisasContext *dc)
{
	TCGMemOp d_ot = mo_pushpop(s, dc, dc->dflag);
	TCGMemOp a_ot = mo_stacksize(s, dc);

	gen_lea_v_seg(s, dc, a_ot, s->cpu_regs[R_EBP], R_SS, -1);
	gen_op_ld_v(s, dc, d_ot, s->cpu_T0, s->cpu_A0);

	tcg_gen_addi_tl(s, s->cpu_T1, s->cpu_regs[R_EBP], 1 << d_ot);

	gen_op_mov_reg_v(s, d_ot, R_EBP, s->cpu_T0);
	gen_op_mov_reg_v(s, a_ot, R_ESP, s->cpu_T1);
}

static void gen_exception(TCGContext * s, DisasContext *dc, int trapno, target_ulong cur_eip)
{
	gen_update_cc_op(s, dc);
	gen_jmp_im(s, cur_eip);
	gen_helper_raise_exception(s, s->cpu_env, tcg_const_i32(s, trapno));
	dc->is_jmp = DISAS_TB_JUMP;
}

/* Generate #UD for the current instruction.  The assumption here is that
   the instruction is known, but it isn't allowed in the current cpu mode.  */
static void gen_illegal_opcode(TCGContext * s, DisasContext *dc)
{
	gen_exception(s, dc, EXCP06_ILLOP, dc->pc_start - dc->cs_base);
}

/* Similarly, except that the assumption here is that we don't decode
   the instruction at all -- either a missing opcode, an unimplemented
   feature, or just a bogus instruction stream.  */
static void gen_unknown_opcode(TCGContext * s, CPUX86State *env, DisasContext *dc)
{
	gen_illegal_opcode(s, dc);
}

/* an interrupt is different from an exception because of the
   privilege checks */
static void gen_interrupt(TCGContext * s, DisasContext *dc, int intno,
	target_ulong cur_eip, target_ulong next_eip)
{
	gen_update_cc_op(s, dc);
	gen_jmp_im(s, cur_eip);
	gen_helper_raise_interrupt(s, s->cpu_env, tcg_const_i32(s, intno),
		tcg_const_i32(s, next_eip - cur_eip));
	dc->is_jmp = DISAS_TB_JUMP;
}

static void gen_debug(TCGContext * s, DisasContext *dc, target_ulong cur_eip)
{
	gen_update_cc_op(s, dc);
	gen_jmp_im(s, cur_eip);
	gen_helper_debug(s, s->cpu_env);
	dc->is_jmp = DISAS_TB_JUMP;
}

static void gen_set_hflag(TCGContext * s, DisasContext *dc, uint32_t mask)
{
	if ((dc->flags & mask) == 0) {
		TCGv_i32 t = tcg_temp_new_i32(s);
		tcg_gen_ld_i32(s, t, s->cpu_env, offsetof(CPUX86State, hflags));
		tcg_gen_ori_i32(s, t, t, mask);
		tcg_gen_st_i32(s, t, s->cpu_env, offsetof(CPUX86State, hflags));
		tcg_temp_free_i32(s, t);
		dc->flags |= mask;
	}
}

static void gen_reset_hflag(TCGContext * s, DisasContext *dc, uint32_t mask)
{
	if (dc->flags & mask) {
		TCGv_i32 t = tcg_temp_new_i32(s);
		tcg_gen_ld_i32(s, t, s->cpu_env, offsetof(CPUX86State, hflags));
		tcg_gen_andi_i32(s, t, t, ~mask);
		tcg_gen_st_i32(s, t, s->cpu_env, offsetof(CPUX86State, hflags));
		tcg_temp_free_i32(s, t);
		dc->flags &= ~mask;
	}
}

/* Clear BND registers during legacy branches.  */
static void gen_bnd_jmp(TCGContext * s, DisasContext *dc)
{
	/* Clear the registers only if BND prefix is missing, MPX is enabled,
	   and if the BNDREGs are known to be in use (non-zero) already.
	   The helper itself will check BNDPRESERVE at runtime.  */
	if ((dc->prefix & PREFIX_REPNZ) == 0
		&& (dc->flags & HF_MPX_EN_MASK) != 0
		&& (dc->flags & HF_MPX_IU_MASK) != 0) {
		gen_helper_bnd_jmp(s, s->cpu_env);
	}
}

/* Generate an end of block. Trace exception is also generated if needed.
   If INHIBIT, set HF_INHIBIT_IRQ_MASK if it isn't already set.
   If RECHECK_TF, emit a rechecking helper for #DB, ignoring the state of
   S->TF.  This is used by the syscall/sysret insns.  */
static void
do_gen_eob_worker(TCGContext * s, DisasContext *dc, bool inhibit, bool recheck_tf, TCGv jr)
{
	gen_update_cc_op(s, dc);

	/* If several instructions disable interrupts, only the first does it.  */
	if (inhibit && !(dc->flags & HF_INHIBIT_IRQ_MASK)) {
		gen_set_hflag(s, dc, HF_INHIBIT_IRQ_MASK);
	}
	else {
		gen_reset_hflag(s, dc, HF_INHIBIT_IRQ_MASK);
	}

	if (dc->tb->flags & HF_RF_MASK) {
		gen_helper_reset_rf(s, s->cpu_env);
	}
	if (dc->singlestep_enabled) {
		gen_helper_debug(s, s->cpu_env);
	}
	else if (recheck_tf) {
		gen_helper_rechecking_single_step(s, s->cpu_env);
		tcg_gen_exit_tb(s, 0);
	}
	else if (dc->tf) {
		gen_helper_single_step(s, s->cpu_env);
	}
	else if (!TCGV_IS_UNUSED(jr)) {
		TCGv vaddr = tcg_temp_new(s);

		tcg_gen_add_tl(s, vaddr, jr, s->cpu_seg_base[R_CS]);
		tcg_gen_lookup_and_goto_ptr(s, vaddr);
		tcg_temp_free(s, vaddr);
	}
	else {
		tcg_gen_exit_tb(s, 0);
	}
	dc->is_jmp = DISAS_TB_JUMP;
}

static inline void
gen_eob_worker(TCGContext * s, DisasContext *dc, bool inhibit, bool recheck_tf)
{
	TCGv unused;

	TCGV_UNUSED(unused);
	do_gen_eob_worker(s, dc, inhibit, recheck_tf, unused);
}

/* End of block.
   If INHIBIT, set HF_INHIBIT_IRQ_MASK if it isn't already set.  */
static void gen_eob_inhibit_irq(TCGContext * s, DisasContext *dc, bool inhibit)
{
	gen_eob_worker(s, dc, inhibit, false);
}

/* End of block, resetting the inhibit irq flag.  */
static void gen_eob(TCGContext * s, DisasContext *dc)
{
	gen_eob_worker(s, dc, false, false);
}

/* Jump to register */
static void gen_jr(TCGContext * s, DisasContext *dc, TCGv dest)
{
	do_gen_eob_worker(s, dc, false, false, dest);
}

/* generate a jump to eip. No segment change must happen before as a
   direct call to the next block may occur */
static void gen_jmp_tb(TCGContext * s, DisasContext *dc, target_ulong eip, int tb_num)
{
	gen_update_cc_op(s, dc);
	set_cc_op(s, dc, CC_OP_EFLAGS);
	if (dc->jmp_opt) {
		gen_goto_tb(s, dc, tb_num, eip);
		dc->is_jmp = DISAS_TB_JUMP;
	}
	else {
		gen_jmp_im(s, eip);
		gen_eob(s, dc);
	}
}

static void gen_jmp(TCGContext * s, DisasContext *dc, target_ulong eip)
{
	gen_jmp_tb(s, dc, eip, 0);
}

static inline void gen_ldq_env_A0(TCGContext * s, DisasContext *dc, int offset)
{
	tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0, dc->mem_index, MO_LEQ);
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset);
}

static inline void gen_stq_env_A0(TCGContext * s, DisasContext *dc, int offset)
{
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset);
	tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0, dc->mem_index, MO_LEQ);
}

static inline void gen_ldo_env_A0(TCGContext * s, DisasContext *dc, int offset)
{
	int mem_index = dc->mem_index;
	tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0, mem_index, MO_LEQ);
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset + offsetof(ZMMReg, ZMM_Q(0)));
	tcg_gen_addi_tl(s, s->cpu_tmp0, s->cpu_A0, 8);
	tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_tmp0, mem_index, MO_LEQ);
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset + offsetof(ZMMReg, ZMM_Q(1)));
}

static inline void gen_sto_env_A0(TCGContext * s, DisasContext *dc, int offset)
{
	int mem_index = dc->mem_index;
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset + offsetof(ZMMReg, ZMM_Q(0)));
	tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0, mem_index, MO_LEQ);
	tcg_gen_addi_tl(s, s->cpu_tmp0, s->cpu_A0, 8);
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, offset + offsetof(ZMMReg, ZMM_Q(1)));
	tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_tmp0, mem_index, MO_LEQ);
}

static inline void gen_op_movo(TCGContext * s, int d_offset, int s_offset)
{
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, s_offset + offsetof(ZMMReg, ZMM_Q(0)));
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, d_offset + offsetof(ZMMReg, ZMM_Q(0)));
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, s_offset + offsetof(ZMMReg, ZMM_Q(1)));
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, d_offset + offsetof(ZMMReg, ZMM_Q(1)));
}

static inline void gen_op_movq(TCGContext * s, int d_offset, int s_offset)
{
	tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env, s_offset);
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, d_offset);
}

static inline void gen_op_movl(TCGContext * s, int d_offset, int s_offset)
{
	tcg_gen_ld_i32(s, s->cpu_tmp2_i32, s->cpu_env, s_offset);
	tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env, d_offset);
}

static inline void gen_op_movq_env_0(TCGContext * s, int d_offset)
{
	tcg_gen_movi_i64(s, s->cpu_tmp1_i64, 0);
	tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env, d_offset);
}

typedef void(*SSEFunc_i_ep)(TCGContext * s, TCGv_i32 val, TCGv_ptr env, TCGv_ptr reg);
typedef void(*SSEFunc_l_ep)(TCGContext * s, TCGv_i64 val, TCGv_ptr env, TCGv_ptr reg);
typedef void(*SSEFunc_0_epi)(TCGContext * s, TCGv_ptr env, TCGv_ptr reg, TCGv_i32 val);
typedef void(*SSEFunc_0_epl)(TCGContext * s, TCGv_ptr env, TCGv_ptr reg, TCGv_i64 val);
typedef void(*SSEFunc_0_epp)(TCGContext * s, TCGv_ptr env, TCGv_ptr reg_a, TCGv_ptr reg_b);
typedef void(*SSEFunc_0_eppi)(TCGContext * s, TCGv_ptr env, TCGv_ptr reg_a, TCGv_ptr reg_b, TCGv_i32 val);
typedef void(*SSEFunc_0_ppi)(TCGContext * s, TCGv_ptr reg_a, TCGv_ptr reg_b, TCGv_i32 val);
typedef void(*SSEFunc_0_eppt)(TCGContext * s, TCGv_ptr env, TCGv_ptr reg_a, TCGv_ptr reg_b, TCGv val);

#define SSE_SPECIAL ((SSEFunc_0_epp)1)
#define SSE_DUMMY ((SSEFunc_0_epp)2)
#define SSE_NULL ((SSEFunc_0_epp)0)

#define SSE_EMPTY { SSE_NULL, SSE_NULL }

#define MMX_OP2(x) { (SSEFunc_0_epp)gen_helper_ ## x ## _mmx, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm, SSE_NULL, SSE_NULL }
#define SSE_FOP(x) { (SSEFunc_0_epp)gen_helper_ ## x ## ps, (SSEFunc_0_epp)gen_helper_ ## x ## pd, (SSEFunc_0_epp)gen_helper_ ## x ## ss, (SSEFunc_0_epp)gen_helper_ ## x ## sd, }

static const SSEFunc_0_epp sse_op_table1[256][4] = {
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	/* 3DNow! extensions */
	/*[0x0e] = */{ SSE_DUMMY, SSE_NULL, SSE_NULL, SSE_NULL }, /* femms */
	/*[0x0f] = */{ SSE_DUMMY, SSE_NULL, SSE_NULL, SSE_NULL }, /* pf... */
	/* pure SSE operations */
	/*[0x10] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* movups, movupd, movss, movsd */
	/*[0x11] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* movups, movupd, movss, movsd */
	/*[0x12] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* movlps, movlpd, movsldup, movddup */
	/*[0x13] = */{ SSE_SPECIAL, SSE_SPECIAL },  /* movlps, movlpd */
	/*[0x14] = */{ (SSEFunc_0_epp)gen_helper_punpckldq_xmm, (SSEFunc_0_epp)gen_helper_punpcklqdq_xmm, SSE_NULL, SSE_NULL },
	/*[0x15] = */{ (SSEFunc_0_epp)gen_helper_punpckhdq_xmm, (SSEFunc_0_epp)gen_helper_punpckhqdq_xmm, SSE_NULL, SSE_NULL },
	/*[0x16] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_NULL },  /* movhps, movhpd, movshdup */
	/*[0x17] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL },  /* movhps, movhpd */
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	/*[0x28] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL },  /* movaps, movapd */
	/*[0x29] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL },  /* movaps, movapd */
	/*[0x2a] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* cvtpi2ps, cvtpi2pd, cvtsi2ss, cvtsi2sd */
	/*[0x2b] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* movntps, movntpd, movntss, movntsd */
	/*[0x2c] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* cvttps2pi, cvttpd2pi, cvttsd2si, cvttss2si */
	/*[0x2d] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL }, /* cvtps2pi, cvtpd2pi, cvtsd2si, cvtss2si */
	/*[0x2e] = */{ (SSEFunc_0_epp)gen_helper_ucomiss, (SSEFunc_0_epp)gen_helper_ucomisd, SSE_NULL, SSE_NULL },
	/*[0x2f] = */{ (SSEFunc_0_epp)gen_helper_comiss, (SSEFunc_0_epp)gen_helper_comisd, SSE_NULL, SSE_NULL },
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	/* SSSE3, SSE4, MOVBE, CRC32, BMI1, BMI2, ADX.  */
	/*[0x38] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL },
	SSE_EMPTY,
	/*[0x3a] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL },
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	/*[0x50] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* movmskps, movmskpd */
	/*[0x51] = */SSE_FOP(sqrt),
	/*[0x52] = */{ (SSEFunc_0_epp)gen_helper_rsqrtps, NULL, (SSEFunc_0_epp)gen_helper_rsqrtss, NULL },
	/*[0x53] = */{ (SSEFunc_0_epp)gen_helper_rcpps, NULL, (SSEFunc_0_epp)gen_helper_rcpss, NULL },
	/*[0x54] = */{ (SSEFunc_0_epp)gen_helper_pand_xmm, (SSEFunc_0_epp)gen_helper_pand_xmm, SSE_NULL, SSE_NULL }, /* andps, andpd */
	/*[0x55] = */{ (SSEFunc_0_epp)gen_helper_pandn_xmm, (SSEFunc_0_epp)gen_helper_pandn_xmm, SSE_NULL, SSE_NULL }, /* andnps, andnpd */
	/*[0x56] = */{ (SSEFunc_0_epp)gen_helper_por_xmm, (SSEFunc_0_epp)gen_helper_por_xmm, SSE_NULL, SSE_NULL }, /* orps, orpd */
	/*[0x57] = */{ (SSEFunc_0_epp)gen_helper_pxor_xmm, (SSEFunc_0_epp)gen_helper_pxor_xmm, SSE_NULL, SSE_NULL }, /* xorps, xorpd */
	/*[0x58] = */SSE_FOP(add),
	/*[0x59] = */SSE_FOP(mul),
	/*[0x5a] = */{ (SSEFunc_0_epp)gen_helper_cvtps2pd, (SSEFunc_0_epp)gen_helper_cvtpd2ps, (SSEFunc_0_epp)gen_helper_cvtss2sd, (SSEFunc_0_epp)gen_helper_cvtsd2ss },
	/*[0x5b] = */{ (SSEFunc_0_epp)gen_helper_cvtdq2ps, (SSEFunc_0_epp)gen_helper_cvtps2dq, (SSEFunc_0_epp)gen_helper_cvttps2dq, SSE_NULL },
	/*[0x5c] = */SSE_FOP(sub),
	/*[0x5d] = */SSE_FOP(min),
	/*[0x5e] = */SSE_FOP(div),
	/*[0x5f] = */SSE_FOP(max),

	/* MMX ops and their SSE extensions */
	/*[0x60] = */MMX_OP2(punpcklbw),
	/*[0x61] = */MMX_OP2(punpcklwd),
	/*[0x62] = */MMX_OP2(punpckldq),
	/*[0x63] = */MMX_OP2(packsswb),
	/*[0x64] = */MMX_OP2(pcmpgtb),
	/*[0x65] = */MMX_OP2(pcmpgtw),
	/*[0x66] = */MMX_OP2(pcmpgtl),
	/*[0x67] = */MMX_OP2(packuswb),
	/*[0x68] = */MMX_OP2(punpckhbw),
	/*[0x69] = */MMX_OP2(punpckhwd),
	/*[0x6a] = */MMX_OP2(punpckhdq),
	/*[0x6b] = */MMX_OP2(packssdw),
	/*[0x6c] = */{ NULL, (SSEFunc_0_epp)gen_helper_punpcklqdq_xmm, SSE_NULL, SSE_NULL },
	/*[0x6d] = */{ NULL, (SSEFunc_0_epp)gen_helper_punpckhqdq_xmm, SSE_NULL, SSE_NULL },
	/*[0x6e] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* movd mm, ea */
	/*[0x6f] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_NULL }, /* movq, movdqa, , movqdu */

	/*[0x70] = */{ (SSEFunc_0_epp)gen_helper_pshufw_mmx,(SSEFunc_0_epp)gen_helper_pshufd_xmm,(SSEFunc_0_epp)gen_helper_pshufhw_xmm,(SSEFunc_0_epp)gen_helper_pshuflw_xmm }, /* XXX: casts */
	/*[0x71] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* shiftw */
	/*[0x72] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* shiftd */
	/*[0x73] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* shiftq */
	/*[0x74] = */MMX_OP2(pcmpeqb),
	/*[0x75] = */MMX_OP2(pcmpeqw),
	/*[0x76] = */MMX_OP2(pcmpeql),
	/*[0x77] = */{ SSE_DUMMY, SSE_NULL, SSE_NULL, SSE_NULL }, /* emms */
	/*[0x78] = */{ NULL, SSE_SPECIAL, NULL, SSE_SPECIAL }, /* extrq_i, insertq_i */
	/*[0x79] = */{ NULL, (SSEFunc_0_epp)gen_helper_extrq_r, NULL, (SSEFunc_0_epp)gen_helper_insertq_r },
	SSE_EMPTY,SSE_EMPTY,
	/*[0x7c] = */{ NULL, (SSEFunc_0_epp)gen_helper_haddpd, NULL, (SSEFunc_0_epp)gen_helper_haddps },
	/*[0x7d] = */{ NULL, (SSEFunc_0_epp)gen_helper_hsubpd, NULL, (SSEFunc_0_epp)gen_helper_hsubps },
	/*[0x7e] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_NULL }, /* movd, movd, , movq */
	/*[0x7f] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL, SSE_NULL }, /* movq, movdqa, movdqu */
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	SSE_EMPTY,SSE_EMPTY,
	/*[0xc2] = */SSE_FOP(cmpeq),
	SSE_EMPTY,
	/*[0xc4] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* pinsrw */
	/*[0xc5] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* pextrw */
	/*[0xc6] = */{ (SSEFunc_0_epp)gen_helper_shufps, (SSEFunc_0_epp)gen_helper_shufpd, SSE_NULL, SSE_NULL }, /* XXX: casts */
	SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,SSE_EMPTY,
	/*[0xd0] = */{ NULL, (SSEFunc_0_epp)gen_helper_addsubpd, NULL, (SSEFunc_0_epp)gen_helper_addsubps },
	/*[0xd1] = */MMX_OP2(psrlw),
	/*[0xd2] = */MMX_OP2(psrld),
	/*[0xd3] = */MMX_OP2(psrlq),
	/*[0xd4] = */MMX_OP2(paddq),
	/*[0xd5] = */MMX_OP2(pmullw),
	/*[0xd6] = */{ NULL, SSE_SPECIAL, SSE_SPECIAL, SSE_SPECIAL },
	/*[0xd7] = */{ SSE_SPECIAL, SSE_SPECIAL, SSE_NULL, SSE_NULL }, /* pmovmskb */
	/*[0xd8] = */MMX_OP2(psubusb),
	/*[0xd9] = */MMX_OP2(psubusw),
	/*[0xda] = */MMX_OP2(pminub),
	/*[0xdb] = */MMX_OP2(pand),
	/*[0xdc] = */MMX_OP2(paddusb),
	/*[0xdd] = */MMX_OP2(paddusw),
	/*[0xde] = */MMX_OP2(pmaxub),
	/*[0xdf] = */MMX_OP2(pandn),

	/*[0xe0] = */MMX_OP2(pavgb),
	/*[0xe1] = */MMX_OP2(psraw),
	/*[0xe2] = */MMX_OP2(psrad),
	/*[0xe3] = */MMX_OP2(pavgw),
	/*[0xe4] = */MMX_OP2(pmulhuw),
	/*[0xe5] = */MMX_OP2(pmulhw),
	/*[0xe6] = */{ NULL, (SSEFunc_0_epp)gen_helper_cvttpd2dq, (SSEFunc_0_epp)gen_helper_cvtdq2pd, (SSEFunc_0_epp)gen_helper_cvtpd2dq },
	/*[0xe7] = */{ SSE_SPECIAL , SSE_SPECIAL, SSE_NULL, SSE_NULL },  /* movntq, movntq */
	/*[0xe8] = */MMX_OP2(psubsb),
	/*[0xe9] = */MMX_OP2(psubsw),
	/*[0xea] = */MMX_OP2(pminsw),
	/*[0xeb] = */MMX_OP2(por),
	/*[0xec] = */MMX_OP2(paddsb),
	/*[0xed] = */MMX_OP2(paddsw),
	/*[0xee] = */MMX_OP2(pmaxsw),
	/*[0xef] = */MMX_OP2(pxor),

	/*[0xf0] = */{ NULL, NULL, NULL, SSE_SPECIAL }, /* lddqu */
	/*[0xf1] = */MMX_OP2(psllw),
	/*[0xf2] = */MMX_OP2(pslld),
	/*[0xf3] = */MMX_OP2(psllq),
	/*[0xf4] = */MMX_OP2(pmuludq),
	/*[0xf5] = */MMX_OP2(pmaddwd),
	/*[0xf6] = */MMX_OP2(psadbw),
	/*[0xf7] = */{ (SSEFunc_0_epp)gen_helper_maskmov_mmx, (SSEFunc_0_epp)gen_helper_maskmov_xmm, SSE_NULL, SSE_NULL }, /* XXX: casts */
	/*[0xf8] = */MMX_OP2(psubb),
	/*[0xf9] = */MMX_OP2(psubw),
	/*[0xfa] = */MMX_OP2(psubl),
	/*[0xfb] = */MMX_OP2(psubq),
	/*[0xfc] = */MMX_OP2(paddb),
	/*[0xfd] = */MMX_OP2(paddw),
	/*[0xfe] = */MMX_OP2(paddl),
	SSE_EMPTY,
};

#undef MMX_OP2
#define MMX_OP2(x) { (SSEFunc_0_epp)gen_helper_ ## x ## _mmx, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm }

static const SSEFunc_0_epp sse_op_table2[3 * 8][2] = {
	{0,0}, {0,0},
	/*[0 + 2] =  2*/ MMX_OP2(psrlw),
	{ 0,0 },
	/*[0 + 4] =  4*/ MMX_OP2(psraw),
	{ 0,0 },
	/*[0 + 6] =  6*/ MMX_OP2(psllw),
	{ 0,0 },{ 0,0 },{ 0,0 },
	/*[8 + 2] = 10*/ MMX_OP2(psrld),
	{ 0,0 },
	/*[8 + 4] = 12*/ MMX_OP2(psrad),
	{ 0,0 },
	/*[8 + 6] = 14*/ MMX_OP2(pslld),
	{ 0,0 },{ 0,0 },{ 0,0 },
	/*[16 + 2] = 18*/ MMX_OP2(psrlq),
	/*[16 + 3] = 19*/{ NULL, (SSEFunc_0_epp)gen_helper_psrldq_xmm },
	{ 0,0 },{ 0,0 },
	/*[16 + 6] = 22*/ MMX_OP2(psllq),
	/*[16 + 7] = 23*/{ NULL, (SSEFunc_0_epp)gen_helper_pslldq_xmm }
};

static const SSEFunc_0_epi sse_op_table3ai[] = {
	(SSEFunc_0_epi)gen_helper_cvtsi2ss,
	(SSEFunc_0_epi)gen_helper_cvtsi2sd
};

#ifdef TARGET_X86_64
static const SSEFunc_0_epl sse_op_table3aq[] = {
	(SSEFunc_0_epl)gen_helper_cvtsq2ss,
	(SSEFunc_0_epl)gen_helper_cvtsq2sd
};
#endif

static const SSEFunc_i_ep sse_op_table3bi[] = {
	(SSEFunc_i_ep)gen_helper_cvttss2si,
	(SSEFunc_i_ep)gen_helper_cvtss2si,
	(SSEFunc_i_ep)gen_helper_cvttsd2si,
	(SSEFunc_i_ep)gen_helper_cvtsd2si
};

#ifdef TARGET_X86_64
static const SSEFunc_l_ep sse_op_table3bq[] = {
	(SSEFunc_l_ep)gen_helper_cvttss2sq,
	(SSEFunc_l_ep)gen_helper_cvtss2sq,
	(SSEFunc_l_ep)gen_helper_cvttsd2sq,
	(SSEFunc_l_ep)gen_helper_cvtsd2sq
};
#endif

static const SSEFunc_0_epp sse_op_table4[8][4] = {
	SSE_FOP(cmpeq),
	SSE_FOP(cmplt),
	SSE_FOP(cmple),
	SSE_FOP(cmpunord),
	SSE_FOP(cmpneq),
	SSE_FOP(cmpnlt),
	SSE_FOP(cmpnle),
	SSE_FOP(cmpord),
};

static const SSEFunc_0_epp sse_op_table5[256] = {
	0,0,0,0,0,0,0,0,0,0,0,0,
	/*[0x0c] = */ (SSEFunc_0_epp)gen_helper_pi2fw,
	/*[0x0d] = */ (SSEFunc_0_epp)gen_helper_pi2fd,
	0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,
	/*[0x1c] = */ (SSEFunc_0_epp)gen_helper_pf2iw,
	/*[0x1d] = */ (SSEFunc_0_epp)gen_helper_pf2id,
	0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,
	/*[0x8a] = */ (SSEFunc_0_epp)gen_helper_pfnacc,
	0,0,0,
	/*[0x8e] = */ (SSEFunc_0_epp)gen_helper_pfpnacc,
	0,
	/*[0x90] = */ (SSEFunc_0_epp)gen_helper_pfcmpge,
	0,0,0,
	/*[0x94] = */ (SSEFunc_0_epp)gen_helper_pfmin,
	0,
	/*[0x96] = */ (SSEFunc_0_epp)gen_helper_pfrcp,
	/*[0x97] = */ (SSEFunc_0_epp)gen_helper_pfrsqrt,
	0,0,
	/*[0x9a] = */ (SSEFunc_0_epp)gen_helper_pfsub,
	0,0,0,
	/*[0x9e] = */ (SSEFunc_0_epp)gen_helper_pfadd,
	0,
	/*[0xa0] = */ (SSEFunc_0_epp)gen_helper_pfcmpgt,
	0,0,0,
	/*[0xa4] = */ (SSEFunc_0_epp)gen_helper_pfmax,
	0,
	/*[0xa6] = */ (SSEFunc_0_epp)gen_helper_movq, /* pfrcpit1; no need to actually increase precision */
	/*[0xa7] = */ (SSEFunc_0_epp)gen_helper_movq, /* pfrsqit1 */
	0,0,
	/*[0xaa] = */ (SSEFunc_0_epp)gen_helper_pfsubr,
	0,0,0,
	/*[0xae] = */ (SSEFunc_0_epp)gen_helper_pfacc,
	0,
	/*[0xb0] = */ (SSEFunc_0_epp)gen_helper_pfcmpeq,
	0,0,0,
	/*[0xb4] = */ (SSEFunc_0_epp)gen_helper_pfmul,
	0,
	/*[0xb6] = */ (SSEFunc_0_epp)gen_helper_movq, /* pfrcpit2 */
	/*[0xb7] = */ (SSEFunc_0_epp)gen_helper_pmulhrw_mmx,
	0,0,0,
	/*[0xbb] = */ (SSEFunc_0_epp)gen_helper_pswapd,
	0,0,0,
	/*[0xbf] = */ (SSEFunc_0_epp)gen_helper_pavgb_mmx, /* pavgusb */
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

struct SSEOpHelper_epp {
	SSEFunc_0_epp op[2];
	uint32_t ext_mask;
};

struct SSEOpHelper_eppi {
	SSEFunc_0_eppi op[2];
	uint32_t ext_mask;
};

#define SSSE3_OP(x) { MMX_OP2(x), CPUID_EXT_SSSE3 }
#define SSE41_OP(x) { { NULL, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm }, CPUID_EXT_SSE41 }
#define SSE42_OP(x) { { NULL, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm }, CPUID_EXT_SSE42 }
#define SSE41_SPECIAL { { NULL, SSE_SPECIAL }, CPUID_EXT_SSE41 }
#define PCLMULQDQ_OP(x) { { NULL, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm }, CPUID_EXT_PCLMULQDQ }
#define AESNI_OP(x) { { NULL, (SSEFunc_0_epp)gen_helper_ ## x ## _xmm }, CPUID_EXT_AES }

static const struct SSEOpHelper_epp sse_op_table6[256] = {
	/*[0x00] = */ SSSE3_OP(pshufb),
	/*[0x01] = */ SSSE3_OP(phaddw),
	/*[0x02] = */ SSSE3_OP(phaddd),
	/*[0x03] = */ SSSE3_OP(phaddsw),
	/*[0x04] = */ SSSE3_OP(pmaddubsw),
	/*[0x05] = */ SSSE3_OP(phsubw),
	/*[0x06] = */ SSSE3_OP(phsubd),
	/*[0x07] = */ SSSE3_OP(phsubsw),
	/*[0x08] = */ SSSE3_OP(psignb),
	/*[0x09] = */ SSSE3_OP(psignw),
	/*[0x0a] = */ SSSE3_OP(psignd),
	/*[0x0b] = */ SSSE3_OP(pmulhrsw),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0x10] = */ SSE41_OP(pblendvb),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0x14] = */ SSE41_OP(blendvps),
	/*[0x15] = */ SSE41_OP(blendvpd),
	{ { 0,0 },0 },
	/*[0x17] = */ SSE41_OP(ptest),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0x1c] = */ SSSE3_OP(pabsb),
	/*[0x1d] = */ SSSE3_OP(pabsw),
	/*[0x1e] = */ SSSE3_OP(pabsd),
	{ { 0,0 },0 },
	/*[0x20] = */ SSE41_OP(pmovsxbw),
	/*[0x21] = */ SSE41_OP(pmovsxbd),
	/*[0x22] = */ SSE41_OP(pmovsxbq),
	/*[0x23] = */ SSE41_OP(pmovsxwd),
	/*[0x24] = */ SSE41_OP(pmovsxwq),
	/*[0x25] = */ SSE41_OP(pmovsxdq),
	{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0x28] = */ SSE41_OP(pmuldq),
	/*[0x29] = */ SSE41_OP(pcmpeqq),
	/*[0x2a] = */ SSE41_SPECIAL, /* movntqda */
	/*[0x2b] = */ SSE41_OP(packusdw),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0x30] = */ SSE41_OP(pmovzxbw),
	/*[0x31] = */ SSE41_OP(pmovzxbd),
	/*[0x32] = */ SSE41_OP(pmovzxbq),
	/*[0x33] = */ SSE41_OP(pmovzxwd),
	/*[0x34] = */ SSE41_OP(pmovzxwq),
	/*[0x35] = */ SSE41_OP(pmovzxdq),
	{ { 0,0 },0 },
	/*[0x37] = */ SSE42_OP(pcmpgtq),
	/*[0x38] = */ SSE41_OP(pminsb),
	/*[0x39] = */ SSE41_OP(pminsd),
	/*[0x3a] = */ SSE41_OP(pminuw),
	/*[0x3b] = */ SSE41_OP(pminud),
	/*[0x3c] = */ SSE41_OP(pmaxsb),
	/*[0x3d] = */ SSE41_OP(pmaxsd),
	/*[0x3e] = */ SSE41_OP(pmaxuw),
	/*[0x3f] = */ SSE41_OP(pmaxud),
	/*[0x40] = */ SSE41_OP(pmulld),
	/*[0x41] = */ SSE41_OP(phminposuw),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	/*[0xdb] = */ AESNI_OP(aesimc),
	/*[0xdc] = */ AESNI_OP(aesenc),
	/*[0xdd] = */ AESNI_OP(aesenclast),
	/*[0xde] = */ AESNI_OP(aesdec),
	/*[0xdf] = */ AESNI_OP(aesdeclast),
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },
	{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 },{ { 0,0 },0 }
};

#undef MMX_OP2
#define MMX_OP2(x) { (SSEFunc_0_eppi)gen_helper_ ## x ## _mmx, (SSEFunc_0_eppi)gen_helper_ ## x ## _xmm }
#undef SSE_SPECIAL
#define SSE_SPECIAL ((SSEFunc_0_eppi)1)
#undef SSE41_OP
#define SSE41_OP(x) { { NULL, (SSEFunc_0_eppi)gen_helper_ ## x ## _xmm }, CPUID_EXT_SSE41 }
#undef SSE42_OP
#define SSE42_OP(x) { { NULL, (SSEFunc_0_eppi)gen_helper_ ## x ## _xmm }, CPUID_EXT_SSE42 }
#undef PCLMULQDQ_OP
#define PCLMULQDQ_OP(x) { { NULL, (SSEFunc_0_eppi)gen_helper_ ## x ## _xmm }, CPUID_EXT_PCLMULQDQ }
#undef AESNI_OP
#define AESNI_OP(x) { { NULL, (SSEFunc_0_eppi)gen_helper_ ## x ## _xmm }, CPUID_EXT_AES }

static const struct SSEOpHelper_eppi sse_op_table7[256] = {
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0x08] = */ SSE41_OP(roundps),
	/*[0x09] = */ SSE41_OP(roundpd),
	/*[0x0a] = */ SSE41_OP(roundss),
	/*[0x0b] = */ SSE41_OP(roundsd),
	/*[0x0c] = */ SSE41_OP(blendps),
	/*[0x0d] = */ SSE41_OP(blendpd),
	/*[0x0e] = */ SSE41_OP(pblendw),
	/*[0x0f] = */ SSSE3_OP(palignr),
	{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0x14] = */ SSE41_SPECIAL, /* pextrb */
	/*[0x15] = */ SSE41_SPECIAL, /* pextrw */
	/*[0x16] = */ SSE41_SPECIAL, /* pextrd/pextrq */
	/*[0x17] = */ SSE41_SPECIAL, /* extractps */
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0x20] = */ SSE41_SPECIAL, /* pinsrb */
	/*[0x21] = */ SSE41_SPECIAL, /* insertps */
	/*[0x22] = */ SSE41_SPECIAL, /* pinsrd/pinsrq */
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0x40] = */ SSE41_OP(dpps),
	/*[0x41] = */ SSE41_OP(dppd),
	/*[0x42] = */ SSE41_OP(mpsadbw),
	{ 0 },
	/*[0x44] = */ PCLMULQDQ_OP(pclmulqdq),
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0x60] = */ SSE42_OP(pcmpestrm),
	/*[0x61] = */ SSE42_OP(pcmpestri),
	/*[0x62] = */ SSE42_OP(pcmpistrm),
	/*[0x63] = */ SSE42_OP(pcmpistri),
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	/*[0xdf] = */ AESNI_OP(aeskeygenassist),
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },
	{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 },{ 0 }
};

static void gen_sse(TCGContext * s, CPUX86State *env, DisasContext *dc, int b,
	target_ulong pc_start, int rex_r)
{
	int b1, op1_offset, op2_offset, is_xmm, val;
	int modrm, mod, rm, reg;
	SSEFunc_0_epp sse_fn_epp;
	SSEFunc_0_eppi sse_fn_eppi;
	SSEFunc_0_ppi sse_fn_ppi;
	SSEFunc_0_eppt sse_fn_eppt;
	TCGMemOp ot;

	b &= 0xff;
	if (dc->prefix & PREFIX_DATA)
		b1 = 1;
	else if (dc->prefix & PREFIX_REPZ)
		b1 = 2;
	else if (dc->prefix & PREFIX_REPNZ)
		b1 = 3;
	else
		b1 = 0;
	sse_fn_epp = sse_op_table1[b][b1];
	if (!sse_fn_epp) {
		goto unknown_op;
	}
	if ((b <= 0x5f && b >= 0x10) || b == 0xc6 || b == 0xc2) {
		is_xmm = 1;
	}
	else {
		if (b1 == 0) {
			/* MMX case */
			is_xmm = 0;
		}
		else {
			is_xmm = 1;
		}
	}
	/* simple MMX/SSE operation */
	if (dc->flags & HF_TS_MASK) {
		gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
		return;
	}
	if (dc->flags & HF_EM_MASK) {
	illegal_op:
		gen_illegal_opcode(s, dc);
		return;
	}
	if (is_xmm
		&& !(dc->flags & HF_OSFXSR_MASK)
		&& ((b != 0x38 && b != 0x3a) || (dc->prefix & PREFIX_DATA))) {
		goto unknown_op;
	}
	if (b == 0x0e) {
		if (!(dc->cpuid_ext2_features & CPUID_EXT2_3DNOW)) {
			/* If we were fully decoding this we might use illegal_op.  */
			goto unknown_op;
		}
		/* femms */
		gen_helper_emms(s, s->cpu_env);
		return;
	}
	if (b == 0x77) {
		/* emms */
		gen_helper_emms(s, s->cpu_env);
		return;
	}
	/* prepare MMX state (XXX: optimize by storing fptt and fptags in
	   the static cpu state) */
	if (!is_xmm) {
		gen_helper_enter_mmx(s, s->cpu_env);
	}

	modrm = cpu_ldub_code(env, dc->pc++);
	reg = ((modrm >> 3) & 7);
	if (is_xmm)
		reg |= rex_r;
	mod = (modrm >> 6) & 3;
	if (sse_fn_epp == (SSEFunc_0_epp)SSE_SPECIAL) {
		b |= (b1 << 8);
		switch (b) {
		case 0x0e7: /* movntq */
			if (mod == 3) {
				goto illegal_op;
			}
			gen_lea_modrm(s, env, dc, modrm);
			gen_stq_env_A0(s, dc, offsetof(CPUX86State, fpregs[reg].mmx));
			break;
		case 0x1e7: /* movntdq */
		case 0x02b: /* movntps */
		case 0x12b: /* movntps */
			if (mod == 3)
				goto illegal_op;
			gen_lea_modrm(s, env, dc, modrm);
			gen_sto_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			break;
		case 0x3f0: /* lddqu */
			if (mod == 3)
				goto illegal_op;
			gen_lea_modrm(s, env, dc, modrm);
			gen_ldo_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			break;
		case 0x22b: /* movntss */
		case 0x32b: /* movntsd */
			if (mod == 3)
				goto illegal_op;
			gen_lea_modrm(s, env, dc, modrm);
			if (b1 & 1) {
				gen_stq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_L(0)));
				gen_op_st_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
			}
			break;
		case 0x6e: /* movd mm, ea */
#ifdef TARGET_X86_64
			if (dc->dflag == MO_64) {
				gen_ldst_modrm(s, env, dc, modrm, MO_64, OR_TMP0, 0);
				tcg_gen_st_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, fpregs[reg].mmx));
			}
			else
#endif
			{
				gen_ldst_modrm(s, env, dc, modrm, MO_32, OR_TMP0, 0);
				tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
					offsetof(CPUX86State, fpregs[reg].mmx));
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_movl_mm_T0_mmx(s, s->cpu_ptr0, s->cpu_tmp2_i32);
			}
			break;
		case 0x16e: /* movd xmm, ea */
#ifdef TARGET_X86_64
			if (dc->dflag == MO_64) {
				gen_ldst_modrm(s, env, dc, modrm, MO_64, OR_TMP0, 0);
				tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[reg]));
				gen_helper_movq_mm_T0_xmm(s, s->cpu_ptr0, s->cpu_T0);
			}
			else
#endif
			{
				gen_ldst_modrm(s, env, dc, modrm, MO_32, OR_TMP0, 0);
				tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[reg]));
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_movl_mm_T0_xmm(s, s->cpu_ptr0, s->cpu_tmp2_i32);
			}
			break;
		case 0x6f: /* movq mm, ea */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State, fpregs[reg].mmx));
			}
			else {
				rm = (modrm & 7);
				tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env,
					offsetof(CPUX86State, fpregs[rm].mmx));
				tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env,
					offsetof(CPUX86State, fpregs[reg].mmx));
			}
			break;
		case 0x010: /* movups */
		case 0x110: /* movupd */
		case 0x028: /* movaps */
		case 0x128: /* movapd */
		case 0x16f: /* movdqa xmm, ea */
		case 0x26f: /* movdqu xmm, ea */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldo_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movo(s, offsetof(CPUX86State, xmm_regs[reg]),
					offsetof(CPUX86State, xmm_regs[rm]));
			}
			break;
		case 0x210: /* movss xmm, ea */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_op_ld_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)));
				tcg_gen_movi_tl(s, s->cpu_T0, 0);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(1)));
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(2)));
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(3)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_L(0)));
			}
			break;
		case 0x310: /* movsd xmm, ea */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
				tcg_gen_movi_tl(s, s->cpu_T0, 0);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(2)));
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(3)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)));
			}
			break;
		case 0x012: /* movlps */
		case 0x112: /* movlpd */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				/* movhlps */
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(1)));
			}
			break;
		case 0x212: /* movsldup */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldo_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_L(0)));
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(2)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_L(2)));
			}
			gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(1)),
				offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)));
			gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(3)),
				offsetof(CPUX86State, xmm_regs[reg].ZMM_L(2)));
			break;
		case 0x312: /* movddup */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)));
			}
			gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(1)),
				offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)));
			break;
		case 0x016: /* movhps */
		case 0x116: /* movhpd */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(1)));
			}
			else {
				/* movlhps */
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(1)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)));
			}
			break;
		case 0x216: /* movshdup */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldo_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(1)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_L(1)));
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(3)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_L(3)));
			}
			gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)),
				offsetof(CPUX86State, xmm_regs[reg].ZMM_L(1)));
			gen_op_movl(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(2)),
				offsetof(CPUX86State, xmm_regs[reg].ZMM_L(3)));
			break;
		case 0x178:
		case 0x378:
		{
			int bit_index, field_length;

			if (b1 == 1 && reg != 0)
				goto illegal_op;
			field_length = cpu_ldub_code(env, dc->pc++) & 0x3F;
			bit_index = cpu_ldub_code(env, dc->pc++) & 0x3F;
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
				offsetof(CPUX86State, xmm_regs[reg]));
			if (b1 == 1)
				gen_helper_extrq_i(s, s->cpu_env, s->cpu_ptr0,
					tcg_const_i32(s, bit_index),
					tcg_const_i32(s, field_length));
			else
				gen_helper_insertq_i(s, s->cpu_env, s->cpu_ptr0,
					tcg_const_i32(s, bit_index),
					tcg_const_i32(s, field_length));
		}
		break;
		case 0x7e: /* movd ea, mm */
#ifdef TARGET_X86_64
			if (dc->dflag == MO_64) {
				tcg_gen_ld_i64(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, fpregs[reg].mmx));
				gen_ldst_modrm(s, env, dc, modrm, MO_64, OR_TMP0, 1);
			}
			else
#endif
			{
				tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, fpregs[reg].mmx.MMX_L(0)));
				gen_ldst_modrm(s, env, dc, modrm, MO_32, OR_TMP0, 1);
			}
			break;
		case 0x17e: /* movd ea, xmm */
#ifdef TARGET_X86_64
			if (dc->dflag == MO_64) {
				tcg_gen_ld_i64(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)));
				gen_ldst_modrm(s, env, dc, modrm, MO_64, OR_TMP0, 1);
			}
			else
#endif
			{
				tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)));
				gen_ldst_modrm(s, env, dc, modrm, MO_32, OR_TMP0, 1);
			}
			break;
		case 0x27e: /* movq xmm, ea */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_ldq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)));
			}
			gen_op_movq_env_0(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(1)));
			break;
		case 0x7f: /* movq ea, mm */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_stq_env_A0(s, dc, offsetof(CPUX86State, fpregs[reg].mmx));
			}
			else {
				rm = (modrm & 7);
				gen_op_movq(s, offsetof(CPUX86State, fpregs[rm].mmx),
					offsetof(CPUX86State, fpregs[reg].mmx));
			}
			break;
		case 0x011: /* movups */
		case 0x111: /* movupd */
		case 0x029: /* movaps */
		case 0x129: /* movapd */
		case 0x17f: /* movdqa ea, xmm */
		case 0x27f: /* movdqu ea, xmm */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_sto_env_A0(s, dc, offsetof(CPUX86State, xmm_regs[reg]));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movo(s, offsetof(CPUX86State, xmm_regs[rm]),
					offsetof(CPUX86State, xmm_regs[reg]));
			}
			break;
		case 0x211: /* movss ea, xmm */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)));
				gen_op_st_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movl(s, offsetof(CPUX86State, xmm_regs[rm].ZMM_L(0)),
					offsetof(CPUX86State, xmm_regs[reg].ZMM_L(0)));
			}
			break;
		case 0x311: /* movsd ea, xmm */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_stq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)));
			}
			break;
		case 0x013: /* movlps */
		case 0x113: /* movlpd */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_stq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				goto illegal_op;
			}
			break;
		case 0x017: /* movhps */
		case 0x117: /* movhpd */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_stq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(1)));
			}
			else {
				goto illegal_op;
			}
			break;
		case 0x71: /* shift mm, im */
		case 0x72:
		case 0x73:
		case 0x171: /* shift xmm, im */
		case 0x172:
		case 0x173:
			if (b1 >= 2) {
				goto unknown_op;
			}
			val = cpu_ldub_code(env, dc->pc++);
			if (is_xmm) {
				tcg_gen_movi_tl(s, s->cpu_T0, val);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_t0.ZMM_L(0)));
				tcg_gen_movi_tl(s, s->cpu_T0, 0);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_t0.ZMM_L(1)));
				op1_offset = offsetof(CPUX86State, xmm_t0);
			}
			else {
				tcg_gen_movi_tl(s, s->cpu_T0, val);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, mmx_t0.MMX_L(0)));
				tcg_gen_movi_tl(s, s->cpu_T0, 0);
				tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, mmx_t0.MMX_L(1)));
				op1_offset = offsetof(CPUX86State, mmx_t0);
			}
			sse_fn_epp = sse_op_table2[((b - 1) & 3) * 8 +
				(((modrm >> 3)) & 7)][b1];
			if (!sse_fn_epp) {
				goto unknown_op;
			}
			if (is_xmm) {
				rm = (modrm & 7) | REX_B(dc);
				op2_offset = offsetof(CPUX86State, xmm_regs[rm]);
			}
			else {
				rm = (modrm & 7);
				op2_offset = offsetof(CPUX86State, fpregs[rm].mmx);
			}
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op2_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op1_offset);
			sse_fn_epp(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
			break;
		case 0x050: /* movmskps */
			rm = (modrm & 7) | REX_B(dc);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
				offsetof(CPUX86State, xmm_regs[rm]));
			gen_helper_movmskps(s, s->cpu_tmp2_i32, s->cpu_env, s->cpu_ptr0);
			tcg_gen_extu_i32_tl(s, s->cpu_regs[reg], s->cpu_tmp2_i32);
			break;
		case 0x150: /* movmskpd */
			rm = (modrm & 7) | REX_B(dc);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env,
				offsetof(CPUX86State, xmm_regs[rm]));
			gen_helper_movmskpd(s, s->cpu_tmp2_i32, s->cpu_env, s->cpu_ptr0);
			tcg_gen_extu_i32_tl(s, s->cpu_regs[reg], s->cpu_tmp2_i32);
			break;
		case 0x02a: /* cvtpi2ps */
		case 0x12a: /* cvtpi2pd */
			gen_helper_enter_mmx(s, s->cpu_env);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				op2_offset = offsetof(CPUX86State, mmx_t0);
				gen_ldq_env_A0(s, dc, op2_offset);
			}
			else {
				rm = (modrm & 7);
				op2_offset = offsetof(CPUX86State, fpregs[rm].mmx);
			}
			op1_offset = offsetof(CPUX86State, xmm_regs[reg]);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			switch (b >> 8) {
			case 0x0:
				gen_helper_cvtpi2ps(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			default:
			case 0x1:
				gen_helper_cvtpi2pd(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			}
			break;
		case 0x22a: /* cvtsi2ss */
		case 0x32a: /* cvtsi2sd */
			ot = mo_64_32(dc->dflag);
			gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
			op1_offset = offsetof(CPUX86State, xmm_regs[reg]);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			if (ot == MO_32) {
				SSEFunc_0_epi sse_fn_epi = sse_op_table3ai[(b >> 8) & 1];
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				sse_fn_epi(s, s->cpu_env, s->cpu_ptr0, s->cpu_tmp2_i32);
			}
			else {
#ifdef TARGET_X86_64
				SSEFunc_0_epl sse_fn_epl = sse_op_table3aq[(b >> 8) & 1];
				sse_fn_epl(s, s->cpu_env, s->cpu_ptr0, s->cpu_T0);
#else
				goto illegal_op;
#endif
			}
			break;
		case 0x02c: /* cvttps2pi */
		case 0x12c: /* cvttpd2pi */
		case 0x02d: /* cvtps2pi */
		case 0x12d: /* cvtpd2pi */
			gen_helper_enter_mmx(s, s->cpu_env);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				op2_offset = offsetof(CPUX86State, xmm_t0);
				gen_ldo_env_A0(s, dc, op2_offset);
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				op2_offset = offsetof(CPUX86State, xmm_regs[rm]);
			}
			op1_offset = offsetof(CPUX86State, fpregs[reg & 7].mmx);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			switch (b) {
			case 0x02c:
				gen_helper_cvttps2pi(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			case 0x12c:
				gen_helper_cvttpd2pi(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			case 0x02d:
				gen_helper_cvtps2pi(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			case 0x12d:
				gen_helper_cvtpd2pi(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
				break;
			}
			break;
		case 0x22c: /* cvttss2si */
		case 0x32c: /* cvttsd2si */
		case 0x22d: /* cvtss2si */
		case 0x32d: /* cvtsd2si */
			ot = mo_64_32(dc->dflag);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				if ((b >> 8) & 1) {
					gen_ldq_env_A0(s, dc, offsetof(CPUX86State, xmm_t0.ZMM_Q(0)));
				}
				else {
					gen_op_ld_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
					tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, xmm_t0.ZMM_L(0)));
				}
				op2_offset = offsetof(CPUX86State, xmm_t0);
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				op2_offset = offsetof(CPUX86State, xmm_regs[rm]);
			}
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op2_offset);
			if (ot == MO_32) {
				SSEFunc_i_ep sse_fn_i_ep =
					sse_op_table3bi[((b >> 7) & 2) | (b & 1)];
				sse_fn_i_ep(s, s->cpu_tmp2_i32, s->cpu_env, s->cpu_ptr0);
				tcg_gen_extu_i32_tl(s, s->cpu_T0, s->cpu_tmp2_i32);
			}
			else {
#ifdef TARGET_X86_64
				SSEFunc_l_ep sse_fn_l_ep = sse_op_table3bq[((b >> 7) & 2) | (b & 1)];
				sse_fn_l_ep(s, s->cpu_T0, s->cpu_env, s->cpu_ptr0);
#else
				goto illegal_op;
#endif
			}
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
			break;
		case 0xc4: /* pinsrw */
		case 0x1c4:
			dc->rip_offset = 1;
			gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
			val = cpu_ldub_code(env, dc->pc++);
			if (b1) {
				val &= 7;
				tcg_gen_st16_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[reg].ZMM_W(val)));
			}
			else {
				val &= 3;
				tcg_gen_st16_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, fpregs[reg].mmx.MMX_W(val)));
			}
			break;
		case 0xc5: /* pextrw */
		case 0x1c5:
			if (mod != 3)
				goto illegal_op;
			ot = mo_64_32(dc->dflag);
			val = cpu_ldub_code(env, dc->pc++);
			if (b1) {
				val &= 7;
				rm = (modrm & 7) | REX_B(dc);
				tcg_gen_ld16u_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, xmm_regs[rm].ZMM_W(val)));
			}
			else {
				val &= 3;
				rm = (modrm & 7);
				tcg_gen_ld16u_tl(s, s->cpu_T0, s->cpu_env,
					offsetof(CPUX86State, fpregs[rm].mmx.MMX_W(val)));
			}
			reg = ((modrm >> 3) & 7) | rex_r;
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
			break;
		case 0x1d6: /* movq ea, xmm */
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_stq_env_A0(s, dc, offsetof(CPUX86State,
					xmm_regs[reg].ZMM_Q(0)));
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_movq(s, offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)),
					offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)));
				gen_op_movq_env_0(s, offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(1)));
			}
			break;
		case 0x2d6: /* movq2dq */
			gen_helper_enter_mmx(s, s->cpu_env);
			rm = (modrm & 7);
			gen_op_movq(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(0)),
				offsetof(CPUX86State, fpregs[rm].mmx));
			gen_op_movq_env_0(s, offsetof(CPUX86State, xmm_regs[reg].ZMM_Q(1)));
			break;
		case 0x3d6: /* movdq2q */
			gen_helper_enter_mmx(s, s->cpu_env);
			rm = (modrm & 7) | REX_B(dc);
			gen_op_movq(s, offsetof(CPUX86State, fpregs[reg & 7].mmx),
				offsetof(CPUX86State, xmm_regs[rm].ZMM_Q(0)));
			break;
		case 0xd7: /* pmovmskb */
		case 0x1d7:
			if (mod != 3)
				goto illegal_op;
			if (b1) {
				rm = (modrm & 7) | REX_B(dc);
				tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, offsetof(CPUX86State, xmm_regs[rm]));
				gen_helper_pmovmskb_xmm(s, s->cpu_tmp2_i32, s->cpu_env, s->cpu_ptr0);
			}
			else {
				rm = (modrm & 7);
				tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, offsetof(CPUX86State, fpregs[rm].mmx));
				gen_helper_pmovmskb_mmx(s, s->cpu_tmp2_i32, s->cpu_env, s->cpu_ptr0);
			}
			reg = ((modrm >> 3) & 7) | rex_r;
			tcg_gen_extu_i32_tl(s, s->cpu_regs[reg], s->cpu_tmp2_i32);
			break;

		case 0x138:
		case 0x038:
			b = modrm;
			if ((b & 0xf0) == 0xf0) {
				goto do_0f_38_fx;
			}
			modrm = cpu_ldub_code(env, dc->pc++);
			rm = modrm & 7;
			reg = ((modrm >> 3) & 7) | rex_r;
			mod = (modrm >> 6) & 3;
			if (b1 >= 2) {
				goto unknown_op;
			}

			sse_fn_epp = sse_op_table6[b].op[b1];
			if (!sse_fn_epp) {
				goto unknown_op;
			}
			if (!(dc->cpuid_ext_features & sse_op_table6[b].ext_mask))
				goto illegal_op;

			if (b1) {
				op1_offset = offsetof(CPUX86State, xmm_regs[reg]);
				if (mod == 3) {
					op2_offset = offsetof(CPUX86State, xmm_regs[rm | REX_B(dc)]);
				}
				else {
					op2_offset = offsetof(CPUX86State, xmm_t0);
					gen_lea_modrm(s, env, dc, modrm);
					switch (b) {
					case 0x20: case 0x30: /* pmovsxbw, pmovzxbw */
					case 0x23: case 0x33: /* pmovsxwd, pmovzxwd */
					case 0x25: case 0x35: /* pmovsxdq, pmovzxdq */
						gen_ldq_env_A0(s, dc, op2_offset +
							offsetof(ZMMReg, ZMM_Q(0)));
						break;
					case 0x21: case 0x31: /* pmovsxbd, pmovzxbd */
					case 0x24: case 0x34: /* pmovsxwq, pmovzxwq */
						tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env, op2_offset +
							offsetof(ZMMReg, ZMM_L(0)));
						break;
					case 0x22: case 0x32: /* pmovsxbq, pmovzxbq */
						tcg_gen_qemu_ld_tl(s, s->cpu_tmp0, s->cpu_A0,
							dc->mem_index, MO_LEUW);
						tcg_gen_st16_tl(s, s->cpu_tmp0, s->cpu_env, op2_offset +
							offsetof(ZMMReg, ZMM_W(0)));
						break;
					case 0x2a:            /* movntqda */
						gen_ldo_env_A0(s, dc, op1_offset);
						return;
					default:
						gen_ldo_env_A0(s, dc, op2_offset);
					}
				}
			}
			else {
				op1_offset = offsetof(CPUX86State, fpregs[reg].mmx);
				if (mod == 3) {
					op2_offset = offsetof(CPUX86State, fpregs[rm].mmx);
				}
				else {
					op2_offset = offsetof(CPUX86State, mmx_t0);
					gen_lea_modrm(s, env, dc, modrm);
					gen_ldq_env_A0(s, dc, op2_offset);
				}
			}
			if (sse_fn_epp == (SSEFunc_0_epp)SSE_SPECIAL) {
				goto unknown_op;
			}

			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			sse_fn_epp(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);

			if (b == 0x17) {
				set_cc_op(s, dc, CC_OP_EFLAGS);
			}
			break;

		case 0x238:
		case 0x338:
		do_0f_38_fx:
			/* Various integer extensions at 0f 38 f[0-f].  */
			b = modrm | (b1 << 8);
			modrm = cpu_ldub_code(env, dc->pc++);
			reg = ((modrm >> 3) & 7) | rex_r;

			switch (b) {
			case 0x3f0: /* crc32 Gd,Eb */
			case 0x3f1: /* crc32 Gd,Ey */
			do_crc32:
				if (!(dc->cpuid_ext_features & CPUID_EXT_SSE42)) {
					goto illegal_op;
				}
				if ((b & 0xff) == 0xf0) {
					ot = MO_8;
				}
				else if (dc->dflag != MO_64) {
					ot = (dc->prefix & PREFIX_DATA ? MO_16 : MO_32);
				}
				else {
					ot = MO_64;
				}

				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[reg]);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				gen_helper_crc32(s, s->cpu_T0, s->cpu_tmp2_i32,
					s->cpu_T0, tcg_const_i32(s, 8 << ot));

				ot = mo_64_32(dc->dflag);
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				break;

			case 0x1f0: /* crc32 or movbe */
			case 0x1f1:
				/* For these insns, the f3 prefix is supposed to have priority
				   over the 66 prefix, but that's not what we implement above
				   setting b1.  */
				if (dc->prefix & PREFIX_REPNZ) {
					goto do_crc32;
				}
				/* FALLTHRU */
			case 0x0f0: /* movbe Gy,My */
			case 0x0f1: /* movbe My,Gy */
				if (!(dc->cpuid_ext_features & CPUID_EXT_MOVBE)) {
					goto illegal_op;
				}
				if (dc->dflag != MO_64) {
					ot = (dc->prefix & PREFIX_DATA ? MO_16 : MO_32);
				}
				else {
					ot = MO_64;
				}

				gen_lea_modrm(s, env, dc, modrm);
				if ((b & 1) == 0) {
					tcg_gen_qemu_ld_tl(s, s->cpu_T0, s->cpu_A0,
						dc->mem_index, (TCGMemOp)(ot | MO_BE));
					gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				}
				else {
					tcg_gen_qemu_st_tl(s, s->cpu_regs[reg], s->cpu_A0,
						dc->mem_index, (TCGMemOp)(ot | MO_BE));
				}
				break;

			case 0x0f2: /* andn Gy, By, Ey */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI1)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				tcg_gen_andc_tl(s, s->cpu_T0, s->cpu_regs[dc->vex_v], s->cpu_T0);
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				gen_op_update1_cc(s);
				set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
				break;

			case 0x0f7: /* bextr Gy, Ey, By */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI1)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				{
					TCGv bound, zero;

					gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
					/* Extract START, and shift the operand.
					   Shifts larger than operand size get zeros.  */
					tcg_gen_ext8u_tl(s, s->cpu_A0, s->cpu_regs[dc->vex_v]);
					tcg_gen_shr_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_A0);

					bound = tcg_const_tl(s, ot == MO_64 ? 63 : 31);
					zero = tcg_const_tl(s, 0);
					tcg_gen_movcond_tl(s, TCG_COND_LEU, s->cpu_T0, s->cpu_A0, bound,
						s->cpu_T0, zero);
					tcg_temp_free(s, zero);

					/* Extract the LEN into a mask.  Lengths larger than
					   operand size get all ones.  */
					tcg_gen_extract_tl(s, s->cpu_A0, s->cpu_regs[dc->vex_v], 8, 8);
					tcg_gen_movcond_tl(s, TCG_COND_LEU, s->cpu_A0, s->cpu_A0, bound,
						s->cpu_A0, bound);
					tcg_temp_free(s, bound);
					tcg_gen_movi_tl(s, s->cpu_T1, 1);
					tcg_gen_shl_tl(s, s->cpu_T1, s->cpu_T1, s->cpu_A0);
					tcg_gen_subi_tl(s, s->cpu_T1, s->cpu_T1, 1);
					tcg_gen_and_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);

					gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
					gen_op_update1_cc(s);
					set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
				}
				break;

			case 0x0f5: /* bzhi Gy, Ey, By */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				tcg_gen_ext8u_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v]);
				{
					TCGv bound = tcg_const_tl(s, ot == MO_64 ? 63 : 31);
					/* Note that since we're using BMILG (in order to get O
					   cleared) we need to store the inverse into C.  */
					tcg_gen_setcond_tl(s, TCG_COND_LT, s->cpu_cc_src,
						s->cpu_T1, bound);
					tcg_gen_movcond_tl(s, TCG_COND_GT, s->cpu_T1, s->cpu_T1,
						bound, bound, s->cpu_T1);
					tcg_temp_free(s, bound);
				}
				tcg_gen_movi_tl(s, s->cpu_A0, -1);
				tcg_gen_shl_tl(s, s->cpu_A0, s->cpu_A0, s->cpu_T1);
				tcg_gen_andc_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_A0);
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				gen_op_update1_cc(s);
				set_cc_op(s, dc, (CCOp)(CC_OP_BMILGB + ot));
				break;

			case 0x3f6: /* mulx By, Gy, rdx, Ey */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				switch (ot) {
				default:
					tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
					tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_regs[R_EDX]);
					tcg_gen_mulu2_i32(s, s->cpu_tmp2_i32, s->cpu_tmp3_i32,
						s->cpu_tmp2_i32, s->cpu_tmp3_i32);
					tcg_gen_extu_i32_tl(s, s->cpu_regs[dc->vex_v], s->cpu_tmp2_i32);
					tcg_gen_extu_i32_tl(s, s->cpu_regs[reg], s->cpu_tmp3_i32);
					break;
#ifdef TARGET_X86_64
				case MO_64:
					tcg_gen_mulu2_i64(s, s->cpu_T0, s->cpu_T1,
						s->cpu_T0, s->cpu_regs[R_EDX]);
					tcg_gen_mov_i64(s, s->cpu_regs[dc->vex_v], s->cpu_T0);
					tcg_gen_mov_i64(s, s->cpu_regs[reg], s->cpu_T1);
					break;
#endif
				}
				break;

			case 0x3f5: /* pdep Gy, By, Ey */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				/* Note that by zero-extending the mask operand, we
				   automatically handle zero-extending the result.  */
				if (ot == MO_64) {
					tcg_gen_mov_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v]);
				}
				else {
					tcg_gen_ext32u_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v]);
				}
				gen_helper_pdep(s, s->cpu_regs[reg], s->cpu_T0, s->cpu_T1);
				break;

			case 0x2f5: /* pext Gy, By, Ey */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				/* Note that by zero-extending the mask operand, we
				   automatically handle zero-extending the result.  */
				if (ot == MO_64) {
					tcg_gen_mov_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v]);
				}
				else {
					tcg_gen_ext32u_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v]);
				}
				gen_helper_pext(s, s->cpu_regs[reg], s->cpu_T0, s->cpu_T1);
				break;

			case 0x1f6: /* adcx Gy, Ey */
			case 0x2f6: /* adox Gy, Ey */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_ADX)) {
					goto illegal_op;
				}
				else {
					TCGv carry_in, carry_out, zero;
					int end_op;

					ot = mo_64_32(dc->dflag);
					gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);

					/* Re-use the carry-out from a previous round.  */
					TCGV_UNUSED(carry_in);
					carry_out = (b == 0x1f6 ? s->cpu_cc_dst : s->cpu_cc_src2);
					switch (dc->cc_op) {
					case CC_OP_ADCX:
						if (b == 0x1f6) {
							carry_in = s->cpu_cc_dst;
							end_op = CC_OP_ADCX;
						}
						else {
							end_op = CC_OP_ADCOX;
						}
						break;
					case CC_OP_ADOX:
						if (b == 0x1f6) {
							end_op = CC_OP_ADCOX;
						}
						else {
							carry_in = s->cpu_cc_src2;
							end_op = CC_OP_ADOX;
						}
						break;
					case CC_OP_ADCOX:
						end_op = CC_OP_ADCOX;
						carry_in = carry_out;
						break;
					default:
						end_op = (b == 0x1f6 ? CC_OP_ADCX : CC_OP_ADOX);
						break;
					}
					/* If we can't reuse carry-out, get it out of EFLAGS.  */
					if (TCGV_IS_UNUSED(carry_in)) {
						if (b == 0x1f6)
						{
							tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
							tcg_gen_or_tl(s, carry_in, carry_in, s->cpu_eflags_c);
						}
						else
						{
							tcg_gen_ld_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));
							tcg_gen_or_tl(s, carry_in, carry_in, s->cpu_eflags_o);
						}
					}

					switch (ot) {
#ifdef TARGET_X86_64
					case MO_32:
						/* If we know TL is 64-bit, and we want a 32-bit
						   result, just do everything in 64-bit arithmetic.  */
						tcg_gen_ext32u_i64(s, s->cpu_regs[reg], s->cpu_regs[reg]);
						tcg_gen_ext32u_i64(s, s->cpu_T0, s->cpu_T0);
						tcg_gen_add_i64(s, s->cpu_T0, s->cpu_T0, s->cpu_regs[reg]);
						tcg_gen_add_i64(s, s->cpu_T0, s->cpu_T0, carry_in);
						tcg_gen_ext32u_i64(s, s->cpu_regs[reg], s->cpu_T0);
						tcg_gen_shri_i64(s, carry_out, s->cpu_T0, 32);
						break;
#endif
					default:
						/* Otherwise compute the carry-out in two steps.  */
						zero = tcg_const_tl(s, 0);
						tcg_gen_add2_tl(s, s->cpu_T0, carry_out,
							s->cpu_T0, zero,
							carry_in, zero);
						tcg_gen_add2_tl(s, s->cpu_regs[reg], carry_out,
							s->cpu_regs[reg], carry_out,
							s->cpu_T0, zero);
						tcg_temp_free(s, zero);
						break;
					}
					set_cc_op(s, dc, (CCOp)end_op);
				}
				break;

			case 0x1f7: /* shlx Gy, Ey, By */
			case 0x2f7: /* sarx Gy, Ey, By */
			case 0x3f7: /* shrx Gy, Ey, By */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				if (ot == MO_64) {
					tcg_gen_andi_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v], 63);
				}
				else {
					tcg_gen_andi_tl(s, s->cpu_T1, s->cpu_regs[dc->vex_v], 31);
				}
				if (b == 0x1f7) {
					tcg_gen_shl_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				}
				else if (b == 0x2f7) {
					if (ot != MO_64) {
						tcg_gen_ext32s_tl(s, s->cpu_T0, s->cpu_T0);
					}
					tcg_gen_sar_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				}
				else {
					if (ot != MO_64) {
						tcg_gen_ext32u_tl(s, s->cpu_T0, s->cpu_T0);
					}
					tcg_gen_shr_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				}
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				break;

			case 0x0f3:
			case 0x1f3:
			case 0x2f3:
			case 0x3f3: /* Group 17 */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI1)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);

				switch (reg & 7) {
				case 1: /* blsr By,Ey */
					tcg_gen_neg_tl(s, s->cpu_T1, s->cpu_T0);
					tcg_gen_and_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
					gen_op_mov_reg_v(s, ot, dc->vex_v, s->cpu_T0);
					gen_op_update2_cc(s);
					set_cc_op(s, dc, (CCOp)(CC_OP_BMILGB + ot));
					break;

				case 2: /* blsmsk By,Ey */
					tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T0);
					tcg_gen_subi_tl(s, s->cpu_T0, s->cpu_T0, 1);
					tcg_gen_xor_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_cc_src);
					tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
					set_cc_op(s, dc, (CCOp)(CC_OP_BMILGB + ot));
					break;

				case 3: /* blsi By, Ey */
					tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T0);
					tcg_gen_subi_tl(s, s->cpu_T0, s->cpu_T0, 1);
					tcg_gen_and_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_cc_src);
					tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
					set_cc_op(s, dc, (CCOp)(CC_OP_BMILGB + ot));
					break;

				default:
					goto unknown_op;
				}
				break;

			default:
				goto unknown_op;
			}
			break;

		case 0x03a:
		case 0x13a:
			b = modrm;
			modrm = cpu_ldub_code(env, dc->pc++);
			rm = modrm & 7;
			reg = ((modrm >> 3) & 7) | rex_r;
			mod = (modrm >> 6) & 3;
			if (b1 >= 2) {
				goto unknown_op;
			}

			sse_fn_eppi = sse_op_table7[b].op[b1];
			if (!sse_fn_eppi) {
				goto unknown_op;
			}
			if (!(dc->cpuid_ext_features & sse_op_table7[b].ext_mask))
				goto illegal_op;

			if (sse_fn_eppi == SSE_SPECIAL) {
				ot = mo_64_32(dc->dflag);
				rm = (modrm & 7) | REX_B(dc);
				dc->rip_offset = 1;
				if (mod != 3)
					gen_lea_modrm(s, env, dc, modrm);
				reg = ((modrm >> 3) & 7) | rex_r;
				val = cpu_ldub_code(env, dc->pc++);
				switch (b) {
				case 0x14: /* pextrb */
					tcg_gen_ld8u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State,
						xmm_regs[reg].ZMM_B(val & 15)));
					if (mod == 3) {
						gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
					}
					else {
						tcg_gen_qemu_st_tl(s, s->cpu_T0, s->cpu_A0,
							dc->mem_index, MO_UB);
					}
					break;
				case 0x15: /* pextrw */
					tcg_gen_ld16u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State,
						xmm_regs[reg].ZMM_W(val & 7)));
					if (mod == 3) {
						gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
					}
					else {
						tcg_gen_qemu_st_tl(s, s->cpu_T0, s->cpu_A0,
							dc->mem_index, MO_LEUW);
					}
					break;
				case 0x16:
					if (ot == MO_32) { /* pextrd */
						tcg_gen_ld_i32(s, s->cpu_tmp2_i32, s->cpu_env,
							offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(val & 3)));
						if (mod == 3) {
							tcg_gen_extu_i32_tl(s, s->cpu_regs[rm], s->cpu_tmp2_i32);
						}
						else {
							tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
								dc->mem_index, MO_LEUL);
						}
					}
					else { /* pextrq */
#ifdef TARGET_X86_64
						tcg_gen_ld_i64(s, s->cpu_tmp1_i64, s->cpu_env,
							offsetof(CPUX86State,
								xmm_regs[reg].ZMM_Q(val & 1)));
						if (mod == 3) {
							tcg_gen_mov_i64(s, s->cpu_regs[rm], s->cpu_tmp1_i64);
						}
						else {
							tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
								dc->mem_index, MO_LEQ);
						}
#else
						goto illegal_op;
#endif
					}
					break;
				case 0x17: /* extractps */
					tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State,
						xmm_regs[reg].ZMM_L(val & 3)));
					if (mod == 3) {
						gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
					}
					else {
						tcg_gen_qemu_st_tl(s, s->cpu_T0, s->cpu_A0,
							dc->mem_index, MO_LEUL);
					}
					break;
				case 0x20: /* pinsrb */
					if (mod == 3) {
						gen_op_mov_v_reg(s, MO_32, s->cpu_T0, rm);
					}
					else {
						tcg_gen_qemu_ld_tl(s, s->cpu_T0, s->cpu_A0,
							dc->mem_index, MO_UB);
					}
					tcg_gen_st8_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State,
						xmm_regs[reg].ZMM_B(val & 15)));
					break;
				case 0x21: /* insertps */
					if (mod == 3) {
						tcg_gen_ld_i32(s, s->cpu_tmp2_i32, s->cpu_env,
							offsetof(CPUX86State, xmm_regs[rm]
								.ZMM_L((val >> 6) & 3)));
					}
					else {
						tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
					}
					tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env,
						offsetof(CPUX86State, xmm_regs[reg]
							.ZMM_L((val >> 4) & 3)));
					if ((val >> 0) & 1)
						tcg_gen_st_i32(s, tcg_const_i32(s, 0 /*float32_zero*/),
							s->cpu_env, offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(0)));
					if ((val >> 1) & 1)
						tcg_gen_st_i32(s, tcg_const_i32(s, 0 /*float32_zero*/),
							s->cpu_env, offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(1)));
					if ((val >> 2) & 1)
						tcg_gen_st_i32(s, tcg_const_i32(s, 0 /*float32_zero*/),
							s->cpu_env, offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(2)));
					if ((val >> 3) & 1)
						tcg_gen_st_i32(s, tcg_const_i32(s, 0 /*float32_zero*/),
							s->cpu_env, offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(3)));
					break;
				case 0x22:
					if (ot == MO_32) { /* pinsrd */
						if (mod == 3) {
							tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[rm]);
						}
						else {
							tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
								dc->mem_index, MO_LEUL);
						}
						tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env,
							offsetof(CPUX86State,
								xmm_regs[reg].ZMM_L(val & 3)));
					}
					else { /* pinsrq */
#ifdef TARGET_X86_64
						if (mod == 3) {
							gen_op_mov_v_reg(s, ot, s->cpu_tmp1_i64, rm);
						}
						else {
							tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
								dc->mem_index, MO_LEQ);
						}
						tcg_gen_st_i64(s, s->cpu_tmp1_i64, s->cpu_env,
							offsetof(CPUX86State,
								xmm_regs[reg].ZMM_Q(val & 1)));
#else
						goto illegal_op;
#endif
					}
					break;
				}
				return;
			}

			if (b1) {
				op1_offset = offsetof(CPUX86State, xmm_regs[reg]);
				if (mod == 3) {
					op2_offset = offsetof(CPUX86State, xmm_regs[rm | REX_B(dc)]);
				}
				else {
					op2_offset = offsetof(CPUX86State, xmm_t0);
					gen_lea_modrm(s, env, dc, modrm);
					gen_ldo_env_A0(s, dc, op2_offset);
				}
			}
			else {
				op1_offset = offsetof(CPUX86State, fpregs[reg].mmx);
				if (mod == 3) {
					op2_offset = offsetof(CPUX86State, fpregs[rm].mmx);
				}
				else {
					op2_offset = offsetof(CPUX86State, mmx_t0);
					gen_lea_modrm(s, env, dc, modrm);
					gen_ldq_env_A0(s, dc, op2_offset);
				}
			}
			val = cpu_ldub_code(env, dc->pc++);

			if ((b & 0xfc) == 0x60) { /* pcmpXstrX */
				set_cc_op(s, dc, CC_OP_EFLAGS);

				if (dc->dflag == MO_64) {
					/* The helper must use entire 64-bit gp registers */
					val |= 1 << 8;
				}
			}

			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			sse_fn_eppi(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1, tcg_const_i32(s, val));
			break;

		case 0x33a:
			/* Various integer extensions at 0f 3a f[0-f].  */
			b = modrm | (b1 << 8);
			modrm = cpu_ldub_code(env, dc->pc++);
			reg = ((modrm >> 3) & 7) | rex_r;

			switch (b) {
			case 0x3f0: /* rorx Gy,Ey, Ib */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI2)
					|| !(dc->prefix & PREFIX_VEX)
					|| dc->vex_l != 0) {
					goto illegal_op;
				}
				ot = mo_64_32(dc->dflag);
				gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
				b = cpu_ldub_code(env, dc->pc++);
				if (ot == MO_64) {
					tcg_gen_rotri_tl(s, s->cpu_T0, s->cpu_T0, b & 63);
				}
				else {
					tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
					tcg_gen_rotri_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, b & 31);
					tcg_gen_extu_i32_tl(s, s->cpu_T0, s->cpu_tmp2_i32);
				}
				gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
				break;

			default:
				goto unknown_op;
			}
			break;

		default:
		unknown_op:
			gen_unknown_opcode(s, env, dc);
			return;
		}
	}
	else {
		/* generic MMX or SSE operation */
		switch (b) {
		case 0x70: /* pshufx insn */
		case 0xc6: /* pshufx insn */
		case 0xc2: /* compare insns */
			dc->rip_offset = 1;
			break;
		default:
			break;
		}
		if (is_xmm) {
			op1_offset = offsetof(CPUX86State, xmm_regs[reg]);
			if (mod != 3) {
				int sz = 4;

				gen_lea_modrm(s, env, dc, modrm);
				op2_offset = offsetof(CPUX86State, xmm_t0);

				switch (b) {
				case 0x50:case 0x51:case 0x52:case 0x53:case 0x54:case 0x55:case 0x56:case 0x57:case 0x58:case 0x59:case 0x5a:
				case 0x5c:case 0x5d:case 0x5e:case 0x5f:
				case 0xc2:
					/* Most sse scalar operations.  */
					if (b1 == 2) {
						sz = 2;
					}
					else if (b1 == 3) {
						sz = 3;
					}
					break;

				case 0x2e:  /* ucomis[sd] */
				case 0x2f:  /* comis[sd] */
					if (b1 == 0) {
						sz = 2;
					}
					else {
						sz = 3;
					}
					break;
				}

				switch (sz) {
				case 2:
					/* 32 bit access */
					gen_op_ld_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
					tcg_gen_st32_tl(s, s->cpu_T0, s->cpu_env,
						offsetof(CPUX86State, xmm_t0.ZMM_L(0)));
					break;
				case 3:
					/* 64 bit access */
					gen_ldq_env_A0(s, dc, offsetof(CPUX86State, xmm_t0.ZMM_D(0)));
					break;
				default:
					/* 128 bit access */
					gen_ldo_env_A0(s, dc, op2_offset);
					break;
				}
			}
			else {
				rm = (modrm & 7) | REX_B(dc);
				op2_offset = offsetof(CPUX86State, xmm_regs[rm]);
			}
		}
		else {
			op1_offset = offsetof(CPUX86State, fpregs[reg].mmx);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				op2_offset = offsetof(CPUX86State, mmx_t0);
				gen_ldq_env_A0(s, dc, op2_offset);
			}
			else {
				rm = (modrm & 7);
				op2_offset = offsetof(CPUX86State, fpregs[rm].mmx);
			}
		}
		switch (b) {
		case 0x0f: /* 3DNow! data insns */
			val = cpu_ldub_code(env, dc->pc++);
			sse_fn_epp = sse_op_table5[val];
			if (!sse_fn_epp) {
				goto unknown_op;
			}
			if (!(dc->cpuid_ext2_features & CPUID_EXT2_3DNOW)) {
				goto illegal_op;
			}
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			sse_fn_epp(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
			break;
		case 0x70: /* pshufx insn */
		case 0xc6: /* pshufx insn */
			val = cpu_ldub_code(env, dc->pc++);
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			/* XXX: introduce a new table? */
			sse_fn_ppi = (SSEFunc_0_ppi)sse_fn_epp;
			sse_fn_ppi(s, s->cpu_ptr0, s->cpu_ptr1, tcg_const_i32(s, val));
			break;
		case 0xc2:
			/* compare insns */
			val = cpu_ldub_code(env, dc->pc++);
			if (val >= 8)
				goto unknown_op;
			sse_fn_epp = sse_op_table4[val][b1];

			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			sse_fn_epp(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
			break;
		case 0xf7:
			/* maskmov : we must prepare A0 */
			if (mod != 3)
				goto illegal_op;
			tcg_gen_mov_tl(s, s->cpu_A0, s->cpu_regs[R_EDI]);
			gen_extu(s, dc->aflag, s->cpu_A0);
			gen_add_A0_ds_seg(s, dc);

			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			/* XXX: introduce a new table? */
			sse_fn_eppt = (SSEFunc_0_eppt)sse_fn_epp;
			sse_fn_eppt(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1, s->cpu_A0);
			break;
		default:
			tcg_gen_addi_ptr(s->cpu_ptr0, s->cpu_env, op1_offset);
			tcg_gen_addi_ptr(s->cpu_ptr1, s->cpu_env, op2_offset);
			sse_fn_epp(s, s->cpu_env, s->cpu_ptr0, s->cpu_ptr1);
			break;
		}
		if (b == 0x2e || b == 0x2f) {
			set_cc_op(s, dc, CC_OP_EFLAGS);
		}
	}
}

/* convert one instruction. dc->is_jmp is set if the translation must
   be stopped. Return the next pc value */
static target_ulong disas_insn(TCGContext * s, CPUX86State *env, DisasContext *dc,
	target_ulong pc_start)
{
	int b, prefixes;
	int shift;
	TCGMemOp ot, aflag, dflag;
	int modrm, reg, rm, mod, op, opreg, val;
	target_ulong next_eip, tval;
	int rex_w, rex_r;

	dc->pc_start = dc->pc = pc_start;
	prefixes = 0;
	dc->override = -1;
	rex_w = -1;
	rex_r = 0;
#ifdef TARGET_X86_64
	dc->rex_x = 0;
	dc->rex_b = 0;
	s->x86_64_hregs = 0;
#endif
	dc->rip_offset = 0; /* for relative ip address */
	dc->vex_l = 0;
	dc->vex_v = 0;
next_byte:
	/* x86 has an upper limit of 15 bytes for an instruction. Since we
	 * do not want to decode and generate IR for an illegal
	 * instruction, the following check limits the instruction size to
	 * 25 bytes: 14 prefix + 1 opc + 6 (modrm+sib+ofs) + 4 imm */
	if (dc->pc - pc_start > 14) {
		goto illegal_op;
	}
	b = cpu_ldub_code(env, dc->pc);
	dc->pc++;
	/* Collect prefixes.  */
	switch (b) {
	case 0xf3:
		prefixes |= PREFIX_REPZ;
		goto next_byte;
	case 0xf2:
		prefixes |= PREFIX_REPNZ;
		goto next_byte;
	case 0xf0:
		prefixes |= PREFIX_LOCK;
		goto next_byte;
	case 0x2e:
		dc->override = R_CS;
		goto next_byte;
	case 0x36:
		dc->override = R_SS;
		goto next_byte;
	case 0x3e:
		dc->override = R_DS;
		goto next_byte;
	case 0x26:
		dc->override = R_ES;
		goto next_byte;
	case 0x64:
		dc->override = R_FS;
		goto next_byte;
	case 0x65:
		dc->override = R_GS;
		goto next_byte;
	case 0x66:
		prefixes |= PREFIX_DATA;
		goto next_byte;
	case 0x67:
		prefixes |= PREFIX_ADR;
		goto next_byte;
#ifdef TARGET_X86_64
	case 0x40:case 0x41:case 0x42:case 0x43:case 0x44:case 0x45:case 0x46:case 0x47:case 0x48:case 0x49:case 0x4a:case 0x4b:case 0x4c:case 0x4d:case 0x4e:case 0x4f:
		if (CODE64(dc)) {
			/* REX prefix */
			rex_w = (b >> 3) & 1;
			rex_r = (b & 0x4) << 1;
			dc->rex_x = (b & 0x2) << 2;
			REX_B(dc) = (b & 0x1) << 3;
			s->x86_64_hregs = 1; /* select uniform byte register addressing */
			goto next_byte;
		}
		break;
#endif
	case 0xc5: /* 2-byte VEX */
	case 0xc4: /* 3-byte VEX */
		/* VEX prefixes cannot be used except in 32-bit mode.
		   Otherwise the instruction is LES or LDS.  */
		if (dc->code32 && !dc->vm86) {
			static const int pp_prefix[4] = {
				0, PREFIX_DATA, PREFIX_REPZ, PREFIX_REPNZ
			};
			int vex3, vex2 = cpu_ldub_code(env, dc->pc);

			if (!CODE64(dc) && (vex2 & 0xc0) != 0xc0) {
				/* 4.1.4.6: In 32-bit mode, bits [7:6] must be 11b,
				   otherwise the instruction is LES or LDS.  */
				break;
			}
			dc->pc++;

			/* 4.1.1-4.1.3: No preceding lock, 66, f2, f3, or rex prefixes. */
			if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ
				| PREFIX_LOCK | PREFIX_DATA)) {
				goto illegal_op;
			}
#ifdef TARGET_X86_64
			if (s->x86_64_hregs) {
				goto illegal_op;
			}
#endif
			rex_r = (~vex2 >> 4) & 8;
			if (b == 0xc5) {
				vex3 = vex2;
				b = cpu_ldub_code(env, dc->pc++) | 0x100;
			}
			else {
#ifdef TARGET_X86_64
				dc->rex_x = (~vex2 >> 3) & 8;
				dc->rex_b = (~vex2 >> 2) & 8;
#endif
				vex3 = cpu_ldub_code(env, dc->pc++);
				rex_w = (vex3 >> 7) & 1;
				switch (vex2 & 0x1f) {
				case 0x01: /* Implied 0f leading opcode bytes.  */
					b = cpu_ldub_code(env, dc->pc++) | 0x100;
					break;
				case 0x02: /* Implied 0f 38 leading opcode bytes.  */
					b = 0x138;
					break;
				case 0x03: /* Implied 0f 3a leading opcode bytes.  */
					b = 0x13a;
					break;
				default:   /* Reserved for future use.  */
					goto unknown_op;
				}
			}
			dc->vex_v = (~vex3 >> 3) & 0xf;
			dc->vex_l = (vex3 >> 2) & 1;
			prefixes |= pp_prefix[vex3 & 3] | PREFIX_VEX;
		}
		break;
	}

	/* Post-process prefixes.  */
	if (CODE64(dc)) {
		/* In 64-bit mode, the default data size is 32-bit.  Select 64-bit
		   data with rex_w, and 16-bit data with 0x66; rex_w takes precedence
		   over 0x66 if both are present.  */
		dflag = (rex_w > 0 ? MO_64 : prefixes & PREFIX_DATA ? MO_16 : MO_32);
		/* In 64-bit mode, 0x67 selects 32-bit addressing.  */
		aflag = (prefixes & PREFIX_ADR ? MO_32 : MO_64);
	}
	else {
		/* In 16/32-bit mode, 0x66 selects the opposite data size.  */
		if (dc->code32 ^ ((prefixes & PREFIX_DATA) != 0)) {
			dflag = MO_32;
		}
		else {
			dflag = MO_16;
		}
		/* In 16/32-bit mode, 0x67 selects the opposite addressing.  */
		if (dc->code32 ^ ((prefixes & PREFIX_ADR) != 0)) {
			aflag = MO_32;
		}
		else {
			aflag = MO_16;
		}
	}

	dc->prefix = prefixes;
	dc->aflag = aflag;
	dc->dflag = dflag;

	/* now check op code */
reswitch:
	switch (b) {
	case 0x0f:
		/**************************/
		/* extended op code */
		b = cpu_ldub_code(env, dc->pc++) | 0x100;
		goto reswitch;

		/**************************/
		/* arith & logic */
	case 0x00:case 0x01:case 0x02:case 0x03:case 0x04:case 0x05:
	case 0x08:case 0x09:case 0x0a:case 0x0b:case 0x0c:case 0x0d:
	case 0x10:case 0x11:case 0x12:case 0x13:case 0x14:case 0x15:
	case 0x18:case 0x19:case 0x1a:case 0x1b:case 0x1c:case 0x1d:
	case 0x20:case 0x21:case 0x22:case 0x23:case 0x24:case 0x25:
	case 0x28:case 0x29:case 0x2a:case 0x2b:case 0x2c:case 0x2d:
	case 0x30:case 0x31:case 0x32:case 0x33:case 0x34:case 0x35:
	case 0x38:case 0x39:case 0x3a:case 0x3b:case 0x3c:case 0x3d:
	{
		int op, f, val;
		op = (b >> 3) & 7;
		f = (b >> 1) & 3;

		ot = mo_b_d(b, dflag);

		switch (f) {
		case 0: /* OP Ev, Gv */
			modrm = cpu_ldub_code(env, dc->pc++);
			reg = ((modrm >> 3) & 7) | rex_r;
			mod = (modrm >> 6) & 3;
			rm = (modrm & 7) | REX_B(dc);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				opreg = OR_TMP0;
			}
			else 
			{
				if (rm == reg) {
					if (op == OP_XORL || op == OP_SUBL) {
					xor_sub_zero:
						// 'sub reg, reg' -> 'mov reg, 0' optimisation
						// 'xor reg, reg' -> 'mov reg, 0' optimisation
						set_cc_op(s, dc, CC_OP_CLR);
						tcg_gen_movi_tl(s, s->cpu_T0, 0);
						gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
						break;
					}
				}
				opreg = rm;
			}
			gen_op_mov_v_reg(s, ot, s->cpu_T1, reg);
			gen_op(s, dc, op, ot, opreg);
			break;
		case 1: /* OP Gv, Ev */
			modrm = cpu_ldub_code(env, dc->pc++);
			mod = (modrm >> 6) & 3;
			reg = ((modrm >> 3) & 7) | rex_r;
			rm = (modrm & 7) | REX_B(dc);
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
			}
			else {
				if (rm == reg) {
					if(op == OP_XORL || op == OP_SUBL)
						goto xor_sub_zero;
				}
				gen_op_mov_v_reg(s, ot, s->cpu_T1, rm);
			}
			gen_op(s, dc, op, ot, reg);
			break;
		case 2: /* OP A, Iv */
			val = insn_get(s, env, dc, ot);
			tcg_gen_movi_tl(s, s->cpu_T1, val);
			gen_op(s, dc, op, ot, OR_EAX);
			break;
		}
	}
	break;

	case 0x82:
		if (CODE64(dc))
			goto illegal_op;
	case 0x80: /* GRP1 */
	case 0x81:
	case 0x83:
	{
		int val;

		ot = mo_b_d(b, dflag);

		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		op = (modrm >> 3) & 7;

		if (mod != 3) {
			if (b == 0x83)
				dc->rip_offset = 1;
			else
				dc->rip_offset = insn_const_size(ot);
			gen_lea_modrm(s, env, dc, modrm);
			opreg = OR_TMP0;
		}
		else {
			opreg = rm;
		}

		switch (b) {
		default:
		case 0x80:
		case 0x81:
		case 0x82:
			val = insn_get(s, env, dc, ot);
			break;
		case 0x83:
			val = (int8_t)insn_get(s, env, dc, MO_8);
			break;
		}

		if (opreg != OR_TMP0 && !(dc->prefix & PREFIX_LOCK))
		{
			if (val == -1)
			{
				if (op == OP_ORL)
				{
					// 'or reg, -1' -> 'mov reg, -1' optimisation
					tcg_gen_movi_tl(s, s->cpu_T0, val);
					gen_op_st_rm_T0_A0(s, dc, ot, opreg);
					gen_op_update1_cc(s);
					set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
					break;
				}
			}
			else if (val == 0)
			{
				if (op == OP_ANDL)
				{
					// 'and reg, 0' -> 'mov reg, 0' optimisation
					set_cc_op(s, dc, CC_OP_CLR);
					tcg_gen_movi_tl(s, s->cpu_T0, 0);
					gen_op_mov_reg_v(s, ot, opreg, s->cpu_T0);
					break;
				}
			}
		}

		tcg_gen_movi_tl(s, s->cpu_T1, val);
		gen_op(s, dc, op, ot, opreg);
	}
	break;

	/**************************/
	/* inc, dec, and other misc arith */
	case 0x40:case 0x41:case 0x42:case 0x43:case 0x44:case 0x45:case 0x46:case 0x47: /* inc Gv */
		ot = dflag;
		gen_inc(s, dc, ot, OR_EAX + (b & 7), 1);
		break;
	case 0x48:case 0x49:case 0x4a:case 0x4b:case 0x4c:case 0x4d:case 0x4e:case 0x4f: /* dec Gv */
		ot = dflag;
		gen_inc(s, dc, ot, OR_EAX + (b & 7), -1);
		break;
	case 0xf6: /* GRP3 */
	case 0xf7:
		ot = mo_b_d(b, dflag);

		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		op = (modrm >> 3) & 7;
		if (mod != 3) {
			if (op == 0) {
				dc->rip_offset = insn_const_size(ot);
			}
			gen_lea_modrm(s, env, dc, modrm);
			/* For those below that handle locked memory, don't load here.  */
			if (!(dc->prefix & PREFIX_LOCK)
				|| op != 2) {
				gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			}
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
		}

		switch (op) {
		case 0: /* test */
			val = insn_get(s, env, dc, ot);
			tcg_gen_movi_tl(s, s->cpu_T1, val);
			gen_op_testl_T0_T1_cc(s);
			set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
			break;
		case 2: /* not */
			if (dc->prefix & PREFIX_LOCK) {
				if (mod == 3) {
					goto illegal_op;
				}
				tcg_gen_movi_tl(s, s->cpu_T0, ~0);
				tcg_gen_atomic_xor_fetch_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_T0,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
			}
			else {
				tcg_gen_not_tl(s, s->cpu_T0, s->cpu_T0);
				if (mod != 3) {
					gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
				}
				else {
					gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
				}
			}
			break;
		case 3: /* neg */
			if (dc->prefix & PREFIX_LOCK) {
				TCGLabel *label1;
				TCGv a0, t0, t1, t2;

				if (mod == 3) {
					goto illegal_op;
				}
				a0 = tcg_temp_local_new(s);
				t0 = tcg_temp_local_new(s);
				label1 = gen_new_label(s);

				tcg_gen_mov_tl(s, a0, s->cpu_A0);
				tcg_gen_mov_tl(s, t0, s->cpu_T0);

				gen_set_label(s, label1);
				t1 = tcg_temp_new(s);
				t2 = tcg_temp_new(s);
				tcg_gen_mov_tl(s, t2, t0);
				tcg_gen_neg_tl(s, t1, t0);
				tcg_gen_atomic_cmpxchg_tl(s, t0, a0, t0, t1,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
				tcg_temp_free(s, t1);
				tcg_gen_brcond_tl(s, TCG_COND_NE, t0, t2, label1);

				tcg_temp_free(s, t2);
				tcg_temp_free(s, a0);
				tcg_gen_mov_tl(s, s->cpu_T0, t0);
				tcg_temp_free(s, t0);
			}
			else {
				tcg_gen_neg_tl(s, s->cpu_T0, s->cpu_T0);
				if (mod != 3) {
					gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
				}
				else {
					gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
				}
			}
			gen_op_update_neg_cc(s);
			set_cc_op(s, dc, (CCOp)(CC_OP_SUBB + ot));
			break;
		case 4: /* mul */
			switch (ot) {
			case MO_8:
				gen_op_mov_v_reg(s, MO_8, s->cpu_T1, R_EAX);
				tcg_gen_ext8u_tl(s, s->cpu_T0, s->cpu_T0);
				tcg_gen_ext8u_tl(s, s->cpu_T1, s->cpu_T1);
				/* XXX: use 32 bit mul which could be faster */
				tcg_gen_mul_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
				tcg_gen_andi_tl(s, s->cpu_cc_src, s->cpu_T0, 0xff00);
				set_cc_op(s, dc, CC_OP_MULB);
				break;
			case MO_16:
				gen_op_mov_v_reg(s, MO_16, s->cpu_T1, R_EAX);
				tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_T0);
				tcg_gen_ext16u_tl(s, s->cpu_T1, s->cpu_T1);
				/* XXX: use 32 bit mul which could be faster */
				tcg_gen_mul_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
				tcg_gen_shri_tl(s, s->cpu_T0, s->cpu_T0, 16);
				gen_op_mov_reg_v(s, MO_16, R_EDX, s->cpu_T0);
				tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T0);
				set_cc_op(s, dc, CC_OP_MULW);
				break;
			default:
			case MO_32:
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_regs[R_EAX]);
				tcg_gen_mulu2_i32(s, s->cpu_tmp2_i32, s->cpu_tmp3_i32,
					s->cpu_tmp2_i32, s->cpu_tmp3_i32);
				tcg_gen_extu_i32_tl(s, s->cpu_regs[R_EAX], s->cpu_tmp2_i32);
				tcg_gen_extu_i32_tl(s, s->cpu_regs[R_EDX], s->cpu_tmp3_i32);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[R_EAX]);
				tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_regs[R_EDX]);
				set_cc_op(s, dc, CC_OP_MULL);
				break;
#ifdef TARGET_X86_64
			case MO_64:
				tcg_gen_mulu2_i64(s, s->cpu_regs[R_EAX], s->cpu_regs[R_EDX],
					s->cpu_T0, s->cpu_regs[R_EAX]);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[R_EAX]);
				tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_regs[R_EDX]);
				set_cc_op(s, dc, CC_OP_MULQ);
				break;
#endif
			}
			break;
		case 5: /* imul */
			switch (ot) {
			case MO_8:
				gen_op_mov_v_reg(s, MO_8, s->cpu_T1, R_EAX);
				tcg_gen_ext8s_tl(s, s->cpu_T0, s->cpu_T0);
				tcg_gen_ext8s_tl(s, s->cpu_T1, s->cpu_T1);
				/* XXX: use 32 bit mul which could be faster */
				tcg_gen_mul_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
				tcg_gen_ext8s_tl(s, s->cpu_tmp0, s->cpu_T0);
				tcg_gen_sub_tl(s, s->cpu_cc_src, s->cpu_T0, s->cpu_tmp0);
				set_cc_op(s, dc, CC_OP_MULB);
				break;
			case MO_16:
				gen_op_mov_v_reg(s, MO_16, s->cpu_T1, R_EAX);
				tcg_gen_ext16s_tl(s, s->cpu_T0, s->cpu_T0);
				tcg_gen_ext16s_tl(s, s->cpu_T1, s->cpu_T1);
				/* XXX: use 32 bit mul which could be faster */
				tcg_gen_mul_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
				tcg_gen_ext16s_tl(s, s->cpu_tmp0, s->cpu_T0);
				tcg_gen_sub_tl(s, s->cpu_cc_src, s->cpu_T0, s->cpu_tmp0);
				tcg_gen_shri_tl(s, s->cpu_T0, s->cpu_T0, 16);
				gen_op_mov_reg_v(s, MO_16, R_EDX, s->cpu_T0);
				set_cc_op(s, dc, CC_OP_MULW);
				break;
			default:
			case MO_32:
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_regs[R_EAX]);
				tcg_gen_muls2_i32(s, s->cpu_tmp2_i32, s->cpu_tmp3_i32,
					s->cpu_tmp2_i32, s->cpu_tmp3_i32);
				tcg_gen_extu_i32_tl(s, s->cpu_regs[R_EAX], s->cpu_tmp2_i32);
				tcg_gen_extu_i32_tl(s, s->cpu_regs[R_EDX], s->cpu_tmp3_i32);
				tcg_gen_sari_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, 31);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[R_EAX]);
				tcg_gen_sub_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
				tcg_gen_extu_i32_tl(s, s->cpu_cc_src, s->cpu_tmp2_i32);
				set_cc_op(s, dc, CC_OP_MULL);
				break;
#ifdef TARGET_X86_64
			case MO_64:
				tcg_gen_muls2_i64(s, s->cpu_regs[R_EAX], s->cpu_regs[R_EDX],
					s->cpu_T0, s->cpu_regs[R_EAX]);
				tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[R_EAX]);
				tcg_gen_sari_tl(s, s->cpu_cc_src, s->cpu_regs[R_EAX], 63);
				tcg_gen_sub_tl(s, s->cpu_cc_src, s->cpu_cc_src, s->cpu_regs[R_EDX]);
				set_cc_op(s, dc, CC_OP_MULQ);
				break;
#endif
			}
			break;
		case 6: /* div */
			switch (ot) {
			case MO_8:
				gen_helper_divb_AL(s, s->cpu_env, s->cpu_T0);
				break;
			case MO_16:
				gen_helper_divw_AX(s, s->cpu_env, s->cpu_T0);
				break;
			default:
			case MO_32:
				gen_helper_divl_EAX(s, s->cpu_env, s->cpu_T0);
				break;
#ifdef TARGET_X86_64
			case MO_64:
				gen_helper_divq_EAX(s, s->cpu_env, s->cpu_T0);
				break;
#endif
			}
			break;
		case 7: /* idiv */
			switch (ot) {
			case MO_8:
				gen_helper_idivb_AL(s, s->cpu_env, s->cpu_T0);
				break;
			case MO_16:
				gen_helper_idivw_AX(s, s->cpu_env, s->cpu_T0);
				break;
			default:
			case MO_32:
				gen_helper_idivl_EAX(s, s->cpu_env, s->cpu_T0);
				break;
#ifdef TARGET_X86_64
			case MO_64:
				gen_helper_idivq_EAX(s, s->cpu_env, s->cpu_T0);
				break;
#endif
			}
			break;
		default:
			goto unknown_op;
		}
		break;

	case 0xfe: /* GRP4 */
	case 0xff: /* GRP5 */
		ot = mo_b_d(b, dflag);

		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		op = (modrm >> 3) & 7;
		if (op >= 2 && b == 0xfe) {
			goto unknown_op;
		}
		if (CODE64(dc)) {
			if (op == 2 || op == 4) {
				/* operand size for jumps is 64 bit */
				ot = MO_64;
			}
			else if (op == 3 || op == 5) {
				ot = dflag != MO_16 ? (TCGMemOp)(MO_32 + (rex_w == 1)) : MO_16;
			}
			else if (op == 6) {
				/* default push size is 64 bit */
				ot = mo_pushpop(s, dc, dflag);
			}
		}
		if (mod != 3) {
			gen_lea_modrm(s, env, dc, modrm);
			if (op >= 2 && op != 3 && op != 5)
				gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
		}

		switch (op) {
		case 0: /* inc Ev */
			if (mod != 3)
				opreg = OR_TMP0;
			else
				opreg = rm;
			gen_inc(s, dc, ot, opreg, 1);
			break;
		case 1: /* dec Ev */
			if (mod != 3)
				opreg = OR_TMP0;
			else
				opreg = rm;
			gen_inc(s, dc, ot, opreg, -1);
			break;
		case 2: /* call Ev */
			/* XXX: optimize if memory (no 'and' is necessary) */
			if (dflag == MO_16) {
				tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_T0);
			}
			next_eip = dc->pc - dc->cs_base;
			tcg_gen_movi_tl(s, s->cpu_T1, next_eip);
			gen_push_v(s, dc, s->cpu_T1);
			gen_op_jmp_v(s, s->cpu_T0);
			gen_bnd_jmp(s, dc);
			gen_jr(s, dc, s->cpu_T0);
			break;
		case 3: /* lcall Ev */
			gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
			gen_add_A0_im(s, dc, 1 << ot);
			gen_op_ld_v(s, dc, MO_16, s->cpu_T0, s->cpu_A0);
		do_lcall:
			if (dc->pe && !dc->vm86) {
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_lcall_protected(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_T1,
					tcg_const_i32(s, dflag - 1),
					tcg_const_tl(s, dc->pc - dc->cs_base));
			}
			else {
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_lcall_real(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_T1,
					tcg_const_i32(s, dflag - 1),
					tcg_const_i32(s, dc->pc - dc->cs_base));
			}
			tcg_gen_ld_tl(s, s->cpu_tmp4, s->cpu_env, offsetof(CPUX86State, eip));
			gen_jr(s, dc, s->cpu_tmp4);
			break;
		case 4: /* jmp Ev */
			if (dflag == MO_16) {
				tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_T0);
			}
			gen_op_jmp_v(s, s->cpu_T0);
			gen_bnd_jmp(s, dc);
			gen_jr(s, dc, s->cpu_T0);
			break;
		case 5: /* ljmp Ev */
			gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
			gen_add_A0_im(s, dc, 1 << ot);
			gen_op_ld_v(s, dc, MO_16, s->cpu_T0, s->cpu_A0);
		do_ljmp:
			if (dc->pe && !dc->vm86) {
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_ljmp_protected(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_T1,
					tcg_const_tl(s, dc->pc - dc->cs_base));
			}
			else {
				gen_op_movl_seg_T0_vm(s, R_CS);
				gen_op_jmp_v(s, s->cpu_T1);
			}
			tcg_gen_ld_tl(s, s->cpu_tmp4, s->cpu_env, offsetof(CPUX86State, eip));
			gen_jr(s, dc, s->cpu_tmp4);
			break;
		case 6: /* push Ev */
			gen_push_v(s, dc, s->cpu_T0);
			break;
		default:
			goto unknown_op;
		}
		break;

	case 0x84: /* test Ev, Gv */
	case 0x85:
		ot = mo_b_d(b, dflag);

		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;

		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
		gen_op_mov_v_reg(s, ot, s->cpu_T1, reg);
		gen_op_testl_T0_T1_cc(s);
		set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
		break;

	case 0xa8: /* test eAX, Iv */
	case 0xa9:
		ot = mo_b_d(b, dflag);
		val = insn_get(s, env, dc, ot);

		gen_op_mov_v_reg(s, ot, s->cpu_T0, OR_EAX);
		tcg_gen_movi_tl(s, s->cpu_T1, val);
		gen_op_testl_T0_T1_cc(s);
		set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));
		break;

	case 0x98: /* CWDE/CBW */
		switch (dflag) {
#ifdef TARGET_X86_64
		case MO_64:
			gen_op_mov_v_reg(s, MO_32, s->cpu_T0, R_EAX);
			tcg_gen_ext32s_tl(s, s->cpu_T0, s->cpu_T0);
			gen_op_mov_reg_v(s, MO_64, R_EAX, s->cpu_T0);
			break;
#endif
		case MO_32:
			gen_op_mov_v_reg(s, MO_16, s->cpu_T0, R_EAX);
			tcg_gen_ext16s_tl(s, s->cpu_T0, s->cpu_T0);
			gen_op_mov_reg_v(s, MO_32, R_EAX, s->cpu_T0);
			break;
		case MO_16:
			gen_op_mov_v_reg(s, MO_8, s->cpu_T0, R_EAX);
			tcg_gen_ext8s_tl(s, s->cpu_T0, s->cpu_T0);
			gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
			break;
		default:
			tcg_abort();
		}
		break;
	case 0x99: /* CDQ/CWD */
		switch (dflag) {
#ifdef TARGET_X86_64
		case MO_64:
			gen_op_mov_v_reg(s, MO_64, s->cpu_T0, R_EAX);
			tcg_gen_sari_tl(s, s->cpu_T0, s->cpu_T0, 63);
			gen_op_mov_reg_v(s, MO_64, R_EDX, s->cpu_T0);
			break;
#endif
		case MO_32:
			gen_op_mov_v_reg(s, MO_32, s->cpu_T0, R_EAX);
			tcg_gen_ext32s_tl(s, s->cpu_T0, s->cpu_T0);
			tcg_gen_sari_tl(s, s->cpu_T0, s->cpu_T0, 31);
			gen_op_mov_reg_v(s, MO_32, R_EDX, s->cpu_T0);
			break;
		case MO_16:
			gen_op_mov_v_reg(s, MO_16, s->cpu_T0, R_EAX);
			tcg_gen_ext16s_tl(s, s->cpu_T0, s->cpu_T0);
			tcg_gen_sari_tl(s, s->cpu_T0, s->cpu_T0, 15);
			gen_op_mov_reg_v(s, MO_16, R_EDX, s->cpu_T0);
			break;
		default:
			tcg_abort();
		}
		break;
	case 0x1af: /* imul Gv, Ev */
	case 0x69: /* imul Gv, Ev, I */
	case 0x6b:
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		if (b == 0x69)
			dc->rip_offset = insn_const_size(ot);
		else if (b == 0x6b)
			dc->rip_offset = 1;
		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
		if (b == 0x69) {
			val = insn_get(s, env, dc, ot);
			tcg_gen_movi_tl(s, s->cpu_T1, val);
		}
		else if (b == 0x6b) {
			val = (int8_t)insn_get(s, env, dc, MO_8);
			tcg_gen_movi_tl(s, s->cpu_T1, val);
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T1, reg);
		}
		switch (ot) {
#ifdef TARGET_X86_64
		case MO_64:
			tcg_gen_muls2_i64(s, s->cpu_regs[reg], s->cpu_T1, s->cpu_T0, s->cpu_T1);
			tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[reg]);
			tcg_gen_sari_tl(s, s->cpu_cc_src, s->cpu_cc_dst, 63);
			tcg_gen_sub_tl(s, s->cpu_cc_src, s->cpu_cc_src, s->cpu_T1);
			break;
#endif
		case MO_32:
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_T1);
			tcg_gen_muls2_i32(s, s->cpu_tmp2_i32, s->cpu_tmp3_i32,
				s->cpu_tmp2_i32, s->cpu_tmp3_i32);
			tcg_gen_extu_i32_tl(s, s->cpu_regs[reg], s->cpu_tmp2_i32);
			tcg_gen_sari_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, 31);
			tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_regs[reg]);
			tcg_gen_sub_i32(s, s->cpu_tmp2_i32, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
			tcg_gen_extu_i32_tl(s, s->cpu_cc_src, s->cpu_tmp2_i32);
			break;
		default:
			tcg_gen_ext16s_tl(s, s->cpu_T0, s->cpu_T0);
			tcg_gen_ext16s_tl(s, s->cpu_T1, s->cpu_T1);
			/* XXX: use 32 bit mul which could be faster */
			tcg_gen_mul_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
			tcg_gen_ext16s_tl(s, s->cpu_tmp0, s->cpu_T0);
			tcg_gen_sub_tl(s, s->cpu_cc_src, s->cpu_T0, s->cpu_tmp0);
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
			break;
		}
		set_cc_op(s, dc, (CCOp)(CC_OP_MULB + ot));
		break;
	case 0x1c0:
	case 0x1c1: /* xadd Ev, Gv */
		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
		if (mod == 3) {
			rm = (modrm & 7) | REX_B(dc);
			gen_op_mov_v_reg(s, ot, s->cpu_T1, rm);
			tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T1);
			gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
		}
		else {
			gen_lea_modrm(s, env, dc, modrm);
			if (dc->prefix & PREFIX_LOCK) {
				tcg_gen_atomic_fetch_add_tl(s, s->cpu_T1, s->cpu_A0, s->cpu_T0,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
				tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
			}
			else {
				gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
				tcg_gen_add_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			}
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T1);
		}
		gen_op_update2_cc(s);
		set_cc_op(s, dc, (CCOp)(CC_OP_ADDB + ot));
		break;
	case 0x1b0:
	case 0x1b1: /* cmpxchg Ev, Gv */
	{
		TCGv oldv, newv, cmpv;

		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		oldv = tcg_temp_new(s);
		newv = tcg_temp_new(s);
		cmpv = tcg_temp_new(s);
		gen_op_mov_v_reg(s, ot, newv, reg);
		tcg_gen_mov_tl(s, cmpv, s->cpu_regs[R_EAX]);

		if (dc->prefix & PREFIX_LOCK) {
			if (mod == 3) {
				goto illegal_op;
			}
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_atomic_cmpxchg_tl(s, oldv, s->cpu_A0, cmpv, newv,
				dc->mem_index, (TCGMemOp)(ot | MO_LE));
			gen_op_mov_reg_v(s, ot, R_EAX, oldv);
		}
		else {
			if (mod == 3) {
				rm = (modrm & 7) | REX_B(dc);
				gen_op_mov_v_reg(s, ot, oldv, rm);
			}
			else {
				gen_lea_modrm(s, env, dc, modrm);
				gen_op_ld_v(s, dc, ot, oldv, s->cpu_A0);
				rm = 0; /* avoid warning */
			}
			gen_extu(s, ot, oldv);
			gen_extu(s, ot, cmpv);
			/* store value = (old == cmp ? new : old);  */
			tcg_gen_movcond_tl(s, TCG_COND_EQ, newv, oldv, cmpv, newv, oldv);
			if (mod == 3) {
				gen_op_mov_reg_v(s, ot, R_EAX, oldv);
				gen_op_mov_reg_v(s, ot, rm, newv);
			}
			else {
				/* Perform an unconditional store cycle like physical cpu;
				   must be before changing accumulator to ensure
				   idempotency if the store faults and the instruction
				   is restarted */
				gen_op_st_v(s, dc, ot, newv, s->cpu_A0);
				gen_op_mov_reg_v(s, ot, R_EAX, oldv);
			}
		}
		tcg_gen_mov_tl(s, s->cpu_cc_src, oldv);
		tcg_gen_mov_tl(s, s->cpu_cc_srcT, cmpv);
		tcg_gen_sub_tl(s, s->cpu_cc_dst, cmpv, oldv);
		set_cc_op(s, dc, (CCOp)(CC_OP_SUBB + ot));
		tcg_temp_free(s, oldv);
		tcg_temp_free(s, newv);
		tcg_temp_free(s, cmpv);
	}
	break;
	case 0x1c7: /* cmpxchg8b */
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		if ((mod == 3) || ((modrm & 0x38) != 0x8))
			goto illegal_op;
#ifdef TARGET_X86_64
		if (dflag == MO_64) {
			if (!(dc->cpuid_ext_features & CPUID_EXT_CX16))
				goto illegal_op;
			gen_lea_modrm(s, env, dc, modrm);
			if ((dc->prefix & PREFIX_LOCK) && s->parallel_cpus) {
				gen_helper_cmpxchg16b(s, s->cpu_env, s->cpu_A0);
			}
			else {
				gen_helper_cmpxchg16b_unlocked(s, s->cpu_env, s->cpu_A0);
			}
		}
		else
#endif        
		{
			if (!(dc->cpuid_features & CPUID_CX8))
				goto illegal_op;
			gen_lea_modrm(s, env, dc, modrm);
			if ((dc->prefix & PREFIX_LOCK) && s->parallel_cpus) {
				gen_helper_cmpxchg8b(s, s->cpu_env, s->cpu_A0);
			}
			else {
				gen_helper_cmpxchg8b_unlocked(s, s->cpu_env, s->cpu_A0);
			}
		}
		set_cc_op(s, dc, CC_OP_EFLAGS);
		break;

		/**************************/
		/* push/pop */
	case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57: /* push */
		gen_op_mov_v_reg(s, MO_32, s->cpu_T0, (b & 7) | REX_B(dc));
		gen_push_v(s, dc, s->cpu_T0);
		break;
	case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5c: case 0x5d: case 0x5e: case 0x5f: /* pop */
		ot = gen_pop_T0(s, dc);
		/* NOTE: order is important for pop %sp */
		gen_pop_update(s, dc, ot);
		gen_op_mov_reg_v(s, ot, (b & 7) | REX_B(dc), s->cpu_T0);
		break;
	case 0x60: /* pusha */
		if (CODE64(dc))
			goto illegal_op;
		gen_pusha(s, dc);
		break;
	case 0x61: /* popa */
		if (CODE64(dc))
			goto illegal_op;
		gen_popa(s, dc);
		break;
	case 0x68: /* push Iv */
	case 0x6a:
		ot = mo_pushpop(s, dc, dflag);
		if (b == 0x68)
			val = insn_get(s, env, dc, ot);
		else
			val = (int8_t)insn_get(s, env, dc, MO_8);
		tcg_gen_movi_tl(s, s->cpu_T0, val);
		gen_push_v(s, dc, s->cpu_T0);
		break;
	case 0x8f: /* pop Ev */
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		ot = gen_pop_T0(s, dc);
		if (mod == 3) {
			/* NOTE: order is important for pop %sp */
			gen_pop_update(s, dc, ot);
			rm = (modrm & 7) | REX_B(dc);
			gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
		}
		else {
			/* NOTE: order is important too for MMU exceptions */
			dc->popl_esp_hack = 1 << ot;
			gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 1);
			dc->popl_esp_hack = 0;
			gen_pop_update(s, dc, ot);
		}
		break;
	case 0xc8: /* enter */
	{
		int level;
		val = cpu_lduw_code(env, dc->pc);
		dc->pc += 2;
		level = cpu_ldub_code(env, dc->pc++);
		gen_enter(s, dc, val, level);
	}
	break;
	case 0xc9: /* leave */
		gen_leave(s, dc);
		break;
	case 0x06: /* push es */
	case 0x0e: /* push cs */
	case 0x16: /* push ss */
	case 0x1e: /* push ds */
		if (CODE64(dc))
			goto illegal_op;
		gen_op_movl_T0_seg(s, b >> 3);
		gen_push_v(s, dc, s->cpu_T0);
		break;
	case 0x1a0: /* push fs */
	case 0x1a8: /* push gs */
		gen_op_movl_T0_seg(s, (b >> 3) & 7);
		gen_push_v(s, dc, s->cpu_T0);
		break;
	case 0x07: /* pop es */
	case 0x17: /* pop ss */
	case 0x1f: /* pop ds */
		if (CODE64(dc))
			goto illegal_op;
		reg = b >> 3;
		ot = gen_pop_T0(s, dc);
		gen_movl_seg_T0(s, dc, reg);
		gen_pop_update(s, dc, ot);
		/* Note that reg == R_SS in gen_movl_seg_T0 always sets is_jmp.  */
		if (dc->is_jmp) {
			gen_jmp_im(s, dc->pc - dc->cs_base);
			if (reg == R_SS) {
				dc->tf = 0;
				gen_eob_inhibit_irq(s, dc, true);
			}
			else {
				gen_eob(s, dc);
			}
		}
		break;
	case 0x1a1: /* pop fs */
	case 0x1a9: /* pop gs */
		ot = gen_pop_T0(s, dc);
		gen_movl_seg_T0(s, dc, (b >> 3) & 7);
		gen_pop_update(s, dc, ot);
		if (dc->is_jmp) {
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
		}
		break;

		/**************************/
		/* mov */
	case 0x88:
	case 0x89: /* mov Gv, Ev */
		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;

		/* generate a generic store */
		gen_ldst_modrm(s, env, dc, modrm, ot, reg, 1);
		break;
	case 0xc6:
	case 0xc7: /* mov Ev, Iv */
		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		if (mod != 3) {
			dc->rip_offset = insn_const_size(ot);
			gen_lea_modrm(s, env, dc, modrm);
		}
		val = insn_get(s, env, dc, ot);
		tcg_gen_movi_tl(s, s->cpu_T0, val);
		if (mod != 3) {
			gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
		}
		else {
			gen_op_mov_reg_v(s, ot, (modrm & 7) | REX_B(dc), s->cpu_T0);
		}
		break;
	case 0x8a:
	case 0x8b: /* mov Ev, Gv */
		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;

		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
		gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
		break;
	case 0x8e: /* mov seg, Gv */
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = (modrm >> 3) & 7;
		if (reg >= 6 || reg == R_CS)
			goto illegal_op;
		gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
		gen_movl_seg_T0(s, dc, reg);
		/* Note that reg == R_SS in gen_movl_seg_T0 always sets is_jmp.  */
		if (dc->is_jmp) {
			gen_jmp_im(s, dc->pc - dc->cs_base);
			if (reg == R_SS) {
				dc->tf = 0;
				gen_eob_inhibit_irq(s, dc, true);
			}
			else {
				gen_eob(s, dc);
			}
		}
		break;
	case 0x8c: /* mov Gv, seg */
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = (modrm >> 3) & 7;
		mod = (modrm >> 6) & 3;
		if (reg >= 6)
			goto illegal_op;
		gen_op_movl_T0_seg(s, reg);
		ot = mod == 3 ? dflag : MO_16;
		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 1);
		break;

	case 0x1b6: /* movzbS Gv, Eb */
	case 0x1b7: /* movzwS Gv, Eb */
	case 0x1be: /* movsbS Gv, Eb */
	case 0x1bf: /* movswS Gv, Eb */
	{
		TCGMemOp d_ot;
		TCGMemOp s_ot;

		/* d_ot is the size of destination */
		d_ot = dflag;
		/* ot is the size of source */
		ot = (TCGMemOp)((b & 1) + MO_8);
		/* s_ot is the sign+size of source */
		s_ot = (TCGMemOp)(b & 8 ? MO_SIGN | ot : ot);

		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);

		if (mod == 3) {
			if (s_ot == MO_SB && byte_reg_is_xH(s, rm)) {
				tcg_gen_sextract_tl(s, s->cpu_T0, s->cpu_regs[rm - 4], 8, 8);
			}
			else {
				gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
				switch (s_ot) {
				case MO_UB:
					tcg_gen_ext8u_tl(s, s->cpu_T0, s->cpu_T0);
					break;
				case MO_SB:
					tcg_gen_ext8s_tl(s, s->cpu_T0, s->cpu_T0);
					break;
				case MO_UW:
					tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_T0);
					break;
				default:
				case MO_SW:
					tcg_gen_ext16s_tl(s, s->cpu_T0, s->cpu_T0);
					break;
				}
			}
			gen_op_mov_reg_v(s, d_ot, reg, s->cpu_T0);
		}
		else {
			gen_lea_modrm(s, env, dc, modrm);
			gen_op_ld_v(s, dc, s_ot, s->cpu_T0, s->cpu_A0);
			gen_op_mov_reg_v(s, d_ot, reg, s->cpu_T0);
		}
	}
	break;

	case 0x8d: /* lea */
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		if (mod == 3)
			goto illegal_op;
		reg = ((modrm >> 3) & 7) | rex_r;
		{
			AddressParts a = gen_lea_modrm_0(env, dc, modrm);
			TCGv ea = gen_lea_modrm_1(s, a);
			gen_lea_v_seg(s, dc, dc->aflag, ea, -1, -1);
			gen_op_mov_reg_v(s, dflag, reg, s->cpu_A0);
		}
		break;

	case 0xa0: /* mov EAX, Ov */
	case 0xa1:
	case 0xa2: /* mov Ov, EAX */
	case 0xa3:
	{
		target_ulong offset_addr;

		ot = mo_b_d(b, dflag);
		switch (dc->aflag) {
#ifdef TARGET_X86_64
		case MO_64:
			offset_addr = cpu_lduq_code(env, dc->pc);
			dc->pc += 8;
			break;
#endif
		default:
			offset_addr = insn_get(s, env, dc, dc->aflag);
			break;
		}
		tcg_gen_movi_tl(s, s->cpu_A0, offset_addr);
		gen_add_A0_ds_seg(s, dc);
		if ((b & 2) == 0) {
			gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			gen_op_mov_reg_v(s, ot, R_EAX, s->cpu_T0);
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, R_EAX);
			gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
		}
	}
	break;
	case 0xd7: /* xlat */
		tcg_gen_mov_tl(s, s->cpu_A0, s->cpu_regs[R_EBX]);
		tcg_gen_ext8u_tl(s, s->cpu_T0, s->cpu_regs[R_EAX]);
		tcg_gen_add_tl(s, s->cpu_A0, s->cpu_A0, s->cpu_T0);
		gen_extu(s, dc->aflag, s->cpu_A0);
		gen_add_A0_ds_seg(s, dc);
		gen_op_ld_v(s, dc, MO_8, s->cpu_T0, s->cpu_A0);
		gen_op_mov_reg_v(s, MO_8, R_EAX, s->cpu_T0);
		break;
	case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6: case 0xb7: /* mov R, Ib */
		val = insn_get(s, env, dc, MO_8);
		tcg_gen_movi_tl(s, s->cpu_T0, val);
		gen_op_mov_reg_v(s, MO_8, (b & 7) | REX_B(dc), s->cpu_T0);
		break;
	case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe: case 0xbf: /* mov R, Iv */
#ifdef TARGET_X86_64
		if (dflag == MO_64) {
			uint64_t tmp;
			/* 64 bit case */
			tmp = cpu_lduq_code(env, dc->pc);
			dc->pc += 8;
			reg = (b & 7) | REX_B(dc);
			tcg_gen_movi_tl(s, s->cpu_T0, tmp);
			gen_op_mov_reg_v(s, MO_64, reg, s->cpu_T0);
		}
		else
#endif
		{
			ot = dflag;
			val = insn_get(s, env, dc, ot);
			reg = (b & 7) | REX_B(dc);
			tcg_gen_movi_tl(s, s->cpu_T0, val);
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
		}
		break;

	case 0x91:case 0x92:case 0x93:case 0x94:case 0x95:case 0x96:case 0x97: /* xchg R, EAX */
	do_xchg_reg_eax:
		ot = dflag;
		reg = (b & 7) | REX_B(dc);
		rm = R_EAX;
		goto do_xchg_reg;
	case 0x86:
	case 0x87: /* xchg Ev, Gv */
		ot = mo_b_d(b, dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		if (mod == 3) {
			rm = (modrm & 7) | REX_B(dc);
		do_xchg_reg:
			gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
			gen_op_mov_v_reg(s, ot, s->cpu_T1, rm);
			gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T1);
		}
		else {
			gen_lea_modrm(s, env, dc, modrm);
			gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
			/* for xchg, lock is implicit */
			tcg_gen_atomic_xchg_tl(s, s->cpu_T1, s->cpu_A0, s->cpu_T0, dc->mem_index, (TCGMemOp)(ot | MO_LE));
			gen_op_mov_reg_v(s, ot, reg, s->cpu_T1);
		}
		break;
	case 0xc4: /* les Gv */
		/* In CODE64 this is VEX3; see above.  */
		op = R_ES;
		goto do_lxx;
	case 0xc5: /* lds Gv */
		/* In CODE64 this is VEX2; see above.  */
		op = R_DS;
		goto do_lxx;
	case 0x1b2: /* lss Gv */
		op = R_SS;
		goto do_lxx;
	case 0x1b4: /* lfs Gv */
		op = R_FS;
		goto do_lxx;
	case 0x1b5: /* lgs Gv */
		op = R_GS;
	do_lxx:
		ot = dflag != MO_16 ? MO_32 : MO_16;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		if (mod == 3)
			goto illegal_op;
		gen_lea_modrm(s, env, dc, modrm);
		gen_op_ld_v(s, dc, ot, s->cpu_T1, s->cpu_A0);
		gen_add_A0_im(s, dc, 1 << ot);
		/* load the segment first to handle exceptions properly */
		gen_op_ld_v(s, dc, MO_16, s->cpu_T0, s->cpu_A0);
		gen_movl_seg_T0(s, dc, op);
		/* then put the data */
		gen_op_mov_reg_v(s, ot, reg, s->cpu_T1);
		if (dc->is_jmp) {
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
		}
		break;

		/************************/
		/* shifts */
	case 0xc0:
	case 0xc1:
		/* shift Ev,Ib */
		shift = 2;
	grp2:
		{
			ot = mo_b_d(b, dflag);
			modrm = cpu_ldub_code(env, dc->pc++);
			mod = (modrm >> 6) & 3;
			op = (modrm >> 3) & 7;

			if (mod != 3) {
				if (shift == 2) {
					dc->rip_offset = 1;
				}
				gen_lea_modrm(s, env, dc, modrm);
				opreg = OR_TMP0;
			}
			else {
				opreg = (modrm & 7) | REX_B(dc);
			}

			/* simpler op */
			if (shift == 0) {
				gen_shift(s, dc, op, ot, opreg, OR_ECX);
			}
			else {
				if (shift == 2) {
					shift = cpu_ldub_code(env, dc->pc++);
				}
				gen_shifti(s, dc, op, ot, opreg, shift);
			}
		}
		break;
	case 0xd0:
	case 0xd1:
		/* shift Ev,1 */
		shift = 1;
		goto grp2;
	case 0xd2:
	case 0xd3:
		/* shift Ev,cl */
		shift = 0;
		goto grp2;

	case 0x1a4: /* shld imm */
		op = 0;
		shift = 1;
		goto do_shiftd;
	case 0x1a5: /* shld cl */
		op = 0;
		shift = 0;
		goto do_shiftd;
	case 0x1ac: /* shrd imm */
		op = 1;
		shift = 1;
		goto do_shiftd;
	case 0x1ad: /* shrd cl */
		op = 1;
		shift = 0;
	do_shiftd:
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		reg = ((modrm >> 3) & 7) | rex_r;
		if (mod != 3) {
			gen_lea_modrm(s, env, dc, modrm);
			opreg = OR_TMP0;
		}
		else {
			opreg = rm;
		}
		gen_op_mov_v_reg(s, ot, s->cpu_T1, reg);

		if (shift) {
			TCGv imm = tcg_const_tl(s, cpu_ldub_code(env, dc->pc++));
			gen_shiftd_rm_T1(s, dc, ot, opreg, op, imm);
			tcg_temp_free(s, imm);
		}
		else {
			gen_shiftd_rm_T1(s, dc, ot, opreg, op, s->cpu_regs[R_ECX]);
		}
		break;

		/************************/
		/* floats */
	case 0xd8: case 0xd9: case 0xda: case 0xdb: case 0xdc: case 0xdd: case 0xde: case 0xdf:
		if (dc->flags & (HF_EM_MASK | HF_TS_MASK)) {
			/* if CR0.EM or CR0.TS are set, generate an FPU exception */
			/* XXX: what to do if illegal op ? */
			gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
			break;
		}
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		rm = modrm & 7;
		op = ((b & 7) << 3) | ((modrm >> 3) & 7);
		if (mod != 3) {
			/* memory op */
			gen_lea_modrm(s, env, dc, modrm);
			switch (op) {
			case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06: case 0x07: /* fxxxs */
			case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: /* fixxxl */
			case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: /* fxxxl */
			case 0x30: case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: /* fixxx */
			{
				int op1;
				op1 = op & 7;

				switch (op >> 4) {
				case 0:
					tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
						dc->mem_index, MO_LEUL);
					gen_helper_flds_FT0(s, s->cpu_env, s->cpu_tmp2_i32);
					break;
				case 1:
					tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
						dc->mem_index, MO_LEUL);
					gen_helper_fildl_FT0(s, s->cpu_env, s->cpu_tmp2_i32);
					break;
				case 2:
					tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
						dc->mem_index, MO_LEQ);
					gen_helper_fldl_FT0(s, s->cpu_env, s->cpu_tmp1_i64);
					break;
				case 3:
				default:
					tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
						dc->mem_index, MO_LESW);
					gen_helper_fildl_FT0(s, s->cpu_env, s->cpu_tmp2_i32);
					break;
				}

				gen_helper_fp_arith_ST0_FT0(s, op1);
				if (op1 == 3) {
					/* fcomp needs pop */
					gen_helper_fpop(s, s->cpu_env);
				}
			}
			break;
			case 0x08: /* flds */
			case 0x0a: /* fsts */
			case 0x0b: /* fstps */
			case 0x18: case 0x19: case 0x1a: case 0x1b: /* fildl, fisttpl, fistl, fistpl */
			case 0x28: case 0x29: case 0x2a: case 0x2b: /* fldl, fisttpll, fstl, fstpl */
			case 0x38: case 0x39: case 0x3a: case 0x3b: /* filds, fisttps, fists, fistps */
				switch (op & 7) {
				case 0:
					switch (op >> 4) {
					case 0:
						tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						gen_helper_flds_ST0(s, s->cpu_env, s->cpu_tmp2_i32);
						break;
					case 1:
						tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						gen_helper_fildl_ST0(s, s->cpu_env, s->cpu_tmp2_i32);
						break;
					case 2:
						tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
							dc->mem_index, MO_LEQ);
						gen_helper_fldl_ST0(s, s->cpu_env, s->cpu_tmp1_i64);
						break;
					case 3:
					default:
						tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LESW);
						gen_helper_fildl_ST0(s, s->cpu_env, s->cpu_tmp2_i32);
						break;
					}
					break;
				case 1:
					/* XXX: the corresponding CPUID bit must be tested ! */
					switch (op >> 4) {
					case 1:
						gen_helper_fisttl_ST0(s, s->cpu_tmp2_i32, s->cpu_env);
						tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						break;
					case 2:
						gen_helper_fisttll_ST0(s, s->cpu_tmp1_i64, s->cpu_env);
						tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
							dc->mem_index, MO_LEQ);
						break;
					case 3:
					default:
						gen_helper_fistt_ST0(s, s->cpu_tmp2_i32, s->cpu_env);
						tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUW);
						break;
					}
					gen_helper_fpop(s, s->cpu_env);
					break;
				default:
					switch (op >> 4) {
					case 0:
						gen_helper_fsts_ST0(s, s->cpu_tmp2_i32, s->cpu_env);
						tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						break;
					case 1:
						gen_helper_fistl_ST0(s, s->cpu_tmp2_i32, s->cpu_env);
						tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUL);
						break;
					case 2:
						gen_helper_fstl_ST0(s, s->cpu_tmp1_i64, s->cpu_env);
						tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0,
							dc->mem_index, MO_LEQ);
						break;
					case 3:
					default:
						gen_helper_fist_ST0(s, s->cpu_tmp2_i32, s->cpu_env);
						tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
							dc->mem_index, MO_LEUW);
						break;
					}
					if ((op & 7) == 3)
						gen_helper_fpop(s, s->cpu_env);
					break;
				}
				break;
			case 0x0c: /* fldenv mem */
				gen_helper_fldenv(s, s->cpu_env, s->cpu_A0, tcg_const_i32(s, dflag - 1));
				break;
			case 0x0d: /* fldcw mem */
				tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
					dc->mem_index, MO_LEUW);
				gen_helper_fldcw(s, s->cpu_env, s->cpu_tmp2_i32);
				break;
			case 0x0e: /* fnstenv mem */
				gen_helper_fstenv(s, s->cpu_env, s->cpu_A0, tcg_const_i32(s, dflag - 1));
				break;
			case 0x0f: /* fnstcw mem */
				gen_helper_fnstcw(s, s->cpu_tmp2_i32, s->cpu_env);
				tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
					dc->mem_index, MO_LEUW);
				break;
			case 0x1d: /* fldt mem */
				gen_helper_fldt_ST0(s, s->cpu_env, s->cpu_A0);
				break;
			case 0x1f: /* fstpt mem */
				gen_helper_fstt_ST0(s, s->cpu_env, s->cpu_A0);
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x2c: /* frstor mem */
				gen_helper_frstor(s, s->cpu_env, s->cpu_A0, tcg_const_i32(s, dflag - 1));
				break;
			case 0x2e: /* fnsave mem */
				gen_helper_fsave(s, s->cpu_env, s->cpu_A0, tcg_const_i32(s, dflag - 1));
				break;
			case 0x2f: /* fnstsw mem */
				gen_helper_fnstsw(s, s->cpu_tmp2_i32, s->cpu_env);
				tcg_gen_qemu_st_i32(s, s->cpu_tmp2_i32, s->cpu_A0,
					dc->mem_index, MO_LEUW);
				break;
			case 0x3c: /* fbld */
				gen_helper_fbld_ST0(s, s->cpu_env, s->cpu_A0);
				break;
			case 0x3e: /* fbstp */
				gen_helper_fbst_ST0(s, s->cpu_env, s->cpu_A0);
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x3d: /* fildll */
				tcg_gen_qemu_ld_i64(s, s->cpu_tmp1_i64, s->cpu_A0, dc->mem_index, MO_LEQ);
				gen_helper_fildll_ST0(s, s->cpu_env, s->cpu_tmp1_i64);
				break;
			case 0x3f: /* fistpll */
				gen_helper_fistll_ST0(s, s->cpu_tmp1_i64, s->cpu_env);
				tcg_gen_qemu_st_i64(s, s->cpu_tmp1_i64, s->cpu_A0, dc->mem_index, MO_LEQ);
				gen_helper_fpop(s, s->cpu_env);
				break;
			default:
				goto unknown_op;
			}
		}
		else {
			/* register float ops */
			opreg = rm;

			switch (op) {
			case 0x08: /* fld sti */
				gen_helper_fpush(s, s->cpu_env);
				gen_helper_fmov_ST0_STN(s, s->cpu_env,
					tcg_const_i32(s, (opreg + 1) & 7));
				break;
			case 0x09: /* fxchg sti */
			case 0x29: /* fxchg4 sti, undocumented op */
			case 0x39: /* fxchg7 sti, undocumented op */
				gen_helper_fxchg_ST0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				break;
			case 0x0a: /* grp d9/2 */
				switch (rm) {
				case 0: /* fnop */
					/* check exceptions (FreeBSD FPU probe) */
					gen_helper_fwait(s, s->cpu_env);
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x0c: /* grp d9/4 */
				switch (rm) {
				case 0: /* fchs */
					gen_helper_fchs_ST0(s, s->cpu_env);
					break;
				case 1: /* fabs */
					gen_helper_fabs_ST0(s, s->cpu_env);
					break;
				case 4: /* ftst */
					gen_helper_fldz_FT0(s, s->cpu_env);
					gen_helper_fcom_ST0_FT0(s, s->cpu_env);
					break;
				case 5: /* fxam */
					gen_helper_fxam_ST0(s, s->cpu_env);
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x0d: /* grp d9/5 */
			{
				switch (rm) {
				case 0:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fld1_ST0(s, s->cpu_env);
					break;
				case 1:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldl2t_ST0(s, s->cpu_env);
					break;
				case 2:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldl2e_ST0(s, s->cpu_env);
					break;
				case 3:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldpi_ST0(s, s->cpu_env);
					break;
				case 4:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldlg2_ST0(s, s->cpu_env);
					break;
				case 5:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldln2_ST0(s, s->cpu_env);
					break;
				case 6:
					gen_helper_fpush(s, s->cpu_env);
					gen_helper_fldz_ST0(s, s->cpu_env);
					break;
				default:
					goto unknown_op;
				}
			}
			break;
			case 0x0e: /* grp d9/6 */
				switch (rm) {
				case 0: /* f2xm1 */
					gen_helper_f2xm1(s, s->cpu_env);
					break;
				case 1: /* fyl2x */
					gen_helper_fyl2x(s, s->cpu_env);
					break;
				case 2: /* fptan */
					gen_helper_fptan(s, s->cpu_env);
					break;
				case 3: /* fpatan */
					gen_helper_fpatan(s, s->cpu_env);
					break;
				case 4: /* fxtract */
					gen_helper_fxtract(s, s->cpu_env);
					break;
				case 5: /* fprem1 */
					gen_helper_fprem1(s, s->cpu_env);
					break;
				case 6: /* fdecstp */
					gen_helper_fdecstp(s, s->cpu_env);
					break;
				default:
				case 7: /* fincstp */
					gen_helper_fincstp(s, s->cpu_env);
					break;
				}
				break;
			case 0x0f: /* grp d9/7 */
				switch (rm) {
				case 0: /* fprem */
					gen_helper_fprem(s, s->cpu_env);
					break;
				case 1: /* fyl2xp1 */
					gen_helper_fyl2xp1(s, s->cpu_env);
					break;
				case 2: /* fsqrt */
					gen_helper_fsqrt(s, s->cpu_env);
					break;
				case 3: /* fsincos */
					gen_helper_fsincos(s, s->cpu_env);
					break;
				case 5: /* fscale */
					gen_helper_fscale(s, s->cpu_env);
					break;
				case 4: /* frndint */
					gen_helper_frndint(s, s->cpu_env);
					break;
				case 6: /* fsin */
					gen_helper_fsin(s, s->cpu_env);
					break;
				default:
				case 7: /* fcos */
					gen_helper_fcos(s, s->cpu_env);
					break;
				}
				break;
			case 0x00: case 0x01: case 0x04: case 0x05: case 0x06: case 0x07: /* fxxx st, sti */
			case 0x20: case 0x21: case 0x24: case 0x25: case 0x26: case 0x27: /* fxxx sti, st */
			case 0x30: case 0x31: case 0x34: case 0x35: case 0x36: case 0x37: /* fxxxp sti, st */
			{
				int op1;

				op1 = op & 7;
				if (op >= 0x20) {
					gen_helper_fp_arith_STN_ST0(s, op1, opreg);
					if (op >= 0x30)
						gen_helper_fpop(s, s->cpu_env);
				}
				else {
					gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
					gen_helper_fp_arith_ST0_FT0(s, op1);
				}
			}
			break;
			case 0x02: /* fcom */
			case 0x22: /* fcom2, undocumented op */
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fcom_ST0_FT0(s, s->cpu_env);
				break;
			case 0x03: /* fcomp */
			case 0x23: /* fcomp3, undocumented op */
			case 0x32: /* fcomp5, undocumented op */
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fcom_ST0_FT0(s, s->cpu_env);
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x15: /* da/5 */
				switch (rm) {
				case 1: /* fucompp */
					gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, 1));
					gen_helper_fucom_ST0_FT0(s, s->cpu_env);
					gen_helper_fpop(s, s->cpu_env);
					gen_helper_fpop(s, s->cpu_env);
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x1c:
				switch (rm) {
				case 0: /* feni (287 only, just do nop here) */
					break;
				case 1: /* fdisi (287 only, just do nop here) */
					break;
				case 2: /* fclex */
					gen_helper_fclex(s, s->cpu_env);
					break;
				case 3: /* fninit */
					gen_helper_fninit(s, s->cpu_env);
					break;
				case 4: /* fsetpm (287 only, just do nop here) */
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x1d: /* fucomi */
				if (!(dc->cpuid_features & CPUID_CMOV)) {
					goto illegal_op;
				}
				gen_update_cc_op(s, dc);
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fucomi_ST0_FT0(s, s->cpu_env);
				set_cc_op(s, dc, CC_OP_EFLAGS);
				break;
			case 0x1e: /* fcomi */
				if (!(dc->cpuid_features & CPUID_CMOV)) {
					goto illegal_op;
				}
				gen_update_cc_op(s, dc);
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fcomi_ST0_FT0(s, s->cpu_env);
				set_cc_op(s, dc, CC_OP_EFLAGS);
				break;
			case 0x28: /* ffree sti */
				gen_helper_ffree_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				break;
			case 0x2a: /* fst sti */
				gen_helper_fmov_STN_ST0(s, s->cpu_env, tcg_const_i32(s, opreg));
				break;
			case 0x2b: /* fstp sti */
			case 0x0b: /* fstp1 sti, undocumented op */
			case 0x3a: /* fstp8 sti, undocumented op */
			case 0x3b: /* fstp9 sti, undocumented op */
				gen_helper_fmov_STN_ST0(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x2c: /* fucom st(i) */
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fucom_ST0_FT0(s, s->cpu_env);
				break;
			case 0x2d: /* fucomp st(i) */
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fucom_ST0_FT0(s, s->cpu_env);
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x33: /* de/3 */
				switch (rm) {
				case 1: /* fcompp */
					gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, 1));
					gen_helper_fcom_ST0_FT0(s, s->cpu_env);
					gen_helper_fpop(s, s->cpu_env);
					gen_helper_fpop(s, s->cpu_env);
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x38: /* ffreep sti, undocumented op */
				gen_helper_ffree_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fpop(s, s->cpu_env);
				break;
			case 0x3c: /* df/4 */
				switch (rm) {
				case 0:
					gen_helper_fnstsw(s, s->cpu_tmp2_i32, s->cpu_env);
					tcg_gen_extu_i32_tl(s, s->cpu_T0, s->cpu_tmp2_i32);
					gen_op_mov_reg_v(s, MO_16, R_EAX, s->cpu_T0);
					break;
				default:
					goto unknown_op;
				}
				break;
			case 0x3d: /* fucomip */
				if (!(dc->cpuid_features & CPUID_CMOV)) {
					goto illegal_op;
				}
				gen_update_cc_op(s, dc);
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fucomi_ST0_FT0(s, s->cpu_env);
				gen_helper_fpop(s, s->cpu_env);
				set_cc_op(s, dc, CC_OP_EFLAGS);
				break;
			case 0x3e: /* fcomip */
				if (!(dc->cpuid_features & CPUID_CMOV)) {
					goto illegal_op;
				}
				gen_update_cc_op(s, dc);
				gen_helper_fmov_FT0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_helper_fcomi_ST0_FT0(s, s->cpu_env);
				gen_helper_fpop(s, s->cpu_env);
				set_cc_op(s, dc, CC_OP_EFLAGS);
				break;
			case 0x10: case 0x11: case 0x12: case 0x13: /* fcmovxx */
			case 0x18: case 0x1a: case 0x1b:
			{
				int op1;
				TCGLabel *l1;
				static const uint8_t fcmov_cc[8] = {
					(JCC_B << 1),
					(JCC_Z << 1),
					(JCC_BE << 1),
					(JCC_P << 1),
				};

				if (!(dc->cpuid_features & CPUID_CMOV)) {
					goto illegal_op;
				}
				op1 = fcmov_cc[op & 3] | (((op >> 3) & 1) ^ 1);
				l1 = gen_new_label(s);
				gen_jcc1_noeob(s, dc, op1, l1);
				gen_helper_fmov_ST0_STN(s, s->cpu_env, tcg_const_i32(s, opreg));
				gen_set_label(s, l1);
			}
			break;
			default:
				goto unknown_op;
			}
		}
		break;
		/************************/
		/* string ops */

	case 0xa4: /* movsS */
	case 0xa5:
		ot = mo_b_d(b, dflag);
		if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) {
			gen_repz_movs(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		else {
			gen_movs(s, dc, ot);
		}
		break;

	case 0xaa: /* stosS */
	case 0xab:
		ot = mo_b_d(b, dflag);
		if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) {
			gen_repz_stos(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		else {
			gen_stos(s, dc, ot);
		}
		break;
	case 0xac: /* lodsS */
	case 0xad:
		ot = mo_b_d(b, dflag);
		if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) {
			gen_repz_lods(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		else {
			gen_lods(s, dc, ot);
		}
		break;
	case 0xae: /* scasS */
	case 0xaf:
		ot = mo_b_d(b, dflag);
		if (prefixes & PREFIX_REPNZ) {
			gen_repz_scas(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base, 1);
		}
		else if (prefixes & PREFIX_REPZ) {
			gen_repz_scas(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base, 0);
		}
		else {
			gen_scas(s, dc, ot);
		}
		break;

	case 0xa6: /* cmpsS */
	case 0xa7:
		ot = mo_b_d(b, dflag);
		if (prefixes & PREFIX_REPNZ) {
			gen_repz_cmps(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base, 1);
		}
		else if (prefixes & PREFIX_REPZ) {
			gen_repz_cmps(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base, 0);
		}
		else {
			gen_cmps(s, dc, ot);
		}
		break;
	case 0x6c: /* insS */
	case 0x6d:
		ot = mo_b_d32(b, dflag);
		tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_regs[R_EDX]);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			SVM_IOIO_TYPE_MASK | svm_is_rep(prefixes) | 4);
		if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) {
			gen_repz_ins(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		else {
			gen_ins(s, dc, ot);
			if (dc->tb->cflags & CF_USE_ICOUNT) {
				gen_jmp(s, dc, dc->pc - dc->cs_base);
			}
		}
		break;
	case 0x6e: /* outsS */
	case 0x6f:
		ot = mo_b_d32(b, dflag);
		tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_regs[R_EDX]);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			svm_is_rep(prefixes) | 4);
		if (prefixes & (PREFIX_REPZ | PREFIX_REPNZ)) {
			gen_repz_outs(s, dc, ot, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		else {
			gen_outs(s, dc, ot);
			if (dc->tb->cflags & CF_USE_ICOUNT) {
				gen_jmp(s, dc, dc->pc - dc->cs_base);
			}
		}
		break;

		/************************/
		/* port I/O */

	case 0xe4:
	case 0xe5:
		ot = mo_b_d32(b, dflag);
		val = cpu_ldub_code(env, dc->pc++);
		tcg_gen_movi_tl(s, s->cpu_T0, val);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			SVM_IOIO_TYPE_MASK | svm_is_rep(prefixes));
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_start(s);
		}
		tcg_gen_movi_i32(s, s->cpu_tmp2_i32, val);
		gen_helper_in_func(s, ot, s->cpu_T1, s->cpu_tmp2_i32);
		gen_op_mov_reg_v(s, ot, R_EAX, s->cpu_T1);
		gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_end(s);
			gen_jmp(s, dc, dc->pc - dc->cs_base);
		}
		break;
	case 0xe6:
	case 0xe7:
		ot = mo_b_d32(b, dflag);
		val = cpu_ldub_code(env, dc->pc++);
		tcg_gen_movi_tl(s, s->cpu_T0, val);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			svm_is_rep(prefixes));
		gen_op_mov_v_reg(s, ot, s->cpu_T1, R_EAX);

		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_start(s);
		}
		tcg_gen_movi_i32(s, s->cpu_tmp2_i32, val);
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_T1);
		gen_helper_out_func(s, ot, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
		gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_end(s);
			gen_jmp(s, dc, dc->pc - dc->cs_base);
		}
		break;
	case 0xec:
	case 0xed:
		ot = mo_b_d32(b, dflag);
		tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_regs[R_EDX]);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			SVM_IOIO_TYPE_MASK | svm_is_rep(prefixes));
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_start(s);
		}
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		gen_helper_in_func(s, ot, s->cpu_T1, s->cpu_tmp2_i32);
		gen_op_mov_reg_v(s, ot, R_EAX, s->cpu_T1);
		gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_end(s);
			gen_jmp(s, dc, dc->pc - dc->cs_base);
		}
		break;
	case 0xee:
	case 0xef:
		ot = mo_b_d32(b, dflag);
		tcg_gen_ext16u_tl(s, s->cpu_T0, s->cpu_regs[R_EDX]);
		gen_check_io(s, dc, ot, pc_start - dc->cs_base,
			svm_is_rep(prefixes));
		gen_op_mov_v_reg(s, ot, s->cpu_T1, R_EAX);

		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_start(s);
		}
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp3_i32, s->cpu_T1);
		gen_helper_out_func(s, ot, s->cpu_tmp2_i32, s->cpu_tmp3_i32);
		gen_bpt_io(s, dc, s->cpu_tmp2_i32, ot);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_end(s);
			gen_jmp(s, dc, dc->pc - dc->cs_base);
		}
		break;

		/************************/
		/* control */
	case 0xc2: /* ret im */
		val = cpu_ldsw_code(env, dc->pc);
		dc->pc += 2;
		ot = gen_pop_T0(s, dc);
		gen_stack_update(s, dc, val + (1 << ot));
		/* Note that gen_pop_T0 uses a zero-extending load.  */
		gen_op_jmp_v(s, s->cpu_T0);
		gen_bnd_jmp(s, dc);
		gen_jr(s, dc, s->cpu_T0);
		break;
	case 0xc3: /* ret */
		ot = gen_pop_T0(s, dc);
		gen_pop_update(s, dc, ot);
		/* Note that gen_pop_T0 uses a zero-extending load.  */
		gen_op_jmp_v(s, s->cpu_T0);
		gen_bnd_jmp(s, dc);
		gen_jr(s, dc, s->cpu_T0);
		break;
	case 0xca: /* lret im */
		val = cpu_ldsw_code(env, dc->pc);
		dc->pc += 2;
	do_lret:
		if (dc->pe && !dc->vm86) {
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_lret_protected(s, s->cpu_env, tcg_const_i32(s, dflag - 1),
				tcg_const_i32(s, val));
		}
		else {
			gen_stack_A0(s, dc);
			/* pop offset */
			gen_op_ld_v(s, dc, dflag, s->cpu_T0, s->cpu_A0);
			/* NOTE: keeping EIP updated is not a problem in case of
			   exception */
			gen_op_jmp_v(s, s->cpu_T0);
			/* pop selector */
			gen_add_A0_im(s, dc, 1 << dflag);
			gen_op_ld_v(s, dc, dflag, s->cpu_T0, s->cpu_A0);
			gen_op_movl_seg_T0_vm(s, R_CS);
			/* add stack offset */
			gen_stack_update(s, dc, val + (2 << dflag));
		}
		gen_eob(s, dc);
		break;
	case 0xcb: /* lret */
		val = 0;
		goto do_lret;
	case 0xcf: /* iret */
		gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_IRET);
		if (!dc->pe) {
			/* real mode */
			gen_helper_iret_real(s, s->cpu_env, tcg_const_i32(s, dflag - 1));
			set_cc_op(s, dc, CC_OP_EFLAGS);
		}
		else if (dc->vm86) {
			if (dc->iopl != 3) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
			}
			else {
				gen_helper_iret_real(s, s->cpu_env, tcg_const_i32(s, dflag - 1));
				set_cc_op(s, dc, CC_OP_EFLAGS);
			}
		}
		else {
			gen_helper_iret_protected(s, s->cpu_env, tcg_const_i32(s, dflag - 1),
				tcg_const_i32(s, dc->pc - dc->cs_base));
			set_cc_op(s, dc, CC_OP_EFLAGS);
		}
		gen_eob(s, dc);
		break;
	case 0xe8: /* call im */
	{
		if (dflag != MO_16) {
			tval = (int32_t)insn_get(s, env, dc, MO_32);
		}
		else {
			tval = (int16_t)insn_get(s, env, dc, MO_16);
		}
		next_eip = dc->pc - dc->cs_base;
		tval += next_eip;
		if (dflag == MO_16) {
			tval &= 0xffff;
		}
		else if (!CODE64(dc)) {
			tval &= 0xffffffff;
		}
		tcg_gen_movi_tl(s, s->cpu_T0, next_eip);
		gen_push_v(s, dc, s->cpu_T0);
		gen_bnd_jmp(s, dc);
		gen_jmp(s, dc, tval);
	}
	break;
	case 0x9a: /* lcall im */
	{
		unsigned int selector, offset;

		if (CODE64(dc))
			goto illegal_op;
		ot = dflag;
		offset = insn_get(s, env, dc, ot);
		selector = insn_get(s, env, dc, MO_16);

		tcg_gen_movi_tl(s, s->cpu_T0, selector);
		tcg_gen_movi_tl(s, s->cpu_T1, offset);
	}
	goto do_lcall;
	case 0xe9: /* jmp im */
		if (dflag != MO_16) {
			tval = (int32_t)insn_get(s, env, dc, MO_32);
		}
		else {
			tval = (int16_t)insn_get(s, env, dc, MO_16);
		}
		tval += dc->pc - dc->cs_base;
		if (dflag == MO_16) {
			tval &= 0xffff;
		}
		else if (!CODE64(dc)) {
			tval &= 0xffffffff;
		}
		gen_bnd_jmp(s, dc);
		gen_jmp(s, dc, tval);
		break;
	case 0xea: /* ljmp im */
	{
		unsigned int selector, offset;

		if (CODE64(dc))
			goto illegal_op;
		ot = dflag;
		offset = insn_get(s, env, dc, ot);
		selector = insn_get(s, env, dc, MO_16);

		tcg_gen_movi_tl(s, s->cpu_T0, selector);
		tcg_gen_movi_tl(s, s->cpu_T1, offset);
	}
	goto do_ljmp;
	case 0xeb: /* jmp Jb */
		tval = (int8_t)insn_get(s, env, dc, MO_8);
		tval += dc->pc - dc->cs_base;
		if (dflag == MO_16) {
			tval &= 0xffff;
		}
		gen_jmp(s, dc, tval);
		break;
	case 0x70: case 0x71: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x77: case 0x78: case 0x79: case 0x7a: case 0x7b: case 0x7c: case 0x7d: case 0x7e: case 0x7f: /* jcc Jb */
		tval = (int8_t)insn_get(s, env, dc, MO_8);
		goto do_jcc;
	case 0x180: case 0x181: case 0x182: case 0x183: case 0x184: case 0x185: case 0x186: case 0x187: case 0x188: case 0x189: case 0x18a: case 0x18b: case 0x18c: case 0x18d: case 0x18e: case 0x18f: /* jcc Jv */
		if (dflag != MO_16) {
			tval = (int32_t)insn_get(s, env, dc, MO_32);
		}
		else {
			tval = (int16_t)insn_get(s, env, dc, MO_16);
		}
	do_jcc:
		next_eip = dc->pc - dc->cs_base;
		tval += next_eip;
		if (dflag == MO_16) {
			tval &= 0xffff;
		}
		gen_bnd_jmp(s, dc);
		gen_jcc(s, dc, b, tval, next_eip);
		break;

	case 0x190: case 0x191: case 0x192: case 0x193: case 0x194: case 0x195: case 0x196: case 0x197: case 0x198: case 0x199: case 0x19a: case 0x19b: case 0x19c: case 0x19d: case 0x19e: case 0x19f: /* setcc Gv */
		modrm = cpu_ldub_code(env, dc->pc++);
		gen_setcc1(s, dc, b, s->cpu_T0);
		gen_ldst_modrm(s, env, dc, modrm, MO_8, OR_TMP0, 1);
		break;
	case 0x140: case 0x141: case 0x142: case 0x143: case 0x144: case 0x145: case 0x146: case 0x147: case 0x148: case 0x149: case 0x14a: case 0x14b: case 0x14c: case 0x14d: case 0x14e: case 0x14f: /* cmov Gv, Ev */
		if (!(dc->cpuid_features & CPUID_CMOV)) {
			goto illegal_op;
		}
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		gen_cmovcc1(s, env, dc, ot, b, modrm, reg);
		break;

		/************************/
		/* flags */
	case 0x9c: /* pushf */
		gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_PUSHF);
		if (dc->vm86 && dc->iopl != 3) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_update_cc_op(s, dc);
			gen_helper_read_eflags(s, s->cpu_T0, s->cpu_env);
			gen_push_v(s, dc, s->cpu_T0);
		}
		break;
	case 0x9d: /* popf */
		gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_POPF);
		if (dc->vm86 && dc->iopl != 3) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			ot = gen_pop_T0(s, dc);
			if (dc->cpl == 0) {
				if (dflag != MO_16) {
					gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
						tcg_const_i32(s, (TF_MASK | AC_MASK |
							ID_MASK | NT_MASK |
							IF_MASK |
							IOPL_MASK)));
				}
				else {
					gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
						tcg_const_i32(s, (TF_MASK | AC_MASK |
							ID_MASK | NT_MASK |
							IF_MASK | IOPL_MASK)
							& 0xffff));
				}
			}
			else {
				if (dc->cpl <= dc->iopl) {
					if (dflag != MO_16) {
						gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
							tcg_const_i32(s, (TF_MASK |
								AC_MASK |
								ID_MASK |
								NT_MASK |
								IF_MASK)));
					}
					else {
						gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
							tcg_const_i32(s, (TF_MASK |
								AC_MASK |
								ID_MASK |
								NT_MASK |
								IF_MASK)
								& 0xffff));
					}
				}
				else {
					if (dflag != MO_16) {
						gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
							tcg_const_i32(s, (TF_MASK | AC_MASK |
								ID_MASK | NT_MASK)));
					}
					else {
						gen_helper_write_eflags(s, s->cpu_env, s->cpu_T0,
							tcg_const_i32(s, (TF_MASK | AC_MASK |
								ID_MASK | NT_MASK)
								& 0xffff));
					}
				}
			}
			gen_pop_update(s, dc, ot);
			set_cc_op(s, dc, CC_OP_EFLAGS);
			/* abort translation because TF/AC flag may change */
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
		}
		break;
	case 0x9e: /* sahf */
	{
		if (CODE64(dc) && !(dc->cpuid_ext3_features & CPUID_EXT3_LAHF_LM))
			goto illegal_op;
		gen_op_mov_v_reg(s, MO_8, s->cpu_T0, R_AH);
		TCGv zero = tcg_const_tl(s, 0);
		TCGv one = tcg_const_tl(s, 1);
		TCGv tmp = tcg_temp_new(s);

		tcg_gen_andi_tl(s, tmp, s->cpu_T0, CC_S);
		tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_s, tmp, zero, one, zero);
		tcg_gen_andi_tl(s, tmp, s->cpu_T0, CC_Z);
		tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_z, tmp, zero, one, zero);
		tcg_gen_andi_tl(s, tmp, s->cpu_T0, CC_A);
		tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_a, tmp, zero, one, zero);
		tcg_gen_andi_tl(s, tmp, s->cpu_T0, CC_P);
		tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_p, tmp, zero, one, zero);
		tcg_gen_andi_tl(s, tmp, s->cpu_T0, CC_C);
		tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_c, tmp, zero, one, zero);

		tcg_gen_st_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		tcg_gen_st_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		tcg_gen_st_tl(s, s->cpu_eflags_a, s->cpu_env, offsetof(CPUX86State, af));
		tcg_gen_st_tl(s, s->cpu_eflags_p, s->cpu_env, offsetof(CPUX86State, pf));
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));

		tcg_temp_free(s, tmp);
		tcg_temp_free(s, one);
		tcg_temp_free(s, zero);
		break;
	}
	case 0x9f: /* lahf */
		if (CODE64(dc) && !(dc->cpuid_ext3_features & CPUID_EXT3_LAHF_LM))
			goto illegal_op;

		tcg_gen_ld_tl(s, s->cpu_eflags_s, s->cpu_env, offsetof(CPUX86State, sf));
		tcg_gen_ld_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		tcg_gen_ld_tl(s, s->cpu_eflags_a, s->cpu_env, offsetof(CPUX86State, af));
		tcg_gen_ld_tl(s, s->cpu_eflags_p, s->cpu_env, offsetof(CPUX86State, pf));
		tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		tcg_gen_ld_tl(s, s->cpu_eflags_o, s->cpu_env, offsetof(CPUX86State, of));

		tcg_gen_movi_tl(s, s->cpu_T0, 0x02);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_s);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_z);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_a);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_p);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_c);
		tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_eflags_o);

		/* Note: only the condition codes are set */
		gen_op_mov_reg_v(s, MO_8, R_AH, s->cpu_T0);
		break;
	case 0xf5: /* cmc */
		gen_compute_eflags(s, dc);
		tcg_gen_ld_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		tcg_gen_xori_tl(s, s->cpu_eflags_c, s->cpu_eflags_c, 1);
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		break;
	case 0xf8: /* clc */
		tcg_gen_movi_tl(s, s->cpu_eflags_c, 0);
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		break;
	case 0xf9: /* stc */
		tcg_gen_movi_tl(s, s->cpu_eflags_c, 1);
		tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
		break;
	case 0xfc: /* cld */
		tcg_gen_movi_i32(s, s->cpu_tmp2_i32, 1);
		tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env, offsetof(CPUX86State, df));
		break;
	case 0xfd: /* std */
		tcg_gen_movi_i32(s, s->cpu_tmp2_i32, -1);
		tcg_gen_st_i32(s, s->cpu_tmp2_i32, s->cpu_env, offsetof(CPUX86State, df));
		break;

		/************************/
		/* bit operations */
	case 0x1ba: /* bt/bts/btr/btc Gv, im */
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		op = (modrm >> 3) & 7;
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		if (mod != 3) {
			dc->rip_offset = 1;
			gen_lea_modrm(s, env, dc, modrm);
			if (!(dc->prefix & PREFIX_LOCK)) {
				gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			}
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
		}
		/* load shift */
		val = cpu_ldub_code(env, dc->pc++);
		tcg_gen_movi_tl(s, s->cpu_T1, val);
		if (op < 4)
			goto unknown_op;
		op -= 4;
		goto bt_op;
	case 0x1a3: /* bt Gv, Ev */
		op = 0;
		goto do_btx;
	case 0x1ab: /* bts */
		op = 1;
		goto do_btx;
	case 0x1b3: /* btr */
		op = 2;
		goto do_btx;
	case 0x1bb: /* btc */
		op = 3;
	do_btx:
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		mod = (modrm >> 6) & 3;
		rm = (modrm & 7) | REX_B(dc);
		gen_op_mov_v_reg(s, MO_32, s->cpu_T1, reg);
		if (mod != 3) {
			AddressParts a = gen_lea_modrm_0(env, dc, modrm);
			/* specific case: we need to add a displacement */
			gen_exts(s, ot, s->cpu_T1);
			tcg_gen_sari_tl(s, s->cpu_tmp0, s->cpu_T1, 3 + ot);
			tcg_gen_shli_tl(s, s->cpu_tmp0, s->cpu_tmp0, ot);
			tcg_gen_add_tl(s, s->cpu_A0, gen_lea_modrm_1(s, a), s->cpu_tmp0);
			gen_lea_v_seg(s, dc, dc->aflag, s->cpu_A0, a.def_seg, dc->override);
			if (!(dc->prefix & PREFIX_LOCK)) {
				gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
			}
		}
		else {
			gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
		}
	bt_op:
		tcg_gen_andi_tl(s, s->cpu_T1, s->cpu_T1, (1 << (3 + ot)) - 1);
		tcg_gen_movi_tl(s, s->cpu_tmp0, 1);
		tcg_gen_shl_tl(s, s->cpu_tmp0, s->cpu_tmp0, s->cpu_T1);
		if (dc->prefix & PREFIX_LOCK) {
			switch (op) {
			case 0: /* bt */
				/* Needs no atomic ops; we surpressed the normal
				   memory load for LOCK above so do it now.  */
				gen_op_ld_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
				break;
			case 1: /* bts */
				tcg_gen_atomic_fetch_or_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_tmp0,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
				break;
			case 2: /* btr */
				tcg_gen_not_tl(s, s->cpu_tmp0, s->cpu_tmp0);
				tcg_gen_atomic_fetch_and_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_tmp0,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
				break;
			default:
			case 3: /* btc */
				tcg_gen_atomic_fetch_xor_tl(s, s->cpu_T0, s->cpu_A0, s->cpu_tmp0,
					dc->mem_index, (TCGMemOp)(ot | MO_LE));
				break;
			}
			tcg_gen_shr_tl(s, s->cpu_tmp4, s->cpu_T0, s->cpu_T1);
		}
		else {
			tcg_gen_shr_tl(s, s->cpu_tmp4, s->cpu_T0, s->cpu_T1);
			switch (op) {
			case 0: /* bt */
				/* Data already loaded; nothing to do.  */
				break;
			case 1: /* bts */
				tcg_gen_or_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp0);
				break;
			case 2: /* btr */
				tcg_gen_andc_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp0);
				break;
			default:
			case 3: /* btc */
				tcg_gen_xor_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_tmp0);
				break;
			}
			if (op != 0) {
				if (mod != 3) {
					gen_op_st_v(s, dc, ot, s->cpu_T0, s->cpu_A0);
				}
				else {
					gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
				}
			}
		}

		/* Delay all CC updates until after the store above.  Note that
		   C is the result of the test, Z is unchanged, and the others
		   are all undefined.  */
		switch (dc->cc_op) {
		case CC_OP_MULB:case CC_OP_MULW:case CC_OP_MULL:case CC_OP_MULQ:
		case CC_OP_ADDB:case CC_OP_ADDW:case CC_OP_ADDL:case CC_OP_ADDQ:
		case CC_OP_ADCB:case CC_OP_ADCW:case CC_OP_ADCL:case CC_OP_ADCQ:
		case CC_OP_SUBB:case CC_OP_SUBW:case CC_OP_SUBL:case CC_OP_SUBQ:
		case CC_OP_SBBB:case CC_OP_SBBW:case CC_OP_SBBL:case CC_OP_SBBQ:
		case CC_OP_LOGICB:case CC_OP_LOGICW:case CC_OP_LOGICL:case CC_OP_LOGICQ:
		case CC_OP_INCB:case CC_OP_INCW:case CC_OP_INCL:case CC_OP_INCQ:
		case CC_OP_DECB:case CC_OP_DECW:case CC_OP_DECL:case CC_OP_DECQ:
		case CC_OP_SHLB:case CC_OP_SHLW:case CC_OP_SHLL:case CC_OP_SHLQ:
		case CC_OP_SARB:case CC_OP_SARW:case CC_OP_SARL:case CC_OP_SARQ:
		case CC_OP_BMILGB:case CC_OP_BMILGW:case CC_OP_BMILGL:case CC_OP_BMILGQ:
			/* Z was going to be computed from the non-zero status of CC_DST.
			   We can get that same Z value (and the new C value) by leaving
			   CC_DST alone, setting CC_SRC, and using a CC_OP_SAR of the
			   same width.  */
			tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_tmp4);
			set_cc_op(s, dc, (CCOp)(((dc->cc_op - CC_OP_MULB) & 3) + CC_OP_SARB));
			break;
		default:
			/* Otherwise, generate EFLAGS and replace the C bit.  */
			assert(CC_C == 1);
			tcg_gen_andi_tl(s, s->cpu_eflags_c, s->cpu_tmp4, CC_C);
			tcg_gen_st_tl(s, s->cpu_eflags_c, s->cpu_env, offsetof(CPUX86State, cf));
			break;
		}
		break;
	case 0x1bc: /* bsf / tzcnt */
	case 0x1bd: /* bsr / lzcnt */
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
		gen_extu(s, ot, s->cpu_T0);

		/* Note that lzcnt and tzcnt are in different extensions.  */
		if ((prefixes & PREFIX_REPZ)
			&& (b & 1
				? dc->cpuid_ext3_features & CPUID_EXT3_ABM
				: dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_BMI1)) {
			int size = 8 << ot;
			/* For lzcnt/tzcnt, C bit is defined related to the input. */
			tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T0);
			if (b & 1) {
				/* For lzcnt, reduce the target_ulong result by the
				   number of zeros that we expect to find at the top.  */
				tcg_gen_clzi_tl(s, s->cpu_T0, s->cpu_T0, TARGET_LONG_BITS);
				tcg_gen_subi_tl(s, s->cpu_T0, s->cpu_T0, TARGET_LONG_BITS - size);
			}
			else {
				/* For tzcnt, a zero input must return the operand size.  */
				tcg_gen_ctzi_tl(s, s->cpu_T0, s->cpu_T0, size);
			}
			/* For lzcnt/tzcnt, Z bit is defined related to the result.  */
			gen_op_update1_cc(s);
			set_cc_op(s, dc, (CCOp)(CC_OP_BMILGB + ot));
		}
		else {
			/* For bsr/bsf, only the Z bit is defined and it is related
			   to the input and not the result.  */
			tcg_gen_mov_tl(s, s->cpu_cc_dst, s->cpu_T0);
			set_cc_op(s, dc, (CCOp)(CC_OP_LOGICB + ot));

			/* ??? The manual says that the output is undefined when the
			   input is zero, but real hardware leaves it unchanged, and
			   real programs appear to depend on that.  Accomplish this
			   by passing the output as the value to return upon zero.  */
			if (b & 1) {
				/* For bsr, return the bit index of the first 1 bit,
				   not the count of leading zeros.  */
				tcg_gen_xori_tl(s, s->cpu_T1, s->cpu_regs[reg], TARGET_LONG_BITS - 1);
				tcg_gen_clz_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_T1);
				tcg_gen_xori_tl(s, s->cpu_T0, s->cpu_T0, TARGET_LONG_BITS - 1);
			}
			else {
				tcg_gen_ctz_tl(s, s->cpu_T0, s->cpu_T0, s->cpu_regs[reg]);
			}
		}
		gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);
		break;
		/************************/
		/* bcd */
	case 0x27: /* daa */
		if (CODE64(dc))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_helper_daa(s, s->cpu_env);
		set_cc_op(s, dc, CC_OP_EFLAGS);
		break;
	case 0x2f: /* das */
		if (CODE64(dc))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_helper_das(s, s->cpu_env);
		set_cc_op(s, dc, CC_OP_EFLAGS);
		break;
	case 0x37: /* aaa */
		if (CODE64(dc))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_helper_aaa(s, s->cpu_env);
		set_cc_op(s, dc, CC_OP_EFLAGS);
		break;
	case 0x3f: /* aas */
		if (CODE64(dc))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_helper_aas(s, s->cpu_env);
		set_cc_op(s, dc, CC_OP_EFLAGS);
		break;
	case 0xd4: /* aam */
		if (CODE64(dc))
			goto illegal_op;
		val = cpu_ldub_code(env, dc->pc++);
		if (val == 0) {
			gen_exception(s, dc, EXCP00_DIVZ, pc_start - dc->cs_base);
		}
		else {
			gen_helper_aam(s, s->cpu_env, tcg_const_i32(s, val));
			set_cc_op(s, dc, CC_OP_LOGICB);
		}
		break;
	case 0xd5: /* aad */
		if (CODE64(dc))
			goto illegal_op;
		val = cpu_ldub_code(env, dc->pc++);
		gen_helper_aad(s, s->cpu_env, tcg_const_i32(s, val));
		set_cc_op(s, dc, CC_OP_LOGICB);
		break;
		/************************/
		/* misc */
	case 0x90: /* nop */
		/* XXX: correct lock test for all insn */
		if (prefixes & PREFIX_LOCK) {
			goto illegal_op;
		}
		/* If REX_B is set, then this is xchg eax, r8d, not a nop.  */
		if (REX_B(dc)) {
			goto do_xchg_reg_eax;
		}
		if (prefixes & PREFIX_REPZ) {
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_pause(s, s->cpu_env, tcg_const_i32(s, dc->pc - pc_start));
			dc->is_jmp = DISAS_TB_JUMP;
		}
		break;
	case 0x9b: /* fwait */
		if ((dc->flags & (HF_MP_MASK | HF_TS_MASK)) ==
			(HF_MP_MASK | HF_TS_MASK)) {
			gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
		}
		else {
			gen_helper_fwait(s, s->cpu_env);
		}
		break;
	case 0xcc: /* int3 */
		gen_interrupt(s, dc, EXCP03_INT3, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		break;
	case 0xcd: /* int N */
		val = cpu_ldub_code(env, dc->pc++);
		if (dc->vm86 && dc->iopl != 3) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_interrupt(s, dc, val, pc_start - dc->cs_base, dc->pc - dc->cs_base);
		}
		break;
	case 0xce: /* into */
		if (CODE64(dc))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, pc_start - dc->cs_base);
		gen_helper_into(s, s->cpu_env, tcg_const_i32(s, dc->pc - pc_start));
		break;
#ifdef WANT_ICEBP
	case 0xf1: /* icebp (undocumented, exits to external debugger) */
		gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_ICEBP);
#if 1
		gen_debug(s, dc, pc_start - dc->cs_base);
#else
		/* start debug */
		tb_flush(CPU(x86_env_get_cpu(env)));
		qemu_set_log(CPU_LOG_INT | CPU_LOG_TB_IN_ASM);
#endif
		break;
#endif
	case 0xfa: /* cli */
		if (!dc->vm86) {
			if (dc->cpl <= dc->iopl) {
				gen_helper_cli(s, s->cpu_env);
			}
			else {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
			}
		}
		else {
			if (dc->iopl == 3) {
				gen_helper_cli(s, s->cpu_env);
			}
			else {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
			}
		}
		break;
	case 0xfb: /* sti */
		if (dc->vm86 ? dc->iopl == 3 : dc->cpl <= dc->iopl) {
			gen_helper_sti(s, s->cpu_env);
			/* interruptions are enabled only the first insn after sti */
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob_inhibit_irq(s, dc, true);
		}
		else {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		break;
	case 0x62: /* bound */
		if (CODE64(dc))
			goto illegal_op;
		ot = dflag;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = (modrm >> 3) & 7;
		mod = (modrm >> 6) & 3;
		if (mod == 3)
			goto illegal_op;
		gen_op_mov_v_reg(s, ot, s->cpu_T0, reg);
		gen_lea_modrm(s, env, dc, modrm);
		tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
		if (ot == MO_16) {
			gen_helper_boundw(s, s->cpu_env, s->cpu_A0, s->cpu_tmp2_i32);
		}
		else {
			gen_helper_boundl(s, s->cpu_env, s->cpu_A0, s->cpu_tmp2_i32);
		}
		break;
	case 0x1c8:case 0x1c9:case 0x1ca:case 0x1cb:case 0x1cc:case 0x1cd:case 0x1ce:case 0x1cf: /* bswap reg */
		reg = (b & 7) | REX_B(dc);
#ifdef TARGET_X86_64
		if (dflag == MO_64) {
			gen_op_mov_v_reg(s, MO_64, s->cpu_T0, reg);
			tcg_gen_bswap64_i64(s, s->cpu_T0, s->cpu_T0);
			gen_op_mov_reg_v(s, MO_64, reg, s->cpu_T0);
		}
		else
#endif
		{
			gen_op_mov_v_reg(s, MO_32, s->cpu_T0, reg);
			tcg_gen_ext32u_tl(s, s->cpu_T0, s->cpu_T0);
			tcg_gen_bswap32_tl(s, s->cpu_T0, s->cpu_T0);
			gen_op_mov_reg_v(s, MO_32, reg, s->cpu_T0);
		}
		break;
	case 0xd6: /* salc */
		if (CODE64(dc))
			goto illegal_op;
		gen_compute_eflags_c(s, dc, s->cpu_T0);
		tcg_gen_neg_tl(s, s->cpu_T0, s->cpu_T0);
		gen_op_mov_reg_v(s, MO_8, R_EAX, s->cpu_T0);
		break;
	case 0xe0: /* loopnz */
	case 0xe1: /* loopz */
	case 0xe2: /* loop */
	case 0xe3: /* jecxz */
	{
		TCGLabel *l1, *l2, *l3;

		tval = (int8_t)insn_get(s, env, dc, MO_8);
		next_eip = dc->pc - dc->cs_base;
		tval += next_eip;
		if (dflag == MO_16) {
			tval &= 0xffff;
		}

		l1 = gen_new_label(s);
		l2 = gen_new_label(s);
		l3 = gen_new_label(s);
		b &= 3;
		switch (b) {
		case 0: /* loopnz */
		case 1: /* loopz */
			gen_op_add_reg_im(s, dc->aflag, R_ECX, -1);
			gen_op_jz_ecx(s, dc->aflag, l3);
			gen_jcc1(s, dc, (JCC_Z << 1) | (b ^ 1), l1);
			break;
		case 2: /* loop */
			gen_op_add_reg_im(s, dc->aflag, R_ECX, -1);
			gen_op_jnz_ecx(s, dc->aflag, l1);
			break;
		default:
		case 3: /* jcxz */
			gen_op_jz_ecx(s, dc->aflag, l1);
			break;
		}

		gen_set_label(s, l3);
		gen_jmp_im(s, next_eip);
		tcg_gen_br(s, l2);

		gen_set_label(s, l1);
		gen_jmp_im(s, tval);
		gen_set_label(s, l2);
		gen_eob(s, dc);
	}
	break;
	case 0x130: /* wrmsr */
	case 0x132: /* rdmsr */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			if (b & 2) {
				gen_helper_rdmsr(s, s->cpu_env);
			}
			else {
				gen_helper_wrmsr(s, s->cpu_env);
			}
		}
		break;
	case 0x131: /* rdtsc */
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, pc_start - dc->cs_base);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_start(s);
		}
		gen_helper_rdtsc(s, s->cpu_env);
		if (dc->tb->cflags & CF_USE_ICOUNT) {
			gen_io_end(s);
			gen_jmp(s, dc, dc->pc - dc->cs_base);
		}
		break;
	case 0x133: /* rdpmc */
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, pc_start - dc->cs_base);
		gen_helper_rdpmc(s, s->cpu_env);
		break;
	case 0x134: /* sysenter */
		/* For Intel SYSENTER is valid on 64-bit */
		if (CODE64(dc) && env->cpuid_vendor1 != CPUID_VENDOR_INTEL_1)
			goto illegal_op;
		if (!dc->pe) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_helper_sysenter(s, s->cpu_env);
			gen_eob(s, dc);
		}
		break;
	case 0x135: /* sysexit */
		/* For Intel SYSEXIT is valid on 64-bit */
		if (CODE64(dc) && env->cpuid_vendor1 != CPUID_VENDOR_INTEL_1)
			goto illegal_op;
		if (!dc->pe) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_helper_sysexit(s, s->cpu_env, tcg_const_i32(s, dflag - 1));
			gen_eob(s, dc);
		}
		break;
#ifdef TARGET_X86_64
	case 0x105: /* syscall */
		/* XXX: is it usable in real mode ? */
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, pc_start - dc->cs_base);
		gen_helper_syscall(s, s->cpu_env, tcg_const_i32(s, dc->pc - pc_start));
		/* TF handling for the syscall insn is different. The TF bit is  checked
		   after the syscall insn completes. This allows #DB to not be
		   generated after one has entered CPL0 if TF is set in FMASK.  */
		gen_eob_worker(s, dc, false, true);
		break;
	case 0x107: /* sysret */
		if (!dc->pe) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_helper_sysret(s, s->cpu_env, tcg_const_i32(s, dflag - 1));
			/* condition codes are modified only in long mode */
			if (dc->lma) {
				set_cc_op(s, dc, CC_OP_EFLAGS);
			}
			/* TF handling for the sysret insn is different. The TF bit is
			   checked after the sysret insn completes. This allows #DB to be
			   generated "as if" the syscall insn in userspace has just
			   completed.  */
			gen_eob_worker(s, dc, false, true);
		}
		break;
#endif
	case 0x1a2: /* cpuid */
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, pc_start - dc->cs_base);
		gen_helper_cpuid(s, s->cpu_env);
		break;
	case 0xf4: /* hlt */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_hlt(s, s->cpu_env, tcg_const_i32(s, dc->pc - pc_start));
			dc->is_jmp = DISAS_TB_JUMP;
		}
		break;
	case 0x100:
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		op = (modrm >> 3) & 7;
		switch (op) {
		case 0: /* sldt */
			if (!dc->pe || dc->vm86)
				goto illegal_op;
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_LDTR_READ);
			tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env,
				offsetof(CPUX86State, ldt.selector));
			ot = mod == 3 ? dflag : MO_16;
			gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 1);
			break;
		case 2: /* lldt */
			if (!dc->pe || dc->vm86)
				goto illegal_op;
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
			}
			else {
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_LDTR_WRITE);
				gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_lldt(s, s->cpu_env, s->cpu_tmp2_i32);
			}
			break;
		case 1: /* str */
			if (!dc->pe || dc->vm86)
				goto illegal_op;
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_TR_READ);
			tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env,
				offsetof(CPUX86State, tr.selector));
			ot = mod == 3 ? dflag : MO_16;
			gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 1);
			break;
		case 3: /* ltr */
			if (!dc->pe || dc->vm86)
				goto illegal_op;
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
			}
			else {
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_TR_WRITE);
				gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
				tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_T0);
				gen_helper_ltr(s, s->cpu_env, s->cpu_tmp2_i32);
			}
			break;
		case 4: /* verr */
		case 5: /* verw */
			if (!dc->pe || dc->vm86)
				goto illegal_op;
			gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
			gen_update_cc_op(s, dc);
			if (op == 4) {
				gen_helper_verr(s, s->cpu_env, s->cpu_T0);
			}
			else {
				gen_helper_verw(s, s->cpu_env, s->cpu_T0);
			}
			set_cc_op(s, dc, CC_OP_EFLAGS);
			break;
		default:
			goto unknown_op;
		}
		break;

	case 0x101:
		modrm = cpu_ldub_code(env, dc->pc++);
		switch (modrm) {
			CASE_MODRM_MEM_OP(0) : /* sgdt */
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_GDTR_READ);
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_ld32u_tl(s, s->cpu_T0,
				s->cpu_env, offsetof(CPUX86State, gdt.limit));
			gen_op_st_v(s, dc, MO_16, s->cpu_T0, s->cpu_A0);
			gen_add_A0_im(s, dc, 2);
			tcg_gen_ld_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, gdt.base));
			if (dflag == MO_16) {
				tcg_gen_andi_tl(s, s->cpu_T0, s->cpu_T0, 0xffffff);
			}
			gen_op_st_v(s, dc, CODE64(dc) + MO_32, s->cpu_T0, s->cpu_A0);
			break;

		case 0xc8: /* monitor */
			if (!(dc->cpuid_ext_features & CPUID_EXT_MONITOR) || dc->cpl != 0) {
				goto illegal_op;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			tcg_gen_mov_tl(s, s->cpu_A0, s->cpu_regs[R_EAX]);
			gen_extu(s, dc->aflag, s->cpu_A0);
			gen_add_A0_ds_seg(s, dc);
			gen_helper_monitor(s, s->cpu_env, s->cpu_A0);
			break;

		case 0xc9: /* mwait */
			if (!(dc->cpuid_ext_features & CPUID_EXT_MONITOR) || dc->cpl != 0) {
				goto illegal_op;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_mwait(s, s->cpu_env, tcg_const_i32(s, dc->pc - pc_start));
			gen_eob(s, dc);
			break;

		case 0xca: /* clac */
			if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_SMAP)
				|| dc->cpl != 0) {
				goto illegal_op;
			}
			gen_helper_clac(s, s->cpu_env);
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

		case 0xcb: /* stac */
			if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_SMAP)
				|| dc->cpl != 0) {
				goto illegal_op;
			}
			gen_helper_stac(s, s->cpu_env);
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

			CASE_MODRM_MEM_OP(1) : /* sidt */
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_IDTR_READ);
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, idt.limit));
			gen_op_st_v(s, dc, MO_16, s->cpu_T0, s->cpu_A0);
			gen_add_A0_im(s, dc, 2);
			tcg_gen_ld_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, idt.base));
			if (dflag == MO_16) {
				tcg_gen_andi_tl(s, s->cpu_T0, s->cpu_T0, 0xffffff);
			}
			gen_op_st_v(s, dc, CODE64(dc) + MO_32, s->cpu_T0, s->cpu_A0);
			break;

		case 0xd0: /* xgetbv */
			if ((dc->cpuid_ext_features & CPUID_EXT_XSAVE) == 0
				|| (dc->prefix & (PREFIX_LOCK | PREFIX_DATA
					| PREFIX_REPZ | PREFIX_REPNZ))) {
				goto illegal_op;
			}
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_ECX]);
			gen_helper_xgetbv(s, s->cpu_tmp1_i64, s->cpu_env, s->cpu_tmp2_i32);
			tcg_gen_extr_i64_tl(s, s->cpu_regs[R_EAX], s->cpu_regs[R_EDX], s->cpu_tmp1_i64);
			break;

		case 0xd1: /* xsetbv */
			if ((dc->cpuid_ext_features & CPUID_EXT_XSAVE) == 0
				|| (dc->prefix & (PREFIX_LOCK | PREFIX_DATA
					| PREFIX_REPZ | PREFIX_REPNZ))) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			tcg_gen_concat_tl_i64(s, s->cpu_tmp1_i64, s->cpu_regs[R_EAX],
				s->cpu_regs[R_EDX]);
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_ECX]);
			gen_helper_xsetbv(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_tmp1_i64);
			/* End TB because translation flags may change.  */
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

		case 0xd8: /* VMRUN */
			if (!(dc->flags & HF_SVME_MASK) || !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_vmrun(s, s->cpu_env, tcg_const_i32(s, dc->aflag - 1),
				tcg_const_i32(s, dc->pc - pc_start));
			tcg_gen_exit_tb(s, 0);
			dc->is_jmp = DISAS_TB_JUMP;
			break;

		case 0xd9: /* VMMCALL */
			if (!(dc->flags & HF_SVME_MASK)) {
				goto illegal_op;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_vmmcall(s, s->cpu_env);
			break;

		case 0xda: /* VMLOAD */
			if (!(dc->flags & HF_SVME_MASK) || !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_vmload(s, s->cpu_env, tcg_const_i32(s, dc->aflag - 1));
			break;

		case 0xdb: /* VMSAVE */
			if (!(dc->flags & HF_SVME_MASK) || !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_vmsave(s, s->cpu_env, tcg_const_i32(s, dc->aflag - 1));
			break;

		case 0xdc: /* STGI */
			if ((!(dc->flags & HF_SVME_MASK)
				&& !(dc->cpuid_ext3_features & CPUID_EXT3_SKINIT))
				|| !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_stgi(s, s->cpu_env);
			break;

		case 0xdd: /* CLGI */
			if (!(dc->flags & HF_SVME_MASK) || !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_clgi(s, s->cpu_env);
			break;

		case 0xde: /* SKINIT */
			if ((!(dc->flags & HF_SVME_MASK)
				&& !(dc->cpuid_ext3_features & CPUID_EXT3_SKINIT))
				|| !dc->pe) {
				goto illegal_op;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_skinit(s, s->cpu_env);
			break;

		case 0xdf: /* INVLPGA */
			if (!(dc->flags & HF_SVME_MASK) || !dc->pe) {
				goto illegal_op;
			}
			if (dc->cpl != 0) {
				gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				break;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_helper_invlpga(s, s->cpu_env, tcg_const_i32(s, dc->aflag - 1));
			break;

			CASE_MODRM_MEM_OP(2) : /* lgdt */
				if (dc->cpl != 0) {
					gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
					break;
				}
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_GDTR_WRITE);
			gen_lea_modrm(s, env, dc, modrm);
			gen_op_ld_v(s, dc, MO_16, s->cpu_T1, s->cpu_A0);
			gen_add_A0_im(s, dc, 2);
			gen_op_ld_v(s, dc, CODE64(dc) + MO_32, s->cpu_T0, s->cpu_A0);
			if (dflag == MO_16) {
				tcg_gen_andi_tl(s, s->cpu_T0, s->cpu_T0, 0xffffff);
			}
			tcg_gen_st_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, gdt.base));
			tcg_gen_st32_tl(s, s->cpu_T1, s->cpu_env, offsetof(CPUX86State, gdt.limit));
			break;

			CASE_MODRM_MEM_OP(3) : /* lidt */
				if (dc->cpl != 0) {
					gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
					break;
				}
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_IDTR_WRITE);
			gen_lea_modrm(s, env, dc, modrm);
			gen_op_ld_v(s, dc, MO_16, s->cpu_T1, s->cpu_A0);
			gen_add_A0_im(s, dc, 2);
			gen_op_ld_v(s, dc, CODE64(dc) + MO_32, s->cpu_T0, s->cpu_A0);
			if (dflag == MO_16) {
				tcg_gen_andi_tl(s, s->cpu_T0, s->cpu_T0, 0xffffff);
			}
			tcg_gen_st_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, idt.base));
			tcg_gen_st32_tl(s, s->cpu_T1, s->cpu_env, offsetof(CPUX86State, idt.limit));
			break;

			CASE_MODRM_OP(4) : /* smsw */
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_READ_CR0);
			tcg_gen_ld_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, cr[0]));
			if (CODE64(dc)) {
				mod = (modrm >> 6) & 3;
				ot = (mod != 3 ? MO_16 : dc->dflag);
			}
			else {
				ot = MO_16;
			}
			gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 1);
			break;
		case 0xee: /* rdpkru */
			if (prefixes & PREFIX_LOCK) {
				goto illegal_op;
			}
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_ECX]);
			gen_helper_rdpkru(s, s->cpu_tmp1_i64, s->cpu_env, s->cpu_tmp2_i32);
			tcg_gen_extr_i64_tl(s, s->cpu_regs[R_EAX], s->cpu_regs[R_EDX], s->cpu_tmp1_i64);
			break;
		case 0xef: /* wrpkru */
			if (prefixes & PREFIX_LOCK) {
				goto illegal_op;
			}
			tcg_gen_concat_tl_i64(s, s->cpu_tmp1_i64, s->cpu_regs[R_EAX],
				s->cpu_regs[R_EDX]);
			tcg_gen_trunc_tl_i32(s, s->cpu_tmp2_i32, s->cpu_regs[R_ECX]);
			gen_helper_wrpkru(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_tmp1_i64);
			break;
			CASE_MODRM_OP(6) : /* lmsw */
				if (dc->cpl != 0) {
					gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
					break;
				}
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_WRITE_CR0);
			gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
			gen_helper_lmsw(s, s->cpu_env, s->cpu_T0);
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

			CASE_MODRM_MEM_OP(7) : /* invlpg */
				if (dc->cpl != 0) {
					gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
					break;
				}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			gen_lea_modrm(s, env, dc, modrm);
			gen_helper_invlpg(s, s->cpu_env, s->cpu_A0);
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

		case 0xf8: /* swapgs */
#ifdef TARGET_X86_64
			if (CODE64(dc)) {
				if (dc->cpl != 0) {
					gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
				}
				else {
					tcg_gen_mov_tl(s, s->cpu_T0, s->cpu_seg_base[R_GS]);
					tcg_gen_ld_tl(s, s->cpu_seg_base[R_GS], s->cpu_env,
						offsetof(CPUX86State, kernelgsbase));
					tcg_gen_st_tl(s, s->cpu_T0, s->cpu_env,
						offsetof(CPUX86State, kernelgsbase));
				}
				break;
			}
#endif
			goto illegal_op;

		case 0xf9: /* rdtscp */
			if (!(dc->cpuid_ext2_features & CPUID_EXT2_RDTSCP)) {
				goto illegal_op;
			}
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, pc_start - dc->cs_base);
			if (dc->tb->cflags & CF_USE_ICOUNT) {
				gen_io_start(s);
			}
			gen_helper_rdtscp(s, s->cpu_env);
			if (dc->tb->cflags & CF_USE_ICOUNT) {
				gen_io_end(s);
				gen_jmp(s, dc, dc->pc - dc->cs_base);
			}
			break;

		default:
			goto unknown_op;
		}
		break;

	case 0x108: /* invd */
	case 0x109: /* wbinvd */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_svm_check_intercept(s, dc, pc_start, (b & 2) ? SVM_EXIT_INVD : SVM_EXIT_WBINVD);
			/* nothing to do */
		}
		break;
	case 0x63: /* arpl or movslS (x86_64) */
#ifdef TARGET_X86_64
		if (CODE64(dc)) {
			int d_ot;
			/* d_ot is the size of destination */
			d_ot = dflag;

			modrm = cpu_ldub_code(env, dc->pc++);
			reg = ((modrm >> 3) & 7) | rex_r;
			mod = (modrm >> 6) & 3;
			rm = (modrm & 7) | REX_B(dc);

			if (mod == 3) {
				gen_op_mov_v_reg(s, MO_32, s->cpu_T0, rm);
				/* sign extend */
				if (d_ot == MO_64) {
					tcg_gen_ext32s_tl(s, s->cpu_T0, s->cpu_T0);
				}
				gen_op_mov_reg_v(s, (TCGMemOp)d_ot, reg, s->cpu_T0);
			}
			else {
				gen_lea_modrm(s, env, dc, modrm);
				gen_op_ld_v(s, dc, MO_32 | MO_SIGN, s->cpu_T0, s->cpu_A0);
				gen_op_mov_reg_v(s, (TCGMemOp)d_ot, reg, s->cpu_T0);
			}
		}
		else
#endif
		{
			TCGLabel *label1;
			TCGv t0, t1, t2, a0;

			if (!dc->pe || dc->vm86)
				goto illegal_op;
			t0 = tcg_temp_local_new(s);
			t1 = tcg_temp_local_new(s);
			t2 = tcg_temp_local_new(s);
			ot = MO_16;
			modrm = cpu_ldub_code(env, dc->pc++);
			reg = (modrm >> 3) & 7;
			mod = (modrm >> 6) & 3;
			rm = modrm & 7;
			if (mod != 3) {
				gen_lea_modrm(s, env, dc, modrm);
				gen_op_ld_v(s, dc, ot, t0, s->cpu_A0);
				a0 = tcg_temp_local_new(s);
				tcg_gen_mov_tl(s, a0, s->cpu_A0);
			}
			else {
				gen_op_mov_v_reg(s, ot, t0, rm);
				TCGV_UNUSED(a0);
			}
			gen_op_mov_v_reg(s, ot, t1, reg);
			tcg_gen_andi_tl(s, s->cpu_tmp0, t0, 3);
			tcg_gen_andi_tl(s, t1, t1, 3);
			tcg_gen_movi_tl(s, t2, 0);
			label1 = gen_new_label(s);
			tcg_gen_brcond_tl(s, TCG_COND_GE, s->cpu_tmp0, t1, label1);
			tcg_gen_andi_tl(s, t0, t0, ~3);
			tcg_gen_or_tl(s, t0, t0, t1);
			tcg_gen_movi_tl(s, t2, CC_Z);
			gen_set_label(s, label1);
			if (mod != 3) {
				gen_op_st_v(s, dc, ot, t0, a0);
				tcg_temp_free(s, a0);
			}
			else {
				gen_op_mov_reg_v(s, ot, rm, t0);
			}

			TCGv zero = tcg_const_tl(s, 0);
			TCGv one = tcg_const_tl(s, 1);
			tcg_gen_movcond_tl(s, TCG_COND_NE, s->cpu_eflags_z, t2, zero, one, zero);
			tcg_gen_st_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
			tcg_temp_free(s, one);
			tcg_temp_free(s, zero);

			tcg_temp_free(s, t0);
			tcg_temp_free(s, t1);
			tcg_temp_free(s, t2);
		}
		break;
	case 0x102: /* lar */
	case 0x103: /* lsl */
	{
		TCGLabel *label1;
		TCGv t0;
		if (!dc->pe || dc->vm86)
			goto illegal_op;
		ot = dflag != MO_16 ? MO_32 : MO_16;
		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;
		gen_ldst_modrm(s, env, dc, modrm, MO_16, OR_TMP0, 0);
		t0 = tcg_temp_local_new(s);
		gen_update_cc_op(s, dc);
		if (b == 0x102) {
			gen_helper_lar(s, t0, s->cpu_env, s->cpu_T0);
		}
		else {
			gen_helper_lsl(s, t0, s->cpu_env, s->cpu_T0);
		}
		tcg_gen_ld_tl(s, s->cpu_eflags_z, s->cpu_env, offsetof(CPUX86State, zf));
		label1 = gen_new_label(s);
		tcg_gen_brcondi_tl(s, TCG_COND_EQ, s->cpu_eflags_z, 0, label1);
		gen_op_mov_reg_v(s, ot, reg, t0);
		gen_set_label(s, label1);
		set_cc_op(s, dc, CC_OP_EFLAGS);
		tcg_temp_free(s, t0);
	}
	break;
	case 0x118:
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		op = (modrm >> 3) & 7;
		switch (op) {
		case 0: /* prefetchnta */
		case 1: /* prefetchnt0 */
		case 2: /* prefetchnt0 */
		case 3: /* prefetchnt0 */
			if (mod == 3)
				goto illegal_op;
			gen_nop_modrm(s, env, dc, modrm);
			/* nothing more to do */
			break;
		default: /* nop (multi byte) */
			gen_nop_modrm(s, env, dc, modrm);
			break;
		}
		break;
	case 0x11a:
		modrm = cpu_ldub_code(env, dc->pc++);
		if (dc->flags & HF_MPX_EN_MASK) {
			mod = (modrm >> 6) & 3;
			reg = ((modrm >> 3) & 7) | rex_r;
			if (prefixes & PREFIX_REPZ) {
				/* bndcl */
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16) {
					goto illegal_op;
				}
				gen_bndck(s, env, dc, modrm, TCG_COND_LTU, s->cpu_bndl[reg]);
			}
			else if (prefixes & PREFIX_REPNZ) {
				/* bndcu */
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16) {
					goto illegal_op;
				}
				TCGv_i64 notu = tcg_temp_new_i64(s);
				tcg_gen_not_i64(s, notu, s->cpu_bndu[reg]);
				gen_bndck(s, env, dc, modrm, TCG_COND_GTU, notu);
				tcg_temp_free_i64(s, notu);
			}
			else if (prefixes & PREFIX_DATA) {
				/* bndmov -- from reg/mem */
				if (reg >= 4 || dc->aflag == MO_16) {
					goto illegal_op;
				}
				if (mod == 3) {
					int reg2 = (modrm & 7) | REX_B(dc);
					if (reg2 >= 4 || (prefixes & PREFIX_LOCK)) {
						goto illegal_op;
					}
					if (dc->flags & HF_MPX_IU_MASK) {
						tcg_gen_mov_i64(s, s->cpu_bndl[reg], s->cpu_bndl[reg2]);
						tcg_gen_mov_i64(s, s->cpu_bndu[reg], s->cpu_bndu[reg2]);
					}
				}
				else {
					gen_lea_modrm(s, env, dc, modrm);
					if (CODE64(dc)) {
						tcg_gen_qemu_ld_i64(s, s->cpu_bndl[reg], s->cpu_A0,
							dc->mem_index, MO_LEQ);
						tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_A0, 8);
						tcg_gen_qemu_ld_i64(s, s->cpu_bndu[reg], s->cpu_A0,
							dc->mem_index, MO_LEQ);
					}
					else {
						tcg_gen_qemu_ld_i64(s, s->cpu_bndl[reg], s->cpu_A0,
							dc->mem_index, MO_LEUL);
						tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_A0, 4);
						tcg_gen_qemu_ld_i64(s, s->cpu_bndu[reg], s->cpu_A0,
							dc->mem_index, MO_LEUL);
					}
					/* bnd registers are now in-use */
					gen_set_hflag(s, dc, HF_MPX_IU_MASK);
				}
			}
			else if (mod != 3) {
				/* bndldx */
				AddressParts a = gen_lea_modrm_0(env, dc, modrm);
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16
					|| a.base < -1) {
					goto illegal_op;
				}
				if (a.base >= 0) {
					tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_regs[a.base], a.disp);
				}
				else {
					tcg_gen_movi_tl(s, s->cpu_A0, 0);
				}
				gen_lea_v_seg(s, dc, dc->aflag, s->cpu_A0, a.def_seg, dc->override);
				if (a.index >= 0) {
					tcg_gen_mov_tl(s, s->cpu_T0, s->cpu_regs[a.index]);
				}
				else {
					tcg_gen_movi_tl(s, s->cpu_T0, 0);
				}
				if (CODE64(dc)) {
					gen_helper_bndldx64(s, s->cpu_bndl[reg], s->cpu_env, s->cpu_A0, s->cpu_T0);
					tcg_gen_ld_i64(s, s->cpu_bndu[reg], s->cpu_env,
						offsetof(CPUX86State, mmx_t0.MMX_Q(0)));
				}
				else {
					gen_helper_bndldx32(s, s->cpu_bndu[reg], s->cpu_env, s->cpu_A0, s->cpu_T0);
					tcg_gen_ext32u_i64(s, s->cpu_bndl[reg], s->cpu_bndu[reg]);
					tcg_gen_shri_i64(s, s->cpu_bndu[reg], s->cpu_bndu[reg], 32);
				}
				gen_set_hflag(s, dc, HF_MPX_IU_MASK);
			}
		}
		gen_nop_modrm(s, env, dc, modrm);
		break;
	case 0x11b:
		modrm = cpu_ldub_code(env, dc->pc++);
		if (dc->flags & HF_MPX_EN_MASK) {
			mod = (modrm >> 6) & 3;
			reg = ((modrm >> 3) & 7) | rex_r;
			if (mod != 3 && (prefixes & PREFIX_REPZ)) {
				/* bndmk */
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16) {
					goto illegal_op;
				}
				AddressParts a = gen_lea_modrm_0(env, dc, modrm);
				if (a.base >= 0) {
					tcg_gen_extu_tl_i64(s, s->cpu_bndl[reg], s->cpu_regs[a.base]);
					if (!CODE64(dc)) {
						tcg_gen_ext32u_i64(s, s->cpu_bndl[reg], s->cpu_bndl[reg]);
					}
				}
				else if (a.base == -1) {
					/* no base register has lower bound of 0 */
					tcg_gen_movi_i64(s, s->cpu_bndl[reg], 0);
				}
				else {
					/* rip-relative generates #ud */
					goto illegal_op;
				}
				tcg_gen_not_tl(s, s->cpu_A0, gen_lea_modrm_1(s, a));
				if (!CODE64(dc)) {
					tcg_gen_ext32u_tl(s, s->cpu_A0, s->cpu_A0);
				}
				tcg_gen_extu_tl_i64(s, s->cpu_bndu[reg], s->cpu_A0);
				/* bnd registers are now in-use */
				gen_set_hflag(s, dc, HF_MPX_IU_MASK);
				break;
			}
			else if (prefixes & PREFIX_REPNZ) {
				/* bndcn */
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16) {
					goto illegal_op;
				}
				gen_bndck(s, env, dc, modrm, TCG_COND_GTU, s->cpu_bndu[reg]);
			}
			else if (prefixes & PREFIX_DATA) {
				/* bndmov -- to reg/mem */
				if (reg >= 4 || dc->aflag == MO_16) {
					goto illegal_op;
				}
				if (mod == 3) {
					int reg2 = (modrm & 7) | REX_B(dc);
					if (reg2 >= 4 || (prefixes & PREFIX_LOCK)) {
						goto illegal_op;
					}
					if (dc->flags & HF_MPX_IU_MASK) {
						tcg_gen_mov_i64(s, s->cpu_bndl[reg2], s->cpu_bndl[reg]);
						tcg_gen_mov_i64(s, s->cpu_bndu[reg2], s->cpu_bndu[reg]);
					}
				}
				else {
					gen_lea_modrm(s, env, dc, modrm);
					if (CODE64(dc)) {
						tcg_gen_qemu_st_i64(s, s->cpu_bndl[reg], s->cpu_A0,
							dc->mem_index, MO_LEQ);
						tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_A0, 8);
						tcg_gen_qemu_st_i64(s, s->cpu_bndu[reg], s->cpu_A0,
							dc->mem_index, MO_LEQ);
					}
					else {
						tcg_gen_qemu_st_i64(s, s->cpu_bndl[reg], s->cpu_A0,
							dc->mem_index, MO_LEUL);
						tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_A0, 4);
						tcg_gen_qemu_st_i64(s, s->cpu_bndu[reg], s->cpu_A0,
							dc->mem_index, MO_LEUL);
					}
				}
			}
			else if (mod != 3) {
				/* bndstx */
				AddressParts a = gen_lea_modrm_0(env, dc, modrm);
				if (reg >= 4
					|| (prefixes & PREFIX_LOCK)
					|| dc->aflag == MO_16
					|| a.base < -1) {
					goto illegal_op;
				}
				if (a.base >= 0) {
					tcg_gen_addi_tl(s, s->cpu_A0, s->cpu_regs[a.base], a.disp);
				}
				else {
					tcg_gen_movi_tl(s, s->cpu_A0, 0);
				}
				gen_lea_v_seg(s, dc, dc->aflag, s->cpu_A0, a.def_seg, dc->override);
				if (a.index >= 0) {
					tcg_gen_mov_tl(s, s->cpu_T0, s->cpu_regs[a.index]);
				}
				else {
					tcg_gen_movi_tl(s, s->cpu_T0, 0);
				}
				if (CODE64(dc)) {
					gen_helper_bndstx64(s, s->cpu_env, s->cpu_A0, s->cpu_T0,
						s->cpu_bndl[reg], s->cpu_bndu[reg]);
				}
				else {
					gen_helper_bndstx32(s, s->cpu_env, s->cpu_A0, s->cpu_T0,
						s->cpu_bndl[reg], s->cpu_bndu[reg]);
				}
			}
		}
		gen_nop_modrm(s, env, dc, modrm);
		break;
	case 0x119: case 0x11c: case 0x11d: case 0x11e: case 0x11f: /* nop (multi byte) */
		modrm = cpu_ldub_code(env, dc->pc++);
		gen_nop_modrm(s, env, dc, modrm);
		break;
	case 0x120: /* mov reg, crN */
	case 0x122: /* mov crN, reg */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			modrm = cpu_ldub_code(env, dc->pc++);
			/* Ignore the mod bits (assume (modrm&0xc0)==0xc0).
			 * AMD documentation (24594.pdf) and testing of
			 * intel 386 and 486 processors all show that the mod bits
			 * are assumed to be 1's, regardless of actual values.
			 */
			rm = (modrm & 7) | REX_B(dc);
			reg = ((modrm >> 3) & 7) | rex_r;
			if (CODE64(dc))
				ot = MO_64;
			else
				ot = MO_32;
			if ((prefixes & PREFIX_LOCK) && (reg == 0) &&
				(dc->cpuid_ext3_features & CPUID_EXT3_CR8LEG)) {
				reg = 8;
			}
			switch (reg) {
			case 0:
			case 2:
			case 3:
			case 4:
			case 8:
				gen_update_cc_op(s, dc);
				gen_jmp_im(s, pc_start - dc->cs_base);
				if (b & 2) {
					if (dc->tb->cflags & CF_USE_ICOUNT) {
						gen_io_start(s);
					}
					gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
					gen_helper_write_crN(s, s->cpu_env, tcg_const_i32(s, reg),
						s->cpu_T0);
					if (dc->tb->cflags & CF_USE_ICOUNT) {
						gen_io_end(s);
					}
					gen_jmp_im(s, dc->pc - dc->cs_base);
					gen_eob(s, dc);
				}
				else {
					if (dc->tb->cflags & CF_USE_ICOUNT) {
						gen_io_start(s);
					}
					gen_helper_read_crN(s, s->cpu_T0, s->cpu_env, tcg_const_i32(s, reg));
					gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
					if (dc->tb->cflags & CF_USE_ICOUNT) {
						gen_io_end(s);
					}
				}
				break;
			default:
				goto unknown_op;
			}
		}
		break;
	case 0x121: /* mov reg, drN */
	case 0x123: /* mov drN, reg */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			modrm = cpu_ldub_code(env, dc->pc++);
			/* Ignore the mod bits (assume (modrm&0xc0)==0xc0).
			 * AMD documentation (24594.pdf) and testing of
			 * intel 386 and 486 processors all show that the mod bits
			 * are assumed to be 1's, regardless of actual values.
			 */
			rm = (modrm & 7) | REX_B(dc);
			reg = ((modrm >> 3) & 7) | rex_r;
			if (CODE64(dc))
				ot = MO_64;
			else
				ot = MO_32;
			if (reg >= 8) {
				goto illegal_op;
			}
			if (b & 2) {
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_WRITE_DR0 + reg);
				gen_op_mov_v_reg(s, ot, s->cpu_T0, rm);
				tcg_gen_movi_i32(s, s->cpu_tmp2_i32, reg);
				gen_helper_set_dr(s, s->cpu_env, s->cpu_tmp2_i32, s->cpu_T0);
				gen_jmp_im(s, dc->pc - dc->cs_base);
				gen_eob(s, dc);
			}
			else {
				gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_READ_DR0 + reg);
				tcg_gen_movi_i32(s, s->cpu_tmp2_i32, reg);
				gen_helper_get_dr(s, s->cpu_T0, s->cpu_env, s->cpu_tmp2_i32);
				gen_op_mov_reg_v(s, ot, rm, s->cpu_T0);
			}
		}
		break;
	case 0x106: /* clts */
		if (dc->cpl != 0) {
			gen_exception(s, dc, EXCP0D_GPF, pc_start - dc->cs_base);
		}
		else {
			gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_WRITE_CR0);
			gen_helper_clts(s, s->cpu_env);
			/* abort block because static cpu state changed */
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
		}
		break;
		/* MMX/3DNow!/SSE/SSE2/SSE3/SSSE3/SSE4 support */
	case 0x1c3: /* MOVNTI reg, mem */
		if (!(dc->cpuid_features & CPUID_SSE2))
			goto illegal_op;
		ot = mo_64_32(dflag);
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		if (mod == 3)
			goto illegal_op;
		reg = ((modrm >> 3) & 7) | rex_r;
		/* generate a generic store */
		gen_ldst_modrm(s, env, dc, modrm, ot, reg, 1);
		break;
	case 0x1ae:
		modrm = cpu_ldub_code(env, dc->pc++);
		switch (modrm) {
			CASE_MODRM_MEM_OP(0) : /* fxsave */
				if (!(dc->cpuid_features & CPUID_FXSR)
					|| (prefixes & PREFIX_LOCK)) {
					goto illegal_op;
				}
			if ((dc->flags & HF_EM_MASK) || (dc->flags & HF_TS_MASK)) {
				gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
				break;
			}
			gen_lea_modrm(s, env, dc, modrm);
			gen_helper_fxsave(s, s->cpu_env, s->cpu_A0);
			break;

			CASE_MODRM_MEM_OP(1) : /* fxrstor */
				if (!(dc->cpuid_features & CPUID_FXSR)
					|| (prefixes & PREFIX_LOCK)) {
					goto illegal_op;
				}
			if ((dc->flags & HF_EM_MASK) || (dc->flags & HF_TS_MASK)) {
				gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
				break;
			}
			gen_lea_modrm(s, env, dc, modrm);
			gen_helper_fxrstor(s, s->cpu_env, s->cpu_A0);
			break;

			CASE_MODRM_MEM_OP(2) : /* ldmxcsr */
				if ((dc->flags & HF_EM_MASK) || !(dc->flags & HF_OSFXSR_MASK)) {
					goto illegal_op;
				}
			if (dc->flags & HF_TS_MASK) {
				gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
				break;
			}
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_qemu_ld_i32(s, s->cpu_tmp2_i32, s->cpu_A0, dc->mem_index, MO_LEUL);
			gen_helper_ldmxcsr(s, s->cpu_env, s->cpu_tmp2_i32);
			break;

			CASE_MODRM_MEM_OP(3) : /* stmxcsr */
				if ((dc->flags & HF_EM_MASK) || !(dc->flags & HF_OSFXSR_MASK)) {
					goto illegal_op;
				}
			if (dc->flags & HF_TS_MASK) {
				gen_exception(s, dc, EXCP07_PREX, pc_start - dc->cs_base);
				break;
			}
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_ld32u_tl(s, s->cpu_T0, s->cpu_env, offsetof(CPUX86State, mxcsr));
			gen_op_st_v(s, dc, MO_32, s->cpu_T0, s->cpu_A0);
			break;

			CASE_MODRM_MEM_OP(4) : /* xsave */
				if ((dc->cpuid_ext_features & CPUID_EXT_XSAVE) == 0
					|| (prefixes & (PREFIX_LOCK | PREFIX_DATA
						| PREFIX_REPZ | PREFIX_REPNZ))) {
					goto illegal_op;
				}
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_concat_tl_i64(s, s->cpu_tmp1_i64, s->cpu_regs[R_EAX],
				s->cpu_regs[R_EDX]);
			gen_helper_xsave(s, s->cpu_env, s->cpu_A0, s->cpu_tmp1_i64);
			break;

			CASE_MODRM_MEM_OP(5) : /* xrstor */
				if ((dc->cpuid_ext_features & CPUID_EXT_XSAVE) == 0
					|| (prefixes & (PREFIX_LOCK | PREFIX_DATA
						| PREFIX_REPZ | PREFIX_REPNZ))) {
					goto illegal_op;
				}
			gen_lea_modrm(s, env, dc, modrm);
			tcg_gen_concat_tl_i64(s, s->cpu_tmp1_i64, s->cpu_regs[R_EAX],
				s->cpu_regs[R_EDX]);
			gen_helper_xrstor(s, s->cpu_env, s->cpu_A0, s->cpu_tmp1_i64);
			/* XRSTOR is how MPX is enabled, which changes how
			   we translate.  Thus we need to end the TB.  */
			gen_update_cc_op(s, dc);
			gen_jmp_im(s, dc->pc - dc->cs_base);
			gen_eob(s, dc);
			break;

			CASE_MODRM_MEM_OP(6) : /* xsaveopt / clwb */
				if (prefixes & PREFIX_LOCK) {
					goto illegal_op;
				}
			if (prefixes & PREFIX_DATA) {
				/* clwb */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_CLWB)) {
					goto illegal_op;
				}
				gen_nop_modrm(s, env, dc, modrm);
			}
			else {
				/* xsaveopt */
				if ((dc->cpuid_ext_features & CPUID_EXT_XSAVE) == 0
					|| (dc->cpuid_xsave_features & CPUID_XSAVE_XSAVEOPT) == 0
					|| (prefixes & (PREFIX_REPZ | PREFIX_REPNZ))) {
					goto illegal_op;
				}
				gen_lea_modrm(s, env, dc, modrm);
				tcg_gen_concat_tl_i64(s, s->cpu_tmp1_i64, s->cpu_regs[R_EAX],
					s->cpu_regs[R_EDX]);
				gen_helper_xsaveopt(s, s->cpu_env, s->cpu_A0, s->cpu_tmp1_i64);
			}
			break;

			CASE_MODRM_MEM_OP(7) : /* clflush / clflushopt */
				if (prefixes & PREFIX_LOCK) {
					goto illegal_op;
				}
			if (prefixes & PREFIX_DATA) {
				/* clflushopt */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_CLFLUSHOPT)) {
					goto illegal_op;
				}
			}
			else {
				/* clflush */
				if ((dc->prefix & (PREFIX_REPZ | PREFIX_REPNZ))
					|| !(dc->cpuid_features & CPUID_CLFLUSH)) {
					goto illegal_op;
				}
			}
			gen_nop_modrm(s, env, dc, modrm);
			break;

		case 0xc0: case 0xc1: case 0xc2: case 0xc3: case 0xc4: case 0xc5: case 0xc6: case 0xc7: /* rdfsbase (f3 0f ae /0) */
		case 0xc8: // ... 0xc8: /* rdgsbase (f3 0f ae /1) */
		case 0xd0:case 0xd1:case 0xd2:case 0xd3:case 0xd4:case 0xd5:case 0xd6:case 0xd7: /* wrfsbase (f3 0f ae /2) */
		case 0xd8: // ... 0xd8: /* wrgsbase (f3 0f ae /3) */
			if (CODE64(dc)
				&& (prefixes & PREFIX_REPZ)
				&& !(prefixes & PREFIX_LOCK)
				&& (dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_FSGSBASE)) {
				TCGv base, treg, src, dst;

				/* Preserve hflags bits by testing CR4 at runtime.  */
				tcg_gen_movi_i32(s, s->cpu_tmp2_i32, CR4_FSGSBASE_MASK);
				gen_helper_cr4_testbit(s, s->cpu_env, s->cpu_tmp2_i32);

				base = s->cpu_seg_base[modrm & 8 ? R_GS : R_FS];
				treg = s->cpu_regs[(modrm & 7) | REX_B(dc)];

				if (modrm & 0x10) {
					/* wr*base */
					dst = base, src = treg;
				}
				else {
					/* rd*base */
					dst = treg, src = base;
				}

				if (dc->dflag == MO_32) {
					tcg_gen_ext32u_tl(s, dst, src);
				}
				else {
					tcg_gen_mov_tl(s, dst, src);
				}
				break;
			}
			goto unknown_op;

		case 0xf8: /* sfence / pcommit */
			if (prefixes & PREFIX_DATA) {
				/* pcommit */
				if (!(dc->cpuid_7_0_ebx_features & CPUID_7_0_EBX_PCOMMIT)
					|| (prefixes & PREFIX_LOCK)) {
					goto illegal_op;
				}
				break;
			}
			/* fallthru */
		case 0xf9:case 0xfa:case 0xfb:case 0xfc:case 0xfd:case 0xfe:case 0xff: /* sfence */
			if (!(dc->cpuid_features & CPUID_SSE)
				|| (prefixes & PREFIX_LOCK)) {
				goto illegal_op;
			}
			tcg_gen_mb(s, (TCGBar)(TCG_MO_ST_ST | TCG_BAR_SC));
			break;
		case 0xe8:case 0xe9:case 0xea:case 0xeb:case 0xec:case 0xed:case 0xee:case 0xef: /* lfence */
			if (!(dc->cpuid_features & CPUID_SSE)
				|| (prefixes & PREFIX_LOCK)) {
				goto illegal_op;
			}
			tcg_gen_mb(s, (TCGBar)(TCG_MO_LD_LD | TCG_BAR_SC));
			break;
		case 0xf0:case 0xf1:case 0xf2:case 0xf3:case 0xf4:case 0xf5:case 0xf6:case 0xf7: /* mfence */
			if (!(dc->cpuid_features & CPUID_SSE2)
				|| (prefixes & PREFIX_LOCK)) {
				goto illegal_op;
			}
			tcg_gen_mb(s, (TCGBar)(TCG_MO_ALL | TCG_BAR_SC));
			break;

		default:
			goto unknown_op;
		}
		break;

	case 0x10d: /* 3DNow! prefetch(w) */
		modrm = cpu_ldub_code(env, dc->pc++);
		mod = (modrm >> 6) & 3;
		if (mod == 3)
			goto illegal_op;
		gen_nop_modrm(s, env, dc, modrm);
		break;
	case 0x1aa: /* rsm */
		gen_svm_check_intercept(s, dc, pc_start, SVM_EXIT_RSM);
		if (!(dc->flags & HF_SMM_MASK))
			goto illegal_op;
		gen_update_cc_op(s, dc);
		gen_jmp_im(s, dc->pc - dc->cs_base);
		gen_helper_rsm(s, s->cpu_env);
		gen_eob(s, dc);
		break;
	case 0x1b8: /* SSE4.2 popcnt */
		if ((prefixes & (PREFIX_REPZ | PREFIX_LOCK | PREFIX_REPNZ)) !=
			PREFIX_REPZ)
			goto illegal_op;
		if (!(dc->cpuid_ext_features & CPUID_EXT_POPCNT))
			goto illegal_op;

		modrm = cpu_ldub_code(env, dc->pc++);
		reg = ((modrm >> 3) & 7) | rex_r;

		if (dc->prefix & PREFIX_DATA) {
			ot = MO_16;
		}
		else {
			ot = mo_64_32(dflag);
		}

		gen_ldst_modrm(s, env, dc, modrm, ot, OR_TMP0, 0);
		gen_extu(s, ot, s->cpu_T0);
		tcg_gen_mov_tl(s, s->cpu_cc_src, s->cpu_T0);
		tcg_gen_ctpop_tl(s, s->cpu_T0, s->cpu_T0);
		gen_op_mov_reg_v(s, ot, reg, s->cpu_T0);

		set_cc_op(s, dc, CC_OP_POPCNT);
		break;
	case 0x10e: case 0x10f:
		/* 3DNow! instructions, ignore prefixes */
		dc->prefix &= ~(PREFIX_REPZ | PREFIX_REPNZ | PREFIX_DATA);
	case 0x110: case 0x111: case 0x112: case 0x113: case 0x114: case 0x115: case 0x116: case 0x117:
	case 0x128: case 0x129: case 0x12a: case 0x12b: case 0x12c: case 0x12d: case 0x12e: case 0x12f:
	case 0x138: case 0x139: case 0x13a:
	case 0x150:case 0x151:case 0x152:case 0x153:case 0x154:case 0x155:case 0x156:case 0x157:case 0x158:case 0x159:case 0x15a:case 0x15b:case 0x15c:case 0x15d:case 0x15e:case 0x15f:
	case 0x160:case 0x161:case 0x162:case 0x163:case 0x164:case 0x165:case 0x166:case 0x167:case 0x168:case 0x169:case 0x16a:case 0x16b:case 0x16c:case 0x16d:case 0x16e:case 0x16f:
	case 0x170:case 0x171:case 0x172:case 0x173:case 0x174:case 0x175:case 0x176:case 0x177:case 0x178:case 0x179:
	case 0x17c: case 0x17d: case 0x17e: case 0x17f:
	case 0x1c2:
	case 0x1c4: case 0x1c5: case 0x1c6:
	case 0x1d0:case 0x1d1:case 0x1d2:case 0x1d3:case 0x1d4:case 0x1d5:case 0x1d6:case 0x1d7:case 0x1d8:case 0x1d9:case 0x1da:case 0x1db:case 0x1dc:case 0x1dd:case 0x1de:case 0x1df:
	case 0x1e0:case 0x1e1:case 0x1e2:case 0x1e3:case 0x1e4:case 0x1e5:case 0x1e6:case 0x1e7:case 0x1e8:case 0x1e9:case 0x1ea:case 0x1eb:case 0x1ec:case 0x1ed:case 0x1ee:case 0x1ef:
	case 0x1f0:case 0x1f1:case 0x1f2:case 0x1f3:case 0x1f4:case 0x1f5:case 0x1f6:case 0x1f7:case 0x1f8:case 0x1f9:case 0x1fa:case 0x1fb:case 0x1fc:case 0x1fd:case 0x1fe:
		gen_sse(s, env, dc, b, pc_start, rex_r);
		break;
	default:
		goto unknown_op;
	}
	return dc->pc;
illegal_op:
	gen_illegal_opcode(s, dc);
	return dc->pc;
unknown_op:
	gen_unknown_opcode(s, env, dc);
	return dc->pc;
}

void i386_translate_init(TCGContext * s)
{
	static const char reg_names[CPU_NB_REGS][4] = {
#ifdef TARGET_X86_64
		"rax","rcx", "rdx", "rbx", "rsp", "rbp", "rsi", "rdi",
		"r8", "r9","r10","r11", "r12","r13","r14","r15",
#else
		 "eax","ecx", "edx", "ebx", "esp", "ebp", "esi", "edi",
#endif
	};

	static const char st_names[8][4] = {
		"st0", "st1", "st2", "st3", "st4", "st5", "st6", "st7"
	};

	static const char mm_names[8][4] = {
		"mm0", "mm1", "mm2", "mm3", "mm4", "mm5", "mm6", "mm7"
	};

	static const char zmm_names[CPU_NB_REGS == 8 ? 8 : 32][8] = {
		"zmm0",  "zmm1",  "zmm2",  "zmm3",  "zmm4",  "zmm5",  "zmm6",  "zmm7",
#ifdef TARGET_X86_64
		"zmm8",  "zmm9",  "zmm10", "zmm11", "zmm12", "zmm13", "zmm14", "zmm15",
		"zmm16", "zmm17", "zmm18", "zmm19", "zmm20", "zmm21", "zmm22", "zmm23",
		"zmm24", "zmm25", "zmm26", "zmm27", "zmm28", "zmm29", "zmm30", "zmm31",
#endif
	};

	static const char seg_base_names[6][8] = {
		"es_base","cs_base","ss_base","ds_base","fs_base","gs_base"
	};

	static const char bnd_regl_names[4][8] = {
		"bnd0_lb", "bnd1_lb", "bnd2_lb", "bnd3_lb"
	};

	static const char bnd_regu_names[4][8] = {
		"bnd0_ub", "bnd1_ub", "bnd2_ub", "bnd3_ub"
	};

	int i;

	s->cpu_env = tcg_global_reg_new_ptr(s, TCG_AREG0, "env");

	for (i = 0; i < CPU_NB_REGS; ++i) {
		s->cpu_regs[i] = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, regs[i]), reg_names[i]);
	}
	
	s->cpu_eip_reg = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, eip), 
#ifdef TARGET_X86_64
		"rip"
#else
		"eip"
#endif
	);

	for (i = 0; i < 8; ++i) {
		// XXX: we need to model these as 80-bit registers, not 64-bit registers.
		s->cpu_st_regs[i] = tcg_global_mem_new_i64(s, s->cpu_env, offsetof(CPUX86State, fpregs[i].d), st_names[i]);
	}

	for (i = 0; i < 8; ++i) {
		s->cpu_mm_regs[i] = tcg_global_mem_new_i64(s, s->cpu_env, offsetof(CPUX86State, fpregs[i].mmx), mm_names[i]);
	}

	for (i = 0; i < (CPU_NB_REGS == 8 ? 8 : 32); ++i) {
		// XXX: we need to model these as 512-bit registers, not 64-bit registers.
		s->cpu_zmm_regs[i] = tcg_global_mem_new_i64(s, s->cpu_env, offsetof(CPUX86State, xmm_regs[i]), zmm_names[i]);
	}

	for (i = 0; i < 6; ++i) {
		s->cpu_seg_base[i] = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, segs[i].base), seg_base_names[i]);
	}

	for (i = 0; i < 4; ++i) {
		s->cpu_bndl[i] = tcg_global_mem_new_i64(s, s->cpu_env, offsetof(CPUX86State, bnd_regs[i].lb), bnd_regl_names[i]);
		s->cpu_bndu[i] = tcg_global_mem_new_i64(s, s->cpu_env, offsetof(CPUX86State, bnd_regs[i].ub), bnd_regu_names[i]);
	}

	s->cpu_eflags_c = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, cf), "CF");
	s->cpu_eflags_p = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, pf), "PF");
	s->cpu_eflags_a = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, af), "AF");
	s->cpu_eflags_z = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, zf), "ZF");
	s->cpu_eflags_s = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, sf), "SF");
	s->cpu_eflags_o = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, of), "OF");
	s->cpu_eflags_d = tcg_global_mem_new_i32(s, s->cpu_env, offsetof(CPUX86State, df), "DF");

	s->cpu_cc_dst = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, cc_dst), "cc_dst");
	s->cpu_cc_src = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, cc_src), "cc_src");
	s->cpu_cc_src2 = tcg_global_mem_new(s, s->cpu_env, offsetof(CPUX86State, cc_src2), "cc_src2");
}

/* generate intermediate code for basic block 'tb'.  */
void gen_intermediate_code_i386(TCGContext * s, CPUX86State *cs, TranslationBlock *tb)
{
	CPUX86State *env = cs;
	DisasContext dc1, *dc = &dc1;
	target_ulong pc_ptr;
	uint32_t flags;
	target_ulong pc_start;
	target_ulong cs_base;
	int num_insns;
	int max_insns;

	/* generate intermediate code */
	pc_start = tb->pc;
	cs_base = tb->cs_base;
	flags = tb->flags;

	dc->pe = (flags >> HF_PE_SHIFT) & 1;
	dc->code32 = (flags >> HF_CS32_SHIFT) & 1;
	dc->ss32 = (flags >> HF_SS32_SHIFT) & 1;
	dc->addseg = (flags >> HF_ADDSEG_SHIFT) & 1;
	dc->f_st = 0;
	dc->vm86 = (flags >> VM_SHIFT) & 1;
	dc->cpl = (flags >> HF_CPL_SHIFT) & 3;
	dc->iopl = (flags >> IOPL_SHIFT) & 3;
	dc->tf = (flags >> TF_SHIFT) & 1;
	dc->singlestep_enabled = cs->singlestep_enabled;
	dc->cc_op = CC_OP_EFLAGS;
	dc->cs_base = cs_base;
	dc->tb = tb;
	dc->popl_esp_hack = 0;
	/* select memory access functions */
	dc->mem_index = 0;
#ifdef CONFIG_SOFTMMU
	dc->mem_index = cpu_mmu_index(env, false);
#endif
	dc->cpuid_features = env->features[FEAT_1_EDX];
	dc->cpuid_ext_features = env->features[FEAT_1_ECX];
	dc->cpuid_ext2_features = env->features[FEAT_8000_0001_EDX];
	dc->cpuid_ext3_features = env->features[FEAT_8000_0001_ECX];
	dc->cpuid_7_0_ebx_features = env->features[FEAT_7_0_EBX];
	dc->cpuid_xsave_features = env->features[FEAT_XSAVE];
#ifdef TARGET_X86_64
	dc->lma = (flags >> HF_LMA_SHIFT) & 1;
	dc->code64 = (flags >> HF_CS64_SHIFT) & 1;
#endif
	dc->flags = flags;
	dc->jmp_opt = !(dc->tf || cs->singlestep_enabled ||
		(flags & HF_INHIBIT_IRQ_MASK));
	/* Do not optimize repz jumps at all in icount mode, because
	   rep movsS instructions are execured with different paths
	   in !repz_opt and repz_opt modes. The first one was used
	   always except single step mode. And this setting
	   disables jumps optimization and control paths become
	   equivalent in run and single step modes.
	   Now there will be no jump optimization for repz in
	   record/replay modes and there will always be an
	   additional step for ecx=0 when icount is enabled.
	 */
	dc->repz_opt = !dc->jmp_opt && !(tb->cflags & CF_USE_ICOUNT);
#if 0
	/* check addseg logic */
	if (!dc->addseg && (dc->vm86 || !dc->pe || !dc->code32))
		printf("ERROR addseg\n");
#endif

	s->cpu_T0 = tcg_temp_new(s);
	s->cpu_T1 = tcg_temp_new(s);
	s->cpu_A0 = tcg_temp_new(s);

	s->cpu_tmp0 = tcg_temp_new(s);
	s->cpu_tmp1_i64 = tcg_temp_new_i64(s);
	s->cpu_tmp2_i32 = tcg_temp_new_i32(s);
	s->cpu_tmp3_i32 = tcg_temp_new_i32(s);
	s->cpu_tmp4 = tcg_temp_new(s);
	s->cpu_ptr0 = tcg_temp_new_ptr(s);
	s->cpu_ptr1 = tcg_temp_new_ptr(s);
	s->cpu_cc_srcT = tcg_temp_local_new(s);

	dc->is_jmp = DISAS_NEXT;
	pc_ptr = pc_start;
	num_insns = 0;
	max_insns = tb->cflags & CF_COUNT_MASK;
	if (max_insns == 0) {
		max_insns = CF_COUNT_MASK;
	}
	if (max_insns > TCG_MAX_INSNS) {
		max_insns = TCG_MAX_INSNS;
	}

	gen_tb_start(s, tb);
	for (;;) {
		tcg_gen_insn_start(s, pc_ptr, dc->cc_op);
		num_insns++;
#if 0
		/* If RF is set, suppress an internally generated breakpoint.  */
		if (unlikely(cpu_breakpoint_test(cs, pc_ptr,
			tb->flags & HF_RF_MASK
			? BP_GDB : BP_ANY))) {
			gen_debug(s, dc, pc_ptr - dc->cs_base);
			/* The address covered by the breakpoint must be included in
			   [tb->pc, tb->pc + tb->size) in order to for it to be
			   properly cleared -- thus we increment the PC here so that
			   the logic setting tb->size below does the right thing.  */
			pc_ptr += 1;
			goto done_generating;
		}
#endif
		if (num_insns == max_insns && (tb->cflags & CF_LAST_IO)) {
			gen_io_start(s);
		}

		pc_ptr = disas_insn(s, env, dc, pc_ptr);
		/* stop translation if indicated */
		if (dc->is_jmp)
			break;
		/* if single step mode, we generate only one instruction and
		   generate an exception */
		   /* if irq were inhibited with HF_INHIBIT_IRQ_MASK, we clear
			  the flag and abort the translation to give the irqs a
			  change to be happen */
		if (dc->tf || dc->singlestep_enabled ||
			(flags & HF_INHIBIT_IRQ_MASK)) {
			gen_jmp_im(s, pc_ptr - dc->cs_base);
			gen_eob(s, dc);
			break;
		}
		/* Do not cross the boundary of the pages in icount mode,
		   it can cause an exception. Do it only when boundary is
		   crossed by the first instruction in the block.
		   If current instruction already crossed the bound - it's ok,
		   because an exception hasn't stopped this code.
		 */
		/*if ((tb->cflags & CF_USE_ICOUNT)
			&& ((pc_ptr & TARGET_PAGE_MASK)
				!= ((pc_ptr + TARGET_MAX_INSN_SIZE - 1) & TARGET_PAGE_MASK)
				|| (pc_ptr & ~TARGET_PAGE_MASK) == 0)) {
			gen_jmp_im(s, pc_ptr - dc->cs_base);
			gen_eob(s, dc);
			break;
		}*/
		/* if too long translation, stop generation too */
		if (tcg_op_buf_full(s) ||
			(pc_ptr - pc_start) >= (TARGET_PAGE_SIZE - 32) ||
			num_insns >= max_insns) {
			gen_jmp_im(s, pc_ptr - dc->cs_base);
			gen_eob(s, dc);
			break;
		}
		/*if (singlestep) {
			gen_jmp_im(s,pc_ptr - dc->cs_base);
			gen_eob(s,dc);
			break;
		}*/
	}
	if (tb->cflags & CF_LAST_IO)
		gen_io_end(s);
done_generating:
	gen_tb_end(s, tb, num_insns);

#ifdef DEBUG_DISAS
	if (qemu_loglevel_mask(CPU_LOG_TB_IN_ASM)
		&& qemu_log_in_addr_range(pc_start)) {
		int disas_flags;
		qemu_log_lock();
		qemu_log("----------------\n");
		qemu_log("IN: %s\n", lookup_symbol(pc_start));
#ifdef TARGET_X86_64
		if (dc->code64)
			disas_flags = 2;
		else
#endif
			disas_flags = !dc->code32;
		log_target_disas(cs, pc_start, pc_ptr - pc_start, disas_flags);
		qemu_log("\n");
		qemu_log_unlock();
	}
#endif

	tb->size = pc_ptr - pc_start;
	tb->icount = num_insns;
}

void restore_state_to_opc(CPUX86State *env, TranslationBlock *tb,
	target_ulong *data)
{
	env->eip = data[0] - tb->cs_base;
	env->cc_op = data[1];
}
