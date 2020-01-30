/*
 * Tiny Code Generator for QEMU
 *
 * Copyright (c) 2008 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef TCG_H
#define TCG_H

#include "lib/tcg/tcg-common.h"
#include "lib/internal.h"

#if TCG_TARGET_REG_BITS == 32
typedef int32_t tcg_target_long;
typedef uint32_t tcg_target_ulong;
#elif TCG_TARGET_REG_BITS == 64
typedef int64_t tcg_target_long;
typedef uint64_t tcg_target_ulong;
#else
#error unsupported
#endif

typedef tcg_target_ulong TCGArg;

 /* XXX: make safe guess about sizes */
#define MAX_OP_PER_INSTR 266

#if HOST_LONG_BITS == 32
#define MAX_OPC_PARAM_PER_ARG 2
#else
#define MAX_OPC_PARAM_PER_ARG 1
#endif
#define MAX_OPC_PARAM_IARGS 5
#define MAX_OPC_PARAM_OARGS 1
#define MAX_OPC_PARAM_ARGS (MAX_OPC_PARAM_IARGS + MAX_OPC_PARAM_OARGS)

/* A Call op needs up to 4 + 2N parameters on 32-bit archs,
 * and up to 4 + N parameters on 64-bit archs
 * (N = number of input arguments + output arguments).  */
#define MAX_OPC_PARAM (4 + (MAX_OPC_PARAM_PER_ARG * MAX_OPC_PARAM_ARGS))
#define OPC_BUF_SIZE 640
#define OPC_MAX_SIZE (OPC_BUF_SIZE - MAX_OP_PER_INSTR)

#define OPPARAM_BUF_SIZE (OPC_BUF_SIZE * MAX_OPC_PARAM)

#define CPU_TEMP_BUF_NLONGS 128

 /* Default target word size to pointer size.  */
#ifndef TCG_TARGET_REG_BITS
# if UINTPTR_MAX == UINT32_MAX
#  define TCG_TARGET_REG_BITS 32
# elif UINTPTR_MAX == UINT64_MAX
#  define TCG_TARGET_REG_BITS 64
# else
#  error Unknown pointer size for tcg target
# endif
#endif

#if TCG_TARGET_REG_BITS == 32
#define TCG_PRIlx PRIx32
#define TCG_PRIld PRId32
#elif TCG_TARGET_REG_BITS == 64
#define TCG_PRIlx PRIx64
#define TCG_PRIld PRId64
#else
#error unsupported
#endif

/* Oversized TCG guests make things like MTTCG hard
 * as we can't use atomics for cputlb updates.
 */
#if TARGET_LONG_BITS > TCG_TARGET_REG_BITS
#define TCG_OVERSIZED_GUEST 1
#else
#define TCG_OVERSIZED_GUEST 0
#endif

#if TCG_TARGET_NB_REGS <= 32
typedef uint32_t TCGRegSet;
#elif TCG_TARGET_NB_REGS <= 64
typedef uint64_t TCGRegSet;
#else
#error unsupported
#endif

#ifndef TCG_TARGET_deposit_i32_valid
#define TCG_TARGET_deposit_i32_valid(ofs, len) 1
#endif
#ifndef TCG_TARGET_deposit_i64_valid
#define TCG_TARGET_deposit_i64_valid(ofs, len) 1
#endif
#ifndef TCG_TARGET_extract_i32_valid
#define TCG_TARGET_extract_i32_valid(ofs, len) 1
#endif
#ifndef TCG_TARGET_extract_i64_valid
#define TCG_TARGET_extract_i64_valid(ofs, len) 1
#endif

 /* Only one of DIV or DIV2 should be defined.  */
#if defined(TCG_TARGET_HAS_div_i32)
#define TCG_TARGET_HAS_div2_i32         0
#elif defined(TCG_TARGET_HAS_div2_i32)
#define TCG_TARGET_HAS_div_i32          0
#define TCG_TARGET_HAS_rem_i32          0
#endif
#if defined(TCG_TARGET_HAS_div_i64)
#define TCG_TARGET_HAS_div2_i64         0
#elif defined(TCG_TARGET_HAS_div2_i64)
#define TCG_TARGET_HAS_div_i64          0
#define TCG_TARGET_HAS_rem_i64          0
#endif

/* For 32-bit targets, some sort of unsigned widening multiply is required.  */
#if TCG_TARGET_REG_BITS == 32 \
    && !(defined(TCG_TARGET_HAS_mulu2_i32) \
         || defined(TCG_TARGET_HAS_muluh_i32))
# error "Missing unsigned widening multiply"
#endif

#ifndef TARGET_INSN_START_EXTRA_WORDS
# define TARGET_INSN_START_WORDS 1
#else
# define TARGET_INSN_START_WORDS (1 + TARGET_INSN_START_EXTRA_WORDS)
#endif

#define tcg_regset_clear(d) (d) = 0
#define tcg_regset_set(d, s) (d) = (s)
#define tcg_regset_set32(d, reg, val32) (d) |= (val32) << (reg)
#define tcg_regset_set_reg(d, r) (d) |= 1L << (r)
#define tcg_regset_reset_reg(d, r) (d) &= ~(1L << (r))
#define tcg_regset_test_reg(d, r) (((d) >> (r)) & 1)
#define tcg_regset_or(d, a, b) (d) = (a) | (b)
#define tcg_regset_and(d, a, b) (d) = (a) & (b)
#define tcg_regset_andnot(d, a, b) (d) = (a) & ~(b)
#define tcg_regset_not(d, a) (d) = ~(a)

#ifndef TCG_TARGET_INSN_UNIT_SIZE
# error "Missing TCG_TARGET_INSN_UNIT_SIZE"
#elif TCG_TARGET_INSN_UNIT_SIZE == 1
typedef uint8_t tcg_insn_unit;
#elif TCG_TARGET_INSN_UNIT_SIZE == 2
typedef uint16_t tcg_insn_unit;
#elif TCG_TARGET_INSN_UNIT_SIZE == 4
typedef uint32_t tcg_insn_unit;
#elif TCG_TARGET_INSN_UNIT_SIZE == 8
typedef uint64_t tcg_insn_unit;
#else
/* The port better have done this.  */
#endif


#if defined CONFIG_DEBUG_TCG || defined QEMU_STATIC_ANALYSIS
# define tcg_debug_assert(X) do { assert(X); } while (0)
#elif QEMU_GNUC_PREREQ(4, 5)
# define tcg_debug_assert(X) \
    do { if (!(X)) { __builtin_unreachable(); } } while (0)
#else
# define tcg_debug_assert(X) do { (void)(X); } while (0)
#endif

typedef struct TCGLabel {
	unsigned has_value : 1;
	unsigned id : 31;
	union {
		uintptr_t value;
		tcg_insn_unit *value_ptr;
	} u;
} TCGLabel;

typedef struct TCGPool {
	struct TCGPool *next;
	int size;
	uint8_t data[0] __attribute__((aligned));
} TCGPool;

#define TCG_POOL_CHUNK_SIZE 32768

#define TCG_MAX_TEMPS 512
#define TCG_MAX_INSNS 512

/* when the size of the arguments of a called function is smaller than
   this value, they are statically allocated in the TB stack frame */
#define TCG_STATIC_CALL_ARGS_SIZE 128

typedef enum TCGType {
	TCG_TYPE_I32,
	TCG_TYPE_I64,
	TCG_TYPE_COUNT, /* number of different types */

	/* An alias for the size of the host register.  */
#if TCG_TARGET_REG_BITS == 32
	TCG_TYPE_REG = TCG_TYPE_I32,
#else
	TCG_TYPE_REG = TCG_TYPE_I64,
#endif

	/* An alias for the size of the native pointer.  */
#if UINTPTR_MAX == UINT32_MAX
	TCG_TYPE_PTR = TCG_TYPE_I32,
#else
	TCG_TYPE_PTR = TCG_TYPE_I64,
#endif

	/* An alias for the size of the target "long", aka register.  */
#if TARGET_LONG_BITS == 64
	TCG_TYPE_TL = TCG_TYPE_I64,
#else
	TCG_TYPE_TL = TCG_TYPE_I32,
#endif
} TCGType;

/**
 * get_alignment_bits
 * @memop: TCGMemOp value
 *
 * Extract the alignment size from the memop.
 */
static inline unsigned get_alignment_bits(TCGMemOp memop)
{
	unsigned a = memop & MO_AMASK;

	if (a == MO_UNALN) {
		/* No alignment required.  */
		a = 0;
	}
	else if (a == MO_ALIGN) {
		/* A natural alignment requirement.  */
		a = memop & MO_SIZE;
	}
	else {
		/* A specific alignment requirement.  */
		a = a >> MO_ASHIFT;
	}
#if defined(CONFIG_SOFTMMU)
	/* The requested alignment cannot overlap the TLB flags.  */
	tcg_debug_assert((TLB_FLAGS_MASK & ((1 << a) - 1)) == 0);
#endif
	return a;
}

/* Define type and accessor macros for TCG variables.

   TCG variables are the inputs and outputs of TCG ops, as described
   in tcg/README. Target CPU front-end code uses these types to deal
   with TCG variables as it emits TCG code via the tcg_gen_* functions.
   They come in several flavours:
	* TCGv_i32 : 32 bit integer type
	* TCGv_i64 : 64 bit integer type
	* TCGv_ptr : a host pointer type
	* TCGv : an integer type the same size as target_ulong
			 (an alias for either TCGv_i32 or TCGv_i64)
   The compiler's type checking will complain if you mix them
   up and pass the wrong sized TCGv to a function.

   Users of tcg_gen_* don't need to know about any of the internal
   details of these, and should treat them as opaque types.
   You won't be able to look inside them in a debugger either.

   Internal implementation details follow:

   Note that there is no definition of the structs TCGv_i32_d etc anywhere.
   This is deliberate, because the values we store in variables of type
   TCGv_i32 are not really pointers-to-structures. They're just small
   integers, but keeping them in pointer types like this means that the
   compiler will complain if you accidentally pass a TCGv_i32 to a
   function which takes a TCGv_i64, and so on. Only the internals of
   TCG need to care about the actual contents of the types, and they always
   box and unbox via the MAKE_TCGV_* and GET_TCGV_* functions.
   Converting to and from intptr_t rather than int reduces the number
   of sign-extension instructions that get implied on 64-bit hosts.  */

typedef struct TCGv_i32_d *TCGv_i32;
typedef struct TCGv_i64_d *TCGv_i64;
typedef struct TCGv_ptr_d *TCGv_ptr;
typedef TCGv_ptr TCGv_env;

#if TARGET_LONG_BITS == 32
#define TCGv TCGv_i32
#elif TARGET_LONG_BITS == 64
#define TCGv TCGv_i64
#else
#error Unhandled TARGET_LONG_BITS value
#endif

static inline TCGv_i32 QEMU_ARTIFICIAL MAKE_TCGV_I32(intptr_t i)
{
	return (TCGv_i32)i;
}

static inline TCGv_i64 QEMU_ARTIFICIAL MAKE_TCGV_I64(intptr_t i)
{
	return (TCGv_i64)i;
}

static inline TCGv_ptr QEMU_ARTIFICIAL MAKE_TCGV_PTR(intptr_t i)
{
	return (TCGv_ptr)i;
}

static inline intptr_t QEMU_ARTIFICIAL GET_TCGV_I32(TCGv_i32 t)
{
	return (intptr_t)t;
}

static inline intptr_t QEMU_ARTIFICIAL GET_TCGV_I64(TCGv_i64 t)
{
	return (intptr_t)t;
}

static inline intptr_t QEMU_ARTIFICIAL GET_TCGV_PTR(TCGv_ptr t)
{
	return (intptr_t)t;
}

#if TCG_TARGET_REG_BITS == 32
#define TCGV_LOW(t) MAKE_TCGV_I32(GET_TCGV_I64(t))
#define TCGV_HIGH(t) MAKE_TCGV_I32(GET_TCGV_I64(t) + 1)
#endif

#define TCGV_EQUAL_I32(a, b) (GET_TCGV_I32(a) == GET_TCGV_I32(b))
#define TCGV_EQUAL_I64(a, b) (GET_TCGV_I64(a) == GET_TCGV_I64(b))
#define TCGV_EQUAL_PTR(a, b) (GET_TCGV_PTR(a) == GET_TCGV_PTR(b))

/* Dummy definition to avoid compiler warnings.  */
#define TCGV_UNUSED_I32(x) x = MAKE_TCGV_I32(-1)
#define TCGV_UNUSED_I64(x) x = MAKE_TCGV_I64(-1)
#define TCGV_UNUSED_PTR(x) x = MAKE_TCGV_PTR(-1)

#define TCGV_IS_UNUSED_I32(x) (GET_TCGV_I32(x) == -1)
#define TCGV_IS_UNUSED_I64(x) (GET_TCGV_I64(x) == -1)
#define TCGV_IS_UNUSED_PTR(x) (GET_TCGV_PTR(x) == -1)

/* call flags */

/* convenience version of most used call flags */
#define TCG_CALL_NO_RWG         TCG_CALL_NO_READ_GLOBALS
#define TCG_CALL_NO_WG          TCG_CALL_NO_WRITE_GLOBALS
#define TCG_CALL_NO_SE          TCG_CALL_NO_SIDE_EFFECTS
#define TCG_CALL_NO_RWG_SE      (TCG_CALL_NO_RWG | TCG_CALL_NO_SE)
#define TCG_CALL_NO_WG_SE       (TCG_CALL_NO_WG | TCG_CALL_NO_SE)

/* used to align parameters */
#define TCG_CALL_DUMMY_TCGV     MAKE_TCGV_I32(-1)
#define TCG_CALL_DUMMY_ARG      ((TCGArg)(-1))

/* Invert the sense of the comparison.  */
static inline TCGCond tcg_invert_cond(TCGCond c)
{
	return (TCGCond)(c ^ 1);
}

/* Swap the operands in a comparison.  */
static inline TCGCond tcg_swap_cond(TCGCond c)
{
	return c & 6 ? (TCGCond)(c ^ 9) : c;
}

/* Create an "unsigned" version of a "signed" comparison.  */
static inline TCGCond tcg_unsigned_cond(TCGCond c)
{
	return c & 2 ? (TCGCond)(c ^ 6) : c;
}

/* Must a comparison be considered unsigned?  */
static inline bool is_unsigned_cond(TCGCond c)
{
	return (c & 4) != 0;
}

/* Create a "high" version of a double-word comparison.
   This removes equality from a LTE or GTE comparison.  */
static inline TCGCond tcg_high_cond(TCGCond c)
{
	switch (c) {
	case TCG_COND_GE:
	case TCG_COND_LE:
	case TCG_COND_GEU:
	case TCG_COND_LEU:
		return (TCGCond)(c ^ 8);
	default:
		return c;
	}
}

typedef enum TCGTempVal {
	TEMP_VAL_DEAD,
	TEMP_VAL_REG,
	TEMP_VAL_MEM,
	TEMP_VAL_CONST,
} TCGTempVal;

typedef struct TCGTemp {
	TCGReg reg : 8;
	TCGTempVal val_type : 8;
	TCGType base_type : 8;
	TCGType type : 8;
	unsigned int fixed_reg : 1;
	unsigned int indirect_reg : 1;
	unsigned int indirect_base : 1;
	unsigned int mem_coherent : 1;
	unsigned int mem_allocated : 1;
	unsigned int temp_local : 1; /* If true, the temp is saved across
								  basic blocks. Otherwise, it is not
								  preserved across basic blocks. */
	unsigned int temp_allocated : 1; /* never used for code gen */

	tcg_target_long val;
	struct TCGTemp *mem_base;
	intptr_t mem_offset;
	const char *name;
} TCGTemp;

typedef struct TCGContext TCGContext;

typedef struct TCGTempSet {
	unsigned long l[BITS_TO_LONGS(TCG_MAX_TEMPS)];
} TCGTempSet;

/* While we limit helpers to 6 arguments, for 32-bit hosts, with padding,
   this imples a max of 6*2 (64-bit in) + 2 (64-bit out) = 14 operands.
   There are never more than 2 outputs, which means that we can store all
   dead + sync data within 16 bits.  */
#define DEAD_ARG  4
#define SYNC_ARG  1
typedef uint16_t TCGLifeData;

/* The layout here is designed to avoid crossing of a 32-bit boundary.
   If we do so, gcc adds padding, expanding the size to 12.  */
typedef struct TCGOp {
	TCGOpcode opc : 8;        /*  8 */

	/* Index of the prev/next op, or 0 for the end of the list.  */
	unsigned prev : 10;       /* 18 */
	unsigned next : 10;       /* 28 */

	/* The number of out and in parameter for a call.  */
	unsigned calli : 4;        /* 32 */
	unsigned callo : 2;        /* 34 */

	/* Index of the arguments for this op, or 0 for zero-operand ops.  */
	unsigned args : 14;       /* 48 */

	/* Lifetime data of the operands.  */
	unsigned life : 16;       /* 64 */
} TCGOp;

/* Make sure operands fit in the bitfields above.  */
QEMU_BUILD_BUG_ON(NB_OPS > (1 << 8));
QEMU_BUILD_BUG_ON(OPC_BUF_SIZE > (1 << 10));
QEMU_BUILD_BUG_ON(OPPARAM_BUF_SIZE > (1 << 14));

/* Make sure that we don't overflow 64 bits without noticing.  */
QEMU_BUILD_BUG_ON(sizeof(TCGOp) > 8);

struct TCGContext {
	TCG_FUNC_MALLOC tcg_malloc;
	TCG_FUNC_FREE tcg_free;
	uint8_t *pool_cur, *pool_end;
	TCGPool *pool_first, *pool_current, *pool_first_large;
	int nb_labels;
	int nb_globals;
	int nb_temps;
	int nb_indirects;

	/* goto_tb support */
	tcg_insn_unit *code_buf;
	uint16_t *tb_jmp_reset_offset; /* tb->jmp_reset_offset */
	uint16_t *tb_jmp_insn_offset; /* tb->jmp_insn_offset if USE_DIRECT_JUMP */
	uintptr_t *tb_jmp_target_addr; /* tb->jmp_target_addr if !USE_DIRECT_JUMP */

	TCGRegSet reserved_regs;
	intptr_t current_frame_offset;
	intptr_t frame_start;
	intptr_t frame_end;
	TCGTemp *frame_temp;

	tcg_insn_unit *code_ptr;

#ifdef CONFIG_PROFILER
	/* profiling info */
	int64_t tb_count1;
	int64_t tb_count;
	int64_t op_count; /* total insn count */
	int op_count_max; /* max insn per TB */
	int64_t temp_count;
	int temp_count_max;
	int64_t del_op_count;
	int64_t code_in_len;
	int64_t code_out_len;
	int64_t search_out_len;
	int64_t interm_time;
	int64_t code_time;
	int64_t la_time;
	int64_t opt_time;
	int64_t restore_count;
	int64_t restore_time;
#endif

#ifdef CONFIG_DEBUG_TCG
	int temps_in_use;
	int goto_tb_issue_mask;
#endif

	int gen_next_op_idx;
	int gen_next_parm_idx;

	/* Code generation.  Note that we specifically do not use tcg_insn_unit
	   here, because there's too much arithmetic throughout that relies
	   on addition and subtraction working on bytes.  Rely on the GCC
	   extension that allows arithmetic on void*.  */
	void *code_gen_prologue;
	void *code_gen_epilogue;
	void *code_gen_buffer;
	size_t code_gen_buffer_size;
	void *code_gen_ptr;

	/* Threshold to flush the translated code buffer.  */
	void *code_gen_highwater;

	//TBContext tb_ctx;

	/* Track which vCPU triggers events */
	struct CPUState * cpu;                      /* *_trans */
	TCGv_env tcg_env;                   /* *_exec  */

	/* The TCGBackendData structure is private to tcg-target.inc.c.  */
	struct TCGBackendData *be;

	TCGTempSet free_temps[TCG_TYPE_COUNT * 2];
	TCGTemp temps[TCG_MAX_TEMPS]; /* globals first, temps after */

	/* Tells which temporary holds a given register.
	   It does not take into account fixed registers */
	TCGTemp *reg_to_temp[TCG_TARGET_NB_REGS];

	TCGOp gen_op_buf[OPC_BUF_SIZE];
	TCGArg gen_opparam_buf[OPPARAM_BUF_SIZE];

	uint16_t gen_insn_end_off[TCG_MAX_INSNS];
	target_ulong gen_insn_data[TCG_MAX_INSNS][TARGET_INSN_START_WORDS];

	bool parallel_cpus;

#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
	TCGv_env cpu_env;
	TCGv_i64 cpu_V0, cpu_V1, cpu_M0;
	TCGv_i32 cpu_R[16];
	TCGv_i32 cpu_CF, cpu_NF, cpu_VF, cpu_ZF;
	TCGv_i64 cpu_exclusive_addr;
	TCGv_i64 cpu_exclusive_val;
	TCGv_i32 cpu_F0s, cpu_F1s;
	TCGv_i64 cpu_F0d, cpu_F1d;
#endif
#if defined(TARGET_AARCH64)
	TCGv_i64 cpu_X[32];
	TCGv_i64 cpu_pc;
	TCGv_i64 cpu_exclusive_high;
	TCGv_i64 cpu_V[32];
#endif

#if defined(TARGET_X86) || defined(TARGET_X86_64)
	TCGv_env cpu_env;
	TCGv cpu_A0;
	TCGv cpu_cc_dst, cpu_cc_src, cpu_cc_src2, cpu_cc_srcT;
#if defined(TARGET_X86)
	TCGv cpu_regs[8];
#elif defined(TARGET_X86_64)
	TCGv cpu_regs[16];
#endif
	TCGv cpu_eip_reg;
	TCGv_i64 cpu_st_regs[8];
	TCGv_i64 cpu_mm_regs[8];
#if defined(TARGET_X86)
	TCGv_i64 cpu_zmm_regs[8];
#elif defined(TARGET_X86_64)
	TCGv_i64 cpu_zmm_regs[32];
#endif
	TCGv cpu_sp0_reg;
	TCGv cpu_seg_base[6];
	TCGv_i64 cpu_bndl[4];
	TCGv_i64 cpu_bndu[4];
	TCGv cpu_eflags_c;
	TCGv cpu_eflags_p;
	TCGv cpu_eflags_a;
	TCGv cpu_eflags_z;
	TCGv cpu_eflags_s;
	TCGv cpu_eflags_o;
	TCGv_i32 cpu_eflags_d;
	TCGv cpu_T0, cpu_T1;
	TCGv cpu_tmp0, cpu_tmp4;
	TCGv_ptr cpu_ptr0, cpu_ptr1;
	TCGv_i32 cpu_tmp2_i32, cpu_tmp3_i32;
	TCGv_i64 cpu_tmp1_i64;
#endif
#if defined(TARGET_X86_64)
	int x86_64_hregs;
#endif
};

static inline void tcg_set_insn_param(TCGContext *s, int op_idx, int arg, TCGArg v)
{
	int op_argi = s->gen_op_buf[op_idx].args;

	s->gen_opparam_buf[op_argi + arg] = v;
}

/* The number of opcodes emitted so far.  */
static inline int tcg_op_buf_count(TCGContext *s)
{
	return s->gen_next_op_idx;
}

/* Test for whether to terminate the TB for using too many opcodes.  */
static inline bool tcg_op_buf_full(TCGContext *s)
{
	return tcg_op_buf_count(s) >= OPC_MAX_SIZE;
}

/* pool based memory allocation */

/* tb_lock must be held for tcg_malloc_internal. */
void * tcg_malloc_internal(TCGContext *s, int size);

void tcg_pool_reset(TCGContext *s);

/* Called with tb_lock held.  */
static inline void * tcg_malloc(TCGContext *s, int size)
{
	uint8_t *ptr, *ptr_end;

	/* ??? This is a weak placeholder for minimum malloc alignment.  */
	size = QEMU_ALIGN_UP(size, 8);

	ptr = s->pool_cur;
	ptr_end = ptr + size;
	if (unlikely(ptr_end > s->pool_end)) {
		return tcg_malloc_internal(s, size);
	}
	else {
		s->pool_cur = ptr_end;
		return ptr;
	}
}

void tcg_context_init(TCGContext *s);

void tcg_context_reset(TCGContext *s);

void liveness_pass_1(TCGContext *s, uint8_t *temp_state);

bool liveness_pass_2(TCGContext *s, uint8_t *temp_state);

int tcg_global_mem_new_internal(TCGContext *s, TCGType, TCGv_ptr, intptr_t, const char *);

TCGv_i32 tcg_global_reg_new_i32(TCGContext *s, TCGReg reg, const char *name);

TCGv_i64 tcg_global_reg_new_i64(TCGContext *s, TCGReg reg, const char *name);

TCGv_i32 tcg_temp_new_internal_i32(TCGContext *s, int temp_local);

TCGv_i64 tcg_temp_new_internal_i64(TCGContext *s, int temp_local);

void tcg_temp_free_i32(TCGContext *s, TCGv_i32 arg);

void tcg_temp_free_i64(TCGContext *s, TCGv_i64 arg);

TCGRegType tcg_get_reg_type(TCGContext *s, int arg);

const char * tcg_find_helper(TCGContext *s, int helper_idx);

const void *tcg_find_helper_func(TCGContext *s, int helper_idx);

static inline TCGv_i32 tcg_global_mem_new_i32(TCGContext *s, TCGv_ptr reg, intptr_t offset, const char *name)
{
	int idx = tcg_global_mem_new_internal(s, TCG_TYPE_I32, reg, offset, name);

	return MAKE_TCGV_I32(idx);
}

static inline TCGv_i32 tcg_temp_new_i32(TCGContext *s)
{
	return tcg_temp_new_internal_i32(s, 0);
}

static inline TCGv_i32 tcg_temp_local_new_i32(TCGContext *s)
{
	return tcg_temp_new_internal_i32(s, 1);
}

static inline TCGv_i64 tcg_global_mem_new_i64(TCGContext *s, TCGv_ptr reg, intptr_t offset, const char *name)
{
	int idx = tcg_global_mem_new_internal(s, TCG_TYPE_I64, reg, offset, name);

	return MAKE_TCGV_I64(idx);
}

static inline TCGv_i64 tcg_temp_new_i64(TCGContext *s)
{
	return tcg_temp_new_internal_i64(s, 0);
}

static inline TCGv_i64 tcg_temp_local_new_i64(TCGContext *s)
{
	return tcg_temp_new_internal_i64(s, 1);
}

#define TCG_CT_ALIAS  0x80
#define TCG_CT_IALIAS 0x40
#define TCG_CT_NEWREG 0x20 /* output requires a new register */
#define TCG_CT_REG    0x01
#define TCG_CT_CONST  0x02 /* any constant of register size */

typedef struct TCGArgConstraint {
	uint16_t ct;
	uint8_t alias_index;
	union {
		TCGRegSet regs;
	} u;
} TCGArgConstraint;

typedef struct TCGOpDef {
	const char *name;
	uint8_t nb_oargs, nb_iargs, nb_cargs, nb_args;
	TCGOpFlags flags;
	TCGArgConstraint *args_ct;
	int *sorted_args;
#if defined(CONFIG_DEBUG_TCG)
	int used;
#endif
} TCGOpDef;

extern TCGOpDef tcg_op_defs[];

extern const size_t tcg_op_defs_max;

extern const char * const cond_name[];

extern const size_t cond_name_max;

extern const char * const ldst_name[];

extern const size_t ldst_name_max;

extern const char * const alignment_name[];

extern const size_t alignment_name_max;

typedef struct TCGTargetOpDef {
	TCGOpcode op;
	const char *args_ct_str[TCG_MAX_OP_ARGS];
} TCGTargetOpDef;

#define tcg_abort() \
do {\
    fprintf(stderr, "%s:%d: tcg fatal error\n", __FILE__, __LINE__);\
    abort();\
} while (0)

#if UINTPTR_MAX == UINT32_MAX
#define TCGV_NAT_TO_PTR(n) MAKE_TCGV_PTR(GET_TCGV_I32(n))
#define TCGV_PTR_TO_NAT(n) MAKE_TCGV_I32(GET_TCGV_PTR(n))

#define tcg_const_ptr(s, V) TCGV_NAT_TO_PTR(tcg_const_i32(s, (intptr_t)(V)))
#define tcg_global_reg_new_ptr(S, R, N) \
    TCGV_NAT_TO_PTR(tcg_global_reg_new_i32((S), (R), (N)))
#define tcg_global_mem_new_ptr(R, O, N) \
    TCGV_NAT_TO_PTR(tcg_global_mem_new_i32((R), (O), (N)))
#define tcg_temp_new_ptr() TCGV_NAT_TO_PTR(tcg_temp_new_i32(s))
#define tcg_temp_free_ptr(T) tcg_temp_free_i32(s,TCGV_PTR_TO_NAT(T))
#else
#define TCGV_NAT_TO_PTR(n) MAKE_TCGV_PTR(GET_TCGV_I64(n))
#define TCGV_PTR_TO_NAT(n) MAKE_TCGV_I64(GET_TCGV_PTR(n))

#define tcg_const_ptr(V) TCGV_NAT_TO_PTR(tcg_const_i64(s, (intptr_t)(V)))
#define tcg_global_reg_new_ptr(S, R, N) \
    TCGV_NAT_TO_PTR(tcg_global_reg_new_i64((S), (R), (N)))
#define tcg_global_mem_new_ptr(R, O, N) \
    TCGV_NAT_TO_PTR(tcg_global_mem_new_i64((R), (O), (N)))
#define tcg_temp_new_ptr() TCGV_NAT_TO_PTR(tcg_temp_new_i64(s))
#define tcg_temp_free_ptr(T) tcg_temp_free_i64(s, TCGV_PTR_TO_NAT(T))
#endif

void tcg_gen_callN(TCGContext *s, int helper_idx, TCGArg ret, int nargs, TCGArg *args);

void tcg_op_remove(TCGContext *s, TCGOp *op);

TCGOp * tcg_op_insert_before(TCGContext *s, TCGOp *op, TCGOpcode opc, int narg);

TCGOp * tcg_op_insert_after(TCGContext *s, TCGOp *op, TCGOpcode opc, int narg);

void tcg_optimize(TCGContext *s);

/* only used for debugging purposes */
void tcg_dump_ops(TCGContext *s);

TCGv_i32 tcg_const_i32(TCGContext *s, int32_t val);

TCGv_i64 tcg_const_i64(TCGContext *s, int64_t val);

TCGv_i32 tcg_const_local_i32(TCGContext *s, int32_t val);

TCGv_i64 tcg_const_local_i64(TCGContext *s, int64_t val);

TCGLabel * gen_new_label(TCGContext *s);

char * tcg_get_arg_str_idx(TCGContext *s, char *buf, int buf_size, int idx);

/**
 * label_arg
 * @l: label
 *
 * Encode a label for storage in the TCG opcode stream.
 */

static inline TCGArg label_arg(TCGLabel *l)
{
	return (uintptr_t)l;
}

/**
 * arg_label
 * @i: value
 *
 * The opposite of label_arg.  Retrieve a label from the
 * encoding of the TCG opcode stream.
 */

static inline TCGLabel *arg_label(TCGArg i)
{
	return (TCGLabel *)(uintptr_t)i;
}

/* Combine the TCGMemOp and mmu_idx parameters into a single value.  */
typedef uint32_t TCGMemOpIdx;

/**
 * make_memop_idx
 * @op: memory operation
 * @idx: mmu index
 *
 * Encode these values into a single parameter.
 */
static inline TCGMemOpIdx make_memop_idx(TCGMemOp op, unsigned idx)
{
	tcg_debug_assert(idx <= 15);
	return (op << 4) | idx;
}

/**
 * get_memop
 * @oi: combined op/idx parameter
 *
 * Extract the memory operation from the combined value.
 */
static inline TCGMemOp get_memop(TCGMemOpIdx oi)
{
	return (TCGMemOp)(oi >> 4);
}

/**
 * get_mmuidx
 * @oi: combined op/idx parameter
 *
 * Extract the mmu index from the combined value.
 */
static inline unsigned get_mmuidx(TCGMemOpIdx oi)
{
	return oi & 15;
}

/*
 * Memory helpers that will be used by TCG generated code.
 */
#ifdef CONFIG_SOFTMMU
 /* Value zero-extended to tcg register size.  */
tcg_target_ulong helper_ret_ldub_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_le_lduw_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_le_ldul_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_le_ldq_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_be_lduw_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_be_ldul_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_be_ldq_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);

/* Value sign-extended to tcg register size.  */
tcg_target_ulong helper_ret_ldsb_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_le_ldsw_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_le_ldsl_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_be_ldsw_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
tcg_target_ulong helper_be_ldsl_mmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);

void helper_ret_stb_mmu(CPUArchState *env, target_ulong addr, uint8_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_le_stw_mmu(CPUArchState *env, target_ulong addr, uint16_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_le_stl_mmu(CPUArchState *env, target_ulong addr, uint32_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_le_stq_mmu(CPUArchState *env, target_ulong addr, uint64_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_be_stw_mmu(CPUArchState *env, target_ulong addr, uint16_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_be_stl_mmu(CPUArchState *env, target_ulong addr, uint32_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);
void helper_be_stq_mmu(CPUArchState *env, target_ulong addr, uint64_t val,
	TCGMemOpIdx oi, uintptr_t retaddr);

uint8_t helper_ret_ldb_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint16_t helper_le_ldw_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_le_ldl_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_le_ldq_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint16_t helper_be_ldw_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_be_ldl_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_be_ldq_cmmu(CPUArchState *env, target_ulong addr,
	TCGMemOpIdx oi, uintptr_t retaddr);

/* Temporary aliases until backends are converted.  */
#ifdef TARGET_WORDS_BIGENDIAN
# define helper_ret_ldsw_mmu  helper_be_ldsw_mmu
# define helper_ret_lduw_mmu  helper_be_lduw_mmu
# define helper_ret_ldsl_mmu  helper_be_ldsl_mmu
# define helper_ret_ldul_mmu  helper_be_ldul_mmu
# define helper_ret_ldl_mmu   helper_be_ldul_mmu
# define helper_ret_ldq_mmu   helper_be_ldq_mmu
# define helper_ret_stw_mmu   helper_be_stw_mmu
# define helper_ret_stl_mmu   helper_be_stl_mmu
# define helper_ret_stq_mmu   helper_be_stq_mmu
# define helper_ret_ldw_cmmu  helper_be_ldw_cmmu
# define helper_ret_ldl_cmmu  helper_be_ldl_cmmu
# define helper_ret_ldq_cmmu  helper_be_ldq_cmmu
#else
# define helper_ret_ldsw_mmu  helper_le_ldsw_mmu
# define helper_ret_lduw_mmu  helper_le_lduw_mmu
# define helper_ret_ldsl_mmu  helper_le_ldsl_mmu
# define helper_ret_ldul_mmu  helper_le_ldul_mmu
# define helper_ret_ldl_mmu   helper_le_ldul_mmu
# define helper_ret_ldq_mmu   helper_le_ldq_mmu
# define helper_ret_stw_mmu   helper_le_stw_mmu
# define helper_ret_stl_mmu   helper_le_stl_mmu
# define helper_ret_stq_mmu   helper_le_stq_mmu
# define helper_ret_ldw_cmmu  helper_le_ldw_cmmu
# define helper_ret_ldl_cmmu  helper_le_ldl_cmmu
# define helper_ret_ldq_cmmu  helper_le_ldq_cmmu
#endif

uint32_t helper_atomic_cmpxchgb_mmu(CPUArchState *env, target_ulong addr,
	uint32_t cmpv, uint32_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_atomic_cmpxchgw_le_mmu(CPUArchState *env, target_ulong addr,
	uint32_t cmpv, uint32_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_atomic_cmpxchgl_le_mmu(CPUArchState *env, target_ulong addr,
	uint32_t cmpv, uint32_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_atomic_cmpxchgq_le_mmu(CPUArchState *env, target_ulong addr,
	uint64_t cmpv, uint64_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_atomic_cmpxchgw_be_mmu(CPUArchState *env, target_ulong addr,
	uint32_t cmpv, uint32_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint32_t helper_atomic_cmpxchgl_be_mmu(CPUArchState *env, target_ulong addr,
	uint32_t cmpv, uint32_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);
uint64_t helper_atomic_cmpxchgq_be_mmu(CPUArchState *env, target_ulong addr,
	uint64_t cmpv, uint64_t newv,
	TCGMemOpIdx oi, uintptr_t retaddr);

#define GEN_ATOMIC_HELPER(NAME, TYPE, SUFFIX)         \
TYPE helper_atomic_ ## NAME ## SUFFIX ## _mmu         \
    (CPUArchState *env, target_ulong addr, TYPE val,  \
     TCGMemOpIdx oi, uintptr_t retaddr);

#ifdef CONFIG_ATOMIC64
#define GEN_ATOMIC_HELPER_ALL(NAME)          \
    GEN_ATOMIC_HELPER(NAME, uint32_t, b)     \
    GEN_ATOMIC_HELPER(NAME, uint32_t, w_le)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, w_be)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, l_le)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, l_be)  \
    GEN_ATOMIC_HELPER(NAME, uint64_t, q_le)  \
    GEN_ATOMIC_HELPER(NAME, uint64_t, q_be)
#else
#define GEN_ATOMIC_HELPER_ALL(NAME)          \
    GEN_ATOMIC_HELPER(NAME, uint32_t, b)     \
    GEN_ATOMIC_HELPER(NAME, uint32_t, w_le)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, w_be)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, l_le)  \
    GEN_ATOMIC_HELPER(NAME, uint32_t, l_be)
#endif

GEN_ATOMIC_HELPER_ALL(fetch_add)
GEN_ATOMIC_HELPER_ALL(fetch_sub)
GEN_ATOMIC_HELPER_ALL(fetch_and)
GEN_ATOMIC_HELPER_ALL(fetch_or)
GEN_ATOMIC_HELPER_ALL(fetch_xor)

GEN_ATOMIC_HELPER_ALL(add_fetch)
GEN_ATOMIC_HELPER_ALL(sub_fetch)
GEN_ATOMIC_HELPER_ALL(and_fetch)
GEN_ATOMIC_HELPER_ALL(or_fetch)
GEN_ATOMIC_HELPER_ALL(xor_fetch)

GEN_ATOMIC_HELPER_ALL(xchg)

#undef GEN_ATOMIC_HELPER_ALL
#undef GEN_ATOMIC_HELPER
#endif /* CONFIG_SOFTMMU */

static void gen_tb_start(TCGContext *s, TranslationBlock *tb)
{

}

static void gen_tb_end(TCGContext *s, TranslationBlock *tb, int num_insns)
{
	s->gen_op_buf[s->gen_op_buf[0].prev].next = 0;
}

#endif /* TCG_H */
