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
#include "lib/internal.h"
#include "lib/tcg/tcg-op.h"
#include "lib/bitops.h"
#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
#include "lib/target/arm/cpu.h"
#endif

TCGLabel *gen_new_label(TCGContext *s)
{
	TCGLabel *l = (TCGLabel *)tcg_malloc(s, sizeof(TCGLabel));

	memset(l, 0, sizeof(TCGLabel));

	l->id = s->nb_labels++;

	return l;
}

/* pool based memory allocation */
void *tcg_malloc_internal(TCGContext *s, int size)
{
	TCGPool *p;
	int pool_size;

	if (size > TCG_POOL_CHUNK_SIZE) {
		/* big malloc: insert a new pool (XXX: could optimize) */
		p = (TCGPool*)s->tcg_malloc(sizeof(TCGPool) + size);
		p->size = size;
		p->next = s->pool_first_large;
		s->pool_first_large = p;
		return p->data;
	}
	else {
		p = s->pool_current;
		if (!p) {
			p = s->pool_first;
			if (!p)
				goto new_pool;
		}
		else {
			if (!p->next) {
			new_pool:
				pool_size = TCG_POOL_CHUNK_SIZE;
				p = (TCGPool *)s->tcg_malloc(sizeof(TCGPool) + pool_size);
				p->size = pool_size;
				p->next = NULL;
				if (s->pool_current)
					s->pool_current->next = p;
				else
					s->pool_first = p;
			}
			else {
				p = p->next;
			}
		}
	}
	s->pool_current = p;
	s->pool_cur = p->data + size;
	s->pool_end = p->data + p->size;
	return p->data;
}

void tcg_pool_reset(TCGContext *s)
{
	TCGPool *p, *t;
	for (p = s->pool_first_large; p; p = t) {
		t = p->next;
		s->tcg_free(p);
	}
	for (p = s->pool_first; p; p = t) {
		t = p->next;
		s->tcg_free(p);
	}
	s->pool_first_large = NULL;
	s->pool_first = NULL;
	s->pool_cur = s->pool_end = NULL;
	s->pool_current = NULL;
}

typedef struct TCGHelperInfo {
	void * func;
	const char * name;
	unsigned flags;
	unsigned sizemask;
} TCGHelperInfo;

#undef _HELPER_PROTO_H
#define DEF_HELPER_PROTO_IMPL
#include "lib/tcg/helper-proto.h"
#undef _HELPER_PROTO_H
#undef DEF_HELPER_PROTO_IMPL

static const TCGHelperInfo all_helpers[] = {
#include "lib/tcg/helper-tcg.h"
};

void tcg_context_init(TCGContext *s)
{

}

void tcg_context_reset(TCGContext *s)
{
	s->nb_temps = s->nb_globals;

	memset(s->free_temps, 0, sizeof(s->free_temps));

	s->nb_labels = 0;

	s->current_frame_offset = s->frame_start;

	s->gen_op_buf[0].next = 1;

	s->gen_op_buf[0].prev = 0;

	s->gen_next_op_idx = 1;

	s->gen_next_parm_idx = 0;
}

static inline int temp_idx(TCGContext *s, TCGTemp *ts)
{
	ptrdiff_t n = ts - s->temps;
	tcg_debug_assert(n >= 0 && n < s->nb_temps);
	return n;
}

static inline TCGTemp *tcg_temp_alloc(TCGContext *s)
{
	int n = s->nb_temps++;
	tcg_debug_assert(n < TCG_MAX_TEMPS);
	memset(&s->temps[n], 0, sizeof(TCGTemp));
	return &s->temps[n];
}

static inline TCGTemp *tcg_global_alloc(TCGContext *s)
{
	tcg_debug_assert(s->nb_globals == s->nb_temps);
	s->nb_globals++;
	return tcg_temp_alloc(s);
}

static int tcg_global_reg_new_internal(TCGContext *s, TCGType type, TCGReg reg, const char *name)
{
	TCGTemp *ts;

	if (TCG_TARGET_REG_BITS == 32 && type != TCG_TYPE_I32) {
		tcg_abort();
	}

	ts = tcg_global_alloc(s);
	ts->base_type = type;
	ts->type = type;
	ts->fixed_reg = 1;
	ts->reg = reg;
	ts->name = name;
	tcg_regset_set_reg(s->reserved_regs, reg);

	return temp_idx(s, ts);
}

TCGv_i32 tcg_global_reg_new_i32(TCGContext *s, TCGReg reg, const char *name)
{
	int idx;

	if (tcg_regset_test_reg(s->reserved_regs, reg)) {
		tcg_abort();
	}
	idx = tcg_global_reg_new_internal(s, TCG_TYPE_I32, reg, name);
	return MAKE_TCGV_I32(idx);
}

TCGv_i64 tcg_global_reg_new_i64(TCGContext *s, TCGReg reg, const char *name)
{
	int idx;

	if (tcg_regset_test_reg(s->reserved_regs, reg)) {
		tcg_abort();
	}
	idx = tcg_global_reg_new_internal(s, TCG_TYPE_I64, reg, name);
	return MAKE_TCGV_I64(idx);
}

char * tcg_strdup(TCGContext *s, const char * src)
{	
	if (src != nullptr)
	{
		size_t len = strlen(src) + 1;

		void * dst = s->tcg_malloc(len);

		if (dst != nullptr)
			return (char *)memcpy(dst, src, len);
	}

	return nullptr;
}

int tcg_global_mem_new_internal(TCGContext *s, TCGType type, TCGv_ptr base, intptr_t offset, const char *name)
{
	TCGTemp *base_ts = &s->temps[GET_TCGV_PTR(base)];
	TCGTemp *ts = tcg_global_alloc(s);
	int indirect_reg = 0, bigendian = 0;
#ifdef HOST_WORDS_BIGENDIAN
	bigendian = 1;
#endif

	if (!base_ts->fixed_reg) {
		/* We do not support double-indirect registers.  */
		tcg_debug_assert(!base_ts->indirect_reg);
		base_ts->indirect_base = 1;
		s->nb_indirects += (TCG_TARGET_REG_BITS == 32 && type == TCG_TYPE_I64
			? 2 : 1);
		indirect_reg = 1;
	}

	if (TCG_TARGET_REG_BITS == 32 && type == TCG_TYPE_I64) {
		TCGTemp *ts2 = tcg_global_alloc(s);
		char buf[64];

		ts->base_type = TCG_TYPE_I64;
		ts->type = TCG_TYPE_I32;
		ts->indirect_reg = indirect_reg;
		ts->mem_allocated = 1;
		ts->mem_base = base_ts;
		ts->mem_offset = offset + bigendian * 4;
		strncpy_s(buf, sizeof(buf), name, strlen(name));
		strncat_s(buf, sizeof(buf), "_0", 2);
		ts->name = tcg_strdup(s,buf);

		tcg_debug_assert(ts2 == ts + 1);
		ts2->base_type = TCG_TYPE_I64;
		ts2->type = TCG_TYPE_I32;
		ts2->indirect_reg = indirect_reg;
		ts2->mem_allocated = 1;
		ts2->mem_base = base_ts;
		ts2->mem_offset = offset + (1 - bigendian) * 4;
		strncpy_s(buf, sizeof(buf), name, strlen(name));
		strncat_s(buf, sizeof(buf), "_1", 2);
		ts2->name = tcg_strdup(s,buf);
	}
	else {
		ts->base_type = type;
		ts->type = type;
		ts->indirect_reg = indirect_reg;
		ts->mem_allocated = 1;
		ts->mem_base = base_ts;
		ts->mem_offset = offset;
		ts->name = name;
	}
	return temp_idx(s, ts);
}

static int tcg_temp_new_internal(TCGContext *s, TCGType type, int temp_local)
{
	TCGTemp *ts;
	int idx, k;

	k = type + (temp_local ? TCG_TYPE_COUNT : 0);
	idx = find_first_bit(s->free_temps[k].l, TCG_MAX_TEMPS);
	if (idx < TCG_MAX_TEMPS) {
		/* There is already an available temp with the right type.  */
		clear_bit(idx, s->free_temps[k].l);

		ts = &s->temps[idx];
		ts->temp_allocated = 1;
		tcg_debug_assert(ts->base_type == type);
		tcg_debug_assert(ts->temp_local == temp_local);
	}
	else {
		ts = tcg_temp_alloc(s);
		if (TCG_TARGET_REG_BITS == 32 && type == TCG_TYPE_I64) {
			TCGTemp *ts2 = tcg_temp_alloc(s);

			ts->base_type = type;
			ts->type = TCG_TYPE_I32;
			ts->temp_allocated = 1;
			ts->temp_local = temp_local;

			tcg_debug_assert(ts2 == ts + 1);
			ts2->base_type = TCG_TYPE_I64;
			ts2->type = TCG_TYPE_I32;
			ts2->temp_allocated = 1;
			ts2->temp_local = temp_local;
		}
		else {
			ts->base_type = type;
			ts->type = type;
			ts->temp_allocated = 1;
			ts->temp_local = temp_local;
		}
		idx = temp_idx(s, ts);
	}

#if defined(CONFIG_DEBUG_TCG)
	s->temps_in_use++;
#endif
	return idx;
}

TCGv_i32 tcg_temp_new_internal_i32(TCGContext *s, int temp_local)
{
	int idx;

	idx = tcg_temp_new_internal(s, TCG_TYPE_I32, temp_local);
	return MAKE_TCGV_I32(idx);
}

TCGv_i64 tcg_temp_new_internal_i64(TCGContext *s, int temp_local)
{
	int idx;

	idx = tcg_temp_new_internal(s, TCG_TYPE_I64, temp_local);
	return MAKE_TCGV_I64(idx);
}

static void tcg_temp_free_internal(TCGContext *s, int idx)
{
	TCGTemp *ts;
	int k;

#if defined(CONFIG_DEBUG_TCG)
	s->temps_in_use--;
	if (s->temps_in_use < 0) {
		fprintf(stderr, "More temporaries freed than allocated!\n");
	}
#endif

	tcg_debug_assert(idx >= s->nb_globals && idx < s->nb_temps);
	ts = &s->temps[idx];
	tcg_debug_assert(ts->temp_allocated != 0);
	ts->temp_allocated = 0;

	k = ts->base_type + (ts->temp_local ? TCG_TYPE_COUNT : 0);
	set_bit(idx, s->free_temps[k].l);
}

void tcg_temp_free_i32(TCGContext *s, TCGv_i32 arg)
{
	tcg_temp_free_internal(s, GET_TCGV_I32(arg));
}

void tcg_temp_free_i64(TCGContext *s, TCGv_i64 arg)
{
	tcg_temp_free_internal(s, GET_TCGV_I64(arg));
}

TCGv_i32 tcg_const_i32(TCGContext *s, int32_t val)
{
	TCGv_i32 t0;
	t0 = tcg_temp_new_i32(s);
	tcg_gen_movi_i32(s, t0, val);
	return t0;
}

TCGv_i64 tcg_const_i64(TCGContext *s, int64_t val)
{
	TCGv_i64 t0;
	t0 = tcg_temp_new_i64(s);
	tcg_gen_movi_i64(s, t0, val);
	return t0;
}

TCGv_i32 tcg_const_local_i32(TCGContext *s, int32_t val)
{
	TCGv_i32 t0;
	t0 = tcg_temp_local_new_i32(s);
	tcg_gen_movi_i32(s, t0, val);
	return t0;
}

TCGv_i64 tcg_const_local_i64(TCGContext *s, int64_t val)
{
	TCGv_i64 t0;
	t0 = tcg_temp_local_new_i64(s);
	tcg_gen_movi_i64(s, t0, val);
	return t0;
}

/* Note: we convert the 64 bit args to 32 bit and do some alignment
   and endian swap. Maybe it would be better to do the alignment
   and endian swap in tcg_reg_alloc_call(). */
void tcg_gen_callN(TCGContext *s, int helper_idx, TCGArg ret, int nargs, TCGArg *args)
{
	int i, real_args, nb_rets, pi, pi_first;
	unsigned sizemask, flags;
	const TCGHelperInfo *info;

	if (helper_idx > ARRAY_SIZE(all_helpers))
		return;

	info = &all_helpers[helper_idx];
	if (!info)
		return;

	flags = info->flags;
	sizemask = info->sizemask;

#if defined(__sparc__) && !defined(__arch64__) \
    && !defined(CONFIG_TCG_INTERPRETER)
	/* We have 64-bit values in one register, but need to pass as two
	   separate parameters.  Split them.  */
	int orig_sizemask = sizemask;
	int orig_nargs = nargs;
	TCGv_i64 retl, reth;

	TCGV_UNUSED_I64(retl);
	TCGV_UNUSED_I64(reth);
	if (sizemask != 0) {
		TCGArg *split_args = __builtin_alloca(sizeof(TCGArg) * nargs * 2);
		for (i = real_args = 0; i < nargs; ++i) {
			int is_64bit = sizemask & (1 << (i + 1) * 2);
			if (is_64bit) {
				TCGv_i64 orig = MAKE_TCGV_I64(args[i]);
				TCGv_i32 h = tcg_temp_new_i32();
				TCGv_i32 l = tcg_temp_new_i32();
				tcg_gen_extr_i64_i32(l, h, orig);
				split_args[real_args++] = GET_TCGV_I32(h);
				split_args[real_args++] = GET_TCGV_I32(l);
			}
			else {
				split_args[real_args++] = args[i];
			}
		}
		nargs = real_args;
		args = split_args;
		sizemask = 0;
	}
#elif defined(TCG_TARGET_EXTEND_ARGS) && TCG_TARGET_REG_BITS == 64
	for (i = 0; i < nargs; ++i) {
		int is_64bit = sizemask & (1 << (i + 1) * 2);
		int is_signed = sizemask & (2 << (i + 1) * 2);
		if (!is_64bit) {
			TCGv_i64 temp = tcg_temp_new_i64();
			TCGv_i64 orig = MAKE_TCGV_I64(args[i]);
			if (is_signed) {
				tcg_gen_ext32s_i64(temp, orig);
			}
			else {
				tcg_gen_ext32u_i64(temp, orig);
			}
			args[i] = GET_TCGV_I64(temp);
		}
	}
#endif /* TCG_TARGET_EXTEND_ARGS */

	pi_first = pi = s->gen_next_parm_idx;
	if (ret != TCG_CALL_DUMMY_ARG) {
#if defined(__sparc__) && !defined(__arch64__) \
    && !defined(CONFIG_TCG_INTERPRETER)
		if (orig_sizemask & 1) {
			/* The 32-bit ABI is going to return the 64-bit value in
			   the %o0/%o1 register pair.  Prepare for this by using
			   two return temporaries, and reassemble below.  */
			retl = tcg_temp_new_i64();
			reth = tcg_temp_new_i64();
			s->gen_opparam_buf[pi++] = GET_TCGV_I64(reth);
			s->gen_opparam_buf[pi++] = GET_TCGV_I64(retl);
			nb_rets = 2;
		}
		else {
			s->gen_opparam_buf[pi++] = ret;
			nb_rets = 1;
		}
#else
		if (TCG_TARGET_REG_BITS < 64 && (sizemask & 1)) {
#ifdef HOST_WORDS_BIGENDIAN
			s->gen_opparam_buf[pi++] = ret + 1;
			s->gen_opparam_buf[pi++] = ret;
#else
			s->gen_opparam_buf[pi++] = ret;
			s->gen_opparam_buf[pi++] = ret + 1;
#endif
			nb_rets = 2;
		}
		else {
			s->gen_opparam_buf[pi++] = ret;
			nb_rets = 1;
		}
#endif
	}
	else {
		nb_rets = 0;
	}
	real_args = 0;
	for (i = 0; i < nargs; i++) {
		int is_64bit = sizemask & (1 << (i + 1) * 2);
		if (TCG_TARGET_REG_BITS < 64 && is_64bit) {
#ifdef TCG_TARGET_CALL_ALIGN_ARGS
			/* some targets want aligned 64 bit args */
			if (real_args & 1) {
				s->gen_opparam_buf[pi++] = TCG_CALL_DUMMY_ARG;
				real_args++;
			}
#endif
			/* If stack grows up, then we will be placing successive
			   arguments at lower addresses, which means we need to
			   reverse the order compared to how we would normally
			   treat either big or little-endian.  For those arguments
			   that will wind up in registers, this still works for
			   HPPA (the only current STACK_GROWSUP target) since the
			   argument registers are *also* allocated in decreasing
			   order.  If another such target is added, this logic may
			   have to get more complicated to differentiate between
			   stack arguments and register arguments.  */
#if defined(HOST_WORDS_BIGENDIAN) != defined(TCG_TARGET_STACK_GROWSUP)
			s->gen_opparam_buf[pi++] = args[i] + 1;
			s->gen_opparam_buf[pi++] = args[i];
#else
			s->gen_opparam_buf[pi++] = args[i];
			s->gen_opparam_buf[pi++] = args[i] + 1;
#endif
			real_args += 2;
			continue;
		}

		s->gen_opparam_buf[pi++] = args[i];
		real_args++;
	}
	s->gen_opparam_buf[pi++] = (uintptr_t)helper_idx;
	s->gen_opparam_buf[pi++] = flags;

	i = s->gen_next_op_idx;
	tcg_debug_assert(i < OPC_BUF_SIZE);
	tcg_debug_assert(pi <= OPPARAM_BUF_SIZE);

	/* Set links for sequential allocation during translation.  */
	s->gen_op_buf[i].opc = INDEX_op_call;
	s->gen_op_buf[i].callo = nb_rets;
	s->gen_op_buf[i].calli = real_args;
	s->gen_op_buf[i].args = pi_first;
	s->gen_op_buf[i].prev = i - 1;
	s->gen_op_buf[i].next = i + 1;
	s->gen_op_buf[i].life = 0;

	/* Make sure the calli field didn't overflow.  */
	tcg_debug_assert(s->gen_op_buf[i].calli == real_args);

	s->gen_op_buf[0].prev = i;
	s->gen_next_op_idx = i + 1;
	s->gen_next_parm_idx = pi;

#if defined(__sparc__) && !defined(__arch64__) \
    && !defined(CONFIG_TCG_INTERPRETER)
	/* Free all of the parts we allocated above.  */
	for (i = real_args = 0; i < orig_nargs; ++i) {
		int is_64bit = orig_sizemask & (1 << (i + 1) * 2);
		if (is_64bit) {
			TCGv_i32 h = MAKE_TCGV_I32(args[real_args++]);
			TCGv_i32 l = MAKE_TCGV_I32(args[real_args++]);
			tcg_temp_free_i32(s, h);
			tcg_temp_free_i32(s, l);
		}
		else {
			real_args++;
		}
	}
	if (orig_sizemask & 1) {
		/* The 32-bit ABI returned two 32-bit pieces.  Re-assemble them.
		   Note that describing these as TCGv_i64 eliminates an unnecessary
		   zero-extension that tcg_gen_concat_i32_i64 would create.  */
		tcg_gen_concat32_i64(MAKE_TCGV_I64(ret), retl, reth);
		tcg_temp_free_i64(s, retl);
		tcg_temp_free_i64(s, reth);
	}
#elif defined(TCG_TARGET_EXTEND_ARGS) && TCG_TARGET_REG_BITS == 64
	for (i = 0; i < nargs; ++i) {
		int is_64bit = sizemask & (1 << (i + 1) * 2);
		if (!is_64bit) {
			TCGv_i64 temp = MAKE_TCGV_I64(args[i]);
			tcg_temp_free_i64(temp);
		}
	}
#endif /* TCG_TARGET_EXTEND_ARGS */
}

TCGRegType tcg_get_reg_type(TCGContext *s, int arg )
{
	if (arg < 0 || arg >= s->nb_temps)
		return TCG_REG_UNKNOWN;

	TCGTemp *ts = &s->temps[arg];

	int idx = temp_idx(s, ts);

	if (idx < s->nb_globals)
		return TCG_REG_GLOBAL;

	if (ts->temp_local) 
		return TCG_REG_LOCAL;

	return TCG_REG_TEMP;
}

static char *tcg_get_arg_str_ptr(TCGContext *s, char *buf, int buf_size, TCGTemp *ts)
{
	int idx = temp_idx(s, ts);

	if (idx < s->nb_globals) {
		strncpy_s(buf, buf_size, ts->name, strlen(ts->name));
	}
	else if (ts->temp_local) {
		snprintf(buf, buf_size, "loc%d", idx - s->nb_globals);
	}
	else {
		snprintf(buf, buf_size, "tmp%d", idx - s->nb_globals);
	}
	return buf;
}

char *tcg_get_arg_str_idx(TCGContext *s, char *buf, int buf_size, int idx)
{
	tcg_debug_assert(idx >= 0 && idx < s->nb_temps);
	return tcg_get_arg_str_ptr(s, buf, buf_size, &s->temps[idx]);
}

/* Find helper name.  */
const char *tcg_find_helper(TCGContext *s, int helper_idx)
{
	if (helper_idx > ARRAY_SIZE(all_helpers))
		return NULL;

	return all_helpers[helper_idx].name;
}

const void *tcg_find_helper_func(TCGContext *s, int helper_idx)
{
	if (helper_idx > ARRAY_SIZE(all_helpers))
		return NULL;

	return all_helpers[helper_idx].func;
}

const char * const cond_name[] =
{
#ifdef _MSC_VER
	"never", "always", "lt", "ge", "ltu", "geu", NULL, NULL, "eq", "ne", "le", "gt", "leu", "gtu", NULL, NULL,
#else
	[TCG_COND_NEVER] = "never",
	[TCG_COND_ALWAYS] = "always",
	[TCG_COND_EQ] = "eq",
	[TCG_COND_NE] = "ne",
	[TCG_COND_LT] = "lt",
	[TCG_COND_GE] = "ge",
	[TCG_COND_LE] = "le",
	[TCG_COND_GT] = "gt",
	[TCG_COND_LTU] = "ltu",
	[TCG_COND_GEU] = "geu",
	[TCG_COND_LEU] = "leu",
	[TCG_COND_GTU] = "gtu"
#endif
};

const size_t cond_name_max = ARRAY_SIZE(cond_name);

const char * const ldst_name[] =
{
#ifdef _MSC_VER
	"ub", "leuw", "leul", "leq", "sb", "lesw", "lesl", NULL, NULL, "beuw", "beul", "beq", NULL, "besw", "besl", NULL,
#else
	[MO_UB] = "ub",
	[MO_SB] = "sb",
	[MO_LEUW] = "leuw",
	[MO_LESW] = "lesw",
	[MO_LEUL] = "leul",
	[MO_LESL] = "lesl",
	[MO_LEQ] = "leq",
	[MO_BEUW] = "beuw",
	[MO_BESW] = "besw",
	[MO_BEUL] = "beul",
	[MO_BESL] = "besl",
	[MO_BEQ] = "beq",
#endif
};

const size_t ldst_name_max = ARRAY_SIZE(ldst_name);

const char * const alignment_name[] = {
#ifdef _MSC_VER
#ifdef ALIGNED_ONLY
	"un+", "",
#else
	"", "al+",
#endif
	"al2+", "al4+", "al8+", "al16+", "al32+", "al64+",
#else
#ifdef ALIGNED_ONLY
	[MO_UNALN >> MO_ASHIFT] = "un+",
	[MO_ALIGN >> MO_ASHIFT] = "",
#else
	[MO_UNALN >> MO_ASHIFT] = "",
	[MO_ALIGN >> MO_ASHIFT] = "al+",
#endif
	[MO_ALIGN_2 >> MO_ASHIFT] = "al2+",
	[MO_ALIGN_4 >> MO_ASHIFT] = "al4+",
	[MO_ALIGN_8 >> MO_ASHIFT] = "al8+",
	[MO_ALIGN_16 >> MO_ASHIFT] = "al16+",
	[MO_ALIGN_32 >> MO_ASHIFT] = "al32+",
	[MO_ALIGN_64 >> MO_ASHIFT] = "al64+",
#endif
};

const size_t alignment_name_max = ARRAY_SIZE(alignment_name);

void tcg_dump_ops(TCGContext *s)
{
	char buf[128];
	TCGOp *op;
	int oi;

	for (oi = s->gen_op_buf[0].next; oi != 0; oi = op->next) {
		int i, k, nb_oargs, nb_iargs, nb_cargs;
		const TCGOpDef *def;
		const TCGArg *args;
		TCGOpcode c;
		int col = 0;

		op = &s->gen_op_buf[oi];
		c = op->opc;
		def = &tcg_op_defs[c];
		args = &s->gen_opparam_buf[op->args];

		if (c == INDEX_op_insn_start) {
			col += qemu_log("%s ----", oi != s->gen_op_buf[0].next ? "\n" : "");

			for (i = 0; i < TARGET_INSN_START_WORDS; ++i) {
				target_ulong a;
#if TARGET_LONG_BITS > TCG_TARGET_REG_BITS
				a = ((target_ulong)args[i * 2 + 1] << 32) | args[i * 2];
#else
				a = args[i];
#endif
				col += qemu_log(" " TARGET_FMT_lx, a);
			}
		}
		else if (c == INDEX_op_call) {
			/* variable number of arguments */
			nb_oargs = op->callo;
			nb_iargs = op->calli;
			nb_cargs = def->nb_cargs;

			/* function name, flags, out args */
			col += qemu_log(" %s %s,$0x%" TCG_PRIlx ",$%d", def->name,
				tcg_find_helper(s, (TCGHelpersIndex)args[nb_oargs + nb_iargs]),
				args[nb_oargs + nb_iargs + 1], nb_oargs);
			for (i = 0; i < nb_oargs; i++) {
				col += qemu_log(",%s", tcg_get_arg_str_idx(s, buf, sizeof(buf),
					args[i]));
			}
			for (i = 0; i < nb_iargs; i++) {
				TCGArg arg = args[nb_oargs + i];
				const char *t = "<dummy>";
				if (arg != TCG_CALL_DUMMY_ARG) {
					t = tcg_get_arg_str_idx(s, buf, sizeof(buf), arg);
				}
				col += qemu_log(",%s", t);
			}
		}
		else {
			col += qemu_log(" %s ", def->name);

			nb_oargs = def->nb_oargs;
			nb_iargs = def->nb_iargs;
			nb_cargs = def->nb_cargs;

			k = 0;
			for (i = 0; i < nb_oargs; i++) {
				if (k != 0) {
					col += qemu_log(",");
				}
				col += qemu_log("%s", tcg_get_arg_str_idx(s, buf, sizeof(buf),
					args[k++]));
			}
			for (i = 0; i < nb_iargs; i++) {
				if (k != 0) {
					col += qemu_log(",");
				}
				col += qemu_log("%s", tcg_get_arg_str_idx(s, buf, sizeof(buf),
					args[k++]));
			}
			switch (c) {
			case INDEX_op_brcond_i32:
			case INDEX_op_setcond_i32:
			case INDEX_op_movcond_i32:
			case INDEX_op_brcond2_i32:
			case INDEX_op_setcond2_i32:
			case INDEX_op_brcond_i64:
			case INDEX_op_setcond_i64:
			case INDEX_op_movcond_i64:
				if (args[k] < ARRAY_SIZE(cond_name) && cond_name[args[k]]) {
					col += qemu_log(",%s", cond_name[args[k++]]);
				}
				else {
					col += qemu_log(",$0x%" TCG_PRIlx, args[k++]);
				}
				i = 1;
				break;
			case INDEX_op_qemu_ld_i32:
			case INDEX_op_qemu_st_i32:
			case INDEX_op_qemu_ld_i64:
			case INDEX_op_qemu_st_i64:
			{
				TCGMemOpIdx oi = args[k++];
				TCGMemOp op = get_memop(oi);
				unsigned ix = get_mmuidx(oi);

				if (op & ~(MO_AMASK | MO_BSWAP | MO_SSIZE)) {
					col += qemu_log(",$0x%x,%u", op, ix);
				}
				else {
					const char *s_al, *s_op;
					s_al = alignment_name[(op & MO_AMASK) >> MO_ASHIFT];
					s_op = ldst_name[op & (MO_BSWAP | MO_SSIZE)];
					col += qemu_log(",%s%s,%u", s_al, s_op, ix);
				}
				i = 1;
			}
			break;
			default:
				i = 0;
				break;
			}
			switch (c) {
			case INDEX_op_set_label:
			case INDEX_op_br:
			case INDEX_op_brcond_i32:
			case INDEX_op_brcond_i64:
			case INDEX_op_brcond2_i32:
				col += qemu_log("%s$L%d", k ? "," : "", arg_label(args[k])->id);
				i++, k++;
				break;
			default:
				break;
			}
			for (; i < nb_cargs; i++, k++) {
				col += qemu_log("%s$0x%" TCG_PRIlx, k ? "," : "", args[k]);
			}
		}
		if (op->life) {
			unsigned life = op->life;

			for (; col < 48; ++col) {
				putc(' ', qemu_logfile);
			}

			if (life & (SYNC_ARG * 3)) {
				qemu_log("  sync:");
				for (i = 0; i < 2; ++i) {
					if (life & (SYNC_ARG << i)) {
						qemu_log(" %d", i);
					}
				}
			}
			life /= DEAD_ARG;
			if (life) {
				qemu_log("  dead:");
				for (i = 0; life; ++i, life >>= 1) {
					if (life & 1) {
						qemu_log(" %d", i);
					}
				}
			}
		}
		qemu_log("\n");
	}
}

void tcg_op_remove(TCGContext *s, TCGOp *op)
{
	int next = op->next;
	int prev = op->prev;

	/* We should never attempt to remove the list terminator.  */
	tcg_debug_assert(op != &s->gen_op_buf[0]);

	s->gen_op_buf[next].prev = prev;
	s->gen_op_buf[prev].next = next;

	memset(op, 0, sizeof(*op));

#ifdef CONFIG_PROFILER
	s->del_op_count++;
#endif
}

TCGOp *tcg_op_insert_before(TCGContext *s, TCGOp *old_op, TCGOpcode opc, int nargs)
{
	int oi = s->gen_next_op_idx;
	int pi = s->gen_next_parm_idx;
	int prev = old_op->prev;
	int next = old_op - s->gen_op_buf;
	TCGOp *new_op;

	tcg_debug_assert(oi < OPC_BUF_SIZE);
	tcg_debug_assert(pi + nargs <= OPPARAM_BUF_SIZE);
	s->gen_next_op_idx = oi + 1;
	s->gen_next_parm_idx = pi + nargs;

	new_op = &s->gen_op_buf[oi];

	new_op->args = pi;
	new_op->calli = 0;
	new_op->callo = 0;
	new_op->life = 0;
	new_op->next = next;
	new_op->opc = opc;
	new_op->prev = prev;

	s->gen_op_buf[prev].next = oi;
	old_op->prev = oi;

	return new_op;
}

TCGOp *tcg_op_insert_after(TCGContext *s, TCGOp *old_op, TCGOpcode opc, int nargs)
{
	int oi = s->gen_next_op_idx;
	int pi = s->gen_next_parm_idx;
	int prev = old_op - s->gen_op_buf;
	int next = old_op->next;
	TCGOp *new_op;

	tcg_debug_assert(oi < OPC_BUF_SIZE);
	tcg_debug_assert(pi + nargs <= OPPARAM_BUF_SIZE);
	s->gen_next_op_idx = oi + 1;
	s->gen_next_parm_idx = pi + nargs;

	new_op = &s->gen_op_buf[oi];

	new_op->args = pi;
	new_op->calli = 0;
	new_op->callo = 0;
	new_op->life = 0;
	new_op->next = next;
	new_op->opc = opc;
	new_op->prev = prev;

	s->gen_op_buf[next].prev = oi;
	old_op->next = oi;

	return new_op;
}

#define TS_DEAD  1
#define TS_MEM   2

#define IS_DEAD_ARG(n)   (arg_life & (DEAD_ARG << (n)))
#define NEED_SYNC_ARG(n) (arg_life & (SYNC_ARG << (n)))

/* liveness analysis: end of function: all temps are dead, and globals
   should be in memory. */
static inline void tcg_la_func_end(TCGContext *s, uint8_t *temp_state)
{
	memset(temp_state, TS_DEAD | TS_MEM, s->nb_globals);
	memset(temp_state + s->nb_globals, TS_DEAD, s->nb_temps - s->nb_globals);
}

/* liveness analysis: end of basic block: all temps are dead, globals
   and local temps should be in memory. */
static inline void tcg_la_bb_end(TCGContext *s, uint8_t *temp_state)
{
	int i, n;

	tcg_la_func_end(s, temp_state);
	for (i = s->nb_globals, n = s->nb_temps; i < n; i++) {
		if (s->temps[i].temp_local) {
			temp_state[i] |= TS_MEM;
		}
	}
}

/* Liveness analysis : update the opc_arg_life array to tell if a
   given input arguments is dead. Instructions updating dead
   temporaries are removed. */
void liveness_pass_1(TCGContext *s, uint8_t *temp_state)
{
	int nb_globals = s->nb_globals;
	int oi, oi_prev;

	tcg_la_func_end(s, temp_state);

	for (oi = s->gen_op_buf[0].prev; oi != 0; oi = oi_prev) {
		int i, nb_iargs, nb_oargs;
		TCGOpcode opc_new, opc_new2;
		bool have_opc_new2;
		TCGLifeData arg_life = 0;
		TCGArg arg;

		TCGOp * const op = &s->gen_op_buf[oi];
		TCGArg * const args = &s->gen_opparam_buf[op->args];
		TCGOpcode opc = op->opc;
		const TCGOpDef *def = &tcg_op_defs[opc];

		oi_prev = op->prev;

		switch (opc) {
		case INDEX_op_call:
		{
			int call_flags;

			nb_oargs = op->callo;
			nb_iargs = op->calli;
			call_flags = args[nb_oargs + nb_iargs + 1];

			/* pure functions can be removed if their result is unused */
			if (call_flags & TCG_CALL_NO_SIDE_EFFECTS) {
				for (i = 0; i < nb_oargs; i++) {
					arg = args[i];
					if (temp_state[arg] != TS_DEAD) {
						goto do_not_remove_call;
					}
				}
				goto do_remove;
			}
			else {
			do_not_remove_call:

				/* output args are dead */
				for (i = 0; i < nb_oargs; i++) {
					arg = args[i];
					if (temp_state[arg] & TS_DEAD) {
						arg_life |= DEAD_ARG << i;
					}
					if (temp_state[arg] & TS_MEM) {
						arg_life |= SYNC_ARG << i;
					}
					temp_state[arg] = TS_DEAD;
				}

				if (!(call_flags & (TCG_CALL_NO_WRITE_GLOBALS |
					TCG_CALL_NO_READ_GLOBALS))) {
					/* globals should go back to memory */
					memset(temp_state, TS_DEAD | TS_MEM, nb_globals);
				}
				else if (!(call_flags & TCG_CALL_NO_READ_GLOBALS)) {
					/* globals should be synced to memory */
					for (i = 0; i < nb_globals; i++) {
						temp_state[i] |= TS_MEM;
					}
				}

				/* record arguments that die in this helper */
				for (i = nb_oargs; i < nb_iargs + nb_oargs; i++) {
					arg = args[i];
					if (arg != TCG_CALL_DUMMY_ARG) {
						if (temp_state[arg] & TS_DEAD) {
							arg_life |= DEAD_ARG << i;
						}
					}
				}
				/* input arguments are live for preceding opcodes */
				for (i = nb_oargs; i < nb_iargs + nb_oargs; i++) {
					arg = args[i];
					if (arg != TCG_CALL_DUMMY_ARG) {
						temp_state[arg] &= ~TS_DEAD;
					}
				}
			}
		}
		break;
		case INDEX_op_insn_start:
			break;
		case INDEX_op_discard:
			/* mark the temporary as dead */
			temp_state[args[0]] = TS_DEAD;
			break;

		case INDEX_op_add2_i32:
			opc_new = INDEX_op_add_i32;
			goto do_addsub2;
		case INDEX_op_sub2_i32:
			opc_new = INDEX_op_sub_i32;
			goto do_addsub2;
		case INDEX_op_add2_i64:
			opc_new = INDEX_op_add_i64;
			goto do_addsub2;
		case INDEX_op_sub2_i64:
			opc_new = INDEX_op_sub_i64;
		do_addsub2:
			nb_iargs = 4;
			nb_oargs = 2;
			/* Test if the high part of the operation is dead, but not
			   the low part.  The result can be optimized to a simple
			   add or sub.  This happens often for x86_64 guest when the
			   cpu mode is set to 32 bit.  */
			if (temp_state[args[1]] == TS_DEAD) {
				if (temp_state[args[0]] == TS_DEAD) {
					goto do_remove;
				}
				/* Replace the opcode and adjust the args in place,
				   leaving 3 unused args at the end.  */
				op->opc = opc = opc_new;
				args[1] = args[2];
				args[2] = args[4];
				/* Fall through and mark the single-word operation live.  */
				nb_iargs = 2;
				nb_oargs = 1;
			}
			goto do_not_remove;

		case INDEX_op_mulu2_i32:
			opc_new = INDEX_op_mul_i32;
			opc_new2 = INDEX_op_muluh_i32;
			have_opc_new2 = TCG_TARGET_HAS_muluh_i32;
			goto do_mul2;
		case INDEX_op_muls2_i32:
			opc_new = INDEX_op_mul_i32;
			opc_new2 = INDEX_op_mulsh_i32;
			have_opc_new2 = TCG_TARGET_HAS_mulsh_i32;
			goto do_mul2;
		case INDEX_op_mulu2_i64:
			opc_new = INDEX_op_mul_i64;
			opc_new2 = INDEX_op_muluh_i64;
			have_opc_new2 = TCG_TARGET_HAS_muluh_i64;
			goto do_mul2;
		case INDEX_op_muls2_i64:
			opc_new = INDEX_op_mul_i64;
			opc_new2 = INDEX_op_mulsh_i64;
			have_opc_new2 = TCG_TARGET_HAS_mulsh_i64;
			goto do_mul2;
		do_mul2:
			nb_iargs = 2;
			nb_oargs = 2;
			if (temp_state[args[1]] == TS_DEAD) {
				if (temp_state[args[0]] == TS_DEAD) {
					/* Both parts of the operation are dead.  */
					goto do_remove;
				}
				/* The high part of the operation is dead; generate the low. */
				op->opc = opc = opc_new;
				args[1] = args[2];
				args[2] = args[3];
			}
			else if (temp_state[args[0]] == TS_DEAD && have_opc_new2) {
				/* The low part of the operation is dead; generate the high. */
				op->opc = opc = opc_new2;
				args[0] = args[1];
				args[1] = args[2];
				args[2] = args[3];
			}
			else {
				goto do_not_remove;
			}
			/* Mark the single-word operation live.  */
			nb_oargs = 1;
			goto do_not_remove;

		default:
			/* XXX: optimize by hardcoding common cases (e.g. triadic ops) */
			nb_iargs = def->nb_iargs;
			nb_oargs = def->nb_oargs;

			/* Test if the operation can be removed because all
			   its outputs are dead. We assume that nb_oargs == 0
			   implies side effects */
			if (!(def->flags & TCG_OPF_SIDE_EFFECTS) && nb_oargs != 0) {
				for (i = 0; i < nb_oargs; i++) {
					if (temp_state[args[i]] != TS_DEAD) {
						goto do_not_remove;
					}
				}
			do_remove:
				tcg_op_remove(s, op);
			}
			else {
			do_not_remove:
				/* output args are dead */
				for (i = 0; i < nb_oargs; i++) {
					arg = args[i];
					if (temp_state[arg] & TS_DEAD) {
						arg_life |= DEAD_ARG << i;
					}
					if (temp_state[arg] & TS_MEM) {
						arg_life |= SYNC_ARG << i;
					}
					temp_state[arg] = TS_DEAD;
				}

				/* if end of basic block, update */
				if (def->flags & TCG_OPF_BB_END) {
					tcg_la_bb_end(s, temp_state);
				}
				else if (def->flags & TCG_OPF_SIDE_EFFECTS) {
					/* globals should be synced to memory */
					for (i = 0; i < nb_globals; i++) {
						temp_state[i] |= TS_MEM;
					}
				}

				/* record arguments that die in this opcode */
				for (i = nb_oargs; i < nb_oargs + nb_iargs; i++) {
					arg = args[i];
					if (temp_state[arg] & TS_DEAD) {
						arg_life |= DEAD_ARG << i;
					}
				}
				/* input arguments are live for preceding opcodes */
				for (i = nb_oargs; i < nb_oargs + nb_iargs; i++) {
					temp_state[args[i]] &= ~TS_DEAD;
				}
			}
			break;
		}
		op->life = arg_life;
	}
}

/* Liveness analysis: Convert indirect regs to direct temporaries.  */
bool liveness_pass_2(TCGContext *s, uint8_t *temp_state)
{
	int nb_globals = s->nb_globals;
	int16_t *dir_temps;
	int i, oi, oi_next;
	bool changes = false;

	dir_temps = (int16_t *)tcg_malloc(s, nb_globals * sizeof(int16_t));
	memset(dir_temps, 0, nb_globals * sizeof(int16_t));

	/* Create a temporary for each indirect global.  */
	for (i = 0; i < nb_globals; ++i) {
		TCGTemp *its = &s->temps[i];
		if (its->indirect_reg) {
			TCGTemp *dts = tcg_temp_alloc(s);
			dts->type = its->type;
			dts->base_type = its->base_type;
			dir_temps[i] = temp_idx(s, dts);
		}
	}

	memset(temp_state, TS_DEAD, nb_globals);

	for (oi = s->gen_op_buf[0].next; oi != 0; oi = oi_next) {
		TCGOp *op = &s->gen_op_buf[oi];
		TCGArg *args = &s->gen_opparam_buf[op->args];
		TCGOpcode opc = op->opc;
		const TCGOpDef *def = &tcg_op_defs[opc];
		TCGLifeData arg_life = op->life;
		int nb_iargs, nb_oargs, call_flags;
		TCGArg arg, dir;

		oi_next = op->next;

		if (opc == INDEX_op_call) {
			nb_oargs = op->callo;
			nb_iargs = op->calli;
			call_flags = args[nb_oargs + nb_iargs + 1];
		}
		else {
			nb_iargs = def->nb_iargs;
			nb_oargs = def->nb_oargs;

			/* Set flags similar to how calls require.  */
			if (def->flags & TCG_OPF_BB_END) {
				/* Like writing globals: save_globals */
				call_flags = 0;
			}
			else if (def->flags & TCG_OPF_SIDE_EFFECTS) {
				/* Like reading globals: sync_globals */
				call_flags = TCG_CALL_NO_WRITE_GLOBALS;
			}
			else {
				/* No effect on globals.  */
				call_flags = (TCG_CALL_NO_READ_GLOBALS |
					TCG_CALL_NO_WRITE_GLOBALS);
			}
		}

		/* Make sure that input arguments are available.  */
		for (i = nb_oargs; i < nb_iargs + nb_oargs; i++) {
			arg = args[i];
			/* Note this unsigned test catches TCG_CALL_ARG_DUMMY too.  */
			if (arg < nb_globals) {
				dir = dir_temps[arg];
				if (dir != 0 && temp_state[arg] == TS_DEAD) {
					TCGTemp *its = &s->temps[arg];
					TCGOpcode lopc = (its->type == TCG_TYPE_I32
						? INDEX_op_ld_i32
						: INDEX_op_ld_i64);
					TCGOp *lop = tcg_op_insert_before(s, op, lopc, 3);
					TCGArg *largs = &s->gen_opparam_buf[lop->args];

					largs[0] = dir;
					largs[1] = temp_idx(s, its->mem_base);
					largs[2] = its->mem_offset;

					/* Loaded, but synced with memory.  */
					temp_state[arg] = TS_MEM;
				}
			}
		}

		/* Perform input replacement, and mark inputs that became dead.
		   No action is required except keeping temp_state up to date
		   so that we reload when needed.  */
		for (i = nb_oargs; i < nb_iargs + nb_oargs; i++) {
			arg = args[i];
			if (arg < nb_globals) {
				dir = dir_temps[arg];
				if (dir != 0) {
					args[i] = dir;
					changes = true;
					if (IS_DEAD_ARG(i)) {
						temp_state[arg] = TS_DEAD;
					}
				}
			}
		}

		/* Liveness analysis should ensure that the following are
		   all correct, for call sites and basic block end points.  */
		if (call_flags & TCG_CALL_NO_READ_GLOBALS) {
			/* Nothing to do */
		}
		else if (call_flags & TCG_CALL_NO_WRITE_GLOBALS) {
			for (i = 0; i < nb_globals; ++i) {
				/* Liveness should see that globals are synced back,
				   that is, either TS_DEAD or TS_MEM.  */
				tcg_debug_assert(dir_temps[i] == 0
					|| temp_state[i] != 0);
			}
		}
		else {
			for (i = 0; i < nb_globals; ++i) {
				/* Liveness should see that globals are saved back,
				   that is, TS_DEAD, waiting to be reloaded.  */
				tcg_debug_assert(dir_temps[i] == 0
					|| temp_state[i] == TS_DEAD);
			}
		}

		/* Outputs become available.  */
		for (i = 0; i < nb_oargs; i++) {
			arg = args[i];
			if (arg >= nb_globals) {
				continue;
			}
			dir = dir_temps[arg];
			if (dir == 0) {
				continue;
			}
			args[i] = dir;
			changes = true;

			/* The output is now live and modified.  */
			temp_state[arg] = 0;

			/* Sync outputs upon their last write.  */
			if (NEED_SYNC_ARG(i)) {
				TCGTemp *its = &s->temps[arg];
				TCGOpcode sopc = (its->type == TCG_TYPE_I32
					? INDEX_op_st_i32
					: INDEX_op_st_i64);
				TCGOp *sop = tcg_op_insert_after(s, op, sopc, 3);
				TCGArg *sargs = &s->gen_opparam_buf[sop->args];

				sargs[0] = dir;
				sargs[1] = temp_idx(s, its->mem_base);
				sargs[2] = its->mem_offset;

				temp_state[arg] = TS_MEM;
			}
			/* Drop outputs that are dead.  */
			if (IS_DEAD_ARG(i)) {
				temp_state[arg] = TS_DEAD;
			}
		}
	}

	return changes;
}
