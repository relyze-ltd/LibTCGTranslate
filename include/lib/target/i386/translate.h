#ifndef TARGET_I386_TRANSLATE_H
#define TARGET_I386_TRANSLATE_H

#include "lib/internal.h"
#include "lib/target/i386/cpu.h"

typedef struct DisasContext {
	/* current insn context */
	int override; /* -1 if no override */
	int prefix;
	TCGMemOp aflag;
	TCGMemOp dflag;
	target_ulong pc_start;
	target_ulong pc; /* pc = eip + cs_base */
	int is_jmp; /* 1 = means jump (stop translation), 2 means CPU
				static state change (stop translation) */
				/* current block context */
	target_ulong cs_base; /* base of CS segment */
	int pe;     /* protected mode */
	int code32; /* 32 bit code segment */
#ifdef TARGET_X86_64
	int lma;    /* long mode active */
	int code64; /* 64 bit code segment */
	int rex_x, rex_b;
#endif
	int vex_l;  /* vex vector length */
	int vex_v;  /* vex vvvv register, without 1's compliment.  */
	int ss32;   /* 32 bit stack segment */
	CCOp cc_op;  /* current CC operation */
	int addseg; /* non zero if either DS/ES/SS have a non zero base */
	int f_st;   /* currently unused */
	int vm86;   /* vm86 mode */
	int cpl;
	int iopl;
	int tf;     /* TF cpu flag */
	int singlestep_enabled; /* "hardware" single step enabled */
	int jmp_opt; /* use direct block chaining for direct jumps */
	int repz_opt; /* optimize jumps within repz instructions */
	int mem_index; /* select memory access functions */
	uint64_t flags; /* all execution flags */
	struct TranslationBlock *tb;
	int popl_esp_hack; /* for correct popl with esp base handling */
	int rip_offset; /* only used in x86_64, but left for simplicity */
	int cpuid_features;
	int cpuid_ext_features;
	int cpuid_ext2_features;
	int cpuid_ext3_features;
	int cpuid_7_0_ebx_features;
	int cpuid_xsave_features;
} DisasContext;

void i386_translate_init(TCGContext * s);

void gen_intermediate_code_i386(TCGContext * s, CPUX86State *cs, TranslationBlock *tb);

#endif