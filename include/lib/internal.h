#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stddef.h>
#include <intrin.h>

#include "lib/tcg/tcg-target.h"

#pragma warning( disable : 4003 4002 4146 ) 

#define NEED_CPU_H	1

#if defined(TARGET_ARM)
#define TARGET_LONG_BITS				32
#define TARGET_LONG_SIZE				4
#define TARGET_INSN_START_EXTRA_WORDS	2

#define arch_gen_intermediate_code		gen_intermediate_code_arm
#define arch_translate_init				arm_translate_init
#define arch_cpu_init					arm_cpu_init
#elif defined(TARGET_AARCH64)
#define TARGET_LONG_BITS				64
#define TARGET_LONG_SIZE				8
#define TARGET_INSN_START_EXTRA_WORDS	2

#define arch_gen_intermediate_code		gen_intermediate_code_a64
#define arch_translate_init				a64_translate_init
#define arch_cpu_init					a64_cpu_init
#elif defined(TARGET_X86)
#define TARGET_LONG_BITS				32
#define TARGET_LONG_SIZE				4
#define TARGET_INSN_START_EXTRA_WORDS	1

#define arch_gen_intermediate_code		gen_intermediate_code_i386
#define arch_translate_init				i386_translate_init
#define arch_cpu_init					i386_cpu_init
#elif defined(TARGET_X86_64)
#define TARGET_LONG_BITS				64
#define TARGET_LONG_SIZE				8
#define TARGET_INSN_START_EXTRA_WORDS	1

#define arch_gen_intermediate_code		gen_intermediate_code_i386
#define arch_translate_init				i386_translate_init
#define arch_cpu_init					i386_cpu_init
#else
#error No Target set
#endif

#define TARGET_PAGE_BITS				12
#define TARGET_PAGE_SIZE				(1 << TARGET_PAGE_BITS)
#define TARGET_PAGE_MASK				~(TARGET_PAGE_SIZE - 1)
#define TARGET_PAGE_ALIGN(addr)			(((addr) + TARGET_PAGE_SIZE - 1) & TARGET_PAGE_MASK)

#define CPU_COMMON \
	bool singlestep_enabled; \
	void * libtct_ctx;

#if TARGET_LONG_SIZE == 4
typedef int32_t target_long;
typedef uint32_t target_ulong;
#define TARGET_FMT_lx "%08x"
#define TARGET_FMT_ld "%d"
#define TARGET_FMT_lu "%u"
#elif TARGET_LONG_SIZE == 8
typedef int64_t target_long;
typedef uint64_t target_ulong;
#define TARGET_FMT_lx "%016" PRIx64
#define TARGET_FMT_ld "%" PRId64
#define TARGET_FMT_lu "%" PRIu64
#else
#error TARGET_LONG_SIZE undefined
#endif

extern size_t cpu_ld_code(void *env, target_ulong addr, uint8_t * buffer, size_t size);
extern uint8_t cpu_ldub_code(void *env, target_ulong addr);
extern uint16_t cpu_lduw_code(void *env, target_ulong addr);
extern uint32_t cpu_ldul_code(void *env, target_ulong addr);
extern uint64_t cpu_lduq_code(void *env, target_ulong addr);
extern int8_t cpu_ldsb_code(void *env, target_ulong addr);
extern int16_t cpu_ldsw_code(void *env, target_ulong addr);
extern int32_t cpu_ldsl_code(void *env, target_ulong addr);
extern int64_t cpu_ldsq_code(void *env, target_ulong addr);

#define g_assert				assert
#define qemu_log				printf
#define qemu_logfile			stdout
#define g_assert_not_reached()
#define qemu_log_mask
#define __builtin_constant_p(x)	0
#define abort()
#define cpu_abort
#define LOG_UNIMP				0
#define likely(x)				(x)
#define unlikely(x)				(x)

#define container_of(ptr, type, member)		((type *)((char *)(ptr) -offsetof(type,member)))
#define ARRAY_SIZE(x)						(sizeof(x) / sizeof((x)[0]))

#define QEMU_NORETURN
#define QEMU_PACKED
#define QEMU_ARTIFICIAL
#define QEMU_BUILD_BUG_ON(x)
#define QEMU_GNUC_PREREQ(x,y)	0
#define QEMU_ALIGN_DOWN(n, m)	((n) / (m) * (m))
#define QEMU_ALIGN_UP(n, m)		QEMU_ALIGN_DOWN((n) + (m) - 1, (m))
#define __attribute__(x)
#define trace_guest_mem_before_tcg(p1,p2,p3,p4)
#define semihosting_enabled()		0
#define gemu_log_mask(p1, p2, p3, p4)

#ifndef MIN
#define MIN(a, b)				(((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)				(((a) > (b)) ? (a) : (b))
#endif

#ifndef glue
#define xglue(x, y)				x##y
#define glue(x, y)				xglue(x, y)
#define stringify(s)			tostring(s)
#define tostring(s)				#s
#endif

#define DIV_ROUND_UP(n,d)		(((n) + (d) - 1) / (d))
#define CHAR_BIT				8
#define BITS_PER_BYTE           CHAR_BIT
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))
#define MAKE_64BIT_MASK(shift, length) (((~0ULL) >> (64 - (length))) << (shift))

#define FIELD(reg, field, shift, length) \
    enum : int { R_ ## reg ## _ ## field ## _SHIFT = (shift)}; \
    enum : int { R_ ## reg ## _ ## field ## _LENGTH = (length)}; \
    enum : __int64 { R_ ## reg ## _ ## field ## _MASK = MAKE_64BIT_MASK(shift, length)};


#define atomic_inc(ptr)			((void) InterlockedIncrement(ptr))
#define atomic_dec(ptr)			((void) InterlockedDecrement(ptr))
#define atomic_add(ptr, n)		((void) InterlockedAdd((LONG*)ptr, (LONG)n))
#define atomic_sub(ptr, n)		((void) InterlockedAdd((LONG*)ptr, (LONG)-n))
#define atomic_or(ptr, n)		((void) InterlockedAdd((LONG*)ptr, (LONG)n))

#define ctzl					ctz32
#define clzl					clz32

static uint32_t __inline ctz32(uint32_t value)
{
	unsigned long trailing_zero = 0;

	if (_BitScanForward(&trailing_zero, value))
		return trailing_zero;

	return 32;
}

static uint32_t __inline ctz64(uint64_t value)
{
	unsigned long trailing_zero = 0;

#ifdef _X86_
	if (_InlineBitScanForward64(&trailing_zero, value))
		return trailing_zero;
#else
	if (_BitScanForward64(&trailing_zero, value))
		return trailing_zero;
#endif

	return 64;
}

static uint32_t __inline clz32(uint32_t value)
{
	unsigned long leading_zero = 0;

	if (_BitScanReverse(&leading_zero, value))
		return 31 - leading_zero;

	return 32;
}

static uint32_t __inline clz64(uint64_t value)
{
	unsigned long leading_zero = 0;

#ifdef _X86_
	if (_InlineBitScanReverse64(&leading_zero, value))
		return 63 - leading_zero;
#else
	if (_BitScanReverse64(&leading_zero, value))
		return 63 - leading_zero;
#endif

	return 64;
}

static uint32_t __inline ctpop32(uint32_t value)
{
	return __popcnt(value);
}

static uint64_t __inline ctpop64(uint64_t value)
{
#ifdef _WIN64
	return __popcnt64(value);
#else
	return ctpop32(value & 0xffffffff) + ctpop32((value >> 32) & 0xffffffff);
#endif
}

#define DISAS_NEXT				0
#define DISAS_JUMP				1
#define DISAS_UPDATE			2
#define DISAS_TB_JUMP			3

#define CF_COUNT_MASK				0x7fff
#define CF_LAST_IO					0x8000 
#define CF_NOCACHE					0x10000
#define CF_USE_ICOUNT				0x20000
#define CF_IGNORE_ICOUNT			0x40000
#define TB_JMP_RESET_OFFSET_INVALID	0xffff

#if defined(TARGET_ARM) || defined(TARGET_AARCH64)

#define CP_REG_SIZE_SHIFT		52
#define CP_REG_SIZE_MASK       0x00f0000000000000ULL
#define CP_REG_SIZE_U32        0x0020000000000000ULL
#define CP_REG_SIZE_U64        0x0030000000000000ULL
#define CP_REG_ARM             0x4000000000000000ULL
#define CP_REG_ARCH_MASK       0xff00000000000000ULL

#define CP_REG_ARM64					0x6000000000000000ULL
#define CP_REG_ARM_COPROC_MASK			0x000000000FFF0000
#define CP_REG_ARM_COPROC_SHIFT			16
#define CP_REG_ARM64_SYSREG				(0x0013 << CP_REG_ARM_COPROC_SHIFT)
#define CP_REG_ARM64_SYSREG_OP0_MASK	0x000000000000c000
#define CP_REG_ARM64_SYSREG_OP0_SHIFT	14
#define CP_REG_ARM64_SYSREG_OP1_MASK	0x0000000000003800
#define CP_REG_ARM64_SYSREG_OP1_SHIFT	11
#define CP_REG_ARM64_SYSREG_CRN_MASK	0x0000000000000780
#define CP_REG_ARM64_SYSREG_CRN_SHIFT	7
#define CP_REG_ARM64_SYSREG_CRM_MASK	0x0000000000000078
#define CP_REG_ARM64_SYSREG_CRM_SHIFT	3
#define CP_REG_ARM64_SYSREG_OP2_MASK	0x0000000000000007
#define CP_REG_ARM64_SYSREG_OP2_SHIFT	0
#define CP_REG_ARM64_SYSREG_CP			(CP_REG_ARM64_SYSREG >> CP_REG_ARM_COPROC_SHIFT)

#define ARM_EL_EC_SHIFT 26
#define ARM_EL_IL_SHIFT 25
#define ARM_EL_ISV_SHIFT 24
#define ARM_EL_IL (1 << ARM_EL_IL_SHIFT)
#define ARM_EL_ISV (1 << ARM_EL_ISV_SHIFT)

#define syn_uncategorized() 0
#define syn_aa32_svc(imm16, is_16bit) 1
#define syn_aa32_hvc(imm16) 2
#define syn_aa32_smc(void) 3
#define syn_aa32_bkpt(imm16, is_16bit) 4
#define syn_cp14_rt_trap(cv, cond, opc1, opc2, crn, crm, rt, isread, is_16bit) 5
#define syn_cp15_rt_trap(cv, cond, opc1, opc2, crn, crm, rt, isread, is_16bit) 6
#define syn_cp14_rrt_trap(cv, cond, opc1, crm, rt, rt2, isread, is_16bit) 7
#define syn_cp15_rrt_trap(cv, cond, opc1, crm, rt, rt2, isread, is_16bit) 8
#define syn_fp_access_trap(cv, cond, is_16bit) 9
#define syn_data_abort_with_iss(same_el, sas, sse, srt, sf, ar, ea, cm, s1ptw, wnr, fsc, is_16bit) 10
#define syn_swstep(same_el, isv, ex) 11
#define syn_aa64_sysregtrap(op0, op1, op2, crn, crm, rt, isread) 12
#define syn_aa64_svc(imm16)	13
#define syn_aa64_hvc(imm16) 14
#define syn_aa64_smc(imm16) 15
#define syn_aa64_bkpt(imm16) 16

#elif defined(TARGET_X86) || defined(TARGET_X86_64)
#define SVM_IOIO_TYPE_MASK  1
#define	SVM_EXIT_READ_CR0 	1
#define	SVM_EXIT_WRITE_CR0 	2
#define SVM_EXIT_IDTR_READ	3
#define SVM_EXIT_GDTR_READ	4
#define SVM_EXIT_LDTR_READ	5
#define SVM_EXIT_TR_READ	6
#define SVM_EXIT_IDTR_WRITE	7
#define SVM_EXIT_GDTR_WRITE	8
#define SVM_EXIT_LDTR_WRITE	9
#define SVM_EXIT_TR_WRITE	10
#define SVM_EXIT_PUSHF		11
#define SVM_EXIT_POPF		12
#define SVM_EXIT_IRET		13
#define SVM_EXIT_INVD		14
#define SVM_EXIT_WBINVD		15
#define	SVM_EXIT_WRITE_DR0 	16
#define	SVM_EXIT_READ_DR0 	17
#define SVM_EXIT_RSM		18
#endif

#define EXCP_INTERRUPT 	0x10000
#define EXCP_HLT        0x10001
#define EXCP_DEBUG      0x10002
#define EXCP_HALTED     0x10003
#define EXCP_YIELD      0x10004
#define EXCP_ATOMIC     0x10005

#define FEATURE_ENABLE( DC, FEATURE )		(DC)->features |= (1ULL << (FEATURE))
#define FEATURE_DISABLE( DC, FEATURE )		(DC)->features &= ~(1ULL << (FEATURE))
#define FEATURE_HAS(DC, FEATURE)			(((DC)->features & (1ULL << FEATURE)) != 0)

typedef void * GHashTable;

typedef int(*fprintf_function)(FILE *handle, const char *format, ...);

typedef uint8_t flag;

typedef uint16_t float16;

typedef uint32_t float32;

typedef uint64_t float64;

typedef struct floatx80
{
	uint64_t low;
	uint16_t high;
} floatx80;

typedef struct float_status {
	signed char float_detect_tininess;
	signed char float_rounding_mode;
	uint8_t     float_exception_flags;
	signed char floatx80_rounding_precision;
	flag flush_to_zero;
	flag flush_inputs_to_zero;
	flag default_nan_mode;
	flag snan_bit_is_one;
} float_status;

typedef union {
	float64 d;
#if defined(HOST_WORDS_BIGENDIAN)
	struct {
		uint32_t upper;
		uint32_t lower;
	} l;
#else
	struct {
		uint32_t lower;
		uint32_t upper;
	} l;
#endif
	uint64_t ll;
} CPU_DoubleU;

typedef struct TCGLabel TCGLabel;

struct TranslationBlock {
	target_ulong pc;
	target_ulong cs_base;
	uint32_t flags;
	uint16_t size;
	uint16_t icount;
	uint32_t cflags;
	uint32_t trace_vcpu_dstate;
	uint16_t invalid;
	void *tc_ptr;
	uint8_t *tc_search;
	struct TranslationBlock *orig_tb;
	struct TranslationBlock *page_next[2];
	uint16_t jmp_reset_offset[2];
#ifdef USE_DIRECT_JUMP
	uint16_t jmp_insn_offset[2];
#else
	uintptr_t jmp_target_addr[2];
#endif
	uintptr_t jmp_list_next[2];
	uintptr_t jmp_list_first;
};

#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
enum arm_exception_class {
	EC_UNCATEGORIZED = 0x00,
	EC_WFX_TRAP = 0x01,
	EC_CP15RTTRAP = 0x03,
	EC_CP15RRTTRAP = 0x04,
	EC_CP14RTTRAP = 0x05,
	EC_CP14DTTRAP = 0x06,
	EC_ADVSIMDFPACCESSTRAP = 0x07,
	EC_FPIDTRAP = 0x08,
	EC_CP14RRTTRAP = 0x0c,
	EC_ILLEGALSTATE = 0x0e,
	EC_AA32_SVC = 0x11,
	EC_AA32_HVC = 0x12,
	EC_AA32_SMC = 0x13,
	EC_AA64_SVC = 0x15,
	EC_AA64_HVC = 0x16,
	EC_AA64_SMC = 0x17,
	EC_SYSTEMREGISTERTRAP = 0x18,
	EC_INSNABORT = 0x20,
	EC_INSNABORT_SAME_EL = 0x21,
	EC_PCALIGNMENT = 0x22,
	EC_DATAABORT = 0x24,
	EC_DATAABORT_SAME_EL = 0x25,
	EC_SPALIGNMENT = 0x26,
	EC_AA32_FPTRAP = 0x28,
	EC_AA64_FPTRAP = 0x2c,
	EC_SERROR = 0x2f,
	EC_BREAKPOINT = 0x30,
	EC_BREAKPOINT_SAME_EL = 0x31,
	EC_SOFTWARESTEP = 0x32,
	EC_SOFTWARESTEP_SAME_EL = 0x33,
	EC_WATCHPOINT = 0x34,
	EC_WATCHPOINT_SAME_EL = 0x35,
	EC_AA32_BKPT = 0x38,
	EC_VECTORCATCH = 0x3a,
	EC_AA64_BKPT = 0x3c,
};
#endif

enum {
	float_round_nearest_even = 0,
	float_round_down = 1,
	float_round_up = 2,
	float_round_to_zero = 3,
	float_round_ties_away = 4,
	float_round_to_odd = 5,
};

enum arm_fprounding {
	FPROUNDING_TIEEVEN = 0,
	FPROUNDING_POSINF,
	FPROUNDING_NEGINF,
	FPROUNDING_ZERO,
	FPROUNDING_TIEAWAY,
	FPROUNDING_ODD
};
