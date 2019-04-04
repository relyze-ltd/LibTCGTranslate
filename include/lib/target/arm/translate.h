#ifndef TARGET_ARM_TRANSLATE_H
#define TARGET_ARM_TRANSLATE_H

#include "lib/internal.h"
#include "lib/target/arm/cpu.h"

typedef struct DisasContext 
{
    target_ulong pc;
    uint32_t insn;
    int is_jmp;
    int condjmp;
    TCGLabel *condlabel;
    int condexec_mask;
    int condexec_cond;
    struct TranslationBlock *tb;
    int singlestep_enabled;
    int thumb;
    int sctlr_b;
    TCGMemOp be_data;
#if !defined(CONFIG_USER_ONLY)
    int user;
#endif
    ARMMMUIdx mmu_idx;
    bool tbi0;
    bool tbi1;
    bool ns;
    int fp_excp_el;
    bool secure_routed_to_el3;
    bool vfp_enabled;
    int vec_len;
    int vec_stride;
    bool v7m_handler_mode;
    uint32_t svc_imm;
    int aarch64;
    int current_el;
    GHashTable *cp_regs;
    uint64_t features;
    bool fp_access_checked;
    bool ss_active;
    bool pstate_ss;
    bool is_ldex;
    bool ss_same_el;
    int c15_cpar;
    int insn_start_idx;
#define TMP_A64_MAX 16
    int tmp_a64_count;
    TCGv_i64 tmp_a64[TMP_A64_MAX];
} DisasContext;

typedef struct DisasCompare 
{
    TCGCond cond;
    TCGv_i32 value;
    bool value_global;
} DisasCompare;

#define DISAS_WFI 4
#define DISAS_SWI 5
#define DISAS_EXC 6
#define DISAS_WFE 7
#define DISAS_HVC 8
#define DISAS_SMC 9
#define DISAS_YIELD 10
#define DISAS_BX_EXCRET 11
#define DISAS_EXIT 12

#if defined(TARGET_ARM)
void arm_translate_init(TCGContext * s);
void gen_intermediate_code_arm(TCGContext * s, CPUARMState *cs, TranslationBlock *tb);
#elif defined(TARGET_AARCH64)
void a64_translate_init(TCGContext * s);
void gen_intermediate_code_a64(TCGContext * s, CPUARMState *cs, TranslationBlock *tb);
#endif

#endif
