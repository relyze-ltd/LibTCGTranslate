#ifndef TCG_TARGET_H
#define TCG_TARGET_H

#define TCG_TARGET_REG_BITS					64
#define TCG_TARGET_NB_REGS					1
#define TCG_TARGET_INSN_UNIT_SIZE			8
#define TCG_TARGET_CALL_ALIGN_ARGS			1

typedef enum TCGReg {
	TCREG_NONE=0,
	TCG_AREG0= TCREG_NONE
} TCGReg;

#define TCG_TARGET_HAS_add2_i32			0
#define TCG_TARGET_HAS_add2_i64			0
#define TCG_TARGET_HAS_andc_i32			0
#define TCG_TARGET_HAS_andc_i64			0
#define TCG_TARGET_HAS_bswap16_i32		0
#define TCG_TARGET_HAS_bswap16_i64		0
#define TCG_TARGET_HAS_bswap32_i32		0
#define TCG_TARGET_HAS_bswap32_i64		0
#define TCG_TARGET_HAS_bswap64_i64		0
#define TCG_TARGET_HAS_clz_i32			0
#define TCG_TARGET_HAS_clz_i64			0
#define TCG_TARGET_HAS_ctpop_i32		0
#define TCG_TARGET_HAS_ctpop_i64		0
#define TCG_TARGET_HAS_ctz_i32			0
#define TCG_TARGET_HAS_ctz_i64			0
#define TCG_TARGET_HAS_deposit_i32		0
#define TCG_TARGET_HAS_deposit_i64		0
#define TCG_TARGET_HAS_div2_i32			0
#define TCG_TARGET_HAS_div2_i64			0
#define TCG_TARGET_HAS_div_i32			1
#define TCG_TARGET_HAS_div_i64			1
#define TCG_TARGET_HAS_eqv_i32			0
#define TCG_TARGET_HAS_eqv_i64			0
#define TCG_TARGET_HAS_ext16s_i32		1
#define TCG_TARGET_HAS_ext16s_i64		1
#define TCG_TARGET_HAS_ext16u_i32		1
#define TCG_TARGET_HAS_ext16u_i64		1
#define TCG_TARGET_HAS_ext32s_i64		1
#define TCG_TARGET_HAS_ext32u_i64		1
#define TCG_TARGET_HAS_ext8s_i32		1
#define TCG_TARGET_HAS_ext8s_i64		1
#define TCG_TARGET_HAS_ext8u_i32		1
#define TCG_TARGET_HAS_ext8u_i64		1
#define TCG_TARGET_HAS_extract_i32		0
#define TCG_TARGET_HAS_extract_i64		0
#define TCG_TARGET_HAS_extrh_i64_i32	0
#define TCG_TARGET_HAS_extrl_i64_i32	0
#define TCG_TARGET_HAS_goto_ptr			1
#define TCG_TARGET_HAS_movcond_i32		1
#define TCG_TARGET_HAS_movcond_i64		1
#define TCG_TARGET_HAS_muls2_i32		0
#define TCG_TARGET_HAS_muls2_i64		0
#define TCG_TARGET_HAS_mulsh_i32		0
#define TCG_TARGET_HAS_mulsh_i64		0
#define TCG_TARGET_HAS_mulu2_i32		0
#define TCG_TARGET_HAS_mulu2_i64		0
#define TCG_TARGET_HAS_muluh_i32		0
#define TCG_TARGET_HAS_muluh_i64		0
#define TCG_TARGET_HAS_nand_i32			0
#define TCG_TARGET_HAS_nand_i64			0
#define TCG_TARGET_HAS_neg_i32			1
#define TCG_TARGET_HAS_neg_i64			1
#define TCG_TARGET_HAS_nor_i32			0
#define TCG_TARGET_HAS_nor_i64			0
#define TCG_TARGET_HAS_not_i32			1
#define TCG_TARGET_HAS_not_i64			1
#define TCG_TARGET_HAS_orc_i32			0
#define TCG_TARGET_HAS_orc_i64			0
#define TCG_TARGET_HAS_rem_i32			1
#define TCG_TARGET_HAS_rem_i64			1
#define TCG_TARGET_HAS_rot_i32			0
#define TCG_TARGET_HAS_rot_i64			0
#define TCG_TARGET_HAS_sextract_i32		0
#define TCG_TARGET_HAS_sextract_i64		0
#define TCG_TARGET_HAS_sub2_i32			0
#define TCG_TARGET_HAS_sub2_i64			0

#endif
