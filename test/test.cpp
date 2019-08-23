/*
* LibTCGTranslate - Test Application
*
* Copyright (c) 2019 Relyze Software Limited
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
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#define LIBTCGT_DYNAMIC_LINKING
#include "LibTCGTranslate.h"

static uint8_t * test_code = nullptr;

static uint64_t test_pc = 0x100000;

size_t LIBTCGT_API read_callback(void * user_ptr, uint64_t addr, uint8_t * buffer, size_t size)
{
	if (buffer == nullptr || test_code == nullptr)
		return 0;

	if (addr < test_pc)
		return 0;

	memcpy(buffer, (uint8_t *)test_code + (addr - test_pc), size);

	return size;
}

static const char * const ldst_name[] = { "uint8_t", "uint16_t", "uint32_t", "uint64_t", "int8_t", "int16_t", "int32_t", NULL, NULL, "beuw", "beul", "beq", NULL, "besw", "besl", NULL };

static const char * const cond_name[] = { "never", "always", "<", ">=", "<u", ">=u", NULL, NULL, "==", "!=", "<=", ">", "<=u", ">u", NULL, NULL };

void get_buffer(const char * input, uint8_t ** buffer)
{
	static const char table[] = "0123456789ABCDEF";

	char * opcd = _strdup(input);

	uint8_t * buff = (uint8_t *)malloc((strlen(opcd) / 2) + 1);

	char * h = opcd;

	for (; *h; h++)
	{
		if (*h == ' ')
		{
			char * k = h + 1;

			for (; ; k++)
			{
				char * q = k - 1;
				*q = *k;
				if (!*k)
					break;
			}
		}

		if (*h >= 'a' && *h <= 'f')
			*h -= ' ';
	}

	h = opcd;

	uint8_t * b = buff;

	for (; *h; h += 2, ++b)
		*b = ((strchr(table, *h) - table) * 16) + ((strchr(table, *(h + 1)) - table));

	if (buffer)
		*buffer = buff;

	free(opcd);
}

int usage(const char * msg)
{
	if (msg)
		printf("[-] %s\n", msg);
	
	printf("Usage: TestLibTCGTranslate.exe [/pseudo] [/opt] [/dce] [/thumb] [/pc 0x10000] [/max_insns 4] [/x86|/x64|/arm|/aarch64] /buffer 90909090");
	
	return -1;
}

int main(int argc, char * argv[])
{
	bool optimize_opt = false;
	bool optimize_dce = false;
	bool thumb = false;
	bool pseudo = false;
	uint16_t max_insns = 0;
	TCGArch arch = TCGArch::TCG_ARCH_UNKNOWN;

	for (int i = 1; i < argc; i++)
	{
		if (_strcmpi(argv[i], "/x86") == 0)
			arch = TCGArch::TCG_ARCH_X86;
		else if (_strcmpi(argv[i], "/x64") == 0)
			arch = TCGArch::TCG_ARCH_X64;
		else if (_strcmpi(argv[i], "/arm") == 0)
			arch = TCGArch::TCG_ARCH_ARM;
		else if (_strcmpi(argv[i], "/aarch64") == 0)
			arch = TCGArch::TCG_ARCH_AARCH64;
		else if (_strcmpi(argv[i], "/pseudo") == 0)
			pseudo = true;
		else if (_strcmpi(argv[i], "/opt") == 0 || _strcmpi(argv[i], "/optimize") == 0)
			optimize_opt = true;
		else if (_strcmpi(argv[i], "/dce") == 0)
			optimize_dce = true;
		else if (_strcmpi(argv[i], "/thumb") == 0)
			thumb = true;
		else if (_strcmpi(argv[i], "/buffer") == 0)
			get_buffer(argv[++i], &test_code);
		else if (_strcmpi(argv[i], "/pc") == 0)
			test_pc = strtoll(argv[++i], nullptr, 16);
		else if (_strcmpi(argv[i], "/max_insns") == 0)
			max_insns = (uint16_t)strtol(argv[++i], nullptr, 16);
	}

	if (arch == TCGArch::TCG_ARCH_UNKNOWN)
	{
		return usage("Must select an arch (/x86, /x64, /arm, /aarch64).");
	}

	if (test_code == nullptr)
	{
		return usage("Must provide a buffer (/buffer).");
	}

	LibTCGTInstance * inst = new LibTCGTInstance(arch, &read_callback, nullptr);
	if (inst == nullptr)
	{
		printf("[-] LibTCGT_init failed.\n");
		free(test_code);
		return -1;
	}

	uint16_t icount = inst->translate((thumb ? (test_pc | 1) : test_pc), max_insns);
	if (icount == 0)
	{
		printf("[-] Failed to translate any instructions.\n");
		return -1;
	}

	if (optimize_opt || optimize_dce)
		inst->optimize(optimize_opt, optimize_dce);

	unsigned oi = inst->first_op();

	while (oi != 0)
	{
		TCGOpcode opc = inst->get_opc(oi);

		if (!pseudo)
			printf("%s%s ", (opc == INDEX_op_insn_start ? "" : "\t"), inst->get_name_opc(opc));

		switch (opc)
		{
		case INDEX_op_mov_i32:
		case INDEX_op_mov_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = %s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			else
			{
				printf(
					"%s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			break;
		}
		case INDEX_op_movi_i32:
		{
			if (pseudo)
			{
				printf(
					"%s = 0x%" PRIx32 ";",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_UINT32(oi, 1)
				);
			}
			else
			{
				printf(
					"%s, 0x%" PRIx32,
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_UINT32(oi, 1)
				);
			}
			break;
		}
		case INDEX_op_movi_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = 0x%" PRIx64 ";",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_UINT64(oi, 1)
				);
			}
			else
			{
				printf(
					"%s, 0x%" PRIx64,
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_UINT64(oi, 1)
				);
			}
			break;
		}
		case INDEX_op_not_i32:
		case INDEX_op_not_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = ~%s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			else
			{
				printf(
					"%s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			break;
		}
		case INDEX_op_neg_i32:
		case INDEX_op_neg_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = -%s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			else
			{
				printf(
					"%s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			break;
		}
		case INDEX_op_add_i32:
		case INDEX_op_sub_i32:
		case INDEX_op_mul_i32:
		case INDEX_op_div_i32:
		case INDEX_op_divu_i32:
		case INDEX_op_rem_i32:
		case INDEX_op_remu_i32:
		case INDEX_op_and_i32:
		case INDEX_op_or_i32:
		case INDEX_op_xor_i32:
		case INDEX_op_shl_i32:
		case INDEX_op_shr_i32:
		case INDEX_op_sar_i32:
		case INDEX_op_add_i64:
		case INDEX_op_sub_i64:
		case INDEX_op_mul_i64:
		case INDEX_op_div_i64:
		case INDEX_op_divu_i64:
		case INDEX_op_rem_i64:
		case INDEX_op_remu_i64:
		case INDEX_op_and_i64:
		case INDEX_op_or_i64:
		case INDEX_op_xor_i64:
		case INDEX_op_shl_i64:
		case INDEX_op_shr_i64:
		case INDEX_op_sar_i64:
		{
			if (pseudo)
			{
				const char * opstr = inst->get_name_opc(opc);
				switch (opc)
				{
				case INDEX_op_add_i32:
				case INDEX_op_add_i64:
					opstr = "+";
					break;
				case INDEX_op_sub_i32:
				case INDEX_op_sub_i64:
					opstr = "-";
					break;
				case INDEX_op_mul_i32:
				case INDEX_op_mul_i64:
					opstr = "*";
					break;
				case INDEX_op_div_i32:
				case INDEX_op_div_i64:
					opstr = "/";
					break;
				case INDEX_op_divu_i32:
				case INDEX_op_divu_i64:
					opstr = "/u";
					break;
				case INDEX_op_rem_i32:
				case INDEX_op_rem_i64:
					opstr = "%";
					break;
				case INDEX_op_remu_i32:
				case INDEX_op_remu_i64:
					opstr = "%u";
					break;
				case INDEX_op_and_i32:
				case INDEX_op_and_i64:
					opstr = "&";
					break;
				case INDEX_op_or_i32:
				case INDEX_op_or_i64:
					opstr = "|";
					break;
				case INDEX_op_xor_i32:
				case INDEX_op_xor_i64:
					opstr = "^";
					break;
				case INDEX_op_shl_i32:
				case INDEX_op_shl_i64:
					opstr = "<<";
					break;
				case INDEX_op_shr_i32:
				case INDEX_op_shr_i64:
					opstr = ">>";
					break;
				case INDEX_op_sar_i32:
				case INDEX_op_sar_i64:
					opstr = ">>a";
					break;
				default:
					break;
				}

				printf(
					"%s = %s %s %s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					opstr,
					inst->get_name_register(inst->get_TCGArg(oi, 2))
				);
			}
			else
			{
				printf(
					"%s, %s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2))
				);
			}
			break;
		}
		case INDEX_op_ext_i32_i64:
		case INDEX_op_extu_i32_i64:
		case INDEX_op_ext8s_i32:
		case INDEX_op_ext8s_i64:
		case INDEX_op_ext8u_i32:
		case INDEX_op_ext8u_i64:
		case INDEX_op_ext16s_i32:
		case INDEX_op_ext16s_i64:
		case INDEX_op_ext16u_i32:
		case INDEX_op_ext16u_i64:
		case INDEX_op_ext32s_i64:
		case INDEX_op_ext32u_i64:
		{
			if (pseudo)
			{
				const char * ext = nullptr;
				switch (opc)
				{
				case INDEX_op_ext8s_i32:
				case INDEX_op_ext8s_i64:
					ext = "int8_t";
					break;
				case INDEX_op_ext8u_i32:
				case INDEX_op_ext8u_i64:
					ext = "uint8_t";
					break;
				case INDEX_op_ext16s_i32:
				case INDEX_op_ext16s_i64:
					ext = "int16_t";
					break;
				case INDEX_op_ext16u_i32:
				case INDEX_op_ext16u_i64:
					ext = "uint16_t";
					break;
				case INDEX_op_ext32s_i64:
					ext = "int32_t";
					break;
				case INDEX_op_ext32u_i64:
					ext = "uint32_t";
					break;
				default:
					ext = inst->get_name_opc(opc);
					break;
				}
				printf(
					"%s = (%s)%s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					ext,
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			else
			{
				printf(
					"%s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1))
				);
			}
			break;
		}
		case INDEX_op_qemu_ld_i32:
		case INDEX_op_qemu_ld_i64:
		case INDEX_op_qemu_st_i32:
		case INDEX_op_qemu_st_i64:
		{
			if (pseudo)
			{
				unsigned ldst_idx = inst->get_TCGMemOp(oi, 2) & (MO_BSWAP | MO_SSIZE);

				if (opc == INDEX_op_qemu_ld_i32 || opc == INDEX_op_qemu_ld_i64)
				{
					printf(
						"%s = *(%s *)%s;",
						inst->get_name_register(inst->get_TCGArg(oi, 0)),
						ldst_name[ldst_idx],
						inst->get_name_register(inst->get_TCGArg(oi, 1))
					);
				}
				else
				{
					printf(
						"*(%s *)%s = %s;",
						ldst_name[ldst_idx],
						inst->get_name_register(inst->get_TCGArg(oi, 1)),
						inst->get_name_register(inst->get_TCGArg(oi, 0))
					);
				}
			}
			else
			{
				printf(
					"%s, %s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_memop(inst->get_TCGMemOp(oi, 2))
				);
			}
			break;
		}
		case INDEX_op_setcond_i32:
		case INDEX_op_setcond_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = (%s %s %s);",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					cond_name[inst->get_TCGCond(oi, 3)],
					inst->get_name_register(inst->get_TCGArg(oi, 2))
				);
			}
			else
			{
				printf(
					"%s, %s, %s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_cond(inst->get_TCGCond(oi, 3))
				);
			}
			break;
		}
		case INDEX_op_setcond2_i32:
		{
			if (pseudo)
			{
				printf(
					"%s = ((%s %s %s) && (%s %s %s));",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					cond_name[inst->get_TCGCond(oi, 5)],
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					cond_name[inst->get_TCGCond(oi, 5)],
					inst->get_name_register(inst->get_TCGArg(oi, 4))
				);
			}
			else
			{
				printf(
					"%s, %s, %s, %s, %s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					inst->get_name_register(inst->get_TCGArg(oi, 4)),
					inst->get_name_cond(inst->get_TCGCond(oi, 5))
				);
			}
			break;
		}
		case INDEX_op_movcond_i32:
		case INDEX_op_movcond_i64:
		{
			if (pseudo)
			{
				printf(
					"%s = ((%s %s %s) ? %s : %s);",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					cond_name[inst->get_TCGCond(oi, 5)],
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					inst->get_name_register(inst->get_TCGArg(oi, 4))
				);
			}
			else
			{
				printf(
					"%s, %s, %s, %s, %s, %s",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					inst->get_name_register(inst->get_TCGArg(oi, 4)),
					inst->get_name_cond(inst->get_TCGCond(oi, 5))
				);
			}
			break;
		}
		case INDEX_op_brcond_i32:
		case INDEX_op_brcond_i64:
		{
			if (pseudo)
			{
				printf(
					"if( %s %s %s ) { goto label_%d; }",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					cond_name[inst->get_TCGCond(oi, 2)],
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_id_label(inst->get_TCGArg(oi, 3))
				);
			}
			else
			{
				printf(
					"%s, %s, %s, $L%d",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_cond(inst->get_TCGCond(oi, 2)),
					inst->get_id_label(inst->get_TCGArg(oi, 3))
				);
			}
			break;
		}
		case INDEX_op_brcond2_i32:
		{
			if (pseudo)
			{
				printf(
					"if( (%s %s %s) && (%s %s %s) ) { goto label_%d; }",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					cond_name[inst->get_TCGCond(oi, 4)],
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					cond_name[inst->get_TCGCond(oi, 4)],
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					inst->get_id_label(inst->get_TCGArg(oi, 5))
				);
			}
			else
			{
				printf(
					"%s, %s, %s, %s, %s, $L%d",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_name_register(inst->get_TCGArg(oi, 2)),
					inst->get_name_register(inst->get_TCGArg(oi, 3)),
					inst->get_name_cond(inst->get_TCGCond(oi, 4)),
					inst->get_id_label(inst->get_TCGArg(oi, 5))
				);
			}
			break;
		}
		case INDEX_op_set_label:
		{
			if (pseudo)
			{
				printf(
					"label_%d:",
					inst->get_id_label(inst->get_TCGArg(oi, 0))
				);
			}
			else
			{
				printf(
					"$L%d",
					inst->get_id_label(inst->get_TCGArg(oi, 0))
				);
			}
			break;
		}
		case INDEX_op_br:
		{
			if (pseudo)
			{
				printf(
					"goto label_%d;",
					inst->get_id_label(inst->get_TCGArg(oi, 0))
				);
			}
			else
			{
				printf(
					"$L%d",
					inst->get_id_label(inst->get_TCGArg(oi, 0))
				);
			}
			break;
		}
		case INDEX_op_goto_ptr:
		{
			if (pseudo)
			{
				printf(
					"jmp %s;",
					inst->get_name_register(inst->get_TCGArg(oi, 0))
				);
			}
			else
			{
				printf(
					"%s",
					inst->get_name_register(inst->get_TCGArg(oi, 0))
				);
			}
			break;
		}
		case INDEX_op_call:
		{
			unsigned out = inst->get_call_op_args_out(oi);
			unsigned in = inst->get_call_op_args_in(oi);

			TCGArgument helper = inst->get_TCGArg(oi, out + in);
			TCGArgument flags = inst->get_TCGArg(oi, out + in + 1);

			if (pseudo)
			{
				for (unsigned i = 0; i < out; i++)
				{
					printf(
						"%s",
						inst->get_TCGArg(oi, i) == -1 ? "<dummy>" : inst->get_name_register(inst->get_TCGArg(oi, i))
					);

					if (i + 1 < out)
						printf(", ");
				}

				if (out > 0)
					printf(" = ");

				printf(
					"%s(",
					inst->get_name_helper(helper)
				);

				for (unsigned i = 0; i < in; i++)
				{
					printf(
						"%s",
						inst->get_TCGArg(oi, i) == -1 ? "<dummy>" : inst->get_name_register(inst->get_TCGArg(oi, out + i))
					);

					if (i + 1 < in)
						printf(", ");
				}

				printf(");");
			}
			else
			{
				printf(
					"%s, %d, ",
					inst->get_name_helper(helper),
					(int)flags
				);

				for (unsigned i = 0; i < out; i++)
				{
					printf(
						"%s, ",
						inst->get_TCGArg(oi, i) == -1 ? "<dummy>" : inst->get_name_register(inst->get_TCGArg(oi, i))
					);
				}

				for (unsigned i = 0; i < in; i++)
				{
					printf(
						"%s, ",
						inst->get_TCGArg(oi, i) == -1 ? "<dummy>" : inst->get_name_register(inst->get_TCGArg(oi, out + i))
					);
				}
			}

			break;
		}
		case INDEX_op_insn_start:
		{
			if (pseudo)
			{
				printf("// %s 0x%" PRIx64,
					inst->get_name_opc(opc),
					inst->get_UINT64(oi, 0)
				);
				break;
			}

			printf(
				"0x%" PRIx64,
				inst->get_UINT64(oi, 0)
			);
			break;
		}
		case INDEX_op_exit_tb:
		case INDEX_op_goto_tb:
		{
			if (pseudo)
			{
				printf("// %s 0x%" PRIx64,
					inst->get_name_opc(opc),
					inst->get_UINT64(oi, 0)
				);
				break;
			}

			printf(
				"0x%" PRIx64,
				inst->get_UINT64(oi, 0)
			);
			break;
		}
		case INDEX_op_ld8u_i32:
		case INDEX_op_ld8s_i32:
		case INDEX_op_ld16u_i32:
		case INDEX_op_ld16s_i32:
		case INDEX_op_ld_i32:
		case INDEX_op_st8_i32:
		case INDEX_op_st16_i32:
		case INDEX_op_st_i32:
		{
			unsigned reg = 0;
			unsigned off = 0;

			inst->get_env_register_from_offset(inst->get_UINT64(oi, 2), &reg, &off);

			if (pseudo)
			{
				if (opc == INDEX_op_st8_i32 || opc == INDEX_op_st16_i32 || opc == INDEX_op_st_i32)
				{
					printf(
						"%s->var_0x%" PRIx32 " = %s; // %s:%d",
						inst->get_name_register(inst->get_TCGArg(oi, 1)),
						inst->get_UINT32(oi, 2),
						inst->get_name_register(inst->get_TCGArg(oi, 0)),
						reg == 0 ? "?" : inst->get_name_register(reg),
						off
					);
				}
				else
				{
					printf(
						"%s = %s->var_0x%" PRIx32 "; // %s:%d",
						inst->get_name_register(inst->get_TCGArg(oi, 0)),
						inst->get_name_register(inst->get_TCGArg(oi, 1)),
						inst->get_UINT32(oi, 2),
						reg == 0 ? "?" : inst->get_name_register(reg),
						off
					);
				}
			}
			else
			{
				printf(
					"%s, %s, 0x%" PRIx32 " ; %s:%d",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_UINT32(oi, 2),
					reg == 0 ? "?" : inst->get_name_register(reg),
					off
				);
			}
			break;
		}
		case INDEX_op_ld8u_i64:
		case INDEX_op_ld8s_i64:
		case INDEX_op_ld16u_i64:
		case INDEX_op_ld16s_i64:
		case INDEX_op_ld32u_i64:
		case INDEX_op_ld32s_i64:
		case INDEX_op_ld_i64:
		case INDEX_op_st8_i64:
		case INDEX_op_st16_i64:
		case INDEX_op_st32_i64:
		case INDEX_op_st_i64:
		{
			unsigned reg = 0;
			unsigned off = 0;

			inst->get_env_register_from_offset(inst->get_UINT64(oi, 2), &reg, &off);

			if (pseudo)
			{
				if (opc == INDEX_op_st8_i64 || opc == INDEX_op_st16_i64 || opc == INDEX_op_st32_i64 || opc == INDEX_op_st_i64)
				{
					printf(
						"%s->var_0x%" PRIx64 " = %s; // %s:%d",
						inst->get_name_register(inst->get_TCGArg(oi, 1)),
						inst->get_UINT64(oi, 2),
						inst->get_name_register(inst->get_TCGArg(oi, 0)),
						reg == 0 ? "?" : inst->get_name_register(reg),
						off
					);
				}
				else
				{
					printf(
						"%s = %s->var_0x%" PRIx64 "; // %s:%d",
						inst->get_name_register(inst->get_TCGArg(oi, 0)),
						inst->get_name_register(inst->get_TCGArg(oi, 1)),
						inst->get_UINT64(oi, 2),
						reg == 0 ? "?" : inst->get_name_register(reg),
						off
					);
				}
			}
			else
			{
				printf(
					"%s, %s, 0x%" PRIx64 " ; %s:%d",
					inst->get_name_register(inst->get_TCGArg(oi, 0)),
					inst->get_name_register(inst->get_TCGArg(oi, 1)),
					inst->get_UINT64(oi, 2),
					reg == 0 ? "?" : inst->get_name_register(reg),
					off
				);
			}
			break;
		}
		case INDEX_op_discard:
		{
			if (pseudo)
			{
				printf("// %s %s",
					inst->get_name_opc(opc),
					inst->get_name_register(inst->get_TCGArg(oi, 0))
				);
				break;
			}

			printf(
				"%s",
				inst->get_name_register(inst->get_TCGArg(oi, 0))
			);
			break;
		}
		case INDEX_op_mb:
		{
			if (pseudo)
			{
				printf("// %s %d",
					inst->get_name_opc(opc),
					inst->get_TCGBar(oi, 0)
				);
				break;
			}

			printf(
				"%d",
				inst->get_TCGBar(oi, 0)
			);
			break;
		}
		default:
		{
			printf("Unknown opc!!!");
			abort();
			break;
		}
		}

		printf("\n");

		oi = inst->next_op(oi);
	}

	inst->reset();

	delete inst;

	free(test_code);

	return 0;
}

