/*
*  LibTCGTranslate
*
*  Copyright (c) 2017 Relyze Software Limited
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
#ifndef _LIB_TCG_TRANSLATE_H
#define _LIB_TCG_TRANSLATE_H

#define LIBTCGT_VER						1

#define LIBTCGT_API                      __cdecl

#include "lib\tcg\tcg-common.h"

struct LibTCGTContext;

typedef uint64_t TCGArgument;

typedef size_t(LIBTCGT_API * LibTCGTReadCallback)(void * user_ptr, uint64_t addr, uint8_t * buffer, size_t size);

namespace ARM
{
	enum Reg : unsigned
	{
		// env (must be 0).
		env = 0,
		// general
		r0,
		r1,
		r2,
		r3,
		r4,
		r5,
		r6,
		r7,
		r8,
		r9,
		r10,
		r11,
		r12,
		r13,
		r14,
		r15,
		// flag
		cf,
		nf,
		vf,
		zf,
		// misc
		exclusive_addr,
		exclusive_val,
		// end.
		RegEnd,
		// aliases
		sb = r9,
		fp = r11,
		ip = r12,
		sp = r13,
		lr = r14,
		pc = r15
	};
}
namespace AARCH64
{
	enum Reg : unsigned
	{
		// env (must be 0)
		env = 0,
		// general
		r0,
		r1,
		r2,
		r3,
		r4,
		r5,
		r6,
		r7,
		r8,
		r9,
		r10,
		r11,
		r12,
		r13,
		r14,
		r15,
		// flag
		cf,
		nf,
		vf,
		zf,
		// misc
		exclusive_addr,
		exclusive_val,
		// general
		pc,
		x0,
		x1,
		x2,
		x3,
		x4,
		x5,
		x6,
		x7,
		x8,
		x9,
		x10,
		x11,
		x12,
		x13,
		x14,
		x15,
		x16,
		x17,
		x18,
		x19,
		x20,
		x21,
		x22,
		x23,
		x24,
		x25,
		x26,
		x27,
		x28,
		x29,
		x30,
		x31,
		// misc
		exclusive_high,
		// end.
		RegEnd,
		// aliases
		fp = x29,
		lr = x30,
		sp = x31
	};
}

namespace X86
{
	enum Reg : unsigned
	{
		// env (must be 0)
		env = 0,
		// general
		eax,
		ecx,
		edx,
		ebx,
		esp,
		ebp,
		esi,
		edi,
		eip,
		// st
		st0,
		st1,
		st2,
		st3,
		st4,
		st5,
		st6,
		st7,
		// mmx
		mm0,
		mm1,
		mm2,
		mm3,
		mm4,
		mm5,
		mm6,
		mm7,
		// zmm (ymm/xmm)
		zmm0,
		zmm1,
		zmm2,
		zmm3,
		zmm4,
		zmm5,
		zmm6,
		zmm7,
		// segment
		es,
		cs,
		ss,
		ds,
		fs,
		gs,
		// bnd
		bnd0_lb,
		bnd0_ub,
		bnd1_lb,
		bnd1_ub,
		bnd2_lb,
		bnd2_ub,
		bnd3_lb,
		bnd3_ub,
		// flag
		cf,
		pf,
		af,
		zf,
		sf,
		of,
		df,
		// cc helper registers
		cc_dst,
		cc_src,
		cc_src2,
		// end.
		RegEnd
		// aliases
		// ...
	};
}

namespace X64
{
	enum Reg : unsigned
	{
        // env (must be 0)
		env = 0,
		// general
		rax,
		rcx,
		rdx,
		rbx,
		rsp,
		rbp,
		rsi,
		rdi,
		r8,
		r9,
		r10,
		r11,
		r12,
		r13,
		r14,
		r15,
		rip,
		// st
		st0,
		st1,
		st2,
		st3,
		st4,
		st5,
		st6,
		st7,
		// mmx
		mm0,
		mm1,
		mm2,
		mm3,
		mm4,
		mm5,
		mm6,
		mm7,
		// zmm (ymm/xmm)
		zmm0,
		zmm1,
		zmm2,
		zmm3,
		zmm4,
		zmm5,
		zmm6,
		zmm7,
		zmm8,
		zmm9,
		zmm10,
		zmm11,
		zmm12,
		zmm13,
		zmm14,
		zmm15,
		zmm16,
		zmm17,
		zmm18,
		zmm19,
		zmm20,
		zmm21,
		zmm22,
		zmm23,
		zmm24,
		zmm25,
		zmm26,
		zmm27,
		zmm28,
		zmm29,
		zmm30,
		zmm31,
		// segment
		es,
		cs,
		ss,
		ds,
		fs,
		gs,
		// bnd
		bnd0_lb,
		bnd0_ub,
		bnd1_lb,
		bnd1_ub,
		bnd2_lb,
		bnd2_ub,
		bnd3_lb,
		bnd3_ub,
		// flag
		cf,
		pf,
		af,
		zf,
		sf,
		of,
		df,
		// cc helper registers
		cc_dst,
		cc_src,
		cc_src2,
		// end.
		RegEnd
		// aliases
		// ...
	};
}

#ifdef LIBTCGT_API_LIBRARY

#define LIBTCGT_API_EXPORT				__declspec(dllexport)

typedef struct LibTCGTContext
{
	LibTCGTReadCallback read_callback;

	void * user_ptr;

	TCGContext s;

#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
	ARMCPU cpu;
#elif defined(TARGET_X86) || defined(TARGET_X86_64)
	X86CPU cpu;
#endif

	TranslationBlock tb;

} LibTCGTContext;

#else // #ifdef LIBTCGT_API_LIBRARY

#if defined(LIBTCGT_DYNAMIC_LINKING)

typedef TCGArch(LIBTCGT_API * LIBTCGT_ARCH)(void);
typedef unsigned (LIBTCGT_API * LIBTCGT_BITS)(void);
typedef unsigned (LIBTCGT_API * LIBTCGT_VERSION)(void);
typedef bool (LIBTCGT_API * LIBTCGT_GET_ENV_REGISTER_FROM_OFFSET)(uint32_t offset, unsigned * reg, unsigned * off);
typedef LibTCGTContext * (LIBTCGT_API * LIBTCGT_INIT)(LibTCGTReadCallback read_callback, void * user_ptr, TCG_FUNC_MALLOC tcg_malloc, TCG_FUNC_FREE tcg_free);
typedef void (LIBTCGT_API * LIBTCGT_REINIT)(LibTCGTContext * ctx, LibTCGTReadCallback read_callback, void * user_ptr);
typedef void (LIBTCGT_API * LIBTCGT_RESET)(LibTCGTContext * ctx);
typedef void (LIBTCGT_API * LIBTCGT_CLEANUP)(LibTCGTContext * ctx);
typedef uint16_t(LIBTCGT_API * LIBTCGT_TRANSLATE)(LibTCGTContext * ctx, uint64_t pc, uint16_t max_insns);
typedef void (LIBTCGT_API * LIBTCGT_OPTIMIZE)(LibTCGTContext * ctx);
typedef unsigned (LIBTCGT_API * LIBTCGT_FIRST_OP)(LibTCGTContext * ctx);
typedef unsigned (LIBTCGT_API * LIBTCGT_LAST_OP)(LibTCGTContext * ctx);
typedef unsigned (LIBTCGT_API * LIBTCGT_NEXT_OP)(LibTCGTContext * ctx, unsigned oi);
typedef unsigned (LIBTCGT_API * LIBTCGT_PREV_OP)(LibTCGTContext * ctx, unsigned oi);
typedef bool (LIBTCGT_API * LIBTCGT_REMOVE_OP)(LibTCGTContext * ctx, unsigned oi);
typedef bool (LIBTCGT_API * LIBTCGT_INSERT_BEFORE_OP2)(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs);
typedef bool (LIBTCGT_API * LIBTCGT_INSERT_BEFORE_OP)(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...);
typedef bool (LIBTCGT_API * LIBTCGT_INSERT_AFTER_OP2)(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs);
typedef bool (LIBTCGT_API * LIBTCGT_INSERT_AFTER_OP)(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...);
typedef TCGOpcode(LIBTCGT_API * LIBTCGT_GET_OPC)(LibTCGTContext * ctx, unsigned oi);
typedef unsigned (LIBTCGT_API * LIBTCGT_GET_ARG_COUNT)(LibTCGTContext * ctx, unsigned oi);
typedef unsigned (LIBTCGT_API * LIBTCGT_GET_ARG_WRITE_COUNT)(LibTCGTContext * ctx, unsigned oi);
typedef TCGArgument(LIBTCGT_API * LIBTCGT_GET_ARG)(LibTCGTContext * ctx, unsigned oi, unsigned ai);
typedef TCGArgAccess(LIBTCGT_API * LIBTCGT_GET_ARG_ACCESS)(LibTCGTContext * ctx, unsigned oi, unsigned ai);
typedef TCGArgType(LIBTCGT_API * LIBTCGT_GET_ARG_TYPE)(LibTCGTContext * ctx, unsigned oi, unsigned ai);
typedef unsigned (LIBTCGT_API * LIBTCGT_GET_CALL_OP_ARGS_IN)(LibTCGTContext * ctx, unsigned oi);
typedef unsigned (LIBTCGT_API * LIBTCGT_GET_CALL_OP_ARGS_OUT)(LibTCGTContext * ctx, unsigned oi);
typedef TCGRegType(LIBTCGT_API * LIBTCGT_GET_REG_TYPE)(LibTCGTContext * ctx, TCGArgument reg);
typedef const void * (LIBTCGT_API * LIBTCGT_GET_PTR_HELPER)(LibTCGTContext * ctx, TCGArgument helper);
typedef const char * (LIBTCGT_API * LIBTCGT_GET_NAME_HELPER)(LibTCGTContext * ctx, TCGArgument helper);
typedef const char * (LIBTCGT_API * LIBTCGT_GET_NAME_OPC)(LibTCGTContext * ctx, TCGOpcode opc);
typedef char * (LIBTCGT_API * LIBTCGT_GET_NAME_REGISTER)(LibTCGTContext * ctx, TCGArgument reg);
typedef const char * (LIBTCGT_API * LIBTCGT_GET_NAME_COND)(LibTCGTContext * ctx, TCGCond cond);
typedef const char * (LIBTCGT_API * LIBTCGT_GET_NAME_MEMOP)(LibTCGTContext * ctx, TCGMemOp memop);
typedef const char * (LIBTCGT_API * LIBTCGT_GET_NAME_ALIGNMENT)(LibTCGTContext * ctx, TCGArgument alignment);
typedef unsigned (LIBTCGT_API * LIBTCGT_GET_ID_LABEL)(LibTCGTContext * ctx, TCGArgument label);
typedef TCGOpFlags(LIBTCGT_API * LIBTCGT_GET_FLAGS_OPC)(LibTCGTContext * ctx, TCGOpcode opc);

#if defined(__cplusplus)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#include <malloc.h>

class LibTCGTInstance
{
protected:
	LibTCGTContext * ctx;
	HMODULE h;

	LIBTCGT_ARCH LibTCGT_arch;
	LIBTCGT_BITS LibTCGT_bits;
	LIBTCGT_VERSION LibTCGT_version;
	LIBTCGT_GET_ENV_REGISTER_FROM_OFFSET LibTCGT_get_env_register_from_offset;
	LIBTCGT_REINIT LibTCGT_reinit;
	LIBTCGT_RESET LibTCGT_reset;
	LIBTCGT_CLEANUP LibTCGT_cleanup;
	LIBTCGT_TRANSLATE LibTCGT_translate;
	LIBTCGT_OPTIMIZE LibTCGT_optimize;
	LIBTCGT_FIRST_OP LibTCGT_first_op;
	LIBTCGT_LAST_OP LibTCGT_last_op;
	LIBTCGT_NEXT_OP LibTCGT_next_op;
	LIBTCGT_PREV_OP LibTCGT_prev_op;
	LIBTCGT_REMOVE_OP LibTCGT_remove_op;
	LIBTCGT_INSERT_BEFORE_OP2 LibTCGT_insert_before_op2;
	LIBTCGT_INSERT_AFTER_OP2 LibTCGT_insert_after_op2;
	LIBTCGT_GET_OPC LibTCGT_get_opc;
	LIBTCGT_GET_ARG_COUNT LibTCGT_get_arg_count;
	LIBTCGT_GET_ARG_WRITE_COUNT LibTCGT_get_arg_write_count;
	LIBTCGT_GET_ARG LibTCGT_get_arg;
	LIBTCGT_GET_ARG_ACCESS LibTCGT_get_arg_access;
	LIBTCGT_GET_ARG_TYPE LibTCGT_get_arg_type;
	LIBTCGT_GET_CALL_OP_ARGS_IN LibTCGT_get_call_op_args_in;
	LIBTCGT_GET_CALL_OP_ARGS_OUT LibTCGT_get_call_op_args_out;
	LIBTCGT_GET_REG_TYPE LibTCGT_get_reg_type;
	LIBTCGT_GET_PTR_HELPER LibTCGT_get_ptr_helper;
	LIBTCGT_GET_NAME_HELPER LibTCGT_get_name_helper;
	LIBTCGT_GET_NAME_OPC LibTCGT_get_name_opc;
	LIBTCGT_GET_NAME_REGISTER LibTCGT_get_name_register;
	LIBTCGT_GET_NAME_COND LibTCGT_get_name_cond;
	LIBTCGT_GET_NAME_MEMOP LibTCGT_get_name_memop;
	LIBTCGT_GET_NAME_ALIGNMENT LibTCGT_get_name_alignment;
	LIBTCGT_GET_ID_LABEL LibTCGT_get_id_label;
	LIBTCGT_GET_FLAGS_OPC LibTCGT_get_flags_opc;

	void LIBTCGT_API _reset(void)
	{
		this->ctx = nullptr;
		this->h = nullptr;
		this->LibTCGT_arch = nullptr;
		this->LibTCGT_bits = nullptr;
		this->LibTCGT_version = nullptr;
		this->LibTCGT_get_env_register_from_offset = nullptr;
		this->LibTCGT_reinit = nullptr;
		this->LibTCGT_reset = nullptr;
		this->LibTCGT_cleanup = nullptr;
		this->LibTCGT_translate = nullptr;
		this->LibTCGT_optimize = nullptr;
		this->LibTCGT_first_op = nullptr;
		this->LibTCGT_last_op = nullptr;
		this->LibTCGT_next_op = nullptr;
		this->LibTCGT_prev_op = nullptr;
		this->LibTCGT_remove_op = nullptr;
		this->LibTCGT_insert_before_op2 = nullptr;
		this->LibTCGT_insert_after_op2 = nullptr;
		this->LibTCGT_get_opc = nullptr;
		this->LibTCGT_get_arg_count = nullptr;
		this->LibTCGT_get_arg_write_count = nullptr;
		this->LibTCGT_get_arg = nullptr;
		this->LibTCGT_get_arg_access = nullptr;
		this->LibTCGT_get_arg_type = nullptr;
		this->LibTCGT_get_call_op_args_in = nullptr;
		this->LibTCGT_get_call_op_args_out = nullptr;
		this->LibTCGT_get_reg_type = nullptr;
		this->LibTCGT_get_ptr_helper = nullptr;
		this->LibTCGT_get_name_helper = nullptr;
		this->LibTCGT_get_name_opc = nullptr;
		this->LibTCGT_get_name_register = nullptr;
		this->LibTCGT_get_name_cond = nullptr;
		this->LibTCGT_get_name_memop = nullptr;
		this->LibTCGT_get_name_alignment = nullptr;
		this->LibTCGT_get_id_label = nullptr;
		this->LibTCGT_get_flags_opc = nullptr;
	}

public:

	LIBTCGT_API LibTCGTInstance(TCGArch arch, LibTCGTReadCallback read_callback, void * user_ptr = nullptr, const char * lib_name = nullptr, TCG_FUNC_MALLOC tcg_malloc = nullptr, TCG_FUNC_FREE tcg_free = nullptr)
	{
		this->_reset();

		if (lib_name == nullptr)
		{
			switch (arch)
			{
			case TCGArch::TCG_ARCH_X86:
				lib_name = "LibTCGTranslateX86.dll";
				break;
			case TCGArch::TCG_ARCH_X64:
				lib_name = "LibTCGTranslateX64.dll";
				break;
			case TCGArch::TCG_ARCH_ARM:
				lib_name = "LibTCGTranslateARM.dll";
				break;
			case TCGArch::TCG_ARCH_AARCH64:
				lib_name = "LibTCGTranslateAArch64.dll";
				break;
			default:
				break;
			}
		}

		bool success = false;

		do
		{
			if (lib_name == nullptr)
				break;

			this->h = ::GetModuleHandleA(lib_name);
			if (this->h == nullptr)
			{
				this->h = ::LoadLibraryA(lib_name);
				if (this->h == nullptr)
					break;
			}

			LIBTCGT_INIT pLibTCGT_init = (LIBTCGT_INIT)GetProcAddress(this->h, "LibTCGT_init");
			if (pLibTCGT_init == nullptr)
				break;

			this->LibTCGT_arch = (LIBTCGT_ARCH)GetProcAddress(this->h, "LibTCGT_arch");
			if (this->LibTCGT_arch == nullptr)
				break;

			this->LibTCGT_bits = (LIBTCGT_BITS)GetProcAddress(this->h, "LibTCGT_bits");
			if (this->LibTCGT_bits == nullptr)
				break;

			this->LibTCGT_version = (LIBTCGT_VERSION)GetProcAddress(this->h, "LibTCGT_version");
			if (this->LibTCGT_version == nullptr)
				break;

			this->LibTCGT_get_env_register_from_offset = (LIBTCGT_GET_ENV_REGISTER_FROM_OFFSET)GetProcAddress(this->h, "LibTCGT_get_env_register_from_offset");
			if (this->LibTCGT_get_env_register_from_offset == nullptr)
				break;

			this->LibTCGT_reinit = (LIBTCGT_REINIT)GetProcAddress(this->h, "LibTCGT_reinit");
			if (this->LibTCGT_reinit == nullptr)
				break;

			this->LibTCGT_reset = (LIBTCGT_RESET)GetProcAddress(this->h, "LibTCGT_reset");
			if (this->LibTCGT_reset == nullptr)
				break;

			this->LibTCGT_cleanup = (LIBTCGT_CLEANUP)GetProcAddress(this->h, "LibTCGT_cleanup");
			if (this->LibTCGT_cleanup == nullptr)
				break;

			this->LibTCGT_translate = (LIBTCGT_TRANSLATE)GetProcAddress(this->h, "LibTCGT_translate");
			if (this->LibTCGT_translate == nullptr)
				break;

			this->LibTCGT_optimize = (LIBTCGT_OPTIMIZE)GetProcAddress(this->h, "LibTCGT_optimize");
			if (this->LibTCGT_optimize == nullptr)
				break;

			this->LibTCGT_first_op = (LIBTCGT_FIRST_OP)GetProcAddress(this->h, "LibTCGT_first_op");
			if (this->LibTCGT_first_op == nullptr)
				break;

			this->LibTCGT_last_op = (LIBTCGT_LAST_OP)GetProcAddress(this->h, "LibTCGT_last_op");
			if (this->LibTCGT_last_op == nullptr)
				break;

			this->LibTCGT_next_op = (LIBTCGT_NEXT_OP)GetProcAddress(this->h, "LibTCGT_next_op");
			if (this->LibTCGT_next_op == nullptr)
				break;

			this->LibTCGT_prev_op = (LIBTCGT_PREV_OP)GetProcAddress(this->h, "LibTCGT_prev_op");
			if (this->LibTCGT_prev_op == nullptr)
				break;

			this->LibTCGT_remove_op = (LIBTCGT_REMOVE_OP)GetProcAddress(this->h, "LibTCGT_remove_op");
			if (this->LibTCGT_remove_op == nullptr)
				break;

			this->LibTCGT_insert_before_op2 = (LIBTCGT_INSERT_BEFORE_OP2)GetProcAddress(this->h, "LibTCGT_insert_before_op2");
			if (this->LibTCGT_insert_before_op2 == nullptr)
				break;

			this->LibTCGT_insert_after_op2 = (LIBTCGT_INSERT_AFTER_OP2)GetProcAddress(this->h, "LibTCGT_insert_after_op2");
			if (this->LibTCGT_insert_after_op2 == nullptr)
				break;

			this->LibTCGT_get_opc = (LIBTCGT_GET_OPC)GetProcAddress(this->h, "LibTCGT_get_opc");
			if (this->LibTCGT_get_opc == nullptr)
				break;

			this->LibTCGT_get_arg_count = (LIBTCGT_GET_ARG_COUNT)GetProcAddress(this->h, "LibTCGT_get_arg_count");
			if (this->LibTCGT_get_arg_count == nullptr)
				break;

			this->LibTCGT_get_arg_write_count = (LIBTCGT_GET_ARG_WRITE_COUNT)GetProcAddress(this->h, "LibTCGT_get_arg_write_count");
			if (this->LibTCGT_get_arg_write_count == nullptr)
				break;

			this->LibTCGT_get_arg = (LIBTCGT_GET_ARG)GetProcAddress(this->h, "LibTCGT_get_arg");
			if (this->LibTCGT_get_arg == nullptr)
				break;

			this->LibTCGT_get_arg_access = (LIBTCGT_GET_ARG_ACCESS)GetProcAddress(this->h, "LibTCGT_get_arg_access");
			if (this->LibTCGT_get_arg_access == nullptr)
				break;

			this->LibTCGT_get_arg_type = (LIBTCGT_GET_ARG_TYPE)GetProcAddress(this->h, "LibTCGT_get_arg_type");
			if (this->LibTCGT_get_arg_type == nullptr)
				break;

			this->LibTCGT_get_call_op_args_in = (LIBTCGT_GET_CALL_OP_ARGS_IN)GetProcAddress(this->h, "LibTCGT_get_call_op_args_in");
			if (this->LibTCGT_get_call_op_args_in == nullptr)
				break;

			this->LibTCGT_get_call_op_args_out = (LIBTCGT_GET_CALL_OP_ARGS_OUT)GetProcAddress(this->h, "LibTCGT_get_call_op_args_out");
			if (this->LibTCGT_get_call_op_args_out == nullptr)
				break;

			this->LibTCGT_get_reg_type = (LIBTCGT_GET_REG_TYPE)GetProcAddress(this->h, "LibTCGT_get_reg_type");
			if (this->LibTCGT_get_reg_type == nullptr)
				break;

			this->LibTCGT_get_ptr_helper = (LIBTCGT_GET_PTR_HELPER)GetProcAddress(this->h, "LibTCGT_get_ptr_helper");
			if (this->LibTCGT_get_ptr_helper == nullptr)
				break;

			this->LibTCGT_get_name_helper = (LIBTCGT_GET_NAME_HELPER)GetProcAddress(this->h, "LibTCGT_get_name_helper");
			if (this->LibTCGT_get_name_helper == nullptr)
				break;

			this->LibTCGT_get_name_opc = (LIBTCGT_GET_NAME_OPC)GetProcAddress(this->h, "LibTCGT_get_name_opc");
			if (this->LibTCGT_get_name_opc == nullptr)
				break;

			this->LibTCGT_get_name_register = (LIBTCGT_GET_NAME_REGISTER)GetProcAddress(this->h, "LibTCGT_get_name_register");
			if (this->LibTCGT_get_name_register == nullptr)
				break;

			this->LibTCGT_get_name_cond = (LIBTCGT_GET_NAME_COND)GetProcAddress(this->h, "LibTCGT_get_name_cond");
			if (this->LibTCGT_get_name_cond == nullptr)
				break;

			this->LibTCGT_get_name_memop = (LIBTCGT_GET_NAME_MEMOP)GetProcAddress(this->h, "LibTCGT_get_name_memop");
			if (this->LibTCGT_get_name_memop == nullptr)
				break;

			this->LibTCGT_get_name_alignment = (LIBTCGT_GET_NAME_ALIGNMENT)GetProcAddress(this->h, "LibTCGT_get_name_alignment");
			if (this->LibTCGT_get_name_alignment == nullptr)
				break;

			this->LibTCGT_get_id_label = (LIBTCGT_GET_ID_LABEL)GetProcAddress(this->h, "LibTCGT_get_id_label");
			if (this->LibTCGT_get_id_label == nullptr)
				break;

			this->LibTCGT_get_flags_opc = (LIBTCGT_GET_FLAGS_OPC)GetProcAddress(this->h, "LibTCGT_get_flags_opc");
			if (this->LibTCGT_get_flags_opc == nullptr)
				break;

			if (this->version() < LIBTCGT_VER)
				break;

			this->ctx = pLibTCGT_init(read_callback, user_ptr, tcg_malloc, tcg_free);
			if (this->ctx == nullptr)
				break;

		} while (0);
	}

	LIBTCGT_API ~LibTCGTInstance(void)
	{
		if (this->LibTCGT_cleanup != nullptr)
			this->LibTCGT_cleanup(this->ctx);

		this->_reset();
	}

	HMODULE LIBTCGT_API get_module(void)
	{
		return this->h;
	}

	// Library proxies...

	TCGArch LIBTCGT_API arch(void)
	{
		if (this->LibTCGT_arch == nullptr)
			return TCGArch::TCG_ARCH_UNKNOWN;

		return this->LibTCGT_arch();
	}

	unsigned LIBTCGT_API bits(void)
	{
		if (this->LibTCGT_bits == nullptr)
			return 0;

		return this->LibTCGT_bits();
	}

	unsigned LIBTCGT_API version(void)
	{
		if (this->LibTCGT_version == nullptr)
			return 0;

		return this->LibTCGT_version();
	}

	bool LIBTCGT_API get_env_register_from_offset(uint32_t offset, unsigned * reg, unsigned * off)
	{
		if (this->LibTCGT_get_env_register_from_offset == nullptr)
			return false;

		return this->LibTCGT_get_env_register_from_offset(offset, reg, off);
	}

	void LIBTCGT_API reinit(LibTCGTReadCallback read_callback, void * user_ptr = nullptr)
	{
		if (this->LibTCGT_reinit == nullptr)
			return;

		this->LibTCGT_reinit(this->ctx, read_callback, user_ptr);
	}

	void LIBTCGT_API reset(void)
	{
		if (this->LibTCGT_reset == nullptr)
			return;

		this->LibTCGT_reset(this->ctx);
	}

	uint16_t LIBTCGT_API translate(uint64_t pc, uint16_t max_insns = 0)
	{
		if (this->LibTCGT_translate == nullptr)
			return 0;

		return this->LibTCGT_translate(this->ctx, pc, max_insns);
	}

	void LIBTCGT_API optimize(void)
	{
		if (this->LibTCGT_optimize == nullptr)
			return;

		this->LibTCGT_optimize(this->ctx);
	}

	unsigned LIBTCGT_API first_op(void)
	{
		if (this->LibTCGT_first_op == nullptr)
			return 0;

		return this->LibTCGT_first_op(this->ctx);
	}

	unsigned LIBTCGT_API last_op(void)
	{
		if (this->LibTCGT_last_op == nullptr)
			return 0;

		return this->LibTCGT_last_op(this->ctx);
	}

	unsigned LIBTCGT_API next_op(unsigned oi)
	{
		if (this->LibTCGT_next_op == nullptr)
			return 0;

		return this->LibTCGT_next_op(this->ctx, oi);
	}

	unsigned LIBTCGT_API prev_op(unsigned oi)
	{
		if (this->LibTCGT_prev_op == nullptr)
			return 0;

		return this->LibTCGT_prev_op(this->ctx, oi);
	}

	bool LIBTCGT_API remove_op(unsigned oi)
	{
		if (this->LibTCGT_remove_op == nullptr)
			return false;

		return this->LibTCGT_remove_op(this->ctx, oi);
	}

	bool LIBTCGT_API insert_before_op(unsigned oi, TCGOpcode opc, int argc, ...)
	{
		if (this->LibTCGT_insert_before_op2 == nullptr)
			return false;

		va_list vargs;

		va_start(vargs, argc);

		bool result = this->LibTCGT_insert_before_op2(this->ctx, oi, opc, argc, vargs);

		va_end(vargs);

		return result;
	}

	bool LIBTCGT_API insert_after_op(unsigned oi, TCGOpcode opc, int argc, ...)
	{
		if (this->LibTCGT_insert_after_op2 == nullptr)
			return false;

		va_list vargs;

		va_start(vargs, argc);

		bool result = this->LibTCGT_insert_after_op2(this->ctx, oi, opc, argc, vargs);

		va_end(vargs);

		return result;
	}

	TCGOpcode LIBTCGT_API get_opc(unsigned oi)
	{
		if (this->LibTCGT_get_opc == nullptr)
			return TCGOpcode::NB_OPS;

		return this->LibTCGT_get_opc(this->ctx, oi);
	}

	unsigned LIBTCGT_API get_arg_count(unsigned oi)
	{
		if (this->LibTCGT_get_arg_count == nullptr)
			return 0;

		return this->LibTCGT_get_arg_count(this->ctx, oi);
	}

	unsigned LIBTCGT_API get_arg_write_count(unsigned oi)
	{
		if (this->LibTCGT_get_arg_write_count == nullptr)
			return 0;

		return this->LibTCGT_get_arg_write_count(this->ctx, oi);
	}

	TCGArgument LIBTCGT_API get_arg(unsigned oi, unsigned ai)
	{
		if (this->LibTCGT_get_arg == nullptr)
			return 0;

		return this->LibTCGT_get_arg(this->ctx, oi, ai);
	}

	TCGArgAccess LIBTCGT_API get_arg_access(unsigned oi, unsigned ai)
	{
		if (this->LibTCGT_get_arg_access == nullptr)
			return TCGArgAccess::TCG_ACCESS_UNKNOWN;

		return this->LibTCGT_get_arg_access(this->ctx, oi, ai);
	}

	TCGArgType LIBTCGT_API get_arg_type(unsigned oi, unsigned ai)
	{
		if (this->LibTCGT_get_arg_type == nullptr)
			return TCGArgType::TCG_TYPE_UNKNOWN;

		return this->LibTCGT_get_arg_type(this->ctx, oi, ai);
	}

	unsigned LIBTCGT_API get_call_op_args_in(unsigned oi)
	{
		if (this->LibTCGT_get_call_op_args_in == nullptr)
			return 0;

		return this->LibTCGT_get_call_op_args_in(this->ctx, oi);
	}

	unsigned LIBTCGT_API get_call_op_args_out(unsigned oi)
	{
		if (this->LibTCGT_get_call_op_args_out == nullptr)
			return 0;

		return this->LibTCGT_get_call_op_args_out(this->ctx, oi);
	}

	TCGRegType LIBTCGT_API get_reg_type(TCGArgument reg)
	{
		if (this->LibTCGT_get_reg_type == nullptr)
			return TCGRegType::TCG_REG_UNKNOWN;

		return this->LibTCGT_get_reg_type(this->ctx, reg);
	}

	const void * LIBTCGT_API get_ptr_helper(TCGArgument helper)
	{
		if (this->LibTCGT_get_ptr_helper == nullptr)
			return nullptr;

		return this->LibTCGT_get_ptr_helper(this->ctx, helper);
	}

	const char * LIBTCGT_API get_name_helper(TCGArgument helper)
	{
		if (this->LibTCGT_get_name_helper == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_helper(this->ctx, helper);
	}

	const char * LIBTCGT_API get_name_opc(TCGOpcode opc)
	{
		if (this->LibTCGT_get_name_opc == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_opc(this->ctx, opc);
	}

	char * LIBTCGT_API get_name_register(TCGArgument reg)
	{
		if (this->LibTCGT_get_name_register == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_register(this->ctx, reg);
	}

	const char * LIBTCGT_API get_name_cond(TCGCond cond)
	{
		if (this->LibTCGT_get_name_cond == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_cond(this->ctx, cond);
	}

	const char * LIBTCGT_API get_name_memop(TCGMemOp memop)
	{
		if (this->LibTCGT_get_name_memop == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_memop(this->ctx, memop);
	}

	const char * LIBTCGT_API get_name_alignment(TCGArgument alignment)
	{
		if (this->LibTCGT_get_name_alignment == nullptr)
			return nullptr;

		return this->LibTCGT_get_name_alignment(this->ctx, alignment);
	}

	unsigned LIBTCGT_API get_id_label(TCGArgument label)
	{
		if (this->LibTCGT_get_id_label == nullptr)
			return 0;

		return this->LibTCGT_get_id_label(this->ctx, label);
	}

	TCGOpFlags LIBTCGT_API get_flags_opc(TCGOpcode opc)
	{
		if (this->LibTCGT_get_flags_opc == nullptr)
			return TCGOpFlags::TCG_OPF_NONE;

		return this->LibTCGT_get_flags_opc(this->ctx, opc);
	}

	// Argument helpers

	TCGArgument LIBTCGT_API get_TCGArg(unsigned oi, unsigned ai)
	{
		return this->get_arg(oi, ai);
	}

	TCGBar LIBTCGT_API get_TCGBar(unsigned oi, unsigned ai)
	{
		return (TCGBar)this->get_arg(oi, ai);
	}

	TCGMemOp LIBTCGT_API get_TCGMemOp(unsigned oi, unsigned ai)
	{
		// right shift 4 to skip past the mmu idx.
		return (TCGMemOp)(this->get_arg(oi, ai) >> 4);
	}

	unsigned LIBTCGT_API get_MMUIdx(unsigned oi, unsigned ai)
	{
		return this->get_arg(oi, ai) & 15;
	}

	TCGCond LIBTCGT_API get_TCGCond(unsigned oi, unsigned ai)
	{
		return (TCGCond)this->get_arg(oi, ai);
	}

	uint32_t LIBTCGT_API get_UINT32(unsigned oi, unsigned ai)
	{
		return (uint32_t)this->get_arg(oi, ai);
	}

	uint64_t LIBTCGT_API get_UINT64(unsigned oi, unsigned ai)
	{
		return (uint64_t)this->get_arg(oi, ai);
	}
};

#endif // #if defined(__cplusplus)

#endif // #if defined(LIBTCGT_DYNAMIC_LINKING)

#define LIBTCGT_API_EXPORT				__declspec(dllimport)

#endif // #ifdef LIBTCGT_API_LIBRARY

#ifdef __cplusplus
extern "C"
{
#endif // #ifdef __cplusplus
	LIBTCGT_API_EXPORT TCGArch LIBTCGT_API LibTCGT_arch(void);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_bits(void);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_version(void);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_get_env_register_from_offset(uint32_t offset, unsigned * reg, unsigned * off);

	LIBTCGT_API_EXPORT LibTCGTContext * LIBTCGT_API LibTCGT_init(LibTCGTReadCallback read_callback, void * user_ptr, TCG_FUNC_MALLOC tcg_malloc, TCG_FUNC_FREE tcg_free);

	LIBTCGT_API_EXPORT void LIBTCGT_API LibTCGT_reinit(LibTCGTContext * ctx, LibTCGTReadCallback read_callback, void * user_ptr);

	LIBTCGT_API_EXPORT void LIBTCGT_API LibTCGT_reset(LibTCGTContext * ctx);

	LIBTCGT_API_EXPORT void LIBTCGT_API LibTCGT_cleanup(LibTCGTContext * ctx);

	LIBTCGT_API_EXPORT uint16_t LIBTCGT_API LibTCGT_translate(LibTCGTContext * ctx, uint64_t pc, uint16_t max_insns);

	LIBTCGT_API_EXPORT void LIBTCGT_API LibTCGT_optimize(LibTCGTContext * ctx);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_first_op(LibTCGTContext * ctx);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_last_op(LibTCGTContext * ctx);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_next_op(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_prev_op(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_remove_op(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_insert_before_op2(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_insert_before_op(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_insert_after_op2(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs);

	LIBTCGT_API_EXPORT bool LIBTCGT_API LibTCGT_insert_after_op(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...);

	LIBTCGT_API_EXPORT TCGOpcode LIBTCGT_API LibTCGT_get_opc(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_get_arg_count(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_get_arg_write_count(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT TCGArgument LIBTCGT_API LibTCGT_get_arg(LibTCGTContext * ctx, unsigned oi, unsigned ai);

	LIBTCGT_API_EXPORT TCGArgAccess LIBTCGT_API LibTCGT_get_arg_access(LibTCGTContext * ctx, unsigned oi, unsigned ai);

	LIBTCGT_API_EXPORT TCGArgType LIBTCGT_API LibTCGT_get_arg_type(LibTCGTContext * ctx, unsigned oi, unsigned ai);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_get_call_op_args_in(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_get_call_op_args_out(LibTCGTContext * ctx, unsigned oi);

	LIBTCGT_API_EXPORT TCGRegType LIBTCGT_API LibTCGT_get_reg_type(LibTCGTContext * ctx, TCGArgument reg);

	LIBTCGT_API_EXPORT const void * LIBTCGT_API LibTCGT_get_ptr_helper(LibTCGTContext * ctx, TCGArgument helper);

	LIBTCGT_API_EXPORT const char * LIBTCGT_API LibTCGT_get_name_helper(LibTCGTContext * ctx, TCGArgument helper);

	LIBTCGT_API_EXPORT const char * LIBTCGT_API LibTCGT_get_name_opc(LibTCGTContext * ctx, TCGOpcode opc);

	LIBTCGT_API_EXPORT char * LIBTCGT_API LibTCGT_get_name_register(LibTCGTContext * ctx, TCGArgument reg);

	LIBTCGT_API_EXPORT const char * LIBTCGT_API LibTCGT_get_name_cond(LibTCGTContext * ctx, TCGCond cond);

	LIBTCGT_API_EXPORT const char * LIBTCGT_API LibTCGT_get_name_memop(LibTCGTContext * ctx, TCGMemOp memop);

	LIBTCGT_API_EXPORT const char * LIBTCGT_API LibTCGT_get_name_alignment(LibTCGTContext * ctx, TCGArgument alignment);

	LIBTCGT_API_EXPORT unsigned LIBTCGT_API LibTCGT_get_id_label(LibTCGTContext * ctx, TCGArgument label);

	LIBTCGT_API_EXPORT TCGOpFlags LIBTCGT_API LibTCGT_get_flags_opc(LibTCGTContext * ctx, TCGOpcode opc);

#ifdef __cplusplus
};
#endif // #ifdef __cplusplus

#endif // #ifndef _LIB_TCG_TRANSLATE_H