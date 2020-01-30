/*
* LibTCGTranslate
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
#include <SDKDDKVer.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "lib\internal.h"
#include "lib\tcg\tcg.h"
#include "lib\tcg\tcg-op.h"
#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
#include "lib\target\arm\translate.h"
#elif defined(TARGET_X86) || defined(TARGET_X86_64)
#include "lib\target\i386\translate.h"
#endif
#include "LibTCGTranslate.h"

size_t cpu_ld_code(void *env, target_ulong addr, uint8_t * buffer, size_t size)
{
	LibTCGTContext * ctx = (LibTCGTContext*)((CPUArchState *)env)->libtct_ctx;
	if (ctx == nullptr)
		return 0;

	return ctx->read_callback(ctx->user_ptr, addr, buffer, size);
}

uint8_t cpu_ldub_code(void *env, target_ulong addr)
{
	uint8_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(uint8_t)) != sizeof(uint8_t))
		return 0;

	return buffer;
}

int8_t cpu_ldsb_code(void *env, target_ulong addr)
{
	int8_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(int8_t)) != sizeof(int8_t))
		return 0;

	return buffer;
}

uint16_t cpu_lduw_code(void *env, target_ulong addr)
{
	uint16_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(uint16_t)) != sizeof(uint16_t))
		return 0;

	return buffer;
}

int16_t cpu_ldsw_code(void *env, target_ulong addr)
{
	int16_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(int16_t)) != sizeof(int16_t))
		return 0;

	return buffer;
}

uint32_t cpu_ldul_code(void *env, target_ulong addr)
{
	uint32_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(uint32_t)) != sizeof(uint32_t))
		return 0;

	return buffer;
}

int32_t cpu_ldsl_code(void *env, target_ulong addr)
{
	int32_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(int32_t)) != sizeof(int32_t))
		return 0;

	return buffer;
}

uint64_t cpu_lduq_code(void *env, target_ulong addr)
{
	uint64_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(uint64_t)) != sizeof(uint64_t))
		return 0;

	return buffer;
}

int64_t cpu_ldsq_code(void *env, target_ulong addr)
{
	int64_t buffer;

	if (cpu_ld_code(env, addr, (uint8_t *)&buffer, sizeof(int64_t)) != sizeof(int64_t))
		return 0;

	return buffer;
}

TCGArch LIBTCGT_API LibTCGT_arch(void)
{
#if defined(TARGET_ARM)
	return TCGArch::TCG_ARCH_ARM;
#elif defined(TARGET_AARCH64)
	return TCGArch::TCG_ARCH_AARCH64;
#elif defined(TARGET_X86)
	return TCGArch::TCG_ARCH_X86;
#elif defined(TARGET_X86_64)
	return TCGArch::TCG_ARCH_X64;
#else
	return TCGArch::Arch_Unknown;
#endif
}

unsigned LIBTCGT_API LibTCGT_bits(void)
{
	return TARGET_LONG_BITS;
}

unsigned LIBTCGT_API LibTCGT_version(void)
{
	return LIBTCGT_VER;
}

#define GET_REG_OFFSET( S, M, R ) \
if( (offset >= offsetof( S, M )) && (offset < offsetof( S, M ) + sizeof( ((S *)0)->M )) ) \
{ \
	if( reg ) \
		*reg = R; \
	if( off ) \
		*off = (offset - offsetof( S, M )) * 8; \
	return true; \
}

bool LIBTCGT_API LibTCGT_get_env_register_from_offset(uint32_t offset, unsigned * reg, unsigned * off)
{
#if defined(TARGET_ARM) 
	// general
	GET_REG_OFFSET(CPUARMState, regs[0], ARM::Reg::r0)
		GET_REG_OFFSET(CPUARMState, regs[1], ARM::Reg::r1)
		GET_REG_OFFSET(CPUARMState, regs[2], ARM::Reg::r2)
		GET_REG_OFFSET(CPUARMState, regs[3], ARM::Reg::r3)
		GET_REG_OFFSET(CPUARMState, regs[4], ARM::Reg::r4)
		GET_REG_OFFSET(CPUARMState, regs[5], ARM::Reg::r5)
		GET_REG_OFFSET(CPUARMState, regs[6], ARM::Reg::r6)
		GET_REG_OFFSET(CPUARMState, regs[7], ARM::Reg::r7)
		GET_REG_OFFSET(CPUARMState, regs[8], ARM::Reg::r8)
		GET_REG_OFFSET(CPUARMState, regs[9], ARM::Reg::r9)
		GET_REG_OFFSET(CPUARMState, regs[10], ARM::Reg::r10)
		GET_REG_OFFSET(CPUARMState, regs[11], ARM::Reg::r11)
		GET_REG_OFFSET(CPUARMState, regs[12], ARM::Reg::r12)
		GET_REG_OFFSET(CPUARMState, regs[13], ARM::Reg::r13)
		GET_REG_OFFSET(CPUARMState, regs[14], ARM::Reg::r14)
		GET_REG_OFFSET(CPUARMState, regs[15], ARM::Reg::r15)
		// flag
		GET_REG_OFFSET(CPUARMState, CF, ARM::Reg::cf)
		GET_REG_OFFSET(CPUARMState, NF, ARM::Reg::nf)
		GET_REG_OFFSET(CPUARMState, VF, ARM::Reg::vf)
		GET_REG_OFFSET(CPUARMState, ZF, ARM::Reg::zf)
		// misc
		GET_REG_OFFSET(CPUARMState, exclusive_addr, ARM::Reg::exclusive_addr)
		GET_REG_OFFSET(CPUARMState, exclusive_val, ARM::Reg::exclusive_val)
		// end.
#elif defined(TARGET_AARCH64)
	// general
	GET_REG_OFFSET(CPUARMState, regs[0], AARCH64::Reg::r0)
		GET_REG_OFFSET(CPUARMState, regs[1], AARCH64::Reg::r1)
		GET_REG_OFFSET(CPUARMState, regs[2], AARCH64::Reg::r2)
		GET_REG_OFFSET(CPUARMState, regs[3], AARCH64::Reg::r3)
		GET_REG_OFFSET(CPUARMState, regs[4], AARCH64::Reg::r4)
		GET_REG_OFFSET(CPUARMState, regs[5], AARCH64::Reg::r5)
		GET_REG_OFFSET(CPUARMState, regs[6], AARCH64::Reg::r6)
		GET_REG_OFFSET(CPUARMState, regs[7], AARCH64::Reg::r7)
		GET_REG_OFFSET(CPUARMState, regs[8], AARCH64::Reg::r8)
		GET_REG_OFFSET(CPUARMState, regs[9], AARCH64::Reg::r9)
		GET_REG_OFFSET(CPUARMState, regs[10], AARCH64::Reg::r10)
		GET_REG_OFFSET(CPUARMState, regs[11], AARCH64::Reg::r11)
		GET_REG_OFFSET(CPUARMState, regs[12], AARCH64::Reg::r12)
		GET_REG_OFFSET(CPUARMState, regs[13], AARCH64::Reg::r13)
		GET_REG_OFFSET(CPUARMState, regs[14], AARCH64::Reg::r14)
		GET_REG_OFFSET(CPUARMState, regs[15], AARCH64::Reg::r15)
		// flag
		GET_REG_OFFSET(CPUARMState, CF, AARCH64::Reg::cf)
		GET_REG_OFFSET(CPUARMState, NF, AARCH64::Reg::nf)
		GET_REG_OFFSET(CPUARMState, VF, AARCH64::Reg::vf)
		GET_REG_OFFSET(CPUARMState, ZF, AARCH64::Reg::zf)
		// misc
		GET_REG_OFFSET(CPUARMState, exclusive_addr, AARCH64::Reg::exclusive_addr)
		GET_REG_OFFSET(CPUARMState, exclusive_val, AARCH64::Reg::exclusive_val)
		// general
		GET_REG_OFFSET(CPUARMState, pc, AARCH64::Reg::pc)
		GET_REG_OFFSET(CPUARMState, xregs[0], AARCH64::Reg::x0)
		GET_REG_OFFSET(CPUARMState, xregs[1], AARCH64::Reg::x1)
		GET_REG_OFFSET(CPUARMState, xregs[2], AARCH64::Reg::x2)
		GET_REG_OFFSET(CPUARMState, xregs[3], AARCH64::Reg::x3)
		GET_REG_OFFSET(CPUARMState, xregs[4], AARCH64::Reg::x4)
		GET_REG_OFFSET(CPUARMState, xregs[5], AARCH64::Reg::x5)
		GET_REG_OFFSET(CPUARMState, xregs[6], AARCH64::Reg::x6)
		GET_REG_OFFSET(CPUARMState, xregs[7], AARCH64::Reg::x7)
		GET_REG_OFFSET(CPUARMState, xregs[8], AARCH64::Reg::x8)
		GET_REG_OFFSET(CPUARMState, xregs[9], AARCH64::Reg::x9)
		GET_REG_OFFSET(CPUARMState, xregs[10], AARCH64::Reg::x10)
		GET_REG_OFFSET(CPUARMState, xregs[11], AARCH64::Reg::x11)
		GET_REG_OFFSET(CPUARMState, xregs[12], AARCH64::Reg::x12)
		GET_REG_OFFSET(CPUARMState, xregs[13], AARCH64::Reg::x13)
		GET_REG_OFFSET(CPUARMState, xregs[14], AARCH64::Reg::x14)
		GET_REG_OFFSET(CPUARMState, xregs[15], AARCH64::Reg::x15)
		GET_REG_OFFSET(CPUARMState, xregs[16], AARCH64::Reg::x16)
		GET_REG_OFFSET(CPUARMState, xregs[17], AARCH64::Reg::x17)
		GET_REG_OFFSET(CPUARMState, xregs[18], AARCH64::Reg::x18)
		GET_REG_OFFSET(CPUARMState, xregs[19], AARCH64::Reg::x19)
		GET_REG_OFFSET(CPUARMState, xregs[20], AARCH64::Reg::x20)
		GET_REG_OFFSET(CPUARMState, xregs[21], AARCH64::Reg::x21)
		GET_REG_OFFSET(CPUARMState, xregs[22], AARCH64::Reg::x22)
		GET_REG_OFFSET(CPUARMState, xregs[23], AARCH64::Reg::x23)
		GET_REG_OFFSET(CPUARMState, xregs[24], AARCH64::Reg::x24)
		GET_REG_OFFSET(CPUARMState, xregs[25], AARCH64::Reg::x25)
		GET_REG_OFFSET(CPUARMState, xregs[26], AARCH64::Reg::x26)
		GET_REG_OFFSET(CPUARMState, xregs[27], AARCH64::Reg::x27)
		GET_REG_OFFSET(CPUARMState, xregs[28], AARCH64::Reg::x28)
		GET_REG_OFFSET(CPUARMState, xregs[29], AARCH64::Reg::x29)
		GET_REG_OFFSET(CPUARMState, xregs[30], AARCH64::Reg::x30)
		GET_REG_OFFSET(CPUARMState, xregs[31], AARCH64::Reg::x31)
		// extension registers
		GET_REG_OFFSET(CPUARMState, vfp.regs[0 * 2], AARCH64::Reg::v0)
		GET_REG_OFFSET(CPUARMState, vfp.regs[1 * 2], AARCH64::Reg::v1)
		GET_REG_OFFSET(CPUARMState, vfp.regs[2 * 2], AARCH64::Reg::v2)
		GET_REG_OFFSET(CPUARMState, vfp.regs[3 * 2], AARCH64::Reg::v3)
		GET_REG_OFFSET(CPUARMState, vfp.regs[4 * 2], AARCH64::Reg::v4)
		GET_REG_OFFSET(CPUARMState, vfp.regs[5 * 2], AARCH64::Reg::v5)
		GET_REG_OFFSET(CPUARMState, vfp.regs[6 * 2], AARCH64::Reg::v6)
		GET_REG_OFFSET(CPUARMState, vfp.regs[7 * 2], AARCH64::Reg::v7)
		GET_REG_OFFSET(CPUARMState, vfp.regs[8 * 2], AARCH64::Reg::v8)
		GET_REG_OFFSET(CPUARMState, vfp.regs[9 * 2], AARCH64::Reg::v9)
		GET_REG_OFFSET(CPUARMState, vfp.regs[10 * 2], AARCH64::Reg::v10)
		GET_REG_OFFSET(CPUARMState, vfp.regs[11 * 2], AARCH64::Reg::v11)
		GET_REG_OFFSET(CPUARMState, vfp.regs[12 * 2], AARCH64::Reg::v12)
		GET_REG_OFFSET(CPUARMState, vfp.regs[13 * 2], AARCH64::Reg::v13)
		GET_REG_OFFSET(CPUARMState, vfp.regs[14 * 2], AARCH64::Reg::v14)
		GET_REG_OFFSET(CPUARMState, vfp.regs[15 * 2], AARCH64::Reg::v15)
		GET_REG_OFFSET(CPUARMState, vfp.regs[16 * 2], AARCH64::Reg::v16)
		GET_REG_OFFSET(CPUARMState, vfp.regs[17 * 2], AARCH64::Reg::v17)
		GET_REG_OFFSET(CPUARMState, vfp.regs[18 * 2], AARCH64::Reg::v18)
		GET_REG_OFFSET(CPUARMState, vfp.regs[19 * 2], AARCH64::Reg::v19)
		GET_REG_OFFSET(CPUARMState, vfp.regs[20 * 2], AARCH64::Reg::v20)
		GET_REG_OFFSET(CPUARMState, vfp.regs[21 * 2], AARCH64::Reg::v21)
		GET_REG_OFFSET(CPUARMState, vfp.regs[22 * 2], AARCH64::Reg::v22)
		GET_REG_OFFSET(CPUARMState, vfp.regs[23 * 2], AARCH64::Reg::v23)
		GET_REG_OFFSET(CPUARMState, vfp.regs[24 * 2], AARCH64::Reg::v24)
		GET_REG_OFFSET(CPUARMState, vfp.regs[25 * 2], AARCH64::Reg::v25)
		GET_REG_OFFSET(CPUARMState, vfp.regs[26 * 2], AARCH64::Reg::v26)
		GET_REG_OFFSET(CPUARMState, vfp.regs[27 * 2], AARCH64::Reg::v27)
		GET_REG_OFFSET(CPUARMState, vfp.regs[28 * 2], AARCH64::Reg::v28)
		GET_REG_OFFSET(CPUARMState, vfp.regs[29 * 2], AARCH64::Reg::v29)
		GET_REG_OFFSET(CPUARMState, vfp.regs[30 * 2], AARCH64::Reg::v30)
		GET_REG_OFFSET(CPUARMState, vfp.regs[31 * 2], AARCH64::Reg::v31)
		// misc
		GET_REG_OFFSET(CPUARMState, exclusive_high, AARCH64::Reg::exclusive_high)
#elif defined(TARGET_X86) 
	// general
	GET_REG_OFFSET(CPUX86State, regs[0], X86::Reg::eax)
		GET_REG_OFFSET(CPUX86State, regs[1], X86::Reg::ecx)
		GET_REG_OFFSET(CPUX86State, regs[2], X86::Reg::edx)
		GET_REG_OFFSET(CPUX86State, regs[3], X86::Reg::ebx)
		GET_REG_OFFSET(CPUX86State, regs[4], X86::Reg::esp)
		GET_REG_OFFSET(CPUX86State, regs[5], X86::Reg::ebp)
		GET_REG_OFFSET(CPUX86State, regs[6], X86::Reg::esi)
		GET_REG_OFFSET(CPUX86State, regs[7], X86::Reg::edi)
		GET_REG_OFFSET(CPUX86State, eip, X86::Reg::eip)
		// st
		GET_REG_OFFSET(CPUX86State, fpregs[0].d, X86::Reg::st0)
		GET_REG_OFFSET(CPUX86State, fpregs[1].d, X86::Reg::st1)
		GET_REG_OFFSET(CPUX86State, fpregs[2].d, X86::Reg::st2)
		GET_REG_OFFSET(CPUX86State, fpregs[3].d, X86::Reg::st3)
		GET_REG_OFFSET(CPUX86State, fpregs[4].d, X86::Reg::st4)
		GET_REG_OFFSET(CPUX86State, fpregs[5].d, X86::Reg::st5)
		GET_REG_OFFSET(CPUX86State, fpregs[6].d, X86::Reg::st6)
		GET_REG_OFFSET(CPUX86State, fpregs[7].d, X86::Reg::st7)
		// mmx
		GET_REG_OFFSET(CPUX86State, fpregs[0].mmx, X86::Reg::mm0)
		GET_REG_OFFSET(CPUX86State, fpregs[1].mmx, X86::Reg::mm1)
		GET_REG_OFFSET(CPUX86State, fpregs[2].mmx, X86::Reg::mm2)
		GET_REG_OFFSET(CPUX86State, fpregs[3].mmx, X86::Reg::mm3)
		GET_REG_OFFSET(CPUX86State, fpregs[4].mmx, X86::Reg::mm4)
		GET_REG_OFFSET(CPUX86State, fpregs[5].mmx, X86::Reg::mm5)
		GET_REG_OFFSET(CPUX86State, fpregs[6].mmx, X86::Reg::mm6)
		GET_REG_OFFSET(CPUX86State, fpregs[7].mmx, X86::Reg::mm7)
		// zmm (ymm/xmm)
		GET_REG_OFFSET(CPUX86State, xmm_regs[0], X86::Reg::zmm0)
		GET_REG_OFFSET(CPUX86State, xmm_regs[1], X86::Reg::zmm1)
		GET_REG_OFFSET(CPUX86State, xmm_regs[2], X86::Reg::zmm2)
		GET_REG_OFFSET(CPUX86State, xmm_regs[3], X86::Reg::zmm3)
		GET_REG_OFFSET(CPUX86State, xmm_regs[4], X86::Reg::zmm4)
		GET_REG_OFFSET(CPUX86State, xmm_regs[5], X86::Reg::zmm5)
		GET_REG_OFFSET(CPUX86State, xmm_regs[6], X86::Reg::zmm6)
		GET_REG_OFFSET(CPUX86State, xmm_regs[7], X86::Reg::zmm7)
		// segment
		GET_REG_OFFSET(CPUX86State, segs[0], X86::Reg::es)
		GET_REG_OFFSET(CPUX86State, segs[1], X86::Reg::cs)
		GET_REG_OFFSET(CPUX86State, segs[2], X86::Reg::ss)
		GET_REG_OFFSET(CPUX86State, segs[3], X86::Reg::ds)
		GET_REG_OFFSET(CPUX86State, segs[4], X86::Reg::fs)
		GET_REG_OFFSET(CPUX86State, segs[5], X86::Reg::gs)
		// bnd
		GET_REG_OFFSET(CPUX86State, bnd_regs[0].lb, X86::Reg::bnd0_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[0].ub, X86::Reg::bnd0_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[1].lb, X86::Reg::bnd1_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[1].ub, X86::Reg::bnd1_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[2].lb, X86::Reg::bnd2_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[2].ub, X86::Reg::bnd2_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[3].lb, X86::Reg::bnd3_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[3].ub, X86::Reg::bnd3_ub)
		// flag
		GET_REG_OFFSET(CPUX86State, cf, X86::Reg::cf)
		GET_REG_OFFSET(CPUX86State, pf, X86::Reg::pf)
		GET_REG_OFFSET(CPUX86State, af, X86::Reg::af)
		GET_REG_OFFSET(CPUX86State, zf, X86::Reg::zf)
		GET_REG_OFFSET(CPUX86State, sf, X86::Reg::sf)
		GET_REG_OFFSET(CPUX86State, of, X86::Reg::of)
		GET_REG_OFFSET(CPUX86State, df, X86::Reg::df)
		// cc helper registers
		GET_REG_OFFSET(CPUX86State, cc_dst, X86::Reg::cc_dst)
		GET_REG_OFFSET(CPUX86State, cc_src, X86::Reg::cc_src)
		GET_REG_OFFSET(CPUX86State, cc_src2, X86::Reg::cc_src2)
		// end
#elif defined(TARGET_X86_64)
	// general
	GET_REG_OFFSET(CPUX86State, regs[0], X64::Reg::rax)
		GET_REG_OFFSET(CPUX86State, regs[1], X64::Reg::rcx)
		GET_REG_OFFSET(CPUX86State, regs[2], X64::Reg::rdx)
		GET_REG_OFFSET(CPUX86State, regs[3], X64::Reg::rbx)
		GET_REG_OFFSET(CPUX86State, regs[4], X64::Reg::rsp)
		GET_REG_OFFSET(CPUX86State, regs[5], X64::Reg::rbp)
		GET_REG_OFFSET(CPUX86State, regs[6], X64::Reg::rsi)
		GET_REG_OFFSET(CPUX86State, regs[7], X64::Reg::rdi)
		GET_REG_OFFSET(CPUX86State, regs[8], X64::Reg::r8)
		GET_REG_OFFSET(CPUX86State, regs[9], X64::Reg::r9)
		GET_REG_OFFSET(CPUX86State, regs[10], X64::Reg::r10)
		GET_REG_OFFSET(CPUX86State, regs[11], X64::Reg::r11)
		GET_REG_OFFSET(CPUX86State, regs[12], X64::Reg::r12)
		GET_REG_OFFSET(CPUX86State, regs[13], X64::Reg::r13)
		GET_REG_OFFSET(CPUX86State, regs[14], X64::Reg::r14)
		GET_REG_OFFSET(CPUX86State, regs[15], X64::Reg::r15)
		GET_REG_OFFSET(CPUX86State, eip, X64::Reg::rip)
		// st
		GET_REG_OFFSET(CPUX86State, fpregs[0].d, X64::Reg::st0)
		GET_REG_OFFSET(CPUX86State, fpregs[1].d, X64::Reg::st1)
		GET_REG_OFFSET(CPUX86State, fpregs[2].d, X64::Reg::st2)
		GET_REG_OFFSET(CPUX86State, fpregs[3].d, X64::Reg::st3)
		GET_REG_OFFSET(CPUX86State, fpregs[4].d, X64::Reg::st4)
		GET_REG_OFFSET(CPUX86State, fpregs[5].d, X64::Reg::st5)
		GET_REG_OFFSET(CPUX86State, fpregs[6].d, X64::Reg::st6)
		GET_REG_OFFSET(CPUX86State, fpregs[7].d, X64::Reg::st7)
		// mmx
		GET_REG_OFFSET(CPUX86State, fpregs[0].mmx, X64::Reg::mm0)
		GET_REG_OFFSET(CPUX86State, fpregs[1].mmx, X64::Reg::mm1)
		GET_REG_OFFSET(CPUX86State, fpregs[2].mmx, X64::Reg::mm2)
		GET_REG_OFFSET(CPUX86State, fpregs[3].mmx, X64::Reg::mm3)
		GET_REG_OFFSET(CPUX86State, fpregs[4].mmx, X64::Reg::mm4)
		GET_REG_OFFSET(CPUX86State, fpregs[5].mmx, X64::Reg::mm5)
		GET_REG_OFFSET(CPUX86State, fpregs[6].mmx, X64::Reg::mm6)
		GET_REG_OFFSET(CPUX86State, fpregs[7].mmx, X64::Reg::mm7)
		// zmm (ymm/xmm)
		GET_REG_OFFSET(CPUX86State, xmm_regs[0], X64::Reg::zmm0)
		GET_REG_OFFSET(CPUX86State, xmm_regs[1], X64::Reg::zmm1)
		GET_REG_OFFSET(CPUX86State, xmm_regs[2], X64::Reg::zmm2)
		GET_REG_OFFSET(CPUX86State, xmm_regs[3], X64::Reg::zmm3)
		GET_REG_OFFSET(CPUX86State, xmm_regs[4], X64::Reg::zmm4)
		GET_REG_OFFSET(CPUX86State, xmm_regs[5], X64::Reg::zmm5)
		GET_REG_OFFSET(CPUX86State, xmm_regs[6], X64::Reg::zmm6)
		GET_REG_OFFSET(CPUX86State, xmm_regs[7], X64::Reg::zmm7)
		GET_REG_OFFSET(CPUX86State, xmm_regs[8], X64::Reg::zmm8)
		GET_REG_OFFSET(CPUX86State, xmm_regs[9], X64::Reg::zmm9)
		GET_REG_OFFSET(CPUX86State, xmm_regs[10], X64::Reg::zmm10)
		GET_REG_OFFSET(CPUX86State, xmm_regs[11], X64::Reg::zmm11)
		GET_REG_OFFSET(CPUX86State, xmm_regs[12], X64::Reg::zmm12)
		GET_REG_OFFSET(CPUX86State, xmm_regs[13], X64::Reg::zmm13)
		GET_REG_OFFSET(CPUX86State, xmm_regs[14], X64::Reg::zmm14)
		GET_REG_OFFSET(CPUX86State, xmm_regs[15], X64::Reg::zmm15)
		GET_REG_OFFSET(CPUX86State, xmm_regs[16], X64::Reg::zmm16)
		GET_REG_OFFSET(CPUX86State, xmm_regs[17], X64::Reg::zmm17)
		GET_REG_OFFSET(CPUX86State, xmm_regs[18], X64::Reg::zmm18)
		GET_REG_OFFSET(CPUX86State, xmm_regs[19], X64::Reg::zmm19)
		GET_REG_OFFSET(CPUX86State, xmm_regs[20], X64::Reg::zmm20)
		GET_REG_OFFSET(CPUX86State, xmm_regs[21], X64::Reg::zmm21)
		GET_REG_OFFSET(CPUX86State, xmm_regs[22], X64::Reg::zmm22)
		GET_REG_OFFSET(CPUX86State, xmm_regs[23], X64::Reg::zmm23)
		GET_REG_OFFSET(CPUX86State, xmm_regs[24], X64::Reg::zmm24)
		GET_REG_OFFSET(CPUX86State, xmm_regs[25], X64::Reg::zmm25)
		GET_REG_OFFSET(CPUX86State, xmm_regs[26], X64::Reg::zmm26)
		GET_REG_OFFSET(CPUX86State, xmm_regs[27], X64::Reg::zmm27)
		GET_REG_OFFSET(CPUX86State, xmm_regs[28], X64::Reg::zmm28)
		GET_REG_OFFSET(CPUX86State, xmm_regs[29], X64::Reg::zmm29)
		GET_REG_OFFSET(CPUX86State, xmm_regs[30], X64::Reg::zmm30)
		GET_REG_OFFSET(CPUX86State, xmm_regs[31], X64::Reg::zmm31)
		// segment
		GET_REG_OFFSET(CPUX86State, segs[0], X64::Reg::es)
		GET_REG_OFFSET(CPUX86State, segs[1], X64::Reg::cs)
		GET_REG_OFFSET(CPUX86State, segs[2], X64::Reg::ss)
		GET_REG_OFFSET(CPUX86State, segs[3], X64::Reg::ds)
		GET_REG_OFFSET(CPUX86State, segs[4], X64::Reg::fs)
		GET_REG_OFFSET(CPUX86State, segs[5], X64::Reg::gs)
		// bnd
		GET_REG_OFFSET(CPUX86State, bnd_regs[0].lb, X64::Reg::bnd0_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[0].ub, X64::Reg::bnd0_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[1].lb, X64::Reg::bnd1_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[1].ub, X64::Reg::bnd1_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[2].lb, X64::Reg::bnd2_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[2].ub, X64::Reg::bnd2_ub)
		GET_REG_OFFSET(CPUX86State, bnd_regs[3].lb, X64::Reg::bnd3_lb)
		GET_REG_OFFSET(CPUX86State, bnd_regs[3].ub, X64::Reg::bnd3_ub)
		// flag
		GET_REG_OFFSET(CPUX86State, cf, X64::Reg::cf)
		GET_REG_OFFSET(CPUX86State, pf, X64::Reg::pf)
		GET_REG_OFFSET(CPUX86State, af, X64::Reg::af)
		GET_REG_OFFSET(CPUX86State, zf, X64::Reg::zf)
		GET_REG_OFFSET(CPUX86State, sf, X64::Reg::sf)
		GET_REG_OFFSET(CPUX86State, of, X64::Reg::of)
		GET_REG_OFFSET(CPUX86State, df, X64::Reg::df)
		// cc helper registers
		GET_REG_OFFSET(CPUX86State, cc_dst, X64::Reg::cc_dst)
		GET_REG_OFFSET(CPUX86State, cc_src, X64::Reg::cc_src)
		GET_REG_OFFSET(CPUX86State, cc_src2, X64::Reg::cc_src2)
		// end.
#endif
	if (reg)
		*reg = 0;

	if (off)
		*off = 0;

	return false;
}

#undef GET_REG_OFFSET

LibTCGTContext * LIBTCGT_API LibTCGT_init(LibTCGTReadCallback read_callback, void * user_ptr, TCG_FUNC_MALLOC tcg_malloc, TCG_FUNC_FREE tcg_free)
{
	if (read_callback == nullptr)
		return nullptr;

	if (tcg_malloc == nullptr)
		tcg_malloc = &malloc;

	if (tcg_free == nullptr)
		tcg_free = &free;

	LibTCGTContext * ctx = (LibTCGTContext *)tcg_malloc(sizeof(LibTCGTContext));
	if (ctx == nullptr)
		return nullptr;

	memset(ctx, 0, sizeof(LibTCGTContext));

	ctx->read_callback = read_callback;

	ctx->user_ptr = user_ptr;

	arch_cpu_init(&ctx->cpu.env);

	tcg_context_init(&ctx->s);

	tcg_context_reset(&ctx->s);

	ctx->s.tcg_free = tcg_free;
	ctx->s.tcg_malloc = tcg_malloc;

	arch_translate_init(&ctx->s);

	ctx->cpu.env.libtct_ctx = ctx;

	return ctx;
}

void LIBTCGT_API LibTCGT_reinit(LibTCGTContext * ctx, LibTCGTReadCallback read_callback, void * user_ptr)
{
	if (ctx == nullptr)
		return;

	LibTCGT_reset(ctx);

	ctx->read_callback = read_callback;

	ctx->user_ptr = user_ptr;
}

void LIBTCGT_API LibTCGT_reset(LibTCGTContext * ctx)
{
	if (ctx == nullptr)
		return;

	tcg_pool_reset(&ctx->s);

	tcg_context_reset(&ctx->s);

	memset(&ctx->tb, 0, sizeof(TranslationBlock));
}

void LIBTCGT_API LibTCGT_cleanup(LibTCGTContext * ctx)
{
	if (ctx == nullptr)
		return;
	
	TCG_FUNC_FREE tcg_free = ctx->s.tcg_free;

	tcg_pool_reset(&ctx->s);

	tcg_free(ctx);
}

uint16_t LIBTCGT_API LibTCGT_translate(LibTCGTContext * ctx, uint64_t pc, uint16_t max_insns)
{
	if (ctx == nullptr)
		return 0;

#if defined(TARGET_ARM)
	ctx->cpu.env.thumb = (pc & 1);
	ctx->cpu.env.pc = (pc & ~1);
	ctx->cpu.env.regs[15] = (target_ulong)ctx->cpu.env.pc;
#elif defined(TARGET_AARCH64)
	ctx->cpu.env.pc = pc;
	ctx->cpu.env.regs[15] = ctx->cpu.env.pc;
#elif defined(TARGET_X86) || defined(TARGET_X86_64)
	ctx->cpu.env.eip = (target_ulong)pc;
#endif

	cpu_get_tb_cpu_state(&ctx->cpu.env, &ctx->tb.pc, &ctx->tb.cs_base, &ctx->tb.flags);

	ctx->tb.cflags = (ctx->tb.cflags & ~CF_COUNT_MASK) | (max_insns & CF_COUNT_MASK);

	arch_gen_intermediate_code(&ctx->s, &ctx->cpu.env, &ctx->tb);

	return ctx->tb.icount;
}

void LIBTCGT_API LibTCGT_optimize(LibTCGTContext * ctx, bool opt, bool dce)
{
	if (ctx == nullptr)
		return;

	if(opt)
		tcg_optimize(&ctx->s);

	if (dce) {
		uint8_t * temp_state = (uint8_t *)tcg_malloc(&ctx->s, ctx->s.nb_temps + ctx->s.nb_indirects);
		if (temp_state == nullptr)
			return;

		liveness_pass_1(&ctx->s, temp_state);

		if (ctx->s.nb_indirects > 0) {
			if (liveness_pass_2(&ctx->s, temp_state))
				liveness_pass_1(&ctx->s, temp_state);
		}
	}
}

unsigned LIBTCGT_API LibTCGT_first_op(LibTCGTContext * ctx)
{
	if (ctx == nullptr)
		return 0;

	return ctx->s.gen_op_buf[0].next;
}

unsigned LIBTCGT_API LibTCGT_last_op(LibTCGTContext * ctx)
{
	if (ctx == nullptr)
		return 0;

	return ctx->s.gen_op_buf[0].prev;
}

unsigned LIBTCGT_API LibTCGT_next_op(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	return ctx->s.gen_op_buf[oi].next;
}

unsigned LIBTCGT_API LibTCGT_prev_op(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	return ctx->s.gen_op_buf[oi].prev;
}

bool LIBTCGT_API LibTCGT_remove_op(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return false;

	tcg_op_remove(&ctx->s, &ctx->s.gen_op_buf[oi]);

	return true;
}

bool LIBTCGT_API LibTCGT_insert_before_op2(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return false;

	TCGOp * op = tcg_op_insert_before(&ctx->s, &ctx->s.gen_op_buf[oi], opc, argc);
	if (op == nullptr)
		return false;

	if (argc > 0)
	{
		TCGArg * args = &ctx->s.gen_opparam_buf[op->args];

		for (int i = 0; i < argc; i++) {
			args[i] = (TCGArg)va_arg(vargs, TCGArgument);
		}
	}

	return true;
}

bool LIBTCGT_API LibTCGT_insert_before_op(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...)
{
	va_list vargs;

	va_start(vargs, argc);

	bool result = LibTCGT_insert_before_op2(ctx, oi, opc, argc, vargs);

	va_end(vargs);

	return result;
}

bool LIBTCGT_API LibTCGT_insert_after_op2(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, va_list vargs)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return false;

	TCGOp * op = tcg_op_insert_after(&ctx->s, &ctx->s.gen_op_buf[oi], opc, argc);
	if (op == nullptr)
		return false;

	if (argc > 0)
	{
		TCGArg * args = &ctx->s.gen_opparam_buf[op->args];

		for (int i = 0; i < argc; i++) {
			args[i] = (TCGArg)va_arg(vargs, TCGArgument);
		}
	}

	return true;
}

bool LIBTCGT_API LibTCGT_insert_after_op(LibTCGTContext * ctx, unsigned oi, TCGOpcode opc, int argc, ...)
{
	va_list vargs;

	va_start(vargs, argc);

	bool result = LibTCGT_insert_after_op2(ctx, oi, opc, argc, vargs);

	va_end(vargs);

	return result;
}

TCGOpcode LIBTCGT_API LibTCGT_get_opc(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return TCGOpcode::NB_OPS;

	return ctx->s.gen_op_buf[oi].opc;
}

unsigned LIBTCGT_API LibTCGT_get_arg_count(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	TCGOpcode opc = ctx->s.gen_op_buf[oi].opc;

	if (opc == INDEX_op_call)
		return ctx->s.gen_op_buf[oi].callo + ctx->s.gen_op_buf[oi].calli + tcg_op_defs[opc].nb_args;

	return tcg_op_defs[opc].nb_args;
}

unsigned LIBTCGT_API LibTCGT_get_arg_write_count(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	TCGOpcode opc = ctx->s.gen_op_buf[oi].opc;

	if (opc == INDEX_op_call)
		return ctx->s.gen_op_buf[oi].callo;

	return tcg_op_defs[opc].nb_oargs;
}

TCGArgument LIBTCGT_API LibTCGT_get_arg(LibTCGTContext * ctx, unsigned oi, unsigned ai)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	TCGOpcode opc = ctx->s.gen_op_buf[oi].opc;

	if (opc == INDEX_op_call)
	{
		if (ai >= ctx->s.gen_op_buf[oi].callo + ctx->s.gen_op_buf[oi].calli + tcg_op_defs[opc].nb_args)
			return 0;
	}
	else
	{
		if (ai >= tcg_op_defs[opc].nb_args)
			return 0;
	}

	unsigned args = ctx->s.gen_op_buf[oi].args;

	if (args + ai >= OPPARAM_BUF_SIZE)
		return 0;

	return ctx->s.gen_opparam_buf[args + ai];
}

TCGArgAccess LIBTCGT_API LibTCGT_get_arg_access(LibTCGTContext * ctx, unsigned oi, unsigned ai)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return TCGArgAccess::TCG_ACCESS_UNKNOWN;

	TCGOpcode opc = ctx->s.gen_op_buf[oi].opc;

	if (opc == INDEX_op_call)
	{
		if (ai < ctx->s.gen_op_buf[oi].callo)
			return TCGArgAccess::TCG_ACCESS_WRITE;

		if (ai >= ctx->s.gen_op_buf[oi].callo + ctx->s.gen_op_buf[oi].calli + tcg_op_defs[opc].nb_args)
			return TCGArgAccess::TCG_ACCESS_UNKNOWN;
	}
	else
	{
		if (ai < tcg_op_defs[opc].nb_oargs)
			return TCGArgAccess::TCG_ACCESS_WRITE;

		if (ai >= tcg_op_defs[opc].nb_args)
			return TCGArgAccess::TCG_ACCESS_UNKNOWN;
	}

	return TCGArgAccess::TCG_ACCESS_READ;
}

TCGArgType LIBTCGT_API LibTCGT_get_arg_type(LibTCGTContext * ctx, unsigned oi, unsigned ai)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return TCGArgType::TCG_TYPE_UNKNOWN;

	TCGOpcode opc = ctx->s.gen_op_buf[oi].opc;

	if (opc == INDEX_op_call)
	{
		if (ai < ctx->s.gen_op_buf[oi].callo + ctx->s.gen_op_buf[oi].calli)
			return TCGArgType::TCG_TYPE_REGISTER;

		if (ai < ctx->s.gen_op_buf[oi].callo + ctx->s.gen_op_buf[oi].calli + tcg_op_defs[opc].nb_args)
			return TCGArgType::TCG_TYPE_CONSTANT;

		return TCGArgType::TCG_TYPE_UNKNOWN;
	}

	if (ai < tcg_op_defs[opc].nb_oargs + tcg_op_defs[opc].nb_iargs)
		return TCGArgType::TCG_TYPE_REGISTER;

	if (ai < tcg_op_defs[opc].nb_oargs + tcg_op_defs[opc].nb_iargs + tcg_op_defs[opc].nb_cargs)
		return TCGArgType::TCG_TYPE_CONSTANT;

	return TCGArgType::TCG_TYPE_UNKNOWN;
}

unsigned LIBTCGT_API LibTCGT_get_call_op_args_in(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	if (ctx->s.gen_op_buf[oi].opc != TCGOpcode::INDEX_op_call)
		return 0;

	return ctx->s.gen_op_buf[oi].calli;
}

unsigned LIBTCGT_API LibTCGT_get_call_op_args_out(LibTCGTContext * ctx, unsigned oi)
{
	if (ctx == nullptr || oi == 0 || oi >= OPC_BUF_SIZE)
		return 0;

	if (ctx->s.gen_op_buf[oi].opc != TCGOpcode::INDEX_op_call)
		return 0;

	return ctx->s.gen_op_buf[oi].callo;
}

TCGRegType LIBTCGT_API LibTCGT_get_reg_type(LibTCGTContext * ctx, TCGArgument reg)
{
	if (ctx == nullptr)
		return TCGRegType::TCG_REG_UNKNOWN;

	return tcg_get_reg_type(&ctx->s, (int)reg);
}

const void * LIBTCGT_API LibTCGT_get_ptr_helper(LibTCGTContext * ctx, TCGArgument helper)
{
	if (ctx == nullptr)
		return nullptr;

	return tcg_find_helper_func(&ctx->s, (int)helper);
}

const char * LIBTCGT_API LibTCGT_get_name_helper(LibTCGTContext * ctx, TCGArgument helper)
{
	if (ctx == nullptr)
		return nullptr;

	return tcg_find_helper(&ctx->s, (int)helper);
}

const char * LIBTCGT_API LibTCGT_get_name_opc(LibTCGTContext * ctx, TCGOpcode opc)
{
	if (ctx == nullptr || opc >= tcg_op_defs_max)
		return nullptr;

	return tcg_op_defs[opc].name;
}

char * LIBTCGT_API LibTCGT_get_name_register(LibTCGTContext * ctx, TCGArgument reg)
{
	if (ctx == nullptr)
		return nullptr;

	char * buffer = (char *)tcg_malloc(&ctx->s, 32);
	if (buffer == nullptr)
		return nullptr;

	return tcg_get_arg_str_idx(&ctx->s, buffer, 32, (int)reg);
}

const char * LIBTCGT_API LibTCGT_get_name_cond(LibTCGTContext * ctx, TCGCond cond)
{
	if (ctx == nullptr || cond >= cond_name_max)
		return nullptr;

	return cond_name[cond];
}

const char * LIBTCGT_API LibTCGT_get_name_memop(LibTCGTContext * ctx, TCGMemOp memop)
{
	if (ctx == nullptr)
		return nullptr;

	unsigned alignment_idx = (memop & MO_AMASK) >> MO_ASHIFT;

	if (alignment_idx >= alignment_name_max)
		return nullptr;

	unsigned ldst_idx = memop & (MO_BSWAP | MO_SSIZE);

	if (ldst_idx >= ldst_name_max)
		return nullptr;

	char * buffer = (char *)tcg_malloc(&ctx->s, 32);
	if (buffer == nullptr)
		return nullptr;

	snprintf(buffer, 32, "%s%s", alignment_name[alignment_idx], ldst_name[ldst_idx]);

	return buffer;
}

const char * LIBTCGT_API LibTCGT_get_name_alignment(LibTCGTContext * ctx, TCGArgument alignment)
{
	if (ctx == nullptr || alignment >= alignment_name_max)
		return nullptr;

	return alignment_name[alignment];
}

unsigned LIBTCGT_API LibTCGT_get_id_label(LibTCGTContext * ctx, TCGArgument label)
{
	if (ctx == nullptr)
		return 0;

	return arg_label((TCGArg)label)->id;
}

TCGOpFlags LIBTCGT_API LibTCGT_get_flags_opc(LibTCGTContext * ctx, TCGOpcode opc)
{
	if (ctx == nullptr || opc >= tcg_op_defs_max)
		return TCGOpFlags::TCG_OPF_NONE;

	return (TCGOpFlags)tcg_op_defs[opc].flags;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

