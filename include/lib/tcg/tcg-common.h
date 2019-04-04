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

#ifndef _TCG_COMMON_H
#define _TCG_COMMON_H

#include <stdint.h>

#include "lib/tcg/tcg-target.h"

typedef void*(__cdecl * TCG_FUNC_MALLOC)(size_t);

typedef void(__cdecl * TCG_FUNC_FREE)(void *);

/* Helper does not read globals (either directly or through an exception). It
   implies TCG_CALL_NO_WRITE_GLOBALS. */
#define TCG_CALL_NO_READ_GLOBALS    0x0010
   /* Helper does not write globals */
#define TCG_CALL_NO_WRITE_GLOBALS   0x0020
/* Helper can be safely suppressed if the return value is not used. */
#define TCG_CALL_NO_SIDE_EFFECTS    0x0040

#define TCG_MAX_OP_ARGS 16

/* Bits for TCGOpDef->flags, 8 bits available.  */
typedef enum TCGOpFlags : uint8_t {
	TCG_OPF_NONE = 0x0,
	/* Instruction defines the end of a basic block.  */
	TCG_OPF_BB_END = 0x01,
	/* Instruction clobbers call registers and potentially update globals.  */
	TCG_OPF_CALL_CLOBBER = 0x02,
	/* Instruction has side effects: it cannot be removed if its outputs
	are not used, and might trigger exceptions.  */
	TCG_OPF_SIDE_EFFECTS = 0x04,
	/* Instruction operands are 64-bits (otherwise 32-bits).  */
	TCG_OPF_64BIT = 0x08,
	/* Instruction is optional and not implemented by the host, or insn
	is generic and should not be implemened by the host.  */
	TCG_OPF_NOT_PRESENT = 0x10,
} TCGOpFlags;

typedef enum TCGOpcode : unsigned char {
#define DEF(name, oargs, iargs, cargs, flags) INDEX_op_ ## name,
#include "lib/tcg/tcg-opc.h"
#undef DEF
	NB_OPS,
} TCGOpcode;

/* Conditions.  Note that these are laid out for easy manipulation by
the functions below:
bit 0 is used for inverting;
bit 1 is signed,
bit 2 is unsigned,
bit 3 is used with bit 0 for swapping signed/unsigned.  */
typedef enum TCGCond : unsigned char {
	/* non-signed */
	TCG_COND_NEVER   = 0 | 0 | 0 | 0,
	TCG_COND_ALWAYS  = 0 | 0 | 0 | 1,
	TCG_COND_EQ      = 8 | 0 | 0 | 0,
	TCG_COND_NE      = 8 | 0 | 0 | 1,
	/* signed */
	TCG_COND_LT      = 0 | 0 | 2 | 0,
	TCG_COND_GE      = 0 | 0 | 2 | 1,
	TCG_COND_LE      = 8 | 0 | 2 | 0,
	TCG_COND_GT      = 8 | 0 | 2 | 1,
	/* unsigned */
	TCG_COND_LTU     = 0 | 4 | 0 | 0,
	TCG_COND_GEU     = 0 | 4 | 0 | 1,
	TCG_COND_LEU     = 8 | 4 | 0 | 0,
	TCG_COND_GTU     = 8 | 4 | 0 | 1,
	/* invalid (all bits set) */
	TCG_COND_INVALID = 8 | 4 | 2 | 1
} TCGCond;

typedef enum TCGBar : unsigned char {
	/* Used to indicate the type of accesses on which ordering
	is to be ensured.  Modeled after SPARC barriers.

	This is of the form TCG_MO_A_B where A is before B in program order.
	*/
	TCG_MO_LD_LD = 0x01,
	TCG_MO_ST_LD = 0x02,
	TCG_MO_LD_ST = 0x04,
	TCG_MO_ST_ST = 0x08,
	TCG_MO_ALL   = 0x0F,  /* OR of the above */

						/* Used to indicate the kind of ordering which is to be ensured by the
						instruction.  These types are derived from x86/aarch64 instructions.
						It should be noted that these are different from C11 semantics.  */
	TCG_BAR_LDAQ = 0x10,  /* Following ops will not come forward */
	TCG_BAR_STRL = 0x20,  /* Previous ops will not be delayed */
	TCG_BAR_SC   = 0x30,  /* No ops cross barrier; OR of the above */
} TCGBar;

/* Constants for qemu_ld and qemu_st for the Memory Operation field.  */
typedef enum TCGMemOp : unsigned char {
	MO_8 = 0,
	MO_16 = 1,
	MO_32 = 2,
	MO_64 = 3,
	MO_SIZE = 3,   /* Mask for the above.  */

	MO_SIGN = 4,   /* Sign-extended, otherwise zero-extended.  */

	MO_BSWAP = 8,   /* Host reverse endian.  */
#ifdef HOST_WORDS_BIGENDIAN
	MO_LE = MO_BSWAP,
	MO_BE = 0,
#else
	MO_LE = 0,
	MO_BE = MO_BSWAP,
#endif
#ifdef TARGET_WORDS_BIGENDIAN
	MO_TE = MO_BE,
#else
	MO_TE = MO_LE,
#endif

	/* MO_UNALN accesses are never checked for alignment.
	* MO_ALIGN accesses will result in a call to the CPU's
	* do_unaligned_access hook if the guest address is not aligned.
	* The default depends on whether the target CPU defines ALIGNED_ONLY.
	*
	* Some architectures (e.g. ARMv8) need the address which is aligned
	* to a size more than the size of the memory access.
	* Some architectures (e.g. SPARCv9) need an address which is aligned,
	* but less strictly than the natural alignment.
	*
	* MO_ALIGN supposes the alignment size is the size of a memory access.
	*
	* There are three options:
	* - unaligned access permitted (MO_UNALN).
	* - an alignment to the size of an access (MO_ALIGN);
	* - an alignment to a specified size, which may be more or less than
	*   the access size (MO_ALIGN_x where 'x' is a size in bytes);
	*/
	MO_ASHIFT = 4,
	MO_AMASK = 7 << MO_ASHIFT,
#ifdef ALIGNED_ONLY
	MO_ALIGN = 0,
	MO_UNALN = MO_AMASK,
#else
	MO_ALIGN = MO_AMASK,
	MO_UNALN = 0,
#endif
	MO_ALIGN_2 = 1 << MO_ASHIFT,
	MO_ALIGN_4 = 2 << MO_ASHIFT,
	MO_ALIGN_8 = 3 << MO_ASHIFT,
	MO_ALIGN_16 = 4 << MO_ASHIFT,
	MO_ALIGN_32 = 5 << MO_ASHIFT,
	MO_ALIGN_64 = 6 << MO_ASHIFT,

	/* Combinations of the above, for ease of use.  */
	MO_UB = MO_8,
	MO_UW = MO_16,
	MO_UL = MO_32,
	MO_SB = MO_SIGN | MO_8,
	MO_SW = MO_SIGN | MO_16,
	MO_SL = MO_SIGN | MO_32,
	MO_Q = MO_64,

	MO_LEUW = MO_LE | MO_UW,
	MO_LEUL = MO_LE | MO_UL,
	MO_LESW = MO_LE | MO_SW,
	MO_LESL = MO_LE | MO_SL,
	MO_LEQ = MO_LE | MO_Q,

	MO_BEUW = MO_BE | MO_UW,
	MO_BEUL = MO_BE | MO_UL,
	MO_BESW = MO_BE | MO_SW,
	MO_BESL = MO_BE | MO_SL,
	MO_BEQ = MO_BE | MO_Q,

	MO_TEUW = MO_TE | MO_UW,
	MO_TEUL = MO_TE | MO_UL,
	MO_TESW = MO_TE | MO_SW,
	MO_TESL = MO_TE | MO_SL,
	MO_TEQ = MO_TE | MO_Q,

	MO_SSIZE = MO_SIZE | MO_SIGN,

	MO_UNKNOWN

} TCGMemOp;

typedef enum TCGRegType : unsigned char
{
	TCG_REG_UNKNOWN = 0,
	TCG_REG_GLOBAL,
	TCG_REG_LOCAL,
	TCG_REG_TEMP
} TCGRegType;

typedef enum TCGArgAccess : unsigned char
{
	TCG_ACCESS_UNKNOWN = 0,
	TCG_ACCESS_READ,
	TCG_ACCESS_WRITE
} TCGArgAccess;

typedef enum TCGArgType : unsigned char
{
	TCG_TYPE_UNKNOWN = 0,
	TCG_TYPE_REGISTER,
	TCG_TYPE_CONSTANT,
	// A TCG_TYPE_CONSTANT may be one of the following depending on the opc.
	TCG_TYPE_BAR,
	TCG_TYPE_COND,
	TCG_TYPE_MEMOP,
	TCG_TYPE_LABEL,
    TCG_TYPE_HELPER,
	TCG_TYPE_IMMEDIATE,
    // Reserved for future use
	TCG_TYPE_RESERVED1,
	TCG_TYPE_RESERVED2,
	TCG_TYPE_RESERVED3,
	TCG_TYPE_RESERVED4,
	TCG_TYPE_RESERVED5,
	TCG_TYPE_RESERVED6,
	TCG_TYPE_RESERVED7,
	TCG_TYPE_RESERVED8
} TCGArgType;

typedef enum TCGArch : unsigned char {
	TCG_ARCH_UNKNOWN = 0,
	TCG_ARCH_ARM,
	TCG_ARCH_AARCH64,
	TCG_ARCH_X86,
	TCG_ARCH_X64
} TCGArch;

#endif