#ifndef _HELPER_HEAD_H
#define _HELPER_HEAD_H

#include "lib/internal.h"
#include "lib/tcg/tcg.h"
#if defined(TARGET_ARM) || defined(TARGET_AARCH64)
#include "lib/target/arm/cpu.h"
#elif defined(TARGET_X86) || defined(TARGET_X86_64)
#include "lib/target/i386/cpu.h"
#endif

#define HELPER(name) glue(helper_, name)

#define HELPER_IDX(name) glue(helper_idx_, name)

#define GET_TCGV_i32 GET_TCGV_I32
#define GET_TCGV_i64 GET_TCGV_I64
#define GET_TCGV_ptr GET_TCGV_PTR

#define dh_alias_i32 i32
#define dh_alias_s32 i32
#define dh_alias_int i32
#define dh_alias_i64 i64
#define dh_alias_s64 i64
#define dh_alias_f32 i32
#define dh_alias_f64 i64
#define dh_alias_ptr ptr
#define dh_alias_void void
#define dh_alias_noreturn noreturn
#define dh_alias(t) glue(dh_alias_, t)

#define dh_ctype_i32 uint32_t
#define dh_ctype_s32 int32_t
#define dh_ctype_int int
#define dh_ctype_i64 uint64_t
#define dh_ctype_s64 int64_t
#define dh_ctype_f32 float32
#define dh_ctype_f64 float64
#define dh_ctype_ptr void *
#define dh_ctype_void void
#define dh_ctype_noreturn void QEMU_NORETURN
#define dh_ctype(t) dh_ctype_##t

#ifdef NEED_CPU_H
# ifdef TARGET_LONG_BITS
#  if TARGET_LONG_BITS == 32
#   define dh_alias_tl i32
#  else
#   define dh_alias_tl i64
#  endif
# endif
# define dh_alias_env ptr
# define dh_ctype_tl target_ulong
# define dh_ctype_env CPUArchState *
#endif

#define dh_retvar_decl0_void void
#define dh_retvar_decl0_noreturn void
#define dh_retvar_decl0_i32 TCGv_i32 retval
#define dh_retvar_decl0_i64 TCGv_i64 retval
#define dh_retvar_decl0_ptr TCGv_ptr retval
#define dh_retvar_decl0(t) glue(dh_retvar_decl0_, dh_alias(t))

#define dh_retvar_decl_void
#define dh_retvar_decl_noreturn
#define dh_retvar_decl_i32 TCGv_i32 retval,
#define dh_retvar_decl_i64 TCGv_i64 retval,
#define dh_retvar_decl_ptr TCGv_ptr retval,
#define dh_retvar_decl(t) glue(dh_retvar_decl_, dh_alias(t))

#define dh_retvar_void TCG_CALL_DUMMY_ARG
#define dh_retvar_noreturn TCG_CALL_DUMMY_ARG
#define dh_retvar_i32 GET_TCGV_i32(retval)
#define dh_retvar_i64 GET_TCGV_i64(retval)
#define dh_retvar_ptr GET_TCGV_ptr(retval)
#define dh_retvar(t) glue(dh_retvar_, dh_alias(t))

#define dh_is_64bit_void 0
#define dh_is_64bit_noreturn 0
#define dh_is_64bit_i32 0
#define dh_is_64bit_i64 1
#define dh_is_64bit_ptr (sizeof(void *) == 8)
#define dh_is_64bit(t) glue(dh_is_64bit_, dh_alias(t))

#define dh_is_signed_void 0
#define dh_is_signed_noreturn 0
#define dh_is_signed_i32 0
#define dh_is_signed_s32 1
#define dh_is_signed_i64 0
#define dh_is_signed_s64 1
#define dh_is_signed_f32 0
#define dh_is_signed_f64 0
#define dh_is_signed_tl  0
#define dh_is_signed_int 1
#define dh_is_signed_ptr 0
#define dh_is_signed_env dh_is_signed_ptr
#define dh_is_signed(t) dh_is_signed_##t

#define dh_sizemask(t, n) \
  ((dh_is_64bit(t) << (n*2)) | (dh_is_signed(t) << (n*2+1)))

#define dh_arg(t, n) \
  glue(GET_TCGV_, dh_alias(t))(glue(arg, n))

#define dh_arg_decl(t, n) glue(TCGv_, dh_alias(t)) glue(arg, n)

#define DEF_HELPER_0(name, ret) \
    DEF_HELPER_FLAGS_0(name, 0, ret)

#define DEF_HELPER_0v(name, ret) \
    DEF_HELPER_FLAGS_0v(name, 0, ret)

#define DEF_HELPER_1(name, ret, t1) \
    DEF_HELPER_FLAGS_1(name, 0, ret, t1)

#define DEF_HELPER_1v(name, ret, t1) \
    DEF_HELPER_FLAGS_1v(name, 0, ret, t1)

#define DEF_HELPER_2(name, ret, t1, t2) \
    DEF_HELPER_FLAGS_2(name, 0, ret, t1, t2)

#define DEF_HELPER_2v(name, ret, t1, t2) \
    DEF_HELPER_FLAGS_2v(name, 0, ret, t1, t2)

#define DEF_HELPER_3(name, ret, t1, t2, t3) \
    DEF_HELPER_FLAGS_3(name, 0, ret, t1, t2, t3)

#define DEF_HELPER_3v(name, ret, t1, t2, t3) \
    DEF_HELPER_FLAGS_3v(name, 0, ret, t1, t2, t3)

#define DEF_HELPER_4(name, ret, t1, t2, t3, t4) \
    DEF_HELPER_FLAGS_4(name, 0, ret, t1, t2, t3, t4)

#define DEF_HELPER_4v(name, ret, t1, t2, t3, t4) \
    DEF_HELPER_FLAGS_4v(name, 0, ret, t1, t2, t3, t4)

#define DEF_HELPER_5(name, ret, t1, t2, t3, t4, t5) \
    DEF_HELPER_FLAGS_5(name, 0, ret, t1, t2, t3, t4, t5)

#define DEF_HELPER_5v(name, ret, t1, t2, t3, t4, t5) \
    DEF_HELPER_FLAGS_5v(name, 0, ret, t1, t2, t3, t4, t5)

#endif