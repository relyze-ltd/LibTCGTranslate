#ifndef _HELPER_PROTO_H
#define _HELPER_PROTO_H

#include "lib/tcg/helper-head.h"

#ifdef DEF_HELPER_PROTO_IMPL
#define DEF_HELPER_BODY		{ return 0; };
#define DEF_HELPER_BODYv	{};
#else
#define DEF_HELPER_BODY		;
#define DEF_HELPER_BODYv	;
#endif

#define DEF_HELPER_FLAGS_0(name, flags, ret) dh_ctype(ret) HELPER(name) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_0v(name, flags, ret) dh_ctype(ret) HELPER(name) DEF_HELPER_BODYv

#define DEF_HELPER_FLAGS_1(name, flags, ret, t1) dh_ctype(ret) HELPER(name) (dh_ctype(t1)) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_1v(name, flags, ret, t1) dh_ctype(ret) HELPER(name) (dh_ctype(t1)) DEF_HELPER_BODYv

#define DEF_HELPER_FLAGS_2(name, flags, ret, t1, t2) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2)) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_2v(name, flags, ret, t1, t2) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2)) DEF_HELPER_BODYv

#define DEF_HELPER_FLAGS_3(name, flags, ret, t1, t2, t3) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3)) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_3v(name, flags, ret, t1, t2, t3) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3)) DEF_HELPER_BODYv

#define DEF_HELPER_FLAGS_4(name, flags, ret, t1, t2, t3, t4) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3), dh_ctype(t4)) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_4v(name, flags, ret, t1, t2, t3, t4) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3), dh_ctype(t4)) DEF_HELPER_BODYv

#define DEF_HELPER_FLAGS_5(name, flags, ret, t1, t2, t3, t4, t5) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3), dh_ctype(t4), dh_ctype(t5)) DEF_HELPER_BODY

#define DEF_HELPER_FLAGS_5v(name, flags, ret, t1, t2, t3, t4, t5) dh_ctype(ret) HELPER(name) (dh_ctype(t1), dh_ctype(t2), dh_ctype(t3), dh_ctype(t4), dh_ctype(t5)) DEF_HELPER_BODYv

#include "lib/tcg/helpers.h"

#undef DEF_HELPER_BODY

#undef DEF_HELPER_BODYv

#undef DEF_HELPER_PROTO_IMPL

#undef DEF_HELPER_FLAGS_0

#undef DEF_HELPER_FLAGS_0v

#undef DEF_HELPER_FLAGS_1

#undef DEF_HELPER_FLAGS_1v

#undef DEF_HELPER_FLAGS_2

#undef DEF_HELPER_FLAGS_2v

#undef DEF_HELPER_FLAGS_3

#undef DEF_HELPER_FLAGS_3v

#undef DEF_HELPER_FLAGS_4

#undef DEF_HELPER_FLAGS_4v

#undef DEF_HELPER_FLAGS_5

#undef DEF_HELPER_FLAGS_5v

#endif