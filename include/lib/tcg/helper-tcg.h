#ifndef _HELPER_TCG_H
#define _HELPER_TCG_H

#include "lib/tcg/helper-head.h"

#define DEF_HELPER_FLAGS_0(NAME, FLAGS, ret) { HELPER(NAME), tostring(NAME), FLAGS, dh_sizemask(ret, 0) },

#define DEF_HELPER_FLAGS_0v DEF_HELPER_FLAGS_0

#define DEF_HELPER_FLAGS_1(NAME, FLAGS, ret, t1) { HELPER(NAME), tostring(NAME), FLAGS, dh_sizemask(ret, 0) | dh_sizemask(t1, 1) },

#define DEF_HELPER_FLAGS_1v DEF_HELPER_FLAGS_1

#define DEF_HELPER_FLAGS_2(NAME, FLAGS, ret, t1, t2) { HELPER(NAME), tostring(NAME), FLAGS, dh_sizemask(ret, 0) | dh_sizemask(t1, 1) | dh_sizemask(t2, 2) },

#define DEF_HELPER_FLAGS_2v DEF_HELPER_FLAGS_2

#define DEF_HELPER_FLAGS_3(NAME, FLAGS, ret, t1, t2, t3) { HELPER(NAME), tostring(NAME), FLAGS, dh_sizemask(ret, 0) | dh_sizemask(t1, 1) | dh_sizemask(t2, 2) | dh_sizemask(t3, 3) },

#define DEF_HELPER_FLAGS_3v DEF_HELPER_FLAGS_3

#define DEF_HELPER_FLAGS_4(NAME, FLAGS, ret, t1, t2, t3, t4) { HELPER(NAME), tostring(NAME), FLAGS,  dh_sizemask(ret, 0) | dh_sizemask(t1, 1) | dh_sizemask(t2, 2) | dh_sizemask(t3, 3) | dh_sizemask(t4, 4) },

#define DEF_HELPER_FLAGS_4v DEF_HELPER_FLAGS_4

#define DEF_HELPER_FLAGS_5(NAME, FLAGS, ret, t1, t2, t3, t4, t5) { HELPER(NAME), tostring(NAME), FLAGS, dh_sizemask(ret, 0) | dh_sizemask(t1, 1) | dh_sizemask(t2, 2) | dh_sizemask(t3, 3) | dh_sizemask(t4, 4) | dh_sizemask(t5, 5) },

#define DEF_HELPER_FLAGS_5v DEF_HELPER_FLAGS_5

#include "lib/tcg/helpers.h"

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