LibTCGTranslate
===============

A library to translate native code for multiple architectures into Tiny Code Generator (TCG) based intermediate representation (IR), based upon the QEMU translators.

Features
--------

 * Supports ARM (including Thumb and Thumb2), AArch64, x86 and x64 translators.
 * C API exposed.
 * C++ Wrapper.
 * Thread Safe (per LibTCGTContext).
 * MSVC 2017 build solution.
 * Explicit condition code eflags access for the x86 and x64 translators.
 
Build
-----

Open the `.\build\msvc\TCGTranslate.sln` solution in Visual Studio 2017 and Build the solution.

Test Usage
----------

After you build the solution you can examine translation using the TestLibTCGTranslate application. For example, the ARM instruction `str r2, [sp, #-0x4]!` is encoded as `04202DE5` and can be translated as follows:

```
>TestLibTCGTranslate.exe /arm /buffer 04202DE5 /max_insns 1
insn_start 0x100000
        mov_i32 tmp5, sp
        movi_i32 tmp6, 0xfffffffc
        add_i32 tmp5, tmp5, tmp6
        mov_i32 tmp6, r2
        mov_i32 tmp7, tmp5
        qemu_st_i32 tmp6, tmp7, leul
        mov_i32 sp, tmp5
        goto_tb 0x1
        movi_i32 pc, 0x100004
        exit_tb 0x31a30c1
```

License
-------
LibTCGTranslate is licensed under the GNU Lesser General Public license. See COPYING.LIB for more information. QEMU is a trademark of Fabrice Bellard.
