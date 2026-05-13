/*
 * veh_stubs.c — Stub implementations of Vectored Exception Handler functions
 * for Phar Lap ETS compatibility.
 *
 * Phar Lap's KERNEL32.dll doesn't have AddVectoredExceptionHandler or
 * RemoveVectoredExceptionHandler. MinGW's DWARF exception handling calls
 * these during DLL init, causing load failure.
 *
 * We provide BOTH:
 *   1. The stdcall function definitions (_AddVectoredExceptionHandler@8)
 *   2. The __imp__ pointer symbols (__imp__AddVectoredExceptionHandler@8)
 *
 * By satisfying both symbols, the linker has no reason to pull in the
 * archive member from libkernel32.a, so no import table entry is created.
 */

/* --- function stubs --- */

void* __attribute__((stdcall))
AddVectoredExceptionHandler(unsigned long First, void* Handler)
{
    (void)First;
    (void)Handler;
    return (void*)1;
}

unsigned long __attribute__((stdcall))
RemoveVectoredExceptionHandler(void* Handle)
{
    (void)Handle;
    return 1;
}

/* --- __imp__ pointers (satisfies dllimport-style references) --- */

__asm__(
    ".section .data\n"
    ".globl __imp__AddVectoredExceptionHandler@8\n"
    "__imp__AddVectoredExceptionHandler@8:\n"
    "  .long _AddVectoredExceptionHandler@8\n"
    ".globl __imp__RemoveVectoredExceptionHandler@4\n"
    "__imp__RemoveVectoredExceptionHandler@4:\n"
    "  .long _RemoveVectoredExceptionHandler@4\n"
);
