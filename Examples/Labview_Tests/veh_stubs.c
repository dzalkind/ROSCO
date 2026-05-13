/*
 * veh_stubs.c — Stub implementations of Vectored Exception Handler functions
 * for Phar Lap ETS compatibility.
 *
 * Phar Lap's KERNEL32.dll doesn't have AddVectoredExceptionHandler or
 * RemoveVectoredExceptionHandler. MinGW's DWARF exception handling calls
 * these during DLL init, causing load failure.
 *
 * Providing local stubs makes the linker use these instead of importing
 * from KERNEL32.dll. DWARF C++ exceptions still work within the DLL.
 */

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
