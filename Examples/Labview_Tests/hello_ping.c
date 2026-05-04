/*
 * hello_ping.c — Minimal deployment sanity check for Phar Lap ETS / LabVIEW.
 *
 * Build as a standalone 32-bit DLL with static linkage:
 *   gcc -m32 -shared -static -static-libgcc -o hello_ping.dll hello_ping.c -Wl,--kill-at
 *
 * The resulting DLL should depend only on KERNEL32.dll and msvcrt.dll.
 * Call hello_ping() from a LabVIEW CLFN — it should return 42.
 */

#ifdef _WIN32
  #define EXPORT __declspec(dllexport)
#else
  #define EXPORT
#endif

EXPORT int hello_ping(void) {
    return 42;
}
