/*
 * PXI Compatibility Test DLLs
 * 
 * Build all tests at once:
 *   build_pxi_tests.bat
 *
 * Deploy each DLL to PXI, test via CLFN with:
 *   Function: test_N  (where N = 1..6)
 *   Return: Signed 32-bit Integer
 *   Convention: C (cdecl)
 *   No parameters
 *
 * Expected return = test number (1, 2, 3, etc.) if it works.
 * If the DLL fails to LOAD, the problem is in that test's category.
 *
 * Compile each test separately - see build_pxi_tests.bat
 */

/* ===== TEST 1: Minimal C++ (no stdlib usage) ===== */
#ifdef BUILD_TEST1
extern "C" __declspec(dllexport) int test_1(void) {
    return 1;
}
#endif

/* ===== TEST 2: C++ with new/delete (heap) ===== */
#ifdef BUILD_TEST2
extern "C" __declspec(dllexport) int test_2(void) {
    int* p = new int(42);
    int val = *p;
    delete p;
    return (val == 42) ? 2 : -2;
}
#endif

/* ===== TEST 3: C++ with std::string ===== */
#ifdef BUILD_TEST3
#include <string>
extern "C" __declspec(dllexport) int test_3(void) {
    std::string s = "hello";
    return (s.size() == 5) ? 3 : -3;
}
#endif

/* ===== TEST 4: C file I/O (fopen/fclose) ===== */
#ifdef BUILD_TEST4
#include <cstdio>
extern "C" __declspec(dllexport) int test_4(void) {
    /* Just test that fopen/fclose resolve - file doesn't need to exist */
    FILE* f = fopen("__pxi_test_dummy.txt", "w");
    if (f) {
        fprintf(f, "test");
        fclose(f);
    }
    return 4;
}
#endif

/* ===== TEST 5: C++ streams (fstream) ===== */
#ifdef BUILD_TEST5
#include <fstream>
#include <string>
extern "C" __declspec(dllexport) int test_5(void) {
    std::ofstream out("__pxi_test_dummy.txt");
    if (out.is_open()) {
        out << "test";
        out.close();
    }
    return 5;
}
#endif

/* ===== TEST 6: Threading + exceptions (closest to ROSCO) ===== */
#ifdef BUILD_TEST6
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" __declspec(dllexport) int test_6(void) {
    /* String ops */
    std::string s = "hello world";
    
    /* malloc/free */
    char* buf = (char*)malloc(100);
    if (buf) {
        strcpy(buf, s.c_str());
        free(buf);
    }
    
    /* snprintf (uses _vsnprintf internally) */
    char msg[64];
    snprintf(msg, sizeof(msg), "val=%d", 42);
    
    /* getenv */
    const char* env = getenv("PATH");
    (void)env;
    
    return 6;
}
#endif
