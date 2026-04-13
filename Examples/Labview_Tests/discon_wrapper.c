/*
 * Build instructions for this wrapper:
 *
 * Unix (Linux):
 *   gcc -O2 -fPIC -shared discon_wrapper.c -o discon_wrapper.so -ldl
 *
 * Unix (macOS):
 *   clang -O2 -fPIC -dynamiclib discon_wrapper.c -o discon_wrapper.dylib
 *
 * Windows 32-bit (for 32-bit LabVIEW):
 *   gcc -O2 -m32 -shared discon_wrapper.c -o discon_wrapper.dll
 *
 * Notes:
 *   - LabVIEW bitness must match the DLL bitness (use 32-bit build with 32-bit LabVIEW).
 *   - Place libdiscon.dll next to this wrapper DLL, or update LIB_NAME accordingly.
 */

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
  #include <windows.h>
  #include <direct.h>
  #define EXPORT __declspec(dllexport)
  #define OPEN_LIB(name)   LoadLibraryA(name)
  #define GET_SYM(h, sym)  GetProcAddress(h, sym)
  typedef HMODULE lib_handle_t;
  #define LIB_NAME "C:\\Users\\schamot\\Documents\\Code\\ROSCO-C\\rosco\\controller\\build\\win32-mingw\\liblibdiscon_win32.dll"
  #define GETCWD _getcwd
#else
  #include <dlfcn.h>
  #define EXPORT
  #define OPEN_LIB(name)   dlopen(name, RTLD_NOW)
  #define GET_SYM(h, sym)  dlsym(h, sym)
  typedef void* lib_handle_t;
  #define LIB_NAME "/Users/dzalkind/Tools/ROSCO-C/rosco/lib/libdiscon.dylib"
#endif

#define DISCON_IN  "USFLOWT_10_DISCON.IN"
#define SIM_NAME   "rosco_test"
#define AVR_SIZE   500
#define MSG_SIZE   1000
#define NUM_BL     3

typedef void (*DISCON_fn)(float*, int*, const char*, const char*, char*);

static lib_handle_t discon_lib = NULL;
static DISCON_fn    discon_fn  = NULL;

static int load_discon(char* avcMSG) {
    if (discon_fn) return 0;  /* already loaded */

    const char* logName = "C:\\Users\\schamot\\Documents\\Code\\ROSCO-USFLOWT\\Examples\\Labview_Tests\\Logs\\discon_log.txt";
    const char* message = "Loading DISCON library";
    FILE* log_file = fopen(logName, "a");
    fprintf(log_file, "%s\n", message);
    
    discon_lib = OPEN_LIB(LIB_NAME);
    if (!discon_lib) {
        #ifdef _WIN32
        fprintf(log_file, "Failed to load %s: error %lu\n", LIB_NAME, GetLastError());
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: error %lu", LIB_NAME, GetLastError());
        #else
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: %s", LIB_NAME, dlerror());
        fprintf(log_file, "Failed to load %s: %s\n", LIB_NAME, dlerror());
        #endif
        return -1;
    }
    fprintf(log_file, "Successfully loaded %s\n", LIB_NAME);
    
    discon_fn = (DISCON_fn) GET_SYM(discon_lib, "DISCON");
    if (!discon_fn) {
        fprintf(log_file, "Failed to find DISCON symbol in %s\n", LIB_NAME);
        snprintf(avcMSG, MSG_SIZE, "Failed to find DISCON symbol in %s", LIB_NAME);
        fclose(log_file);
        return -1;
    }
    
    fclose(log_file);
    return 0;
}

EXPORT void run_discon(
    /* inputs */
    int   iStatus,
    float time,
    float dt,
    float bld_pitch,
    float gen_speed,
    float rot_speed,
    float wind_speed,
    /* outputs */
    float* gen_torque,
    float* pitch1,
    float* pitch2,
    float* pitch3,
    float* yaw_rate,
    /* error handling */
    int*  aviFAIL,
    char* avcMSG
) {
    *aviFAIL = 0;
    avcMSG[0] = '\0';
    #ifdef _WIN32
    _chdir("C:\\Users\\schamot\\Documents\\Code\\ROSCO-USFLOWT\\Examples\\Labview_Tests");
    #else
        chdir("/Users/dzalkind/Tools/ROSCO-USFLOWT/Examples/Labview_Tests");
    #endif



    char cwd[FILENAME_MAX];  
    GETCWD(cwd, sizeof(cwd));

    const char* message = cwd; // Message to log, which is the current working directory
    FILE* log_file;

    const char* logName = "C:\\Users\\schamot\\Documents\\Code\\ROSCO-USFLOWT\\Examples\\Labview_Tests\\Logs\\discon_log.txt";
    log_file = fopen(logName, "w");
    fprintf(log_file, "%s\n", message);
    
    if (load_discon(avcMSG) != 0) {
        *aviFAIL = -1;
        return;
    }

    /* Build avrSWAP from scalar inputs */
    float avrSWAP[AVR_SIZE] = {0};
    
    avrSWAP[0]  = (float)iStatus;
    avrSWAP[1]  = time;
    avrSWAP[2]  = dt;
    avrSWAP[3]  = bld_pitch;
    avrSWAP[19] = gen_speed;
    avrSWAP[20] = rot_speed;
    avrSWAP[26] = wind_speed;
    avrSWAP[60] = (float)NUM_BL;

    avrSWAP[32] = bld_pitch; // Set all blade pitches to the same value for this test
    avrSWAP[33] = bld_pitch;
    avrSWAP[22] = gen_torque ? *gen_torque : 0.0f; // Initial guess for gen torque, if provided


    /* String buffer sizes */
    avrSWAP[48] = (float)MSG_SIZE;
    avrSWAP[49] = (float)strlen(DISCON_IN);
    avrSWAP[50] = (float)strlen(SIM_NAME);
    fprintf(log_file, "DISCON_IN length: %d, SIM_NAME length: %d\n", (int)avrSWAP[49], (int)avrSWAP[50]);
    fclose(log_file);
    char msg_buf[MSG_SIZE] = {0};
    discon_fn(avrSWAP, aviFAIL, DISCON_IN, SIM_NAME, msg_buf);
    fprintf(log_file, "DISCON returned aviFAIL=%d, msg=%s\n", *aviFAIL, msg_buf);
    strncpy(avcMSG, msg_buf, MSG_SIZE - 1);
    
    /* Extract outputs */
    *gen_torque = avrSWAP[46];
    *pitch1     = avrSWAP[41];
    *pitch2     = avrSWAP[42];
    *pitch3     = avrSWAP[43];
    *yaw_rate   = avrSWAP[47];

    /* Log errors/warnings */
    if (*aviFAIL != 0) {
        const char* logName = "C:\\Users\\schamot\\Documents\\Code\\ROSCO-USFLOWT\\Examples\\Labview_Tests\\Logs\\discon_log.txt";
        FILE* f = fopen(logName, "a");
        if (f) {
            fprintf(f, "iStatus=%d t=%.3f aviFAIL=%d msg=%s\n",
                    iStatus, time, *aviFAIL, avcMSG);
            fclose(f);
        }
    }
}
