#include <stdio.h>
#include <string.h>

#ifdef _WIN32
  #include <windows.h>
  #define EXPORT __declspec(dllexport)
  #define OPEN_LIB(name)   LoadLibraryA(name)
  #define GET_SYM(h, sym)  GetProcAddress(h, sym)
  typedef HMODULE lib_handle_t;
  #define LIB_NAME "libdiscon.dll"
#else
  #include <dlfcn.h>
  #define EXPORT
  #define OPEN_LIB(name)   dlopen(name, RTLD_NOW)
  #define GET_SYM(h, sym)  dlsym(h, sym)
  typedef void* lib_handle_t;
  #define LIB_NAME "/Users/dzalkind/Tools/ROSCO-USFLOWT/rosco/lib/libdiscon.dylib"
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

    discon_lib = OPEN_LIB(LIB_NAME);
    if (!discon_lib) {
#ifdef _WIN32
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: error %lu", LIB_NAME, GetLastError());
#else
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: %s", LIB_NAME, dlerror());
#endif
        return -1;
    }

    discon_fn = (DISCON_fn) GET_SYM(discon_lib, "DISCON");
    if (!discon_fn) {
        snprintf(avcMSG, MSG_SIZE, "Failed to find DISCON symbol in %s", LIB_NAME);
        return -1;
    }

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

    /* String buffer sizes */
    avrSWAP[48] = (float)MSG_SIZE;
    avrSWAP[49] = (float)strlen(DISCON_IN);
    avrSWAP[50] = (float)strlen(SIM_NAME);

    char msg_buf[MSG_SIZE] = {0};
    discon_fn(avrSWAP, aviFAIL, DISCON_IN, SIM_NAME, msg_buf);
    strncpy(avcMSG, msg_buf, MSG_SIZE - 1);

    /* Extract outputs */
    *gen_torque = avrSWAP[46];
    *pitch1     = avrSWAP[41];
    *pitch2     = avrSWAP[42];
    *pitch3     = avrSWAP[43];
    *yaw_rate   = avrSWAP[47];

    /* Log errors/warnings */
    if (*aviFAIL != 0) {
        FILE* f = fopen("Logs/discon_log.txt", "a");
        if (f) {
            fprintf(f, "iStatus=%d t=%.3f aviFAIL=%d msg=%s\n",
                    iStatus, time, *aviFAIL, avcMSG);
            fclose(f);
        }
    }
}
