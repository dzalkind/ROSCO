/*
 * discon_wrapper.c — LabVIEW-friendly C ABI shim for ROSCO's DISCON controller.
 *
 * This wrapper converts individual scalar inputs/outputs into the Bladed-style
 * avrSWAP float array that DISCON expects.  ROSCO's libdiscon DLL is loaded
 * at runtime via LoadLibraryA/GetProcAddress and unloaded on shutdown to
 * avoid memory leaks across repeated runs.
 *
 * Exported functions (called from LabVIEW CLFN):
 *   hello_ping     — returns 42; deployment sanity check
 *   init_discon    — load libdiscon, open log file; call once before run_discon
 *   run_discon     — packs scalars into avrSWAP, calls DISCON, unpacks outputs
 *   shutdown_discon— unload libdiscon, close log file; call when done
 *
 * Build (MinGW 32-bit, MSVCRT variant):
 *   gcc -m32 -c -o pharlap_compat.o pharlap_compat.c
 *   gcc -m32 -O2 -shared -static -static-libgcc \
 *       -o discon_wrapper.dll discon_wrapper.c pharlap_compat.o \
 *       -Wl,--kill-at
 *
 * Target: Phar Lap ETS 13.1 on NI PXIe-8133, LabVIEW 2019 (32-bit).
 * Only KERNEL32.dll and msvcrt.dll APIs are used (Phar Lap safe).
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#ifdef _WIN32
  #include <windows.h>
  #define EXPORT __declspec(dllexport)
  #define OPEN_LIB(name)   LoadLibraryA(name)
  #define CLOSE_LIB(h)     FreeLibrary(h)
  #define GET_SYM(h, sym)  GetProcAddress(h, sym)
  typedef HMODULE lib_handle_t;
#else
  #include <dlfcn.h>
  #define EXPORT
  #define OPEN_LIB(name)   dlopen(name, RTLD_NOW)
  #define CLOSE_LIB(h)     dlclose(h)
  #define GET_SYM(h, sym)  dlsym(h, sym)
  typedef void* lib_handle_t;
#endif

/* ------------------------------------------------------------------ */
/*  Configuration — adjust these for your deployment                   */
/* ------------------------------------------------------------------ */
#define DISCON_IN  "USFLOWT_10_DISCON.IN"   /* Controller config file name  */
#define SIM_NAME   "rosco_test"             /* Output/log name prefix       */
#define AVR_SIZE   2048                     /* Bladed swap array length     */
#define MSG_SIZE   1000                     /* DISCON message buffer size   */
#define NUM_BL     3                        /* Number of blades             */
#define LOG_NAME   "discon_log.txt"         /* Log file (written to CWD)    */
#define LIB_NAME   "libdiscon.dll"          /* ROSCO DLL (same dir or PATH) */

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */
typedef void (*DISCON_fn)(float*, int*, const char*, const char*, char*);

static lib_handle_t discon_lib  = NULL;
static DISCON_fn    discon_fn   = NULL;
static FILE*        log_file    = NULL;
static int          initialized = 0;
static int          call_count  = 0;

/* ------------------------------------------------------------------ */
/*  Logging helper                                                     */
/* ------------------------------------------------------------------ */
static void log_msg(const char* fmt, ...) {
    if (!log_file) return;
    fprintf(log_file, "[%06d] ", call_count);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(log_file, fmt, ap);
    va_end(ap);
    fprintf(log_file, "\n");
    fflush(log_file);
}

/* ------------------------------------------------------------------ */
/*  hello_ping — deployment sanity check                               */
/*  Returns 42.  Use to verify DLL loads before involving ROSCO.       */
/* ------------------------------------------------------------------ */
EXPORT int hello_ping(void) {
    return 42;
}

/* ------------------------------------------------------------------ */
/*  load_discon — load libdiscon DLL and resolve DISCON symbol         */
/*  Returns 0 on success, -1 on failure (writes error to avcMSG).     */
/* ------------------------------------------------------------------ */
static int load_discon(char* avcMSG) {
    if (discon_fn) return 0;  /* already loaded */

    discon_lib = OPEN_LIB(LIB_NAME);
    if (!discon_lib) {
#ifdef _WIN32
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: error %lu",
                 LIB_NAME, GetLastError());
#else
        snprintf(avcMSG, MSG_SIZE, "Failed to load %s: %s",
                 LIB_NAME, dlerror());
#endif
        log_msg("load_discon: %s", avcMSG);
        return -1;
    }

    discon_fn = (DISCON_fn)GET_SYM(discon_lib, "DISCON");
    if (!discon_fn) {
        snprintf(avcMSG, MSG_SIZE, "DISCON symbol not found in %s", LIB_NAME);
        log_msg("load_discon: %s", avcMSG);
        CLOSE_LIB(discon_lib);
        discon_lib = NULL;
        return -1;
    }

    log_msg("load_discon: loaded %s OK", LIB_NAME);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  init_discon — call once from LabVIEW before the control loop       */
/*  Opens the log file, loads libdiscon, and marks the wrapper ready.  */
/* ------------------------------------------------------------------ */
EXPORT int init_discon(void) {
    if (initialized) return 0;
    log_file = fopen(LOG_NAME, "a");
    initialized = 1;
    call_count  = 0;
    log_msg("init_discon: wrapper ready (AVR_SIZE=%d)", AVR_SIZE);

    char err[MSG_SIZE] = {0};
    if (load_discon(err) != 0) {
        log_msg("init_discon: FAILED — %s", err);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/*  shutdown_discon — call once from LabVIEW after the control loop    */
/*  Unloads libdiscon, closes the log file, and resets state.          */
/* ------------------------------------------------------------------ */
EXPORT void shutdown_discon(void) {
    log_msg("shutdown_discon: closing");
    if (discon_lib) {
        CLOSE_LIB(discon_lib);
        discon_lib = NULL;
        discon_fn  = NULL;
    }
    if (log_file) {
        fclose(log_file);
        log_file = NULL;
    }
    initialized = 0;
    call_count  = 0;
}

/* ------------------------------------------------------------------ */
/*  run_discon — main controller interface for LabVIEW                 */
/*                                                                     */
/*  Packs scalar inputs into the Bladed avrSWAP array, calls DISCON,   */
/*  then unpacks the relevant outputs back to scalar pointers.         */
/*                                                                     */
/*  Parameters (LabVIEW CLFN types in parentheses):                    */
/*    iStatus    — simulation status: 0=first call, 1=normal, -1=last  */
/*    time       — current simulation time [s]                         */
/*    dt         — timestep [s]                                        */
/*    bld_pitch  — measured blade pitch angle [rad]                    */
/*    gen_speed  — generator speed [rad/s]                             */
/*    rot_speed  — rotor speed [rad/s]                                 */
/*    wind_speed — hub-height wind speed [m/s]                         */
/*    gen_torque — (in/out) generator torque demand [Nm]               */
/*    pitch1..3  — (out) individual blade pitch commands [rad]         */
/*    yaw_rate   — (out) nacelle yaw rate command [rad/s]              */
/*    aviFAIL    — (out) error flag from DISCON                        */
/*    avcMSG     — (out) message buffer, pre-allocate >=1000 chars     */
/* ------------------------------------------------------------------ */
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

    if (!initialized) {
        init_discon();
    }
    if (!discon_fn) {
        if (load_discon(avcMSG) != 0) {
            *aviFAIL = -1;
            return;
        }
    }
    call_count++;

    log_msg("run_discon: iStatus=%d t=%.3f dt=%.4f ws=%.1f rpm=%.2f pitch=%.4f torque=%.1f",
            iStatus, time, dt, wind_speed, rot_speed, bld_pitch,
            gen_torque ? *gen_torque : 0.0f);

    /* Build avrSWAP from scalar inputs */
    float avrSWAP[AVR_SIZE] = {0};

    avrSWAP[0]  = (float)iStatus;       /* iStatus                  */
    avrSWAP[1]  = time;                 /* current time [s]         */
    avrSWAP[2]  = dt;                   /* timestep [s]             */
    avrSWAP[3]  = bld_pitch;            /* measured pitch blade 1   */
    avrSWAP[19] = gen_speed;            /* generator speed [rad/s]  */
    avrSWAP[20] = rot_speed;            /* rotor speed [rad/s]      */
    avrSWAP[26] = wind_speed;           /* hub-height wind [m/s]    */
    avrSWAP[60] = (float)NUM_BL;        /* number of blades         */

    avrSWAP[32] = bld_pitch;            /* blade 2 pitch            */
    avrSWAP[33] = bld_pitch;            /* blade 3 pitch            */
    avrSWAP[22] = gen_torque ? *gen_torque : 0.0f;  /* initial torque guess */

    /* String buffer sizes (Bladed convention) */
    avrSWAP[48] = (float)MSG_SIZE;
    avrSWAP[49] = (float)strlen(DISCON_IN);
    avrSWAP[50] = (float)strlen(SIM_NAME);

    /* Call ROSCO — dynamically loaded */
    char msg_buf[MSG_SIZE] = {0};
    discon_fn(avrSWAP, aviFAIL, DISCON_IN, SIM_NAME, msg_buf);

    /* Copy message to caller's buffer */
    strncpy(avcMSG, msg_buf, MSG_SIZE - 1);
    avcMSG[MSG_SIZE - 1] = '\0';

    if (*aviFAIL != 0) {
        log_msg("run_discon: aviFAIL=%d msg=%s", *aviFAIL, msg_buf);
    }

    /* Extract outputs from avrSWAP */
    *gen_torque = avrSWAP[46];           /* demanded generator torque */
    *pitch1     = avrSWAP[41];           /* blade 1 pitch command     */
    *pitch2     = avrSWAP[42];           /* blade 2 pitch command     */
    *pitch3     = avrSWAP[43];           /* blade 3 pitch command     */
    *yaw_rate   = avrSWAP[47];           /* yaw rate command          */
}
