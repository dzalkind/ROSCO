/*
 * discon_wrapper.c — LabVIEW-friendly C ABI shim for ROSCO's DISCON controller.
 *
 * This wrapper converts individual scalar inputs/outputs into the Bladed-style
 * avrSWAP float array that DISCON expects.  ROSCO (libdiscon) is statically
 * linked at build time — no runtime DLL loading required.
 *
 * Exported functions (called from LabVIEW CLFN):
 *   hello_ping     — returns 42; deployment sanity check
 *   run_discon     — packs scalars into avrSWAP, calls DISCON, unpacks outputs
 *
 * Build (MinGW 32-bit, MSVCRT variant, static linkage):
 *   gcc -m32 -O2 -shared -static -static-libgcc \
 *       -o discon_wrapper.dll discon_wrapper.c \
 *       -L<path-to-libdiscon> -ldiscon \
 *       -Wl,--kill-at
 *
 * Target: Phar Lap ETS 13.1 on NI PXIe-8133, LabVIEW 2019 (32-bit).
 * The resulting DLL must depend only on KERNEL32.dll and msvcrt.dll.
 */

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
  #define EXPORT __declspec(dllexport)
#else
  #define EXPORT
#endif

/* ------------------------------------------------------------------ */
/*  Configuration — adjust these for your deployment                   */
/* ------------------------------------------------------------------ */
#define DISCON_IN  "USFLOWT_10_DISCON.IN"   /* Controller config file name  */
#define SIM_NAME   "rosco_test"             /* Output/log name prefix       */
#define AVR_SIZE   500                      /* Bladed swap array length     */
#define MSG_SIZE   1000                     /* DISCON message buffer size   */
#define NUM_BL     3                        /* Number of blades             */

/* ------------------------------------------------------------------ */
/*  Forward declaration of ROSCO's DISCON entry point                  */
/*                                                                     */
/*  Linked statically from libdiscon.a at build time.                  */
/*  Original Fortran: BIND(C, NAME='DISCON'); now C++ with C linkage.  */
/* ------------------------------------------------------------------ */
extern void DISCON(float* avrSWAP, int* aviFAIL,
                   const char* accINFILE, const char* avcOUTNAME,
                   char* avcMSG);

/* ------------------------------------------------------------------ */
/*  hello_ping — deployment sanity check                               */
/*  Returns 42.  Use to verify DLL loads before involving ROSCO.       */
/* ------------------------------------------------------------------ */
EXPORT int hello_ping(void) {
    return 42;
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

    /* Call ROSCO — statically linked, no runtime loading */
    char msg_buf[MSG_SIZE] = {0};
    DISCON(avrSWAP, aviFAIL, DISCON_IN, SIM_NAME, msg_buf);

    /* Copy message to caller's buffer */
    strncpy(avcMSG, msg_buf, MSG_SIZE - 1);
    avcMSG[MSG_SIZE - 1] = '\0';

    /* Extract outputs from avrSWAP */
    *gen_torque = avrSWAP[46];           /* demanded generator torque */
    *pitch1     = avrSWAP[41];           /* blade 1 pitch command     */
    *pitch2     = avrSWAP[42];           /* blade 2 pitch command     */
    *pitch3     = avrSWAP[43];           /* blade 3 pitch command     */
    *yaw_rate   = avrSWAP[47];           /* yaw rate command          */
}
