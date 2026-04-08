#include <stdio.h>

#ifdef _WIN32
  #include <windows.h>
  #define LIB_NAME "discon_wrapper.dll"
  #define OPEN_LIB(name)   LoadLibraryA(name)
  #define GET_SYM(h, sym)  GetProcAddress(h, sym)
  #define CLOSE_LIB(h)     FreeLibrary(h)
  typedef HMODULE lib_handle_t;
#else
  #include <dlfcn.h>
  #define LIB_NAME "./discon_wrapper.so"
  #define OPEN_LIB(name)   dlopen(name, RTLD_NOW)
  #define GET_SYM(h, sym)  dlsym(h, sym)
  #define CLOSE_LIB(h)     dlclose(h)
  typedef void* lib_handle_t;
#endif

typedef void (*run_discon_fn)(
    int iStatus, float time, float dt,
    float bld_pitch, float gen_speed, float rot_speed, float wind_speed,
    float* gen_torque, float* pitch1, float* pitch2, float* pitch3, float* yaw_rate,
    int* aviFAIL, char* avcMSG
);

int main(void) {
    lib_handle_t lib = OPEN_LIB(LIB_NAME);
    if (!lib) {
#ifdef _WIN32
        fprintf(stderr, "Failed to load %s: error %lu\n", LIB_NAME, GetLastError());
#else
        fprintf(stderr, "Failed to load %s: %s\n", LIB_NAME, dlerror());
#endif
        return 1;
    }

    run_discon_fn run_discon = (run_discon_fn) GET_SYM(lib, "run_discon");
    if (!run_discon) {
        fprintf(stderr, "Failed to find run_discon symbol\n");
        CLOSE_LIB(lib);
        return 1;
    }

    float gen_torque, pitch1, pitch2, pitch3, yaw_rate;
    int   aviFAIL;
    char  avcMSG[1000] = {0};

    /* Turbine state: below-rated, 10 m/s wind */
    float gen_speed  = 100.0f;  /* rad/s */
    float rot_speed  =   1.0f;  /* rad/s */
    float wind_speed =  10.0f;  /* m/s   */
    float bld_pitch  =   0.0f;  /* rad   */
    float dt         =   0.025f;

    /* Initialization call */
    run_discon(0, 0.0f, dt, bld_pitch, gen_speed, rot_speed, wind_speed,
               &gen_torque, &pitch1, &pitch2, &pitch3, &yaw_rate,
               &aviFAIL, avcMSG);
    printf("Init:  aviFAIL=%d  msg=%s\n", aviFAIL, avcMSG);

    /* First timestep */
    run_discon(1, dt, dt, bld_pitch, gen_speed, rot_speed, wind_speed,
               &gen_torque, &pitch1, &pitch2, &pitch3, &yaw_rate,
               &aviFAIL, avcMSG);
    printf("Run:   aviFAIL=%d  msg=%s\n", aviFAIL, avcMSG);
    printf("  gen_torque = %.1f N·m\n", gen_torque);
    printf("  pitch1     = %.4f rad\n", pitch1);
    printf("  pitch2     = %.4f rad\n", pitch2);
    printf("  pitch3     = %.4f rad\n", pitch3);
    printf("  yaw_rate   = %.4f rad/s\n", yaw_rate);

    CLOSE_LIB(lib);
    return 0;
}
