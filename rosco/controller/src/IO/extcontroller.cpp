#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include "../include/rosco_objects.hpp"
#include <dlfcn.h>
#include <cstdio>
#include <cstring>

// Bladed DLL legacy interface — function pointer typedef
typedef void (*bladed_dll_proc_t)(float*, int*, char*, char*, char*);

void ExtController(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar, ExtControlType& ExtDLL, errorvariables_t* ErrVar) {
    static void* dll_handle = nullptr;
    static bladed_dll_proc_t dll_proc = nullptr;

    const int max_avr_entries = 2000;
    const char* ExtRootName = "external_control";

    // Initialize strings for external controller
    int aviFAIL = 0;

    // Build null-terminated C strings from Fortran CHARACTER fields
    // DLL_InFile
    char accINFILE[1024];
    int infile_len = 0;
    while (infile_len < 1023 && CntrPar.DLL_InFile[infile_len] != ' ' && CntrPar.DLL_InFile[infile_len] != '\0') {
        accINFILE[infile_len] = CntrPar.DLL_InFile[infile_len];
        infile_len++;
    }
    accINFILE[infile_len] = '\0';

    // OUTNAME
    char avcOUTNAME[128];
    int outname_len = (int)strlen(ExtRootName);
    memcpy(avcOUTNAME, ExtRootName, outname_len);
    avcOUTNAME[outname_len] = '\0';

    // MSG buffer
    char avcMSG[1025];
    memset(avcMSG, 0, sizeof(avcMSG));

    // First call: load dynamic library
    if (LocalVar.iStatus == 0) {
        // Build null-terminated filename
        char dll_filename[1024];
        int fn_len = 0;
        while (fn_len < 1023 && CntrPar.DLL_FileName[fn_len] != ' ' && CntrPar.DLL_FileName[fn_len] != '\0') {
            dll_filename[fn_len] = CntrPar.DLL_FileName[fn_len];
            fn_len++;
        }
        dll_filename[fn_len] = '\0';

        // Build null-terminated proc name
        char dll_procname[1024];
        int pn_len = 0;
        while (pn_len < 1023 && CntrPar.DLL_ProcName[pn_len] != ' ' && CntrPar.DLL_ProcName[pn_len] != '\0') {
            dll_procname[pn_len] = CntrPar.DLL_ProcName[pn_len];
            pn_len++;
        }
        dll_procname[pn_len] = '\0';

        printf("ROSCO is calling an external dynamic library for control input:\n");
        printf("DLL_FileName: %s\n", dll_filename);
        printf("DLL_InFile: %s\n", accINFILE);
        printf("DLL_ProcName: %s\n", dll_procname);

        // dlopen
        dll_handle = dlopen(dll_filename, RTLD_LAZY);
        if (!dll_handle) {
            ErrVar->ErrStat = -1;
            snprintf(ErrVar->ErrMsg, 1024, "ExtController:The dynamic library %s could not be loaded.", dll_filename);
            int len = (int)strlen(ErrVar->ErrMsg);
            for (int k = len; k < 1024; k++) ErrVar->ErrMsg[k] = ' ';
            return;
        }

        // dlsym
        dll_proc = (bladed_dll_proc_t)dlsym(dll_handle, dll_procname);
        if (!dll_proc) {
            ErrVar->ErrStat = -1;
            snprintf(ErrVar->ErrMsg, 1024, "ExtController:The procedure %s could not be loaded.", dll_procname);
            int len = (int)strlen(ErrVar->ErrMsg);
            for (int k = len; k < 1024; k++) ErrVar->ErrMsg[k] = ' ';
            return;
        }

        printf("Library loaded successfully\n");
    }

    // Copy avrSWAP to ExtDLL's swap array
    for (int i = 0; i < max_avr_entries; i++) {
        ExtDLL.avrSWAP[i] = avrSWAP[i];
    }

    // Set length parameters (0-based: records 49,50,51 → indices 48,49,50)
    ExtDLL.avrSWAP[48] = (float)(sizeof(avcMSG));           // Record 49: max MSG length
    ExtDLL.avrSWAP[49] = (float)(infile_len + 1);           // Record 50: INFILE length
    ExtDLL.avrSWAP[50] = (float)(outname_len + 1);          // Record 51: OUTNAME length

    // Call the external DLL
    dll_proc(ExtDLL.avrSWAP.data(), &aviFAIL, accINFILE, avcOUTNAME, avcMSG);

    // Check for errors from the DLL
    if (aviFAIL < 0) {
        ErrVar->aviFAIL = aviFAIL;
        char tmp[1024];
        snprintf(tmp, sizeof(tmp), "ExtController:%s", avcMSG);
        int len = (int)strlen(tmp);
        if (len > 1024) len = 1024;
        memcpy(ErrVar->ErrMsg, tmp, len);
        for (int k = len; k < 1024; k++) ErrVar->ErrMsg[k] = ' ';
        printf("%s\n", tmp);
    }
}
