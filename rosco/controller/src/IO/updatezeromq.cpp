#include "../include/vit_types.h"
#include "../include/rosco_types.hpp"
#include <cstring>

#ifdef ZMQ_CLIENT
extern "C" {
    void zmq_client(char* zmq_address, double* measurements, double* setpoints);
}
#endif

void UpdateZeroMQ(localvariables_t* LocalVar, const ControlParameters& CntrPar, errorvariables_t* ErrVar) {
    // Only communicate at ZMQ update interval or on final timestep
    if (LocalVar->n_DT % CntrPar.n_DT_ZMQ == 0 || LocalVar->iStatus == -1) {

        // Pack 17 turbine measurements
        double turbine_measurements[17];
        turbine_measurements[0]  = LocalVar->ZMQ_ID;
        turbine_measurements[1]  = LocalVar->iStatus;
        turbine_measurements[2]  = LocalVar->Time;
        turbine_measurements[3]  = LocalVar->VS_MechGenPwr;
        turbine_measurements[4]  = LocalVar->VS_GenPwr;
        turbine_measurements[5]  = LocalVar->GenSpeed;
        turbine_measurements[6]  = LocalVar->RotSpeed;
        turbine_measurements[7]  = LocalVar->GenTqMeas;
        turbine_measurements[8]  = LocalVar->NacHeading;
        turbine_measurements[9]  = LocalVar->NacVane;
        turbine_measurements[10] = LocalVar->HorWindV;
        turbine_measurements[11] = LocalVar->rootMOOP[0];
        turbine_measurements[12] = LocalVar->rootMOOP[1];
        turbine_measurements[13] = LocalVar->rootMOOP[2];
        turbine_measurements[14] = LocalVar->FA_Acc_TT;
        turbine_measurements[15] = LocalVar->NacIMU_FA_RAcc;
        turbine_measurements[16] = LocalVar->Azimuth;

        // Format ZMQ address with null terminator
        char zmq_address[256];
        int len = 0;
        while (len < 255 && CntrPar.ZMQ_CommAddress[len] != ' ' && CntrPar.ZMQ_CommAddress[len] != '\0') {
            zmq_address[len] = CntrPar.ZMQ_CommAddress[len];
            len++;
        }
        zmq_address[len] = '\0';

        double setpoints[8] = {0};

#ifdef ZMQ_CLIENT
        zmq_client(zmq_address, turbine_measurements, setpoints);
#else
        // ZMQ client not compiled — set error if ZMQ_Mode > 0
        ErrVar->aviFAIL = -1;
        if (CntrPar.ZMQ_Mode > 0) {
            const char* msg = "UpdateZeroMQ: >> The ZeroMQ client has not been properly installed, "
                              "please install it to use ZMQ_Mode > 0.";
            int mlen = (int)strlen(msg);
            if (mlen > 1024) mlen = 1024;
            memcpy(ErrVar->ErrMsg, msg, mlen);
            for (int k = mlen; k < 1024; k++) ErrVar->ErrMsg[k] = ' ';
        }
#endif

        // Unpack 8 setpoints
        LocalVar->ZMQ_TorqueOffset = setpoints[0];
        LocalVar->ZMQ_YawOffset    = setpoints[1];
        LocalVar->ZMQ_PitOffset[0] = setpoints[2];
        LocalVar->ZMQ_PitOffset[1] = setpoints[3];
        LocalVar->ZMQ_PitOffset[2] = setpoints[4];
        LocalVar->ZMQ_R_Speed      = setpoints[5];
        LocalVar->ZMQ_R_Torque     = setpoints[6];
        LocalVar->ZMQ_R_Pitch      = setpoints[7];
    }
}
