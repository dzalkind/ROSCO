#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include "../ControlElements/picontroller.hpp"

double FloatingFeedback(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar) {
    // FloatingFeedback: pitch contribution from nacelle velocity feedback
    //   Fl_Mode = 1: proportional feedback of translational nacelle velocity
    //   Fl_Mode = 2: proportional feedback of rotational nacelle velocity

    // Gain scheduling — interpolate Kp_Float from wind speed
    LocalVar->Kp_Float = interp1d(CntrPar.Fl_U,
                                    CntrPar.Fl_Kp,
                                    LocalVar->WE_Vw_F, ErrVar);

    // Integrate fore-aft acceleration to get velocity (KP=0, KI=1 → pure integrator)
    static PIController faVelPI;
    double FA_vel;
    if (LocalVar->iStatus == 0 || LocalVar->restart) {
        faVelPI.init(0.0);
        FA_vel = 0.0;
    } else {
        FA_vel = faVelPI.step(LocalVar->FA_AccF, 0.0, 1.0, -100.0, 100.0, LocalVar->DT);
    }

    static PIController nacImuFaVelPI;
    double NacIMU_FA_vel;
    if (LocalVar->iStatus == 0 || LocalVar->restart) {
        nacImuFaVelPI.init(0.0);
        NacIMU_FA_vel = 0.0;
    } else {
        NacIMU_FA_vel = nacImuFaVelPI.step(LocalVar->NACIMU_FA_AccF, 0.0, 1.0, -100.0, 100.0, LocalVar->DT);
    }

    // Select velocity signal based on mode and apply gain
    double result = 0.0;
    if (CntrPar.Fl_Mode == 1) {
        result = (0.0 - FA_vel) * LocalVar->Kp_Float;
    } else if (CntrPar.Fl_Mode == 2) {
        result = (0.0 - NacIMU_FA_vel) * LocalVar->Kp_Float;
    }

    return result;
}
