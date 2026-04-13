#include "../include/vit_types.h"
#include "../include/vit_translated.h"

double FloatingFeedback(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, errorvariables_t* ErrVar) {
    // FloatingFeedback: pitch contribution from nacelle velocity feedback
    //   Fl_Mode = 1: proportional feedback of translational nacelle velocity
    //   Fl_Mode = 2: proportional feedback of rotational nacelle velocity

    // Gain scheduling — interpolate Kp_Float from wind speed
    LocalVar->Kp_Float = interp1d(CntrPar.Fl_U,
                                    CntrPar.Fl_Kp,
                                    LocalVar->WE_Vw_F, ErrVar);

    // Integrate fore-aft acceleration to get velocity (KP=0, KI=1 → pure integrator)
    double FA_vel = PIController(
        LocalVar->FA_AccF,          // error: fore-aft acceleration
        0.0,                         // kp
        1.0,                         // ki
        -100.0,                      // minValue
        100.0,                       // maxValue
        LocalVar->DT,                // time step
        0.0,                         // I0
        &LocalVar->piP,              // PI parameters struct
        (LocalVar->restart != 0),    // reset flag
        &objInst->instPI             // instance counter
    );

    double NacIMU_FA_vel = PIController(
        LocalVar->NACIMU_FA_AccF,   // error: nacelle IMU fore-aft acceleration
        0.0,                         // kp
        1.0,                         // ki
        -100.0,                      // minValue
        100.0,                       // maxValue
        LocalVar->DT,                // time step
        0.0,                         // I0
        &LocalVar->piP,              // PI parameters struct
        (LocalVar->restart != 0),    // reset flag
        &objInst->instPI             // instance counter
    );

    // Select velocity signal based on mode and apply gain
    double result = 0.0;
    if (CntrPar.Fl_Mode == 1) {
        result = (0.0 - FA_vel) * LocalVar->Kp_Float;
    } else if (CntrPar.Fl_Mode == 2) {
        result = (0.0 - NacIMU_FA_vel) * LocalVar->Kp_Float;
    }

    return result;
}
