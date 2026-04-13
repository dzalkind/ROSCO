#include "../include/vit_types.h"
#include "../include/vit_translated.h"

void RefSpeedExclusion(localvariables_t* LocalVar, const ControlParameters& CntrPar, objectinstances_t* objInst, debugvariables_t* DebugVar) {

    // Get LSS Ref speed
    double VS_RefSpeed_LSS = LocalVar->VS_RefSpd / CntrPar.WE_GearboxRatio;

    if ((VS_RefSpeed_LSS > CntrPar.TRA_ExclSpeed - CntrPar.TRA_ExclBand / 2) &&
        (VS_RefSpeed_LSS < CntrPar.TRA_ExclSpeed + CntrPar.TRA_ExclBand / 2)) {
        LocalVar->FA_Hist = 1;
    } else {
        LocalVar->FA_Hist = 0;
    }

    // Initialize last reference speed state
    if (LocalVar->restart != 0) {
        if (LocalVar->FA_Hist > 0) {
            if (VS_RefSpeed_LSS > CntrPar.TRA_ExclSpeed) {
                LocalVar->TRA_LastRefSpd = CntrPar.TRA_ExclSpeed + CntrPar.TRA_ExclBand / 2;
            } else {
                LocalVar->TRA_LastRefSpd = CntrPar.TRA_ExclSpeed - CntrPar.TRA_ExclBand / 2;
            }
        } else {
            LocalVar->TRA_LastRefSpd = VS_RefSpeed_LSS;
        }
    }

    if (LocalVar->FA_Hist > 0) {
        LocalVar->VS_RefSpd_TRA = LocalVar->TRA_LastRefSpd;
    } else {
        LocalVar->VS_RefSpd_TRA = VS_RefSpeed_LSS;
    }

    LocalVar->TRA_LastRefSpd = LocalVar->VS_RefSpd_TRA;

    // Rate limit reference speed
    LocalVar->VS_RefSpd_RL = ratelimit(LocalVar->VS_RefSpd_TRA, -CntrPar.TRA_RateLimit, CntrPar.TRA_RateLimit,
                                          LocalVar->DT, (LocalVar->restart != 0) ? 1 : 0,
                                          &LocalVar->rlP, &objInst->instRL, 0, 0.0);
    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd_RL * CntrPar.WE_GearboxRatio;
}
