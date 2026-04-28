#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include "../ControlElements/ratelimiter.hpp"

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
    static RateLimiter refSpdRL;
    if (LocalVar->iStatus == 0 || LocalVar->restart) refSpdRL.init(LocalVar->VS_RefSpd_TRA);
    LocalVar->VS_RefSpd_RL = refSpdRL.step(LocalVar->VS_RefSpd_TRA, -CntrPar.TRA_RateLimit, CntrPar.TRA_RateLimit, LocalVar->DT);
    LocalVar->VS_RefSpd = LocalVar->VS_RefSpd_RL * CntrPar.WE_GearboxRatio;
}
