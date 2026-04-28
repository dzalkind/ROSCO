#include "../include/vit_types.h"
#include "../include/vit_translated.h"
#include <algorithm>
#include "../ControlElements/picontroller.hpp"

void ForeAftDamping(const ControlParameters& CntrPar, localvariables_t* LocalVar, objectinstances_t* objInst) {
    // Fore-aft damping controller: reduces tower vibrations using pitch

    // PI controller on fore-aft acceleration (high-pass filtered)
    static PIController faAccPI;
    if (LocalVar->iStatus == 0 || LocalVar->restart) {
        faAccPI.init(0.0);
        LocalVar->FA_AccHPFI = 0.0;
    } else {
        LocalVar->FA_AccHPFI = faAccPI.step(LocalVar->FA_AccHPF, 0.0, CntrPar.FA_KI,
            -CntrPar.FA_IntSat, CntrPar.FA_IntSat, LocalVar->DT);
    }

    // Store the fore-aft pitch contribution for all blades
    for (int K = 0; K < LocalVar->NumBl; K++) {
        LocalVar->FA_PitCom[K] = LocalVar->FA_AccHPFI;
    }
}
