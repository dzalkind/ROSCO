#include <vector>
#include "../include/vit_translated.h"

void StructuralControl(float* avrSWAP, const ControlParameters& CntrPar, LocalVariables& LocalVar) {
    // StructuralControl: structural control input assignment
    //   StC_Mode = 1: user-defined step inputs
    //   StC_Mode = 2: open-loop from lookup table

    if (CntrPar.StC_Mode == 1) {
        // User-defined control — step change at t > 500
        if (LocalVar.Time > 500) {
            LocalVar.StC_Input[0] = -1.234e+06;
            LocalVar.StC_Input[1] = 2.053e+06;
            LocalVar.StC_Input[2] = -7.795e+05;
        }

    } else if (CntrPar.StC_Mode == 2) {
        // Open loop control — interp1d on OL_StructControl(I_GROUP,:)
        for (int I_GROUP = 0; I_GROUP < CntrPar.StC_Group_N; I_GROUP++) {
            if (CntrPar.Ind_StructControl[I_GROUP] > 0) {
                // Extract row from column-major 2D array:
                // Fortran OL_StructControl(I_GROUP,:) = all columns for this row
                // Column-major: element (row, col) = flat[col * n_rows + row]
                int n_rows = CntrPar.OL_StructControl_rows;
                int n_cols = CntrPar.OL_StructControl_cols;
                std::vector<double> row_slice(n_cols);
                for (int col = 0; col < n_cols; col++) {
                    row_slice[col] = CntrPar.OL_StructControl[col * n_rows + I_GROUP];
                }
                LocalVar.StC_Input[I_GROUP] = interp1d(CntrPar.OL_Breakpoints,
                                                        {row_slice.data(), n_cols},
                                                        LocalVar.Time);
            }
        }
    }

    // Assign to avrSWAP
    for (int I_GROUP = 0; I_GROUP < CntrPar.StC_Group_N; I_GROUP++) {
        // Fortran: avrSWAP(StC_GroupIndex(I_GROUP)) — 1-indexed
        // C++: avrSWAP[StC_GroupIndex[I_GROUP] - 1] — 0-indexed
        avrSWAP[CntrPar.StC_GroupIndex[I_GROUP] - 1] = LocalVar.StC_Input[I_GROUP];
    }
}
