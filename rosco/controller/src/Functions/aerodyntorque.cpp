#include "../include/rosco_objects.hpp"
#include "../include/vit_translated.h"
#include <cmath>
#include <limits>

#include "../include/rosco_constants.h"

double AeroDynTorque(double RotSpeed, double BldPitch, double WE_Vw,
                     double WE_BladeRadius, double WE_RhoAir,
                     const PerformanceData& PerfData) {

    // Find Torque
    double RotorArea = PI * (WE_BladeRadius * WE_BladeRadius);
    double WindSpeed = WE_Vw > std::numeric_limits<double>::epsilon()
                     ? WE_Vw : std::numeric_limits<double>::epsilon();
    double Lambda = RotSpeed * WE_BladeRadius / WindSpeed;

    // Compute Cp via 2D interpolation on performance surface
    double Cp = interp2d(PerfData.Beta_vec.data(), (int)PerfData.Beta_vec.size(),
                         PerfData.TSR_vec.data(),  (int)PerfData.TSR_vec.size(),
                         PerfData.Cp_mat.data(),   (int)PerfData.TSR_vec.size(), (int)PerfData.Beta_vec.size(),
                         BldPitch * R2D, Lambda);

    double result = 0.5 * (WE_RhoAir * RotorArea) * (WE_Vw * WE_Vw * WE_Vw / RotSpeed) * Cp;
    result = result > 0.0 ? result : 0.0;

    return result;
}
