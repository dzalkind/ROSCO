// ControllerObjects — every filter, PI/PID controller and rate limiter whose
// state has to survive from one timestep to the next.
//
// These used to be function-local `static` objects scattered across the
// controller modules. That works for a normal run, but a warm restart
// (iStatus == -9) only restores LocalVar from the checkpoint file, so the
// statics kept zeroed coefficients and the first filtered signal came out NaN.
// Collecting them here gives WriteRestartFile/ReadRestartFile a single POD blob
// to serialise, so a restarted run resumes on exactly the state it left off on.
//
// Members are grouped by the module that owns them, which keeps names that
// repeat across modules (genSpeedFilter, rotSpeedFilter, nacVaneCosFilter)
// distinct and makes the owning call site obvious.
//
// Adding new persistent state: add the member here, use it as
// ObjState.<module>.<name>, and do NOT declare a new function-local static
// filter, controller or rate limiter. A stray static works for a normal run and
// silently corrupts a warm restart — no compiler error, no test failure unless
// restart is exercised on that path. .github/prompts/plan-checkpointState.prompt.md
// records why it is a hand-maintained list and what the alternatives cost.

#pragma once

#include "../ControlElements/picontroller.hpp"
#include "../ControlElements/pidcontroller.hpp"
#include "../ControlElements/piicontroller.hpp"
#include "../ControlElements/ratelimiter.hpp"
#include "../ControlElements/rescontroller.hpp"
#include "../Filters/hpfilter.hpp"
#include "../Filters/lpfilter.hpp"
#include "../Filters/notchfilter.hpp"
#include "../Filters/notchfilterslopes.hpp"
#include "../Filters/seclpfilter.hpp"
#include "../Filters/seclpfilter_vel.hpp"

#include <type_traits>

struct ControllerObjects {

    // PreFilterMeasuredSignals
    struct {
        LPFilter          genSpeedFilter;
        LPFilter          rotSpeedFilter;
        SecLPFilter       genSpeedFilter2;
        SecLPFilter       rotSpeedFilter2;
        NotchFilter       genSpdNotch[10];
        SecLPFilter       nacImuFaAccLPF;
        SecLPFilter       faAccLPF;
        HPFilter          nacImuFaAccHPF;
        HPFilter          faAccHPF;
        NotchFilter       twrTopNotchNacImu[10];
        NotchFilter       twrTopNotchFaAcc[10];
        HPFilter          faAccDampHPF;
        LPFilter          weVwFilter;
        NotchFilterSlopes rootMOOPNotchSlopes[3];
        SecLPFilter       rootMOOPSecLPF[3];
        HPFilter          rootMOOPHPF[3];
        NotchFilter       rootMOOPNotch[3 * 10];
        SecLPFilter       lastGenTrqFilter;
        SecLPFilter       blPitchCMeasFilter;
        LPFilter          nacVaneCosFilter;
        LPFilter          nacVaneSinFilter;
    } prefilter;

    // Shutdown
    struct {
        LPFilter pitchFilter;
        LPFilter genSpeedFilter;
        LPFilter nacVaneCosFilter;
        LPFilter nacVaneSinFilter;
    } shutdown;

    // Startup
    struct {
        LPFilter rotSpeedFilter;
    } startup;

    // SetpointSmoother
    struct {
        LPFilter ssFilter;
    } setpoint_smoother;

    // ComputeVariablesSetpoints
    struct {
        LPFilter prcWindFilter;
        LPFilter refSpdFilter;
    } speed_setpoints;

    // RefSpeedExclusion
    struct {
        RateLimiter refSpdRL;
    } ref_speed_exclusion;

    // IPC
    struct {
        LPFilter     yawErrFilter;
        PIController yawIpcPI;
        PIController ipcTilt1pPI;
        PIController ipcYaw1pPI;
        PIController ipcTilt2pPI;
        PIController ipcYaw2pPI;
        LPFilter     ipcActFilter[3];
    } ipc;

    // FloatingFeedback
    struct {
        PIController faVelPI;
        PIController nacImuFaVelPI;
    } floating;

    // ActiveWakeControl
    struct {
        ResController awcResCtrl[2];
        PIController  awcPI[2];
        PIController  awcStrPI;
    } awc;

    // ForeAftDamping
    struct {
        PIController faAccPI;
    } fore_aft;

    // SetParameters
    struct {
        LPFilter olIndexFilter;
    } setparameters;

    // WindSpeedEstimator
    struct {
        LPFilter horWindFilter;
    } wse;

    // FlapControl
    struct {
        PIIController flpPII[3];
        PIController  flpTiltPI;
        PIController  flpYawPI;
    } flap;

    // PitchControl
    struct {
        PIController pcPitComTPI;
        RateLimiter  pitComTRL;
        RateLimiter  pitComRL[3];
        LPFilter     pitchActFilter[3];
        SecLPFilter  pitchActFilter2[3];
        RateLimiter  pitComActRL[3];
    } pitch;

    // VariableSpeedControl
    struct {
        PIController  genTqPI;
        PIController  genArTqPI;
        PIController  genBrTqPI;
        RateLimiter   genTqRL;
        PIDController genTqAzPID;
    } torque;

    // YawRateControl
    struct {
        int      Tidx = 0;
        LPFilter windDirCosFilter;
        LPFilter windDirSinFilter;
    } yaw;

    // CableControl
    struct {
        SecLPFilterVel ccActFilter[10];
        PIController   ccActPI[10];
    } cable;
};

// The checkpoint writes ControllerObjects as one raw block, so it must stay a
// flat bag of arithmetic members. Adding a pointer, std::vector or std::string
// to any of the classes above breaks that and trips this assert.
static_assert(std::is_trivially_copyable<ControllerObjects>::value,
              "ControllerObjects must be trivially copyable — it is checkpointed as raw bytes");

// Single instance, defined in controller_objects.cpp. Lives for the lifetime of
// the loaded library, exactly as the function-local statics it replaced did.
extern ControllerObjects ObjState;
