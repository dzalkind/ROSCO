! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Read and set the parameters used by the controller

! Submodules:
!           Assert: Initial condition and input check
!           ComputeVariablesSetpoints: Compute setpoints used by controllers
!           ReadAvrSWAP: Read AvrSWAP array
!           ReadControlParameterFileSub: Read DISCON.IN input file
!           ReadCPFile: Read text file containing Cp Surface
!           SetParameters: Define initial conditions 

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

USE Constants
USE Functions

    IMPLICIT NONE

CONTAINS
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadControlParameterFileSub(CntrPar, accINFILE, accINFILE_size)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters

        INTEGER(4)                              :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size), INTENT(IN)   :: accINFILE(accINFILE_size)    ! DISCON input filename
        INTEGER(4), PARAMETER                   :: UnControllerParameters = 89  ! Unit number to open file
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar                      ! Control parameter type

        CHARACTER(1024)                         :: OL_String                    ! Open description loop string
        INTEGER(4)                              :: OL_Count                     ! Number of open loop channels
       

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *)

        !----------------------- DEBUG --------------------------
        READ(UnControllerParameters, *) 
        READ(UnControllerParameters, *) CntrPar%LoggingLevel
        READ(UnControllerParameters, *) 

        !----------------- CONTROLLER FLAGS ---------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%F_LPFType
        READ(UnControllerParameters, *) CntrPar%F_NotchType
        READ(UnControllerParameters, *) CntrPar%IPC_ControlMode
        READ(UnControllerParameters, *) CntrPar%VS_ControlMode
        READ(UnControllerParameters, *) CntrPar%PC_ControlMode
        READ(UnControllerParameters, *) CntrPar%Y_ControlMode        
        READ(UnControllerParameters, *) CntrPar%SS_Mode        
        READ(UnControllerParameters, *) CntrPar%WE_Mode        
        READ(UnControllerParameters, *) CntrPar%PS_Mode        
        READ(UnControllerParameters, *) CntrPar%SD_Mode        
        READ(UnControllerParameters, *) CntrPar%FL_Mode        
        READ(UnControllerParameters, *) CntrPar%Flp_Mode   
        READ(UnControllerParameters, *) CntrPar%PwC_Mode       
        READ(UnControllerParameters, *) CntrPar%OL_Mode        
        READ(UnControllerParameters, *)

        !----------------- FILTER CONSTANTS ---------------------
        READ(UnControllerParameters, *)        
        READ(UnControllerParameters, *) CntrPar%F_LPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%F_LPFDamping
        READ(UnControllerParameters, *) CntrPar%F_NotchCornerFreq
        ALLOCATE(CntrPar%F_NotchBetaNumDen(2))
        READ(UnControllerParameters,*) CntrPar%F_NotchBetaNumDen
        READ(UnControllerParameters,*) CntrPar%F_SSCornerFreq
        READ(UnControllerParameters,*) CntrPar%F_FlCornerFreq, CntrPar%F_FlDamping
        READ(UnControllerParameters,*) CntrPar%F_FlpCornerFreq, CntrPar%F_FlpDamping
        READ(UnControllerParameters, *)

        !----------- BLADE PITCH CONTROLLER CONSTANTS -----------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%PC_GS_n
        ALLOCATE(CntrPar%PC_GS_angles(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_angles
        ALLOCATE(CntrPar%PC_GS_KP(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KP
        ALLOCATE(CntrPar%PC_GS_KI(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KI
        ALLOCATE(CntrPar%PC_GS_KD(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_KD
        ALLOCATE(CntrPar%PC_GS_TF(CntrPar%PC_GS_n))
        READ(UnControllerParameters,*) CntrPar%PC_GS_TF
        READ(UnControllerParameters, *) CntrPar%PC_MaxPit
        READ(UnControllerParameters, *) CntrPar%PC_MinPit
        READ(UnControllerParameters, *) CntrPar%PC_MaxRat
        READ(UnControllerParameters, *) CntrPar%PC_MinRat
        READ(UnControllerParameters, *) CntrPar%PC_RefSpd
        READ(UnControllerParameters, *) CntrPar%PC_FinePit
        READ(UnControllerParameters, *) CntrPar%PC_Switch
        READ(UnControllerParameters, *)

        !------------------- IPC CONSTANTS -----------------------
        READ(UnControllerParameters, *) 
        READ(UnControllerParameters, *) CntrPar%IPC_IntSat
        ALLOCATE(CntrPar%IPC_KI(2))
        READ(UnControllerParameters,*) CntrPar%IPC_KI
        ALLOCATE(CntrPar%IPC_aziOffset(2))
        READ(UnControllerParameters,*) CntrPar%IPC_aziOffset
        READ(UnControllerParameters, *) CntrPar%IPC_CornerFreqAct
        READ(UnControllerParameters, *)

        !------------ VS TORQUE CONTROL CONSTANTS ----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%VS_GenEff
        READ(UnControllerParameters, *) CntrPar%VS_ArSatTq
        READ(UnControllerParameters, *) CntrPar%VS_MaxRat
        READ(UnControllerParameters, *) CntrPar%VS_MaxTq
        READ(UnControllerParameters, *) CntrPar%VS_MinTq
        READ(UnControllerParameters, *) CntrPar%VS_MinOMSpd
        READ(UnControllerParameters, *) CntrPar%VS_Rgn2K
        READ(UnControllerParameters, *) CntrPar%VS_RtPwr
        READ(UnControllerParameters, *) CntrPar%VS_RtTq
        READ(UnControllerParameters, *) CntrPar%VS_RefSpd
        READ(UnControllerParameters, *) CntrPar%VS_n
        ALLOCATE(CntrPar%VS_KP(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KP
        ALLOCATE(CntrPar%VS_KI(CntrPar%VS_n))
        READ(UnControllerParameters,*) CntrPar%VS_KI
        READ(UnControllerParameters,*) CntrPar%VS_TSRopt
        READ(UnControllerParameters, *)

        !------- Setpoint Smoother --------------------------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%SS_VSGain
        READ(UnControllerParameters, *) CntrPar%SS_PCGain
        READ(UnControllerParameters, *) 

        !------------ WIND SPEED ESTIMATOR CONTANTS --------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%WE_BladeRadius
        READ(UnControllerParameters, *) CntrPar%WE_CP_n
        ALLOCATE(CntrPar%WE_CP(CntrPar%WE_CP_n))
        READ(UnControllerParameters, *) CntrPar%WE_CP
        READ(UnControllerParameters, *) CntrPar%WE_Gamma
        READ(UnControllerParameters, *) CntrPar%WE_GearboxRatio
        READ(UnControllerParameters, *) CntrPar%WE_Jtot
        READ(UnControllerParameters, *) CntrPar%WE_RhoAir
        READ(UnControllerParameters, *) CntrPar%PerfFileName
        ALLOCATE(CntrPar%PerfTableSize(2))
        READ(UnControllerParameters, *) CntrPar%PerfTableSize
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_N
        ALLOCATE(CntrPar%WE_FOPoles_v(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles_v
        ALLOCATE(CntrPar%WE_FOPoles(CntrPar%WE_FOPoles_n))
        READ(UnControllerParameters, *) CntrPar%WE_FOPoles
        READ(UnControllerParameters, *)

        !-------------- YAW CONTROLLER CONSTANTS -----------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Y_ErrThresh
        READ(UnControllerParameters, *) CntrPar%Y_IPC_IntSat
        READ(UnControllerParameters, *) CntrPar%Y_IPC_n
        ALLOCATE(CntrPar%Y_IPC_KP(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KP
        ALLOCATE(CntrPar%Y_IPC_KI(CntrPar%Y_IPC_n))
        READ(UnControllerParameters,*) CntrPar%Y_IPC_KI
        READ(UnControllerParameters, *) CntrPar%Y_IPC_omegaLP
        READ(UnControllerParameters, *) CntrPar%Y_IPC_zetaLP
        READ(UnControllerParameters, *) CntrPar%Y_MErrSet
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPFast
        READ(UnControllerParameters, *) CntrPar%Y_omegaLPSlow
        READ(UnControllerParameters, *) CntrPar%Y_Rate
        READ(UnControllerParameters, *)

        !------------ FORE-AFT TOWER DAMPER CONSTANTS ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%FA_KI  
        READ(UnControllerParameters, *) CntrPar%FA_HPFCornerFreq
        READ(UnControllerParameters, *) CntrPar%FA_IntSat
        READ(UnControllerParameters, *)      

        !------------ PEAK SHAVING ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin_N  
        ALLOCATE(CntrPar%PS_WindSpeeds(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_WindSpeeds
        ALLOCATE(CntrPar%PS_BldPitchMin(CntrPar%PS_BldPitchMin_N))
        READ(UnControllerParameters, *) CntrPar%PS_BldPitchMin
        READ(UnControllerParameters, *) 

        !------------ POWER CONTROL ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%PwC_PwrRating_N  
        ALLOCATE(CntrPar%PwC_PwrRating(CntrPar%PwC_PwrRating_N))
        READ(UnControllerParameters, *) CntrPar%PwC_PwrRating
        ALLOCATE(CntrPar%PwC_BldPitchMin(CntrPar%PwC_PwrRating_N))
        READ(UnControllerParameters, *) CntrPar%PwC_BldPitchMin
        READ(UnControllerParameters, *) CntrPar%PwC_ConstPwr
        READ(UnControllerParameters, *) CntrPar%PwC_OpenLoopInp
        READ(UnControllerParameters, *) 

        ! Read Open Loop input if CntrPar%PwC_Mode == 2
        IF (CntrPar%PwC_Mode == 1) THEN
            PRINT *, 'Implementing power control with constant R'
        ELSE IF (CntrPar%PwC_Mode == 2) THEN
            CALL Read_OL_Input(CntrPar%PwC_OpenLoopInp,109,1,CntrPar%PwC_Time,CntrPar%PwC_R_Time)
            PRINT *, 'Implementing open loop power control with R defined in '//TRIM(CntrPar%PwC_OpenLoopInp)
        ELSE IF (CntrPar%PwC_Mode == 3) THEN
            CALL Read_OL_Input(CntrPar%PwC_OpenLoopInp,109,1,CntrPar%PwC_WindSpeed,CntrPar%PwC_R_WindSpeed)
            PRINT *, 'Implementing open loop power control w.r.t. wind speed defined in '//TRIM(CntrPar%PwC_OpenLoopInp)
        END IF
        ! PRINT *, 'Implementing constant power control with R = '//LocalVar%PwC_R

        PRINT *, CntrPar%PwC_WindSpeed
        PRINT *, CntrPar%PwC_R_WindSpeed
        

        !------------ SHUTDOWN ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%SD_MaxPit  
        READ(UnControllerParameters, *) CntrPar%SD_CornerFreq
        READ(UnControllerParameters, *)      

        !------------ FLOATING ------------
        READ(UnControllerParameters, *)
        READ(UnControllerParameters, *) CntrPar%Fl_Kp  
        READ(UnControllerParameters, *) 

        !------------ Flaps ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%Flp_Angle  
        READ(UnControllerParameters, *) CntrPar%Flp_Kp  
        READ(UnControllerParameters, *) CntrPar%Flp_Ki  
        READ(UnControllerParameters, *) CntrPar%Flp_MaxPit  
        READ(UnControllerParameters, *) 


        !------------ Open loop input ------------
        READ(UnControllerParameters, *)      
        READ(UnControllerParameters, *) CntrPar%OL_Filename  
        READ(UnControllerParameters, *) CntrPar%Ind_Breakpoint  
        READ(UnControllerParameters, *) CntrPar%Ind_BldPitch  
        READ(UnControllerParameters, *) CntrPar%Ind_GenTq
        READ(UnControllerParameters, *) CntrPar%Ind_YawRate
        
        ! Read open loop input, if desired
        IF (CntrPar%OL_Mode == 1) THEN
            OL_String = ''
            OL_Count  = 0
            IF (CntrPar%Ind_BldPitch > 0) THEN
                OL_String   = TRIM(OL_String)//' BldPitch '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                OL_String   = TRIM(OL_String)//' GenTq '
                OL_Count    = OL_Count + 1
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                OL_String   = TRIM(OL_String)//' YawRate '
                OL_Count    = OL_Count + 1
            ENDIF

            PRINT *, 'ROSCO: Implementing open loop control for'//TRIM(OL_String)
            CALL Read_OL_Input(CntrPar%OL_Filename,110,OL_Count,CntrPar%OL_Breakpoints,CntrPar%OL_Channels)

            IF (CntrPar%Ind_BldPitch > 0) THEN
                CntrPar%OL_BldPitch = CntrPar%OL_Channels(:,CntrPar%Ind_BldPitch-1)
            ENDIF

            IF (CntrPar%Ind_GenTq > 0) THEN
                CntrPar%OL_GenTq = CntrPar%OL_Channels(:,CntrPar%Ind_GenTq-1)
            ENDIF

            IF (CntrPar%Ind_YawRate > 0) THEN
                CntrPar%OL_YawRate = CntrPar%OL_Channels(:,CntrPar%Ind_YawRate-1)
            ENDIF
        END IF

        ! Debugging outputs:
        write(400,*) CntrPar%OL_Breakpoints
        write(401,*) CntrPar%OL_BldPitch
        write(402,*) CntrPar%OL_GenTq
        write(403,*) CntrPar%OL_YawRate

        ! END OF INPUT FILE    
        
        !------------------- CALCULATED CONSTANTS -----------------------
        CntrPar%PC_RtTq99 = CntrPar%VS_RtTq*0.99
        CntrPar%VS_MinOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_MinOMSpd**2
        CntrPar%VS_MaxOMTq = CntrPar%VS_Rgn2K*CntrPar%VS_RefSpd**2
        CLOSE(UnControllerParameters)
        
        !------------------- HOUSEKEEPING -----------------------
        CntrPar%PerfFileName = TRIM(CntrPar%PerfFileName)

    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Calculate setpoints for primary control actions    
    SUBROUTINE ComputeVariablesSetpoints(CntrPar, LocalVar, DebugVar, objInst)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables
        
        ! Allocate variables
        TYPE(ControlParameters), INTENT(INOUT)  :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)     :: LocalVar
        TYPE(DebugVariables), INTENT(INOUT)     :: DebugVar
        TYPE(ObjectInstances), INTENT(INOUT)    :: objInst

        REAL(8)                                 :: VS_RefSpd        ! Referece speed for variable speed torque controller, [rad/s] 
        REAL(8)                                 :: PC_RefSpd        ! Referece speed for pitch controller, [rad/s] 
        REAL(8)                                 :: Omega_op         ! Optimal TSR-tracking generator speed, [rad/s]
        

        ! Calculate Power reference
        LocalVar%WE_SlowEst = SecLPFilter(LocalVar%WE_Vw,LocalVar%DT,0.0628,0.7,LocalVar%iStatus,.FALSE.,objInst%instSecLPF) ! 30 second time constant
        IF (CntrPar%PwC_Mode == 0) THEN
            LocalVar%PwC_R  = 1.0
        ELSEIF (CntrPar%PwC_Mode == 1) THEN  ! constant power
            LocalVar%PwC_R  = CntrPar%PwC_ConstPwr
        ELSEIF (CntrPar%PwC_Mode == 2) THEN  ! open loop power
            LocalVar%PwC_R  = interp1d(CntrPar%PwC_Time,RESHAPE(CntrPar%PwC_R_Time(:,1),(/size(CntrPar%PwC_Time)/)),LocalVar%Time)
        ELSEIF (CntrPar%PwC_Mode == 3) THEN  ! open loop power vs. wind speed
                LocalVar%PwC_R  = interp1d(CntrPar%PwC_WindSpeed,RESHAPE(CntrPar%PwC_R_WindSpeed(:,1),(/size(CntrPar%PwC_WindSpeed)/)),LocalVar%WE_SlowEst)
        ENDIF  ! add open loop next

        ! ----- Calculate yaw misalignment error -----
        LocalVar%Y_MErr = LocalVar%Y_M + CntrPar%Y_MErrSet ! Yaw-alignment error
        
        ! ----- Pitch controller speed and power error -----
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF < 0) THEN
            PC_RefSpd = LocalVar%PwC_R * CntrPar%PC_RefSpd - LocalVar%SS_DelOmegaF
        ELSE
            PC_RefSpd = LocalVar%PwC_R * CntrPar%PC_RefSpd
        ENDIF

        LocalVar%PC_SpdErr = PC_RefSpd - LocalVar%GenSpeedF            ! Speed error
        LocalVar%PC_PwrErr = CntrPar%VS_RtPwr - LocalVar%VS_GenPwr             ! Power error
        
        ! ----- Torque controller reference errors -----
        ! Define VS reference generator speed [rad/s]
        IF ((CntrPar%VS_ControlMode == 2) .OR. (CntrPar%VS_ControlMode == 3)) THEN
            VS_RefSpd = (CntrPar%VS_TSRopt * LocalVar%We_Vw_F / CntrPar%WE_BladeRadius) * CntrPar%WE_GearboxRatio
            VS_RefSpd = saturate(LocalVar%PwC_R * VS_RefSpd,CntrPar%VS_MinOMSpd, CntrPar%VS_RefSpd)
        ELSE
            VS_RefSpd = LocalVar%PwC_R * CntrPar%VS_RefSpd
        ENDIF 
        
        ! Implement setpoint smoothing
        IF (LocalVar%SS_DelOmegaF > 0) THEN
            VS_RefSpd = VS_RefSpd - LocalVar%SS_DelOmegaF
        ENDIF

        ! Force zero torque in shutdown mode
        IF (LocalVar%SD) THEN
            VS_RefSpd = CntrPar%VS_MinOMSpd
        ENDIF

        ! Force minimum rotor speed
        VS_RefSpd = max(VS_RefSpd, CntrPar%VS_MinOmSpd)

        ! TSR-tracking reference error
        IF ((CntrPar%VS_ControlMode == 2) .OR. (CntrPar%VS_ControlMode == 3)) THEN
            LocalVar%VS_SpdErr = VS_RefSpd - LocalVar%GenSpeedF
        ENDIF

        ! Define transition region setpoint errors
        LocalVar%VS_SpdErrAr = VS_RefSpd - LocalVar%GenSpeedF               ! Current speed error - Region 2.5 PI-control (Above Rated)
        LocalVar%VS_SpdErrBr = CntrPar%VS_MinOMSpd - LocalVar%GenSpeedF     ! Current speed error - Region 1.5 PI-control (Below Rated)
        
        ! Region 3 minimum pitch angle for state machine
        LocalVar%VS_Rgn3Pitch = LocalVar%PC_MinPit + CntrPar%PC_Switch
    
        ! Save Debug Variables
        DebugVar%PwC_R      = LocalVar%PwC_R
        DebugVar%Om_tau     = VS_RefSpd
        DebugVar%Om_theta   = PC_RefSpd
        DebugVar%WE_SlowEst = LocalVar%WE_SlowEst
    
    END SUBROUTINE ComputeVariablesSetpoints
    ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar)
        USE ROSCO_Types, ONLY : LocalVariables
    
        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LocalVar%iStatus = NINT(avrSWAP(1))
        LocalVar%Time = avrSWAP(2)
        LocalVar%DT = avrSWAP(3)
        LocalVar%VS_MechGenPwr = avrSWAP(14)
        LocalVar%VS_GenPwr = avrSWAP(15)
        LocalVar%GenSpeed = avrSWAP(20)
        LocalVar%RotSpeed = avrSWAP(21)
        LocalVar%GenTqMeas = avrSWAP(23)
        LocalVar%Y_M = avrSWAP(24)
        LocalVar%HorWindV = avrSWAP(27)
        LocalVar%rootMOOP(1) = avrSWAP(30)
        LocalVar%rootMOOP(2) = avrSWAP(31)
        LocalVar%rootMOOP(3) = avrSWAP(32)
        LocalVar%FA_Acc = avrSWAP(53)
        LocalVar%NacIMU_FA_Acc = avrSWAP(83)
        LocalVar%Azimuth = avrSWAP(60)
        LocalVar%NumBl = NINT(avrSWAP(61))

          ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does not...
        IF (LocalVar%iStatus == 0) THEN
            LocalVar%BlPitch(1) = avrSWAP(4)
            LocalVar%BlPitch(2) = avrSWAP(33)
            LocalVar%BlPitch(3) = avrSWAP(34)
        ELSE
            LocalVar%BlPitch(1) = LocalVar%PitCom(1)
            LocalVar%BlPitch(2) = LocalVar%PitCom(2)
            LocalVar%BlPitch(3) = LocalVar%PitCom(3)      
        ENDIF

    END SUBROUTINE ReadAvrSWAP
    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution
    SUBROUTINE Assert(LocalVar, CntrPar, avrSWAP, aviFAIL, ErrMsg, size_avcMSG)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters
    
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN)     :: CntrPar
        TYPE(LocalVariables), INTENT(IN)        :: LocalVar
        INTEGER(4), INTENT(IN)                  :: size_avcMSG
        REAL(C_FLOAT), INTENT(IN)               :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        ! Outputs
        INTEGER(C_INT), INTENT(OUT)             :: aviFAIL             ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
        CHARACTER(size_avcMSG-1), INTENT(OUT)   :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
        
        ! Local
        
        !..............................................................................................................................
        ! Check validity of input parameters:
        !..............................................................................................................................
        
        IF ((CntrPar%F_LPFType > 2.0) .OR. (CntrPar%F_LPFType < 1.0)) THEN
            aviFAIL = -1
            ErrMsg  = 'F_LPFType must be 1 or 2.'
        ENDIF
        
        IF ((CntrPar%F_LPFDamping > 1.0) .OR. (CntrPar%F_LPFDamping < 0.0)) THEN
            aviFAIL = -1
            ErrMsg  = 'Filter damping coefficient must be between [0, 1]'
        ENDIF
        
        IF (CntrPar%IPC_CornerFreqAct < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'Corner frequency of IPC actuator model must be positive, or set to 0 to disable.'
        ENDIF
        
        IF (CntrPar%F_LPFCornerFreq <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'CornerFreq must be greater than zero.'
        ENDIF
        
        IF ((CntrPar%IPC_ControlMode > 0) .AND. (CntrPar%Y_ControlMode > 1)) THEN
            aviFAIL = -1
            ErrMsg  = 'IPC control for load reductions and yaw-by-IPC cannot be activated simultaneously'
        ENDIF
        
        IF (LocalVar%DT <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'DT must be greater than zero.'
        ENDIF
        
        IF (CntrPar%VS_MaxRat <= 0.0) THEN
            aviFAIL =  -1
            ErrMsg  = 'VS_MaxRat must be greater than zero.'
        ENDIF
        
        IF (CntrPar%VS_RtTq < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_RtTq must not be negative.'
        ENDIF
        
        IF (CntrPar%VS_Rgn2K < 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_Rgn2K must not be negative.'
        ENDIF
        
        IF (CntrPar%VS_MaxTq < CntrPar%VS_RtTq) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_RtTq must not be greater than VS_MaxTq.'
        ENDIF
        
        IF (CntrPar%VS_KP(1) > 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_KP must be less than zero.'
        ENDIF
        
        IF (CntrPar%VS_KI(1) > 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'VS_KI must be less than zero.'
        ENDIF
        
        IF (CntrPar%PC_RefSpd <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'PC_RefSpd must be greater than zero.'
        ENDIF
        
        IF (CntrPar%PC_MaxRat <= 0.0) THEN
            aviFAIL = -1
            ErrMsg  = 'PC_MaxRat must be greater than zero.'
        ENDIF
        
        IF (CntrPar%PC_MinPit >= CntrPar%PC_MaxPit)  THEN
            aviFAIL = -1
            ErrMsg  = 'PC_MinPit must be less than PC_MaxPit.'
        ENDIF
        
        IF (CntrPar%IPC_KI(1) < 0.0)  THEN
            aviFAIL = -1
            ErrMsg  = 'IPC_KI(1) must be zero or greater than zero.'
        ENDIF
        
        IF (CntrPar%IPC_KI(2) < 0.0)  THEN
            aviFAIL = -1
            ErrMsg  = 'IPC_KI(2) must be zero or greater than zero.'
        ENDIF
        
        ! ---- Yaw Control ----
        IF (CntrPar%Y_ControlMode > 0) THEN
            IF (CntrPar%Y_IPC_omegaLP <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_IPC_omegaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_IPC_zetaLP <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_IPC_zetaLP must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_ErrThresh <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_ErrThresh must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_Rate <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'CntrPar%Y_Rate must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPFast <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_omegaLPFast must be greater than zero.'
            ENDIF
            
            IF (CntrPar%Y_omegaLPSlow <= 0.0)  THEN
                aviFAIL = -1
                ErrMsg  = 'Y_omegaLPSlow must be greater than zero.'
            ENDIF
        ENDIF

        ! --- Floating Control ---
        IF (CntrPar%Fl_Mode > 0) THEN
            IF (CntrPar%F_NotchType <= 1 .OR. CntrPar%F_NotchCornerFreq == 0.0) THEN
                aviFAIL = -1
                ErrMsg = 'F_NotchType and F_NotchCornerFreq must be specified for Fl_Mode greater than zero.'
            ENDIF
        ENDIF
        
        ! Abort if the user has not requested a pitch angle actuator (See Appendix A
        ! of Bladed User's Guide):
        IF (NINT(avrSWAP(10)) /= 0)  THEN ! .TRUE. if a pitch angle actuator hasn't been requested
            aviFAIL = -1
            ErrMsg  = 'Pitch angle actuator not requested.'
        ENDIF
        
        IF (NINT(avrSWAP(28)) == 0 .AND. ((CntrPar%IPC_ControlMode > 0) .OR. (CntrPar%Y_ControlMode > 1))) THEN
            aviFAIL = -1
            ErrMsg  = 'IPC enabled, but Ptch_Cntrl in ServoDyn has a value of 0. Set it to 1.'
        ENDIF

    END SUBROUTINE Assert
    ! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, aviFAIL, accINFILE, ErrMsg, size_avcMSG, CntrPar, LocalVar, objInst, PerfData)
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData
        
        INTEGER(4), INTENT(IN) :: size_avcMSG
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT) :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT) :: objInst
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData

        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        INTEGER(C_INT), INTENT(OUT) :: aviFAIL              ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
        CHARACTER(KIND=C_CHAR), INTENT(IN)      :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
        CHARACTER(size_avcMSG-1), INTENT(OUT) :: ErrMsg     ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
        INTEGER(4) :: K    ! Index used for looping through blades.
        CHARACTER(200) :: git_version
        ! Set aviFAIL to 0 in each iteration:
        aviFAIL = 0
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF = 1
        objInst%instSecLPF = 1
        objInst%instHPF = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch = 1
        objInst%instPI = 1
        
        ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
        avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
        avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
        avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
        avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
        avrSWAP(55) = 0.0 ! Pitch override: 0=yes
        avrSWAP(56) = 0.0 ! Torque override: 0=yes
        avrSWAP(65) = 0.0 ! Number of variables returned for logging
        avrSWAP(72) = 0.0 ! Generator start-up resistance
        avrSWAP(79) = 0.0 ! Request for loads: 0=none
        avrSWAP(80) = 0.0 ! Variable slip current status
        avrSWAP(81) = 0.0 ! Variable slip current demand
        
        ! Read any External Controller Parameters specified in the User Interface
        !   and initialize variables:
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Inform users that we are using this user-defined routine:
            aviFAIL = 1
            git_version = QueryGitVersion()
            ! ErrMsg = '                                                                              '//NEW_LINE('A')// &
            !          '------------------------------------------------------------------------------'//NEW_LINE('A')// &
            !          'Running a controller implemented through NREL''s ROSCO Toolbox                    '//NEW_LINE('A')// &
            !          'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
            !          'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
            !          '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
            !          'Primary development by (listed alphabetically): Nikhar J. Abbas               '//NEW_LINE('A')// &
            !          '                                                Sebastiaan P. Mulders         '//NEW_LINE('A')// &
            !          '                                                Jan-Willem van Wingerden      '//NEW_LINE('A')// &
            !          'Visit our GitHub-page to contribute to this project:                          '//NEW_LINE('A')// &
            !          'https://github.com/NREL/ROSCO                                                 '//NEW_LINE('A')// &
            !          '------------------------------------------------------------------------------'
            ErrMsg = '                                                                              '//NEW_LINE('A')// &
                    '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                    'Running ROSCO-'//TRIM(git_version)//NEW_LINE('A')// &
                    'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                    'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                    '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                    '------------------------------------------------------------------------------'

            CALL ReadControlParameterFileSub(CntrPar, accINFILE, NINT(avrSWAP(50)))

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData)
            ENDIF
            ! Initialize testValue (debugging variable)
            LocalVar%TestType = 0
        
            ! Initialize the SAVED variables:
            LocalVar%PitCom = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call
            LocalVar%Y_AccErr = 0.0  ! This will ensure that the accumulated yaw error starts at zero
            LocalVar%Y_YawEndT = -1.0 ! This will ensure that the initial yaw end time is lower than the actual time to prevent initial yawing
            
            ! Wind speed estimator initialization
            LocalVar%WE_Vw = LocalVar%HorWindV
            LocalVar%WE_VwI = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            
            ! Setpoint Smoother initialization to zero
            LocalVar%SS_DelOmegaF = 0

            ! Generator Torque at K omega^2 or rated
            IF (LocalVar%GenSpeed > 0.98 * CntrPar%PC_RefSpd) THEN
                LocalVar%GenTq = CntrPar%VS_RtTq
            ELSE
                LocalVar%GenTq = min(CntrPar%VS_RtTq, CntrPar%VS_Rgn2K*LocalVar%GenSpeed*LocalVar%GenSpeed)
            ENDIF            
            LocalVar%VS_LastGenTrq = LocalVar%GenTq       
            LocalVar%VS_MaxTq      = CntrPar%VS_MaxTq
            ! Check validity of input parameters:
            CALL Assert(LocalVar, CntrPar, avrSWAP, aviFAIL, ErrMsg, size_avcMSG)
            

        ENDIF
    END SUBROUTINE SetParameters
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters

        INTEGER(4), PARAMETER :: UnPerfParameters = 89
        TYPE(PerformanceData), INTENT(INOUT) :: PerfData
        TYPE(ControlParameters), INTENT(INOUT) :: CntrPar
        ! Local variables
        INTEGER(4)                  :: i ! iteration index
        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read') ! Should put input file into DISCON.IN
        
        ! ----------------------- Axis Definitions ------------------------
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        READ(UnPerfParameters, *)
        ALLOCATE(PerfData%Beta_vec(CntrPar%PerfTableSize(1)))
        READ(UnPerfParameters, *) PerfData%Beta_vec
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%TSR_vec(CntrPar%PerfTableSize(2)))
        READ(UnPerfParameters, *) PerfData%TSR_vec

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) ! Input file should contains wind speed information here - unneeded for now
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cp_mat(i,:) ! Read Cp table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Ct_mat(i,:) ! Read Ct table
        END DO
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        READ(UnPerfParameters, *) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(1),CntrPar%PerfTableSize(2)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *) PerfData%Cq_mat(i,:) ! Read Cq table
        END DO
    
    END SUBROUTINE ReadCpFile

    ! ------------------------------------------------------
    ! Read Open Loop Control Inputs
    ! 
    ! Timeseries or lookup tables of the form
    ! index (time or wind speed)   channel_1 \t channel_2 \t channel_3 ...
    SUBROUTINE Read_OL_Input(OL_InputFileName, Unit_OL_Input, NumChannels, Breakpoints, Channels)

        CHARACTER(1024), INTENT(IN)                             :: OL_InputFileName    ! DISCON input filename
        INTEGER(4), INTENT(IN)                                  :: Unit_OL_Input 
        INTEGER(4), INTENT(IN)                                  :: NumChannels     ! Number of open loop channels being defined

        LOGICAL                                                 :: FileExists
        INTEGER                                                 :: IOS                                                 ! I/O status of OPEN.
        CHARACTER(1024)                                         :: Line              ! Temp variable for reading whole line from file
        INTEGER(4)                                              :: NumComments
        INTEGER(4)                                              :: NumDataLines
        INTEGER(4)                                              :: NumCols 
        REAL(8)                                                 :: TmpData(NumChannels+1)  ! Temp variable for reading all columns from a line
        CHARACTER(15)                                           :: NumString

        REAL(8), INTENT(OUT), DIMENSION(:), ALLOCATABLE         :: Breakpoints    ! Breakpoints of open loop Channels
        REAL(8), INTENT(OUT), DIMENSION(:,:), ALLOCATABLE       :: Channels         ! Open loop channels
        INTEGER(4)                                              :: I,J

        NumCols             = NumChannels + 1

        !-------------------------------------------------------------------------------------------------
        ! Read from input file, borrowed (read: copied) from (Open)FAST team...thanks!
        !-------------------------------------------------------------------------------------------------

        !-------------------------------------------------------------------------------------------------
        ! Open the file for reading
        !-------------------------------------------------------------------------------------------------

        INQUIRE (FILE = OL_InputFileName, EXIST = FileExists)

        IF ( .NOT. FileExists) THEN
            PRINT *, TRIM(OL_InputFileName)// ' does not exist, setting Channels = 1 for all time'
            ALLOCATE(Breakpoints(2))
            ALLOCATE(Channels(2,1))
            Channels(1,1) = 1;              Channels(2,1)           = 1
            Breakpoints(1) = 0;             Breakpoints(2)          = 90000;

        ELSE

            OPEN( Unit_OL_Input, FILE=TRIM(OL_InputFileName), STATUS='OLD', FORM='FORMATTED', IOSTAT=IOS, ACTION='READ' )

            IF (IOS /= 0) THEN
                PRINT *, 'Cannot open ' // TRIM(OL_InputFileName) // ', setting R = 1 for all time'
                ALLOCATE(Breakpoints(2))
                ALLOCATE(Channels(2,1))
                Channels(1,1) = 1;              Channels(2,1) = 1
                Breakpoints(1) = 0;             Breakpoints(2)       = 90000;
                CLOSE(Unit_OL_Input)
            
            ELSE
                ! Do all the stuff!

                !-------------------------------------------------------------------------------------------------
                ! Find the number of comment lines
                !-------------------------------------------------------------------------------------------------

                LINE = '!'                          ! Initialize the line for the DO WHILE LOOP
                NumComments = -1                    ! the last line we read is not a comment, so we'll initialize this to -1 instead of 0

                DO WHILE ( (INDEX( LINE, '!' ) > 0) .OR. (INDEX( LINE, '#' ) > 0) .OR. (INDEX( LINE, '%' ) > 0) ) ! Lines containing "!" are treated as comment lines
                    NumComments = NumComments + 1
                    
                    READ(Unit_OL_Input,'( A )',IOSTAT=IOS) LINE

                    ! NWTC_IO has some error catching here that we'll skip for now
            
                END DO !WHILE

                !-------------------------------------------------------------------------------------------------
                ! Find the number of data lines
                !-------------------------------------------------------------------------------------------------

                NumDataLines = 0

                READ(LINE,*,IOSTAT=IOS) ( TmpData(I), I=1,NumCols ) ! this line was read when we were figuring out the comment lines; let's make sure it contains

                DO WHILE (IOS == 0)  ! read the rest of the file (until an error occurs)
                    NumDataLines = NumDataLines + 1
                    
                    READ(Unit_OL_Input,*,IOSTAT=IOS) ( TmpData(I), I=1,NumCols )
                
                END DO !WHILE
            
            
                IF (NumDataLines < 1) THEN
                    WRITE (NumString,'(I11)')  NumComments
                    PRINT *, 'Error: '//TRIM(NumString)//' comment lines were found in the uniform wind file, '// &
                                'but the first data line does not contain the proper format.'
                    CLOSE(Unit_OL_Input)
                END IF

                !-------------------------------------------------------------------------------------------------
                ! Allocate arrays for the uniform wind data
                !-------------------------------------------------------------------------------------------------
                ALLOCATE(Breakpoints(NumDataLines))
                ALLOCATE(Channels(NumDataLines,NumChannels))

                !-------------------------------------------------------------------------------------------------
                ! Rewind the file (to the beginning) and skip the comment lines
                !-------------------------------------------------------------------------------------------------

                REWIND( Unit_OL_Input )

                DO I=1,NumComments
                    READ(Unit_OL_Input,'( A )',IOSTAT=IOS) LINE
                END DO !I
            

                !-------------------------------------------------------------------------------------------------
                ! Read the data arrays
                !-------------------------------------------------------------------------------------------------
            
                DO I=1,NumDataLines
                
                    READ(Unit_OL_Input,*,IOSTAT=IOS) ( TmpData(J), J=1,NumCols )

                    IF (IOS > 0) THEN
                        CLOSE(Unit_OL_Input)
                    END IF

                    Breakpoints(I)          = TmpData(1)
                    Channels(I,:)        = TmpData(2:)
            
                END DO !I     
            END IF
        END IF
    
    END SUBROUTINE Read_OL_Input


END MODULE ReadSetParameters
