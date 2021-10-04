! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! High level run script

!=======================================================================
SUBROUTINE DISCON(avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG) BIND (C, NAME='DISCON')
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

USE, INTRINSIC  :: ISO_C_Binding
USE             :: ROSCO_Types
USE             :: ReadSetParameters
USE             :: ControllerBlocks
USE             :: Controllers
USE             :: Constants
USE             :: Filters
USE             :: Functions
USE             :: ExtControl

IMPLICIT NONE
! Enable .dll export
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
#endif

!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------

! Passed Variables:
!REAL(ReKi), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(ReKi), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller

REAL(ReKi), INTENT(INOUT)            :: avrSWAP(*)                       ! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT), INTENT(INOUT)           :: aviFAIL                          ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR), INTENT(IN)      :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
CHARACTER(KIND=C_CHAR), INTENT(IN)      :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR), INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME)-1)           :: RootName                         ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)               :: ErrMsg                           ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]

TYPE(ControlParameters), SAVE         :: CntrPar
TYPE(LocalVariables), SAVE            :: LocalVar
TYPE(ObjectInstances), SAVE           :: objInst
TYPE(PerformanceData), SAVE           :: PerfData
TYPE(DebugVariables), SAVE            :: DebugVar
TYPE(ErrorVariables), SAVE            :: ErrVar

CHARACTER(*), PARAMETER               :: RoutineName = 'ROSCO'

RootName = TRANSFER(avcOUTNAME, RootName)
!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------
! Read avrSWAP array into derived types/variables
PRINT *, "AvrSWAP:", avrSWAP(1:85)
CALL ReadAvrSWAP(avrSWAP, LocalVar)

PRINT *, "hello from build"
PRINT *, "RootName:", avcOUTNAME
! Set Control Parameters
CALL SetParameters(avrSWAP, accINFILE, SIZE(avcMSG), CntrPar, LocalVar, objInst, PerfData, ErrVar)

! Call external controller, if desired
CALL ExtController(avrSWAP, CntrPar, LocalVar, ErrVar)
PRINT *, "Back in DISCON"

! Overwrite with ROSCO, where desired

! Filter signals
PRINT *, "Starting PFMS"
CALL PreFilterMeasuredSignals(CntrPar, LocalVar, objInst, ErrVar)
PRINT *, "Finished PFMS"

IF ((LocalVar%iStatus >= 0) .AND. (ErrVar%aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step
    
    PRINT *, "Starting WSE"
    CALL WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    PRINT *, "Starting CVS"
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
    PRINT *, "Starting SM"
    CALL StateMachine(CntrPar, LocalVar)
    PRINT *, "Starting SS"
    CALL SetpointSmoother(LocalVar, CntrPar, objInst)
    
    PRINT *, "Starting CVS2"
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)

    PRINT *, 'Finished calling ComputeVariablesSetpoints2'


    CALL VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst)

    PRINT *, 'Finished calling VariableSpeedControl'


    CALL PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)

    PRINT *, 'Finished calling PitchControl'

    
    IF (CntrPar%Y_ControlMode > 0) THEN
        CALL YawRateControl(avrSWAP, CntrPar, LocalVar, objInst)
    END IF
    
    IF (CntrPar%Flp_Mode > 0) THEN
        CALL FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    END IF
    
    IF (CntrPar%LoggingLevel > 0) THEN
        CALL Debug(LocalVar, CntrPar, DebugVar, avrSWAP, RootName, SIZE(avcOUTNAME))
    END IF

    PRINT *, 'Finished calling Debug'

END IF

! print *, avrSWAP(43)

! Add RoutineName to error message
IF (ErrVar%aviFAIL < 0) THEN
    ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    print * , TRIM(ErrVar%ErrMsg)
ENDIF
ErrMsg = ADJUSTL(TRIM(ErrVar%ErrMsg))
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, LEN(ErrMsg)+1)
avcMSG = TRANSFER(ErrMsg//C_NULL_CHAR, avcMSG, SIZE(avcMSG))
aviFAIL = ErrVar%aviFAIL

ErrVar%ErrMsg = ''

PRINT *, 'Finished calling ROSCO'


RETURN
END SUBROUTINE DISCON
