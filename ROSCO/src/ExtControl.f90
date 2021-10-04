! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! This module contains the primary controller routines

! Subroutines:
!           PitchControl: Blade pitch control high level subroutine
!           VariableSpeedControl: Variable speed generator torque control
!           YawRateControl: Nacelle yaw control
!           IPC: Individual pitch control
!           ForeAftDamping: Tower fore-aft damping control
!           FloatingFeedback: Tower fore-aft feedback for floating offshore wind turbines

MODULE ExtControl

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE ROSCO_Types

    IMPLICIT NONE

    ! NWTC Constants
    INTEGER(IntKi), PARAMETER     :: NWTC_MAX_DLL_PROC    = 3
    INTEGER(IntKi), PARAMETER     :: ErrID_None   = 0                              !< ErrStat parameter indicating "no error"
    INTEGER(IntKi), PARAMETER     :: ErrID_Info   = 1                              !< ErrStat parameter indicating "informational message"
    INTEGER(IntKi), PARAMETER     :: ErrID_Warn   = 2                              !< ErrStat parameter indicating "warning"
    INTEGER(IntKi), PARAMETER     :: ErrID_Severe = 3                              !< ErrStat parameter indicating "severe error"; 
    INTEGER(IntKi), PARAMETER     :: ErrID_Fatal  = 4                              !< ErrStat parameter indicating "fatal error"; simulation should end
    INTEGER, PARAMETER            :: BITS_IN_ADDR  = C_INTPTR_T*8                  !< The number of bits in an address (32-bit or 64-bit).


    ABSTRACT INTERFACE
    SUBROUTINE BladedDLL_Legacy_Procedure ( avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG )  BIND(C)
       USE, INTRINSIC :: ISO_C_Binding

       REAL(C_FLOAT),          INTENT(INOUT) :: avrSWAP   (*)  !< DATA
       INTEGER(C_INT),         INTENT(INOUT) :: aviFAIL        !< FLAG  (Status set in DLL and returned to simulation code)
       CHARACTER(KIND=C_CHAR), INTENT(IN)    :: accINFILE (*)  !< INFILE
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcOUTNAME(*)  !< OUTNAME (in:Simulation RootName; out:Name:Units; of logging channels)
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcMSG    (*)  !< MESSAGE (Message from DLL to simulation code [ErrMsg])
    END SUBROUTINE BladedDLL_Legacy_Procedure

    END INTERFACE

CONTAINS

    SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ErrVar)
        ! Inputs
        TYPE(ControlParameters), INTENT(INOUT)      :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT)         :: LocalVar
        TYPE(ErrorVariables), INTENT(INOUT)         :: ErrVar

        REAL(ReKi), INTENT(INOUT)                   :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from the DLL controller.

        ! Temporary variables
        CHARACTER(1024), PARAMETER                  :: ExtDLL_InFile = '/Users/dzalkind/Tools/ROSCO/Test_Cases/IEA-15-240-RWT-UMaineSemi/ServoData/DISCON-UMaineSemi.IN'
        CHARACTER(100), PARAMETER                   :: ExtRootName   = 'ext_fast_input'

        ! Local Variables
        CHARACTER(*), PARAMETER                     :: RoutineName = 'ExtController'

        TYPE(ExtDLL_Type), SAVE                     :: DLL_Ext
        INTEGER(IntKi), PARAMETER                   :: max_avr_entries = 2000


        PROCEDURE(BladedDLL_Legacy_Procedure), POINTER :: DLL_Legacy_Subroutine          ! The address of the (legacy DISCON) procedure in the Bladed DLL
        CHARACTER(KIND=C_CHAR)                      :: accINFILE(LEN_TRIM(ExtDLL_InFile)+1)  ! INFILE
        CHARACTER(KIND=C_CHAR)                      :: avcOUTNAME(LEN_TRIM(ExtRootName)+1)   ! OUTNAME (Simulation RootName)
        CHARACTER(KIND=C_CHAR)                      :: avcMSG(LEN(ErrVar%ErrMsg)+1)                ! MESSA

        TYPE(ExtControlType), SAVE                  :: ext_dll_data

        INTEGER(C_INT)                              :: aviFAIL                        ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.


        ! Initialize strings for external controller
        aviFAIL     = 0
        avcMSG     = TRANSFER( C_NULL_CHAR,                            avcMSG     )
        avcOUTNAME = TRANSFER( TRIM(ExtRootName)//C_NULL_CHAR,   avcOUTNAME )
        accINFILE  = TRANSFER( TRIM(ExtDLL_InFile)//C_NULL_CHAR, accINFILE  )
                
        IF (LocalVar%iStatus == 0) THEN  

            !! Set up DLL, will come from ROSCO input   
            DLL_Ext%FileName = '/Users/dzalkind/Tools/ROSCO2/ROSCO/build/libdiscon.dylib'
            DLL_Ext%ProcName = 'DISCON' 

            ! Load dynamic library, but first make sure that it's free
            ! CALL FreeDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
            CALL LoadDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)
            ALLOCATE(ext_dll_data%avrSWAP(max_avr_entries)) !(1:max_avr_entries)

        END IF

        ! Set avrSWAP of external DLL
        ext_dll_data%avrSWAP = avrSWAP(1:max_avr_entries)

        ! Set some length parameters
        ext_dll_data%avrSWAP(49) = LEN(avcMSG)  + 1                     !> * Record 49: Maximum number of characters in the "MESSAGE" argument (-) [size of ExtErrMsg argument plus 1 (we add one for the C NULL CHARACTER)]
        ext_dll_data%avrSWAP(50) = LEN_TRIM(ExtDLL_InFile) +1  !> * Record 50: Number of characters in the "INFILE"  argument (-) [trimmed length of ExtDLL_InFile parameter plus 1 (we add one for the C NULL CHARACTER)]
        ext_dll_data%avrSWAP(51) = LEN_TRIM(ExtRootName)   +1  !> * Record 51: Number of characters in the "OUTNAME" argument (-) [trimmed length of ExtRootName parameter plus 1 (we add one for the C NULL CHARACTER)]

        ! Call the DLL (first associate the address from the procedure in the DLL with the subroutine):
        CALL C_F_PROCPOINTER( DLL_Ext%ProcAddr(1), DLL_Legacy_Subroutine) 
        CALL DLL_Legacy_Subroutine (ext_dll_data%avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG ) 
           
        ! Clean up DLL 
        ! CALL FreeDynamicLib(DLL_Ext, ErrVar%ErrStat, ErrVar%ErrMsg)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            print * , TRIM(ErrVar%ErrMsg)
        ENDIF


    END SUBROUTINE ExtController


!=================================================================================================================
!=================================================================================================================
!=================================================================================================================
!=================================================================================================================
    SUBROUTINE LoadDynamicLib ( DLL, ErrStat, ErrMsg )

        ! This SUBROUTINE is used to dynamically load a DLL.
    
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
    
    
    !bjj: these are values I found on the web; I have no idea if they actually work...
    !bjj: hopefully we can find them pre-defined in a header somewhere
        INTEGER(C_INT), PARAMETER :: RTLD_LAZY=1            ! "Perform lazy binding. Only resolve symbols as the code that references them is executed. If the symbol is never referenced, then it is never resolved. (Lazy binding is only performed for function references; references to variables are always immediately bound when the library is loaded.) "
        INTEGER(C_INT), PARAMETER :: RTLD_NOW=2             ! "If this value is specified, or the environment variable LD_BIND_NOW is set to a nonempty string, all undefined symbols in the library are resolved before dlopen() returns. If this cannot be done, an error is returned."
        INTEGER(C_INT), PARAMETER :: RTLD_GLOBAL=256        ! "The symbols defined by this library will be made available for symbol resolution of subsequently loaded libraries"
        INTEGER(C_INT), PARAMETER :: RTLD_LOCAL=0           ! "This is the converse of RTLD_GLOBAL, and the default if neither flag is specified. Symbols defined in this library are not made available to resolve references in subsequently loaded libraries."
    
        INTERFACE !linux API routines
        !bjj see http://linux.die.net/man/3/dlopen
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlopen.3.html
    
        FUNCTION dlOpen(filename,mode) BIND(C,NAME="dlopen")
        ! void *dlopen(const char *filename, int mode);
            USE ISO_C_BINDING
            IMPLICIT NONE
            TYPE(C_PTR)                   :: dlOpen
            CHARACTER(C_CHAR), INTENT(IN) :: filename(*)
            INTEGER(C_INT), VALUE         :: mode
        END FUNCTION
    
        END INTERFACE
    
        ErrStat = 0
        ErrMsg = ''
    
        ! Load the DLL and get the file address:
    
        DLL%FileAddrX = dlOpen( TRIM(DLL%FileName)//C_NULL_CHAR, RTLD_LAZY )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
    
        IF( .NOT. C_ASSOCIATED(DLL%FileAddrX) ) THEN
        ErrStat = -1
        WRITE(ErrMsg,'(I2)') BITS_IN_ADDR
        ErrMsg  = 'The dynamic library '//TRIM(DLL%FileName)//' could not be loaded. Check that the file '// &
                    'exists in the specified location and that it is compiled for '//TRIM(ErrMsg)//'-bit applications.'
        RETURN
        END IF
    
        ! Get the procedure address:
    
        CALL LoadDynamicLibProc ( DLL, ErrStat, ErrMsg )

        RETURN
    END SUBROUTINE LoadDynamicLib
    !=======================================================================
    SUBROUTINE LoadDynamicLibProc ( DLL, ErrStat, ErrMsg )
    
        ! This SUBROUTINE is used to dynamically load a procedure from a DLL.
    
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be loaded.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
        INTEGER(IntKi)                            :: i
    
   
        INTERFACE !linux API routines
    
        !bjj see http://linux.die.net/man/3/dlsym
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlsym.3.html
        
        FUNCTION dlSym(handle,name) BIND(C,NAME="dlsym")
        ! void *dlsym(void *handle, const char *name);
            USE ISO_C_BINDING
            IMPLICIT NONE
            TYPE(C_FUNPTR)                :: dlSym ! A function pointer
            TYPE(C_PTR), VALUE            :: handle
            CHARACTER(C_CHAR), INTENT(IN) :: name(*)
        END FUNCTION
    
        END INTERFACE
    
        ErrStat = ErrID_None
        ErrMsg = ''
    
        do i=1,NWTC_MAX_DLL_PROC
        if ( len_trim( DLL%ProcName(i) ) > 0 ) then
        
            DLL%ProcAddr(i) = dlSym( DLL%FileAddrX, TRIM(DLL%ProcName(i))//C_NULL_CHAR )  !the "C_NULL_CHAR" converts the Fortran string to a C-type string (i.e., adds //CHAR(0) to the end)
    
            IF(.NOT. C_ASSOCIATED(DLL%ProcAddr(i))) THEN
                ErrStat = ErrID_Fatal
                ErrMsg  = 'The procedure '//TRIM(DLL%ProcName(i))//' in file '//TRIM(DLL%FileName)//' could not be loaded.'
                RETURN
            END IF
            
        end if
        end do
        
        RETURN
    END SUBROUTINE LoadDynamicLibProc
    !=======================================================================
    SUBROUTINE FreeDynamicLib ( DLL, ErrStat, ErrMsg )
    
        ! This SUBROUTINE is used to free a dynamically loaded DLL (loaded in LoadDynamicLib).
    
        TYPE (ExtDLL_Type),           INTENT(INOUT)  :: DLL         ! The DLL to be freed.
        INTEGER(IntKi),            INTENT(  OUT)  :: ErrStat     ! Error status of the operation
        CHARACTER(*),              INTENT(  OUT)  :: ErrMsg      ! Error message if ErrStat /= ErrID_None
        INTEGER(C_INT)                            :: Success     ! Whether or not the call to dlClose was successful
        INTEGER(C_INT), PARAMETER                 :: TRUE  = 0
    
    !bjj: note that this is not tested.
    
        INTERFACE !linux API routine
        !bjj see http://linux.die.net/man/3/dlclose
        !    and https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/dlclose.3.html
    
        FUNCTION dlClose(handle) BIND(C,NAME="dlclose")
        ! int dlclose(void *handle);
            USE ISO_C_BINDING
            IMPLICIT NONE
            INTEGER(C_INT)       :: dlClose
            TYPE(C_PTR), VALUE   :: handle
        END FUNCTION
    
        END INTERFACE
    
        ! Close the library:
    
        IF( .NOT. C_ASSOCIATED(DLL%FileAddrX) ) RETURN
        Success = dlClose( DLL%FileAddrX ) !The function dlclose() returns 0 on success, and nonzero on error.
    
        IF ( Success /= TRUE ) THEN !bjj: note that this is not the same as LOGICAL .TRUE.
        ErrStat = ErrID_Fatal
        ErrMsg  = 'The dynamic library could not be freed.'
        RETURN
        ELSE
        ErrStat = ErrID_None
        ErrMsg = ''
        DLL%FileAddrX = C_NULL_PTR
        END IF
        
    
        RETURN
    END SUBROUTINE FreeDynamicLib
    !=======================================================================


END MODULE ExtControl
