@echo off
REM Build all PXI compatibility test DLLs
REM Run from: ROSCO-USFLOWT\Examples\Labview_Tests\
REM
REM veh_stubs.c provides local implementations of AddVectoredExceptionHandler
REM and RemoveVectoredExceptionHandler so they are NOT imported from KERNEL32.dll
REM (Phar Lap ETS doesn't have them).

echo Compiling VEH stubs...
gcc -m32 -c -o veh_stubs.o veh_stubs.c
if errorlevel 1 (echo FAILED to compile veh_stubs.c & goto :eof)

set LFLAGS=-m32 -shared -static -static-libgcc -static-libstdc++ -Wl,--kill-at

echo Building test_1 (minimal C++) ...
g++ %LFLAGS% -DBUILD_TEST1 -o test_1.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo Building test_2 (new/delete) ...
g++ %LFLAGS% -DBUILD_TEST2 -o test_2.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo Building test_3 (std::string) ...
g++ %LFLAGS% -DBUILD_TEST3 -o test_3.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo Building test_4 (C file I/O) ...
g++ %LFLAGS% -DBUILD_TEST4 -o test_4.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo Building test_5 (C++ streams) ...
g++ %LFLAGS% -DBUILD_TEST5 -o test_5.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo Building test_6 (strings + malloc + snprintf + getenv) ...
g++ %LFLAGS% -DBUILD_TEST6 -o test_6.dll pxi_tests.c veh_stubs.o
if errorlevel 1 (echo FAILED & goto :eof)

echo.
echo === All builds succeeded ===
echo.
echo Verifying dependencies...
echo.

for %%f in (test_1.dll test_2.dll test_3.dll test_4.dll test_5.dll test_6.dll) do (
    echo --- %%f ---
    objdump -p %%f | findstr "DLL Name"
    echo.
)

echo === Deploy all test_*.dll to PXI C:\ni-rt\system\ ===
echo === Test each via CLFN: function=test_N, return=i32, convention=C ===
echo === Expected return value = test number (1, 2, 3, ...) ===
echo === First DLL that fails to LOAD identifies the problem category ===
