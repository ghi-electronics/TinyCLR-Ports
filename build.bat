@ECHO OFF

SETLOCAL EnableDelayedExpansion

SET ScriptRoot=%~dp0

IF %ScriptRoot:~-1%==\ SET ScriptRoot=%ScriptRoot:~0,-1%

PUSHD "%ScriptRoot%"

SET DeviceName=%1
SET BuildTarget=%2
SET BuildConfiguration=%3
SET BuildVerbosity=%4
SET GccDirectory=%5

IF "%BuildTarget%" == "" SET BuildTarget=build
IF "%BuildConfiguration%" == "" SET BuildConfiguration=release
IF "%BuildVerbosity%" == "" SET BuildVerbosity=quiet

IF NOT EXIST "%ScriptRoot%\Devices\%DeviceName%" (
    ECHO Unsupported device passed: %DeviceName%
    GOTO :EOF
)

IF NOT "%BuildTarget%" == "build" IF NOT "%BuildTarget%" == "cleanbuild" IF NOT "%BuildTarget%" == "clean" (
    ECHO Unsupported target passed: %BuildTarget%
    GOTO :EOF
)

IF NOT "%BuildConfiguration%" == "debug" IF NOT "%BuildConfiguration%" == "release" (
    ECHO Unsupported configuration passed: %BuildConfiguration%
    GOTO :EOF
)

IF NOT EXIST "%GccDirectory%" SET GccDirectory=C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update
IF NOT EXIST "%GccDirectory%" SET GccDirectory=C:\Program Files\GNU Tools ARM Embedded\6 2017-q2-update

IF NOT EXIST "%GccDirectory%" (
    ECHO Cannot find GCC. Try passing the directory explicitly like `build.bat device build release normal "C:\Program Files (x86)\GNU Tools ARM Embedded\6 2017-q2-update"`
    GOTO :EOF
)

IF "%GccDirectory:~-1%"=="\" SET "GccDirectory=%GccDirectory:~0,-1%"

SET DeviceBuildConfiguration=%ScriptRoot%\Devices\%DeviceName%\BuildConfiguration.txt

IF NOT EXIST "%DeviceBuildConfiguration%" (
    ECHO Cannot find device configuration at "%DeviceBuildConfiguration%".
    GOTO :EOF
)

FOR /F "usebackq tokens=1,2 delims=:" %%A IN ("%DeviceBuildConfiguration%") DO SET %%A=%%B

IF "%TargetName%" == "" (
    ECHO TargetName not defined in "%DeviceBuildConfiguration%".
    GOTO :EOF
)

SET TargetBuildConfiguration=%ScriptRoot%\Targets\%TargetName%\BuildConfiguration.txt

IF NOT EXIST "%TargetBuildConfiguration%" (
    ECHO Cannot find target configuration at "%TargetBuildConfiguration%".
    GOTO :EOF
)

FOR /F "usebackq tokens=1,2 delims=:" %%A IN ("%TargetBuildConfiguration%") DO SET %%A=%%B

IF "%TargetArchitecture%" == "" (
    ECHO TargetArchitecture not defined in "%TargetBuildConfiguration%".
    GOTO :EOF
)

SET LibraryBuildConfiguration=%ScriptRoot%\Core\%TargetArchitecture%BuildConfiguration.txt

IF NOT EXIST "%LibraryBuildConfiguration%" (
    ECHO Cannot find library configuration at "%LibraryBuildConfiguration%".
    GOTO :EOF
)

FOR /F "usebackq tokens=1,2 delims=:" %%A IN ("%LibraryBuildConfiguration%") DO SET "%%A=%%B"

IF "%NeedCmsis%" == "1" (
    IF NOT EXIST "%ScriptRoot%\CMSIS\ARM.CMSIS.pdsc" (
        ECHO Cannot find CMSIS. Please make sure it is downloaded and extracted to repo_root\CMSIS.
        GOTO :EOF
    )
)

SET AdditionalIncludes=%AdditionalIncludes% -I"%GccDirectory%\lib\gcc\arm-none-eabi\6.3.1\include"
SET AdditionalIncludes=%AdditionalIncludes% -I"%ScriptRoot%\Targets\%TargetName%"
SET AdditionalIncludes=%AdditionalIncludes% -I"%ScriptRoot%\Devices\%DeviceName%"
SET AdditionalIncludes=%AdditionalIncludes% -I"%ScriptRoot%\Core"

SET AdditionalDefines=%AdditionalDefines% -DGCC
SET AdditionalCompilerArguments=-mstructure-size-boundary=8 -fno-exceptions -ffunction-sections -fdata-sections -fshort-wchar -funsigned-char -mlong-calls

IF "%BuildConfiguration%" == "debug" (
    SET AdditionalDefines=%AdditionalDefines% -DDEBUG -D_DEBUG
    SET AdditionalAssemblerArguments=%AdditionalAssemblerArguments% -g
    SET AdditionalCompilerArguments=-O0 -femit-class-debug-always -g3 -ggdb %AdditionalCompilerArguments%
) ELSE (
    SET AdditionalDefines=%AdditionalDefines%
    SET AssemblerCompileArguments=%AdditionalAssemblerArguments%
    SET AdditionalCompilerArguments=%OptimizeLevel% %AdditionalCompilerArguments%
)

SET OutputDirectory=%ScriptRoot%\Build\%BuildConfiguration%\%DeviceName%
SET CoreDirectory=%ScriptRoot%\Core
SET CoreLibraryFile=%CoreDirectory%\TinyCLR_%TargetArchitecture%.lib
SET CoreHeaderFile=%CoreDirectory%\TinyCLR.h

IF "%BuildTarget%" == "cleanbuild" (
    SET DoClean=1
    SET DoBuild=1
) ELSE IF "%BuildTarget%" == "build" (
    SET DoClean=0
    SET DoBuild=1
) ELSE IF "%BuildTarget%" == "clean" (
    SET DoClean=1
    SET DoBuild=0
)

IF "%DoClean%" == "1" IF EXIST "%OutputDirectory%" (
    RMDIR /S /Q "%OutputDirectory%"
)

IF "%DoBuild%" == "1" (
    IF NOT EXIST "%OutputDirectory%" MD "%OutputDirectory%"

    IF NOT EXIST "%CoreLibraryFile%" (
        ECHO Cannot find the core library file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR_%TargetArchitecture%.lib.
        GOTO :EOF
    )

    IF NOT EXIST "%CoreHeaderFile%" (
        ECHO Cannot find the core header file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR.h.
        GOTO :EOF
    )

    FOR %%A IN ("%ScriptRoot%\Targets\%TargetName%", "%ScriptRoot%\Devices\%DeviceName%", "%ScriptRoot%\Main") DO (
        PUSHD "%%A"

        FOR /R %%B IN ("*.gcc.s") DO (
            ECHO %%B

            "%GccDirectory%\bin\arm-none-eabi-as.exe" %AdditionalAssemblerArguments% -mcpu=%MCpu% -mlittle-endian %FloatCompileArguments% -o "%OutputDirectory%\%%~nB.obj" "%%B"
        )

        FOR /R %%B IN ("*.cpp") DO (
            ECHO %%B

            "%GccDirectory%\bin\arm-none-eabi-g++.exe" -std=c++11 -xc++ %AdditionalCompilerArguments% -mcpu=%MCpu% -mlittle-endian %FloatCompileArguments% %AdditionalDefines% %AdditionalIncludes% -o "%OutputDirectory%\%%~nB.obj" -c "%%B"
        )

        POPD
    )

    "%GccDirectory%\bin\arm-none-eabi-g++.exe" -mcpu=%MCpu% -mlittle-endian -nostartfiles %FloatCompileArguments% -Xlinker %AdditionalCompilerArguments% -L "%GccLibrary%" -Wl,-static,--gc-sections,--no-wchar-size-warning,-Map="%OutputDirectory%\%DeviceName% Firmware.map" -specs="%GccLibrary%\nano.specs" -T"%ScriptRoot%\Devices\%DeviceName%\Scatterfile.gcc.ldf" "%OutputDirectory%\*.obj" "%CoreLibraryFile%" -o "%OutputDirectory%\%DeviceName% Firmware.axf"

    "%GccDirectory%\bin\arm-none-eabi-objcopy.exe" -S -j ER_FLASH -j ER_RAM_RO -j ER_RAM_RW -O binary "%OutputDirectory%\%DeviceName% Firmware.axf" "%OutputDirectory%\%DeviceName% Firmware.bin"
    "%GccDirectory%\bin\arm-none-eabi-objcopy.exe" -S -R ER_DAT -R ER_CONFIG -O ihex "%OutputDirectory%\%DeviceName% Firmware.axf" "%OutputDirectory%\%DeviceName% Firmware.hex"

    IF NOT "%ImageGenParameters%" == ""  "%ScriptRoot%\imagegen.exe" %ImageGenParameters% "%OutputDirectory%\%DeviceName% Firmware.bin"
)

ECHO Done

POPD

ENDLOCAL
