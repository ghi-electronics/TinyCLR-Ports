@ECHO OFF

SETLOCAL

SET ScriptRoot=%~dp0

IF %ScriptRoot:~-1%==\ SET ScriptRoot=%ScriptRoot:~0,-1%

PUSHD "%ScriptRoot%"

SET Device=%1
SET BuildTarget=%2
SET BuildConfiguration=%3
SET BuildVerbosity=%4
SET GccDirectory=%5

IF "%BuildTarget%" == "" SET BuildTarget=build
IF "%BuildConfiguration%" == "" SET BuildConfiguration=release
IF "%BuildVerbosity%" == "" SET BuildVerbosity=quiet

IF "%Device%" == "FEZ" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
    SET ImageGenParameters=0x884DED08 0x3671259A 0x08008000 0x00038000
) ELSE (IF "%Device%" == "G30" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
    SET ImageGenParameters=0x69FA3B0D 0xF754B64C 0x08008000 0x00038000 0x78A2A46B 0xA817BB9F 0x47B6A877 0x57C56E3E
) ELSE (IF "%Device%" == "G80" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
    SET ImageGenParameters=0x5FB39ABC 0x14EF5B6A 0x08008000 0x000B8000 0x45023756 0xBCBFA856 0x28A347EB 0xCDACEBAF
) ELSE (IF "%Device%" == "FEZCerberus" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
    SET ImageGenParameters=0x526603B1 0xE16B218D 0x08008000 0x00038000
) ELSE (IF "%Device%" == "Netduino3" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
) ELSE (IF "%Device%" == "Quail" (
    SET ProcessorPart=STM32F4
    SET ProcessorArchitecture=CortexM4
) ELSE (
    ECHO Unsupported device passed: %Device%
    GOTO :EOF
))))))

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

IF "%ProcessorArchitecture%" == "CortexM3" (
    SET InstructionType=THUMB2
    SET MCpu=cortex-m3
    SET FloatCompileArguments=
    SET AdditionalIncludes=-I"%ScriptRoot%\CMSIS\CMSIS\Include"
    SET AdditionalDefines=-DPLATFORM_ARM_CORTEX_M3 -D__CORTEX_M3F -DCORTEX_M3 -DCOMPILE_THUMB2
    SET AdditionalAssemblerArguments=--defsym COMPILE_THUMB2=1 -mthumb
    SET AdditionalCompilerArguments=-mthumb
    SET "GccLibrary=%GccDirectory%\arm-none-eabi\lib\thumb\v7-m"
    SET NeedCmsis=1
) ELSE IF "%ProcessorArchitecture%" == "CortexM4" (
    SET InstructionType=THUMB2FP
    SET MCpu=cortex-m4
    SET FloatCompileArguments=-mfloat-abi=hard -mfpu=fpv4-sp-d16
    SET AdditionalIncludes=-I"%ScriptRoot%\CMSIS\CMSIS\Include"
    SET AdditionalDefines=-DPLATFORM_ARM_CORTEX_M4 -D__CORTEX_M4F -DCORTEX_M4 -DCOMPILE_THUMB2
    SET AdditionalAssemblerArguments=--defsym COMPILE_THUMB2=1 -mthumb
    SET AdditionalCompilerArguments=-mthumb
    SET "GccLibrary=%GccDirectory%\arm-none-eabi\lib\thumb\v7e-m\fpv4-sp\hard"
    SET NeedCmsis=1
) ELSE IF "%ProcessorArchitecture%" == "ARM9" (
    SET InstructionType=ARM
    SET MCpu=arm926ej-s
    SET FloatCompileArguments=-mfloat-abi=soft
    SET AdditionalIncludes=
    SET AdditionalDefines=-DCOMPILE_ARM -DPLATFORM_ARM_ARM9
    SET AdditionalAssemblerArguments=--defsym COMPILE_ARM=1
    SET AdditionalCompilerArguments=-marm -mthumb-interwork
    SET "GccLibrary=%GccDirectory%\arm-none-eabi\lib"
)

IF "%NeedCmsis%" == "1" (
    IF NOT EXIST "%ScriptRoot%\CMSIS\ARM.CMSIS.pdsc" (
        ECHO Cannot find CMSIS. Please make sure it is downloaded and extracted to repo_root\CMSIS.
        GOTO :EOF
    )
)

SET AdditionalIncludes=%AdditionalIncludes% -I"%GccDirectory%\lib\gcc\arm-none-eabi\6.3.1\include"
SET AdditionalIncludes=%AdditionalIncludes% -I"%ScriptRoot%\Targets\%ProcessorPart%"
SET AdditionalIncludes=%AdditionalIncludes% -I"%ScriptRoot%\Devices\%Device%"
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
    SET AdditionalCompilerArguments=-Os %AdditionalCompilerArguments%
)

SET OutputDirectory=%ScriptRoot%\Build\%BuildConfiguration%\%Device%
SET CoreDirectory=%ScriptRoot%\Core
SET CoreLibraryFile=%CoreDirectory%\TinyCLR_%ProcessorArchitecture%.lib
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
        ECHO Cannot find the core library file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR_%ProcessorArchitecture%.lib.
        GOTO :EOF
    )

    IF NOT EXIST "%CoreHeaderFile%" (
        ECHO Cannot find the core header file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR.h.
        GOTO :EOF
    )

    FOR %%A IN ("%ScriptRoot%\Targets\%ProcessorPart%", "%ScriptRoot%\Main") DO (
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

    "%GccDirectory%\bin\arm-none-eabi-g++.exe" -mcpu=%MCpu% -mlittle-endian -nostartfiles %FloatCompileArguments% -Xlinker %AdditionalCompilerArguments% -L "%GccLibrary%" -Wl,-static,--gc-sections,--no-wchar-size-warning,-Map="%OutputDirectory%\tinyclr.map" -specs="%GccLibrary%\nano.specs" -T"%ScriptRoot%\Devices\%Device%\tinyclr_scatterfile_gcc.ldf" "%OutputDirectory%\*.obj" "%CoreLibraryFile%" -o "%OutputDirectory%\tinyclr.axf"

    "%GccDirectory%\bin\arm-none-eabi-objcopy.exe" -S -j ER_FLASH -j ER_RAM_RO -j ER_RAM_RW -O binary "%OutputDirectory%\tinyclr.axf" "%OutputDirectory%\%Device% Firmware.bin"
    "%GccDirectory%\bin\arm-none-eabi-objcopy.exe" -S -R ER_DAT -R ER_CONFIG -O ihex "%OutputDirectory%\tinyclr.axf" "%OutputDirectory%\%Device% Firmware.hex"

    IF NOT "%ImageGenParameters%" == ""  "%ScriptRoot%\imagegen.exe" %ImageGenParameters% "%OutputDirectory%\%Device% Firmware.bin"
)

ECHO Done

POPD

ENDLOCAL
