#!/bin/bash
# In WSL, don't use default gcc compiler
# Instead, install gcc-arm-none-eabi version greater than 6
# Instructions are here: https://gnu-mcu-eclipse.github.io/toolchain/arm/install/
#
# Note that default installation for compiler is ~/opt/gcc-arm-none-eabi-6-2017-q2-update

ScriptRoot=$(pwd)
ScriptRoot=${ScriptRoot%/}

pushd "$ScriptRoot" > /dev/null

DeviceName=$1
BuildTarget=$2
BuildConfiguration=$3
BuildVerbosity=$4
GccDirectory=$5

if [ -z "$2" ]; then BuildTarget=build; fi
if [ -z "$3" ]; then BuildConfiguration=release; fi
if [ -z "$4" ]; then BuildVerbosity=quiet; fi

if [ ! -d "$ScriptRoot/Devices/$DeviceName" ] || [ -z $DeviceName ]; then 
	echo "Unsupported device passed: $DeviceName"; exit 1;
fi

if [ ! $BuildTarget = "build" ] && [ ! $BuildTarget = "cleanbuild" ] && [ ! $BuildTarget = "clean" ]; then
	echo "Unsupported target passed: $BuildTarget"; popd; exit 1;
fi

if [ ! $BuildConfiguration = "debug" ] && [ ! $BuildConfiguration = "release" ]; then
	echo "Unsupported configuration passed: $BuildConfiguration"; popd; exit 1;
fi

if [ ! -d "$GccDirectory" ]; then GccDirectory=~/opt/gcc-arm-none-eabi-6-2017-q2-update; fi
if [ ! -d "$GccDirectory" ]; then
	echo "Cannot find GCC. Try passing the directory explicitly like `build.bat device build release normal "/usr/bin"`"; popd; exit 1;
fi

GccDirectory=${GccDirectory%/}

DeviceBuildConfiguration=$ScriptRoot/Devices/$DeviceName/BuildConfiguration.txt
if [ ! -e "$DeviceBuildConfiguration" ]; then
	echo "Cannot find device configuration at '%DeviceBuildConfiguration%.'"; popd; exit 1;
fi

while IFS=: read -r var value  || [ -n "$var" ]
do
	tmp=${value//\\//}
	tmp=${tmp//!ScriptRoot!/$ScriptRoot}
	tmp=${tmp//!GccDirectory!/$GccDirectory}
	eval "$var=\${tmp//[$'\n\r']}"
done < "$DeviceBuildConfiguration"

if [ -z $TargetName ]; then
	echo "TargetName not defined in '%DeviceBuildConfiguration%'."; popd; exit 1;
fi
TargetBuildConfiguration=$ScriptRoot/Targets/$TargetName/BuildConfiguration.txt
if [ ! -e "$TargetBuildConfiguration" ]; then
	echo "Cannot find target configuration at '$TargetBuildConfiguration.'"; popd; exit 1;
fi

while IFS=: read -r var value || [ -n "$var" ]
do
	tmp=${value//\\//}
	tmp=${tmp//!ScriptRoot!/$ScriptRoot}
	tmp=${tmp//!GccDirectory!/$GccDirectory}
	eval "$var=\${tmp//[$'\n\r']}"
done < "$TargetBuildConfiguration"

if [ -z $TargetArchitecture ]; then
	echo "TargetArchitecture not defined in '$TargetBuildConfiguration'."; popd; exit 1;
fi
LibraryBuildConfiguration=$ScriptRoot/Core/$TargetArchitecture'BuildConfiguration.txt'

if [ ! -e "$LibraryBuildConfiguration" ]; then
	echo "Cannot find library configuration at '$LibraryBuildConfiguration.'"; popd; exit 1;
fi

while IFS=: read -r var value || [ -n "$var" ]
do
	tmp=${value//\\//}
	tmp=${tmp//!ScriptRoot!/$ScriptRoot}
	tmp=${tmp//!GccDirectory!/$GccDirectory}
	eval "$var=\${tmp//[$'\n\r']}"
done < "$LibraryBuildConfiguration"

if [ $NeedCmsis = "1" ]; then
	if [ ! -e "$ScriptRoot/CMSIS/ARM.CMSIS.pdsc" ]; then
		echo Cannot find CMSIS. Please make sure it is downloaded and extracted to repo_root\CMSIS.; popd; exit 1;
	fi
fi

AdditionalIncludes=-I$ScriptRoot/CMSIS/CMSIS/Include
AdditionalIncludes="$AdditionalIncludes -I/usr/lib/gcc/arm-none-eabi/4.9.3/include"
AdditionalIncludes="$AdditionalIncludes -I$ScriptRoot/Targets/$TargetName"
AdditionalIncludes="$AdditionalIncludes -I$ScriptRoot/Devices/$DeviceName"
AdditionalIncludes="$AdditionalIncludes -I$ScriptRoot/Core"

AdditionalDefines="$AdditionalDefines -DGCC"
AdditionalCompilerArguments="-mstructure-size-boundary=8 -fno-exceptions -ffunction-sections -fdata-sections -fshort-wchar -funsigned-char -mlong-calls"

if [ $BuildConfiguration = "debug" ]; then
	AdditionalDefines="$AdditionalDefines -DDEBUG -D_DEBUG"
	AdditionalAssemblerArguments="$AdditionalAssemblerArguments -g"
	AdditionalCompilerArguments="-O0 -femit-class-debug-always -g3 -ggdb $AdditionalCompilerArguments"
else
	AdditionalDefines="$AdditionalDefines"
	AdditionalAssemblerArguments="$AdditionalAssemblerArguments"
	AdditionalCompilerArguments="$OptimizeLevel $AdditionalCompilerArguments"
fi

OutputDirectory="$ScriptRoot/Build/$BuildConfiguration/$DeviceName"
CoreDirectory="$ScriptRoot/Core"
CoreLibraryFile="$CoreDirectory/TinyCLR_$TargetArchitecture.lib"
CoreHeaderFile="$CoreDirectory/TinyCLR.h"

if [ $BuildTarget = "cleanbuild" ]; then
	DoClean="1"
	DoBuild="1"
elif [ $BuildTarget = "build" ]; then
	DoClean="0"
	DoBuild="1"
elif [ $BuildTarget = "clean" ]; then
	DoClean="1"
	DoBuild="0"
fi

if [ $DoClean = "1" ]; then
	if [ -d "$OutputDirectory" ]; then
		rm -R "$OutputDirectory"
	fi
fi

if [ $DoBuild = "1" ]; then
	if [ ! -d "$OutputDirectory" ]; then mkdir -p "$OutputDirectory"; fi
	if [ ! -e "$CoreLibraryFile" ]; then
		echo "Cannot find the core library file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR_$TargetArchitecture.lib."; popd;
		exit 1;
	fi

	if [ ! -e "$CoreHeaderFile" ]; then
		echo "Cannot find the core library file. Please make sure it is downloaded and extracted to repo_root\Core\TinyCLR.h."; popd;
		exit 1;
	fi
	CompilePaths=("$ScriptRoot/Targets/$TargetName" "$ScriptRoot/Devices/$DeviceName" "$ScriptRoot/Main")

	if [ ! -z $AdditionalDrivers ]; then
		for a in "${AdditionalDrivers[@]}"; do
			CompilePaths+=("$ScriptRoot/Drivers/$a")
		done
	fi
	for i in "${CompilePaths[@]}"; do
		pushd "$i" > /dev/null
		for f in *.gcc.s
		do
			if [ -e $f ]; then
			echo "${f##*/}"
			"$GccDirectory/bin/arm-none-eabi-as" ${AdditionalAssemblerArguments//[$'\n\r']} -mcpu=$MCpu -mlittle-endian $FloatCompileArguments -o "$OutputDirectory/${f##*/}.obj" "$f"
			fi
		done
		for f in *.cpp; do
			if [ -e $f ]; then
			echo "${f##*/}"
			"$GccDirectory/bin/arm-none-eabi-g++" -std=c++11 -xc++ $AdditionalCompilerArguments -mcpu=$MCpu -mlittle-endian $FloatCompileArguments $AdditionalDefines $AdditionalIncludes -o "$OutputDirectory/${f##*/}.obj" -c "$f"
			fi
		done
		popd > /dev/null 
	done
	"$GccDirectory/bin/arm-none-eabi-g++" -mcpu=$MCpu -mlittle-endian -nostartfiles $FloatCompileArguments -Xlinker $AdditionalCompilerArguments -L "$GccLibrary" -Wl,-static,--gc-sections,--no-wchar-size-warning,-Map="$OutputDirectory/$DeviceName Firmware.map" -specs="$GccLibrary/nano.specs" -T"$ScriptRoot/Devices/$DeviceName/Scatterfile.gcc.ldf" "$OutputDirectory/"*.obj "$CoreLibraryFile" -o "$OutputDirectory/$DeviceName Firmware.axf"
    "$GccDirectory/bin/arm-none-eabi-objcopy" -S -j ER_FLASH -j ER_RAM_RO -j ER_RAM_RW -O binary "$OutputDirectory/$DeviceName Firmware.axf" "$OutputDirectory/$DeviceName Firmware.bin"
    "$GccDirectory/bin/arm-none-eabi-objcopy" -S -R ER_DAT -R ER_CONFIG -O ihex "$OutputDirectory/$DeviceName Firmware.axf" "$OutputDirectory/$DeviceName Firmware.hex"
	
	# TODO : include tools to make .ghi or .glb files
	if [ ! -z $ImageGenParameters ]; then $ScriptRoot/imagen; fi
fi

echo "Done"
popd > /dev/null 