set(SRCS
	glslang/GenericCodeGen/CodeGen.cpp
	glslang/GenericCodeGen/Link.cpp
	#glslang/HLSL/hlslAttributes.cpp
	#glslang/HLSL/hlslGrammar.cpp
	#glslang/HLSL/hlslOpMap.cpp
	#glslang/HLSL/hlslParseables.cpp
	#glslang/HLSL/hlslParseHelper.cpp
	#glslang/HLSL/hlslScanContext.cpp
	#glslang/HLSL/hlslTokenStream.cpp
	glslang/MachineIndependent/attribute.cpp
	glslang/MachineIndependent/Constant.cpp
	glslang/MachineIndependent/glslang_tab.cpp
	glslang/MachineIndependent/InfoSink.cpp
	glslang/MachineIndependent/Initialize.cpp
	glslang/MachineIndependent/Intermediate.cpp
	glslang/MachineIndependent/intermOut.cpp
	glslang/MachineIndependent/IntermTraverse.cpp
	glslang/MachineIndependent/iomapper.cpp
	glslang/MachineIndependent/limits.cpp
	glslang/MachineIndependent/linkValidate.cpp
	glslang/MachineIndependent/parseConst.cpp
	glslang/MachineIndependent/ParseContextBase.cpp
	glslang/MachineIndependent/ParseHelper.cpp
	glslang/MachineIndependent/PoolAlloc.cpp
	glslang/MachineIndependent/preprocessor/Pp.cpp
	glslang/MachineIndependent/preprocessor/PpAtom.cpp
	glslang/MachineIndependent/preprocessor/PpContext.cpp
	glslang/MachineIndependent/preprocessor/PpScanner.cpp
	glslang/MachineIndependent/preprocessor/PpTokens.cpp
	glslang/MachineIndependent/propagateNoContraction.cpp
	glslang/MachineIndependent/reflection.cpp
	glslang/MachineIndependent/RemoveTree.cpp
	glslang/MachineIndependent/Scan.cpp
	glslang/MachineIndependent/ShaderLang.cpp
	glslang/MachineIndependent/SpirvIntrinsics.cpp
	glslang/MachineIndependent/SymbolTable.cpp
	glslang/MachineIndependent/Versions.cpp
	glslang/ResourceLimits/ResourceLimits.cpp
	#OGLCompilersDLL/InitializeDll.cpp
	SPIRV/disassemble.cpp
	SPIRV/doc.cpp
	SPIRV/GlslangToSpv.cpp
	SPIRV/InReadableOrder.cpp
	SPIRV/Logger.cpp
	SPIRV/SpvBuilder.cpp
	SPIRV/SpvPostProcess.cpp
	SPIRV/SPVRemapper.cpp
	SPIRV/SpvTools.cpp
)

if(WIN32)
	list(APPEND SRCS 	glslang/OSDependent/Windows/ossource.cpp)
else()
	list(APPEND SRCS 	glslang/OSDependent/Unix/ossource.cpp)
endif()

if(NOT MSVC)
# glslang requires C++17 at a minimum to compile.
add_compile_options(-std=c++17)

# Root directory for build-time generated include files
set(GLSLANG_GENERATED_INCLUDEDIR "${CMAKE_BINARY_DIR}/include")

################################################################################
# Build version information generation
################################################################################
include(parse_version.cmake)
set(GLSLANG_CHANGES_FILE      "${CMAKE_CURRENT_SOURCE_DIR}/CHANGES.md")
set(GLSLANG_BUILD_INFO_H_TMPL "${CMAKE_CURRENT_SOURCE_DIR}/build_info.h.tmpl")
set(GLSLANG_BUILD_INFO_H      "${GLSLANG_GENERATED_INCLUDEDIR}/glslang/build_info.h")

parse_version(${GLSLANG_CHANGES_FILE} GLSLANG)

function(configurate_version)
    set(major ${GLSLANG_VERSION_MAJOR})
    set(minor ${GLSLANG_VERSION_MINOR})
    set(patch ${GLSLANG_VERSION_PATCH})
    set(flavor ${GLSLANG_VERSION_FLAVOR})
    configure_file(${GLSLANG_BUILD_INFO_H_TMPL} ${GLSLANG_BUILD_INFO_H} @ONLY)
endfunction()

configurate_version()

# Silence some warnings that occur frequently to reduce noise in build logs.
add_compile_options(-Wno-shadow)
add_compile_options(-Wno-reorder)
add_compile_options(-Wno-sign-compare)
add_compile_options(-Wno-parentheses)
add_compile_options(-Wno-unused-variable)
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	add_compile_options(-Wno-unused-but-set-variable)
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(-Wno-missing-variable-declarations)
endif()
endif()

add_library(glslang STATIC ${SRCS})
dolphin_disable_warnings(glslang)

target_include_directories(glslang
PRIVATE
	${CMAKE_SOURCE_DIR}/Externals/glslang
	${GLSLANG_GENERATED_INCLUDEDIR}
)
