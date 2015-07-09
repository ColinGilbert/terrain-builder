cmake_minimum_required(VERSION 3.0)
project(terrain-builder)

set(CMAKE_CXX_FLAGS "-std=c++11 -fno-strict-aliasing -fpermissive -Wall ${CMAKE_CXX_FLAGS}")


include_directories(lib/cppformat)
list (APPEND cppformat_src
	lib/cppformat/format.cc
	lib/cppformat/posix.cc
	)

include_directories(lib/noisepp/core)
include_directories(lib/noisepp/threadpp)
include_directories(lib/noisepp/utils)

list (APPEND noisepp_src
	lib/noisepp/utils/NoiseBuilders.cpp
	lib/noisepp/utils/NoiseGradientRenderer.cpp
	lib/noisepp/utils/NoiseJobQueue.cpp
	lib/noisepp/utils/NoiseOutStream.cpp
	lib/noisepp/utils/NoiseEndianUtils.cpp
	lib/noisepp/utils/NoiseInStream.cpp
	lib/noisepp/utils/NoiseSystem.cpp
	lib/noisepp/utils/NoiseWriter.cpp
	lib/noisepp/utils/NoiseColourValue.cpp
	lib/noisepp/utils/NoiseImage.cpp
	lib/noisepp/utils/NoiseModules.cpp
	lib/noisepp/utils/NoiseReader.cpp
	)

include_directories(lib/polyvox/include)

list(APPEND openmesh_src
	#lib/openmesh/src/OpenMesh/Tools/Decimater/BaseDecimaterT.cc
	lib/openmesh/src/OpenMesh/Tools/Decimater/ModQuadricT.cc
	lib/openmesh/src/OpenMesh/Tools/Decimater/DecimaterT.cc
	lib/openmesh/src/OpenMesh/Tools/Decimater/Observer.cc
	lib/openmesh/src/OpenMesh/Core/Mesh/ArrayKernelT.cc
	)


list(APPEND common_src
	terrain.cpp
	)

add_executable(terrain-builder ${common_src} ${cppformat_src} ${noisepp_src} ${openmesh_src})

target_link_libraries(terrain-builder LINK_PUBLIC rt pthread dl OpenMeshCore OpenMeshTools)

