cmake_minimum_required(VERSION 3.16)
project(recon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# -----------------------------------------------------------------------------
# Versiones fijadas
#
# IMPORTANTE: el API de KISS-ICP ha cambiado entre versiones. Hubo un refactor
# que elimino la trayectoria completa y dejo una sola pose (poses() -> pose()),
# y otro que cambio los accesores a referencias. Fijar el tag es lo que hace
# reproducible la build y lo que exige el enunciado del curso.
#
# Antes de la primera compilacion: abran cpp/kiss_icp/pipeline/KissICP.hpp de la
# version que quedo en _deps/ y verifiquen la firma de RegisterFrame y los
# campos de KISSConfig. Si no coinciden con lo que usa src/main.cpp, ajusten
# main.cpp (no la biblioteca) y anoten la version en el paper.
# -----------------------------------------------------------------------------
set(KISS_ICP_TAG "v1.0.0"   CACHE STRING "Tag de PRBonn/kiss-icp")
set(VDBFUSION_TAG "v0.1.6"  CACHE STRING "Tag de PRBonn/vdbfusion")

include(FetchContent)

FetchContent_Declare(kiss_icp
  GIT_REPOSITORY https://github.com/PRBonn/kiss-icp.git
  GIT_TAG        ${KISS_ICP_TAG}
  SOURCE_SUBDIR  cpp/kiss_icp)
FetchContent_MakeAvailable(kiss_icp)

# OpenVDB del sistema (Ubuntu/Kria: primera ruta; Fedora: segunda)
list(APPEND CMAKE_MODULE_PATH
  "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/OpenVDB"
  "/usr/lib64/cmake/OpenVDB"
  "/usr/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

FetchContent_Declare(vdbfusion
  GIT_REPOSITORY https://github.com/PRBonn/vdbfusion.git
  GIT_TAG        ${VDBFUSION_TAG}
  SOURCE_SUBDIR  src/vdbfusion/vdbfusion)
FetchContent_MakeAvailable(vdbfusion)

find_package(Eigen3 REQUIRED)

# -----------------------------------------------------------------------------
# Codigo propio
# -----------------------------------------------------------------------------
add_library(recon_core
  src/PcdReader.cpp
  src/RangeFilter.cpp
  src/Io.cpp)

target_include_directories(recon_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(recon_core PUBLIC Eigen3::Eigen)

# Flags de optimizacion.
# -ffast-math NO se usa a proposito: cambia la semantica de NaN/inf y el filtro
# depende de que los valores no finitos ya fueron descartados en la lectura.
target_compile_options(recon_core PRIVATE -O3 -Wall -Wextra)

# En ARM, -mcpu=native activa NEON y ajusta al nucleo concreto (Cortex-A57 en la
# Jetson Nano, Cortex-A53 en la Kria). En AArch64 NEON ya viene por defecto.
include(CheckCXXCompilerFlag)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  check_cxx_compiler_flag(-mcpu=native HAS_MCPU_NATIVE)
  if(HAS_MCPU_NATIVE)
    target_compile_options(recon_core PRIVATE -mcpu=native)
  endif()
  message(STATUS "Compilando para ARM: camino NEON activo")
else()
  message(STATUS "Compilando para ${CMAKE_SYSTEM_PROCESSOR}: camino escalar")
endif()

add_executable(recon src/main.cpp)
target_link_libraries(recon PRIVATE recon_core kiss_icp_pipeline vdbfusion)
target_compile_options(recon PRIVATE -O3 -Wall -Wextra)
