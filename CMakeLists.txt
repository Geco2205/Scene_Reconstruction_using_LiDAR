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

# -----------------------------------------------------------------------------
# Optimizaciones (ver docs/OPTIMIZATIONS.md)
#
# Cada una se activa por separado para poder medir el antes y el despues. Con
# todas en su valor por defecto se obtiene la LINEA BASE:
#
#   cmake -B build -DCMAKE_BUILD_TYPE=Release                       # base
#   cmake -B build-opt -DRECON_OPT_NATIVE=ON -DRECON_OPT_LTO=ON \
#         -DRECON_OPT_SOA_ALIGNED=ON                                # optimizada
#
# El paralelismo por tareas no es una opcion de compilacion: se elige al correr
# con --pipeline off|prefetch|full, asi se compara con el mismo binario.
#
# NATIVE y LTO se aplican ANTES de FetchContent para que alcancen tambien a
# KISS-ICP y VDBFusion, que es donde se va ~97 % del tiempo por scan. OpenVDB y
# TBB vienen del sistema ya compilados, asi que a ellos no los alcanza.
# -----------------------------------------------------------------------------
option(RECON_OPT_NATIVE      "-march=native (x86) / -mcpu=native (ARM)"        OFF)
option(RECON_OPT_LTO         "Link-Time Optimization (-flto)"                  OFF)
option(RECON_OPT_SOA_ALIGNED "SoA alineado a 64 B + compactacion sin ramas"    OFF)
option(RECON_OPT_NEON        "Filtro de rango con intrinsics NEON (solo ARM)"  ON)

include(CheckCXXCompilerFlag)
if(RECON_OPT_NATIVE)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    # En ARM GCC no acepta -march=native en todas las versiones; -mcpu=native
    # fija el conjunto de instrucciones y el ajuste al nucleo (Cortex-A53/A57).
    check_cxx_compiler_flag(-mcpu=native HAS_MCPU_NATIVE)
    if(HAS_MCPU_NATIVE)
      add_compile_options(-mcpu=native)
    endif()
  else()
    add_compile_options(-march=native -mtune=native)
  endif()
endif()

if(RECON_OPT_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT HAS_IPO OUTPUT IPO_MSG LANGUAGES CXX)
  if(HAS_IPO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    message(WARNING "LTO no soportado por este compilador: ${IPO_MSG}")
  endif()
endif()

message(STATUS "Optimizaciones: NATIVE=${RECON_OPT_NATIVE} LTO=${RECON_OPT_LTO} "
               "SOA_ALIGNED=${RECON_OPT_SOA_ALIGNED} NEON=${RECON_OPT_NEON}")

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
  src/Io.cpp
  src/Profiler.cpp)

target_include_directories(recon_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(recon_core PUBLIC Eigen3::Eigen)

# Flags de optimizacion.
# -ffast-math NO se usa a proposito: cambia la semantica de NaN/inf y el filtro
# depende de que los valores no finitos ya fueron descartados en la lectura.
target_compile_options(recon_core PRIVATE -O3 -Wall -Wextra)

# Interruptores que cambian el codigo (no solo los flags). Se exponen como macros
# para que el binario imprima con que se compilo (queda en run.log).
target_compile_definitions(recon_core PUBLIC
  RECON_SOA_ALIGNED=$<BOOL:${RECON_OPT_SOA_ALIGNED}>
  RECON_NEON_REQUESTED=$<BOOL:${RECON_OPT_NEON}>
  RECON_OPT_NATIVE_ON=$<BOOL:${RECON_OPT_NATIVE}>
  RECON_OPT_LTO_ON=$<BOOL:${CMAKE_INTERPROCEDURAL_OPTIMIZATION}>)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  message(STATUS "Compilando para ARM: camino NEON ${RECON_OPT_NEON}")
else()
  message(STATUS "Compilando para ${CMAKE_SYSTEM_PROCESSOR}: camino escalar")
endif()

# Hilos para el pipeline por tareas (--pipeline).
find_package(Threads REQUIRED)

add_executable(recon src/main.cpp)
target_link_libraries(recon PRIVATE recon_core kiss_icp_pipeline vdbfusion Threads::Threads)
target_compile_options(recon PRIVATE -O3 -Wall -Wextra)
