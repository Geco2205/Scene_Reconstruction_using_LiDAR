#!/usr/bin/env bash
# run_profile.sh -- corre el pipeline con perfilado y deja los resultados en
# <out-dir>/<label>/, listos para comparar maquinas.
#
# Uso:
#   tools/run_profile.sh --label <nombre-maquina> --scans <dir> [opciones]
#
# Opciones:
#   --label <nombre>      Nombre corto de la maquina, p. ej. pc1, kria (obligatorio)
#   --scans <dir>         Directorio con los .pcd                   (obligatorio)
#   --mesh-repeats <n>    Repeticiones de la extraccion de malla           (101)
#   --out-dir <dir>       Carpeta base de resultados                       (results)
#   --bin <ruta>          Ejecutable                                       (build/recon)
#   Cualquier otra opcion se pasa tal cual a recon (p. ej. --icp-voxel 1.0).
#
# Salida en <out-dir>/<label>/:
#   timings.csv   una fila por scan
#   mesh.csv      una fila por repeticion de la extraccion de malla
#   system.txt    CPU, memoria, SO, compilador, commit y flags de la build
#   run.log       salida completa del programa (+ /usr/bin/time -v si existe)

set -euo pipefail

LABEL=""
SCANS=""
# El proyecto exige "mas de 100 muestras por etapa". La extraccion de malla
# ocurre una sola vez por secuencia, asi que se repite sobre el volumen final.
# 101 es el menor valor que cumple; con 100 el papel no se sostiene.
MESH_REPEATS=101
OUT_BASE="results"
BIN="build/recon"
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --label)        LABEL="$2"; shift 2 ;;
        --scans)        SCANS="$2"; shift 2 ;;
        --mesh-repeats) MESH_REPEATS="$2"; shift 2 ;;
        --out-dir)      OUT_BASE="$2"; shift 2 ;;
        --bin)          BIN="$2"; shift 2 ;;
        -h|--help)      sed -n '2,22p' "$0"; exit 0 ;;
        *)              EXTRA+=("$1"); shift ;;
    esac
done

if [[ -z "$LABEL" || -z "$SCANS" ]]; then
    echo "Faltan --label y/o --scans. Ver: $0 --help" >&2
    exit 1
fi
if [[ ! -x "$BIN" ]]; then
    echo "No existe $BIN. Compilen primero (ver README, seccion Build)." >&2
    exit 1
fi

OUT="$OUT_BASE/$LABEL"
mkdir -p "$OUT"

# --- Informacion del sistema ------------------------------------------------
# Todo lo que puede explicar una diferencia de tiempos entre maquinas.
{
    echo "label:        $LABEL"
    echo "date:         $(date -Iseconds)"
    echo "hostname:     $(hostname)"
    echo "kernel:       $(uname -srm)"
    if [[ -r /etc/os-release ]]; then
        echo "os:           $(. /etc/os-release && echo "$PRETTY_NAME")"
    fi
    echo "cpu_model:    $(lscpu | sed -n 's/^Model name:[[:space:]]*//p' | head -1)"
    echo "cpu_arch:     $(lscpu | sed -n 's/^Architecture:[[:space:]]*//p')"
    echo "cpu_count:    $(nproc)"
    echo "cpu_max_mhz:  $(lscpu | sed -n 's/^CPU max MHz:[[:space:]]*//p')"
    gov=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    echo "cpu_governor: $( [[ -r $gov ]] && cat $gov || echo n/a )"
    echo "mem_total:    $(awk '/MemTotal/ {printf "%.1f GiB", $2/1048576}' /proc/meminfo)"
    echo "compiler:     $(${CXX:-c++} --version | head -1)"
    echo "cmake:        $(cmake --version | head -1)"
    echo "git_commit:   $(git rev-parse --short HEAD 2>/dev/null || echo n/a)"
    echo "git_dirty:    $( [[ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ]] && echo yes || echo no )"
    # El cache se busca junto al ejecutable, para que --bin build-opt/recon
    # reporte los flags de esa build y no los de build/.
    cache="$(dirname "$BIN")/CMakeCache.txt"
    if [[ -r $cache ]]; then
        echo "build_type:   $(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' $cache)"
        # Flags exactos de la build. Sin esto no se puede distinguir una corrida
        # BASE de una OPTIMIZADA (-march=native, LTO, etc.) al comparar maquinas.
        echo "cxx_flags:    $(sed -n 's/^CMAKE_CXX_FLAGS:STRING=//p' $cache)"
        echo "cxx_flags_rel: $(sed -n 's/^CMAKE_CXX_FLAGS_RELEASE:STRING=//p' $cache)"
        echo "cxx_compiler: $(sed -n 's/^CMAKE_CXX_COMPILER:[A-Z]*=//p' $cache)"
        # Interruptores de docs/OPTIMIZATIONS.md (todos OFF salvo NEON = base).
        for opt in RECON_OPT_NATIVE RECON_OPT_LTO RECON_OPT_SOA_ALIGNED RECON_OPT_NEON; do
            printf '%-13s %s\n' "${opt#RECON_}:" "$(sed -n "s/^$opt:BOOL=//p" $cache)"
        done
    fi
    echo "scans_dir:    $SCANS"
    echo "mesh_repeats: $MESH_REPEATS"
    echo "extra_args:   ${EXTRA[*]:-}"
    # Modo de paralelismo por tareas (off si no se paso --pipeline).
    pipe=off
    for ((k = 0; k < ${#EXTRA[@]}; k++)); do
        [[ "${EXTRA[$k]}" == "--pipeline" ]] && pipe="${EXTRA[$((k + 1))]:-off}"
    done
    echo "pipeline:     $pipe"
    echo "load_avg:     $(cut -d' ' -f1-3 /proc/loadavg)"
} > "$OUT/system.txt"

echo "== Perfilado en '$LABEL' -> $OUT"
cat "$OUT/system.txt"
echo

# --- Corrida -----------------------------------------------------------------
CMD=("$BIN" --scans "$SCANS" --out "$OUT/mesh.ply"
     --timing-csv "$OUT/timings.csv" --mesh-csv "$OUT/mesh.csv"
     --mesh-repeats "$MESH_REPEATS" "${EXTRA[@]}")

if [[ -x /usr/bin/time ]]; then
    /usr/bin/time -v "${CMD[@]}" 2>&1 | tee "$OUT/run.log"
else
    "${CMD[@]}" 2>&1 | tee "$OUT/run.log"
fi

# mesh.ply se deja para compararla contra otras corridas; .gitignore la excluye.
echo
echo "== Listo. Resultados en $OUT"
echo "   Analisis: python3 tools/analyze_profile.py $OUT_BASE"
