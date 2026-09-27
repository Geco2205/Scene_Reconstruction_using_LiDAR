#!/usr/bin/env bash
# run_optimizations.sh -- mide el antes y el despues de cada optimizacion en
# UNA maquina y deja los resultados listos para analyze_profile.py.
#
# Uso:
#   tools/run_optimizations.sh --label <maquina> --scans <dir> [opciones]
#
# Opciones:
#   --label <nombre>     Nombre corto de la maquina, p. ej. pc2-keilin, kria (obligatorio)
#   --scans <dir>        Directorio con los .pcd                              (obligatorio)
#   --variants "<lista>" Variantes a correr, separadas por espacio  (todas, ver abajo)
#   --mesh-repeats <n>   Repeticiones de la malla por corrida                        (101)
#   --cmake-args "<...>" Argumentos extra para cmake en todas las builds
#   --skip-build         No recompila; usa las carpetas build-* que ya existan
#   Cualquier otra opcion se pasa tal cual a recon (p. ej. --icp-voxel 1.0).
#
# Variantes (build + modo de ejecucion). Cada una cambia UNA cosa respecto de
# la base, salvo "all", que junta todo:
#   base        todas las opciones OFF, --pipeline off   (= linea base)
#   prefetch    base + --pipeline prefetch (leer/filtrar k+1 mientras se procesa k)
#   full        base + --pipeline full     (ademas integrar k mientras se registra k+1)
#   native      -DRECON_OPT_NATIVE=ON
#   lto         -DRECON_OPT_LTO=ON
#   soa         -DRECON_OPT_SOA_ALIGNED=ON
#   all         native + lto + soa + --pipeline full
#   noneon      -DRECON_OPT_NEON=OFF (solo tiene sentido en ARM: Kria/Jetson)
#
# Salida: results/opt-<label>/<variante>/ (timings.csv, mesh.csv, system.txt,
# run.log) y el analisis en results/opt-<label>/analysis/.

set -euo pipefail

LABEL=""
SCANS=""
MESH_REPEATS=101
CMAKE_EXTRA=""
SKIP_BUILD=0
VARIANTS=""
EXTRA=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --label)        LABEL="$2"; shift 2 ;;
        --scans)        SCANS="$2"; shift 2 ;;
        --variants)     VARIANTS="$2"; shift 2 ;;
        --mesh-repeats) MESH_REPEATS="$2"; shift 2 ;;
        --cmake-args)   CMAKE_EXTRA="$2"; shift 2 ;;
        --skip-build)   SKIP_BUILD=1; shift ;;
        -h|--help)      sed -n '2,30p' "$0"; exit 0 ;;
        *)              EXTRA+=("$1"); shift ;;
    esac
done

if [[ -z "$LABEL" || -z "$SCANS" ]]; then
    echo "Faltan --label y/o --scans. Ver: $0 --help" >&2
    exit 1
fi

if [[ -z "$VARIANTS" ]]; then
    VARIANTS="base prefetch full native lto soa all"
    if [[ "$(uname -m)" =~ ^(aarch64|arm) ]]; then
        VARIANTS="$VARIANTS noneon"
    fi
fi

# variante -> "carpeta_de_build|flags_de_cmake|modo_pipeline"
spec() {
    case "$1" in
        base)     echo "build-base|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=ON|off" ;;
        prefetch) echo "build-base|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=ON|prefetch" ;;
        full)     echo "build-base|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=ON|full" ;;
        native)   echo "build-native|-DRECON_OPT_NATIVE=ON -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=ON|off" ;;
        lto)      echo "build-lto|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=ON -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=ON|off" ;;
        soa)      echo "build-soa|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=ON -DRECON_OPT_NEON=ON|off" ;;
        all)      echo "build-all|-DRECON_OPT_NATIVE=ON -DRECON_OPT_LTO=ON -DRECON_OPT_SOA_ALIGNED=ON -DRECON_OPT_NEON=ON|full" ;;
        noneon)   echo "build-noneon|-DRECON_OPT_NATIVE=OFF -DRECON_OPT_LTO=OFF -DRECON_OPT_SOA_ALIGNED=OFF -DRECON_OPT_NEON=OFF|off" ;;
        *)        echo "Variante desconocida: $1" >&2; exit 1 ;;
    esac
}

OUT_BASE="results/opt-$LABEL"
mkdir -p "$OUT_BASE"

# --- Builds (una por carpeta; base/prefetch/full comparten build-base) -------
if [[ $SKIP_BUILD -eq 0 ]]; then
    declare -A built=()
    for v in $VARIANTS; do
        IFS='|' read -r dir flags _ <<< "$(spec "$v")"
        [[ -n "${built[$dir]:-}" ]] && continue
        echo "== Compilando $dir ($flags)"
        # shellcheck disable=SC2086
        cmake -B "$dir" -DCMAKE_BUILD_TYPE=Release $flags $CMAKE_EXTRA > "$dir.cmake.log" 2>&1 \
            || { echo "Fallo cmake, ver $dir.cmake.log" >&2; exit 1; }
        cmake --build "$dir" -j"$(nproc)" > "$dir.build.log" 2>&1 \
            || { echo "Fallo la compilacion, ver $dir.build.log" >&2; exit 1; }
        built[$dir]=1
    done
fi

# --- Corridas -----------------------------------------------------------------
# Se intercalan en el orden dado; conviene no usar la maquina mientras tanto.
for v in $VARIANTS; do
    IFS='|' read -r dir _ pipe <<< "$(spec "$v")"
    echo
    echo "================ $LABEL / $v  (bin $dir/recon, pipeline $pipe)"
    tools/run_profile.sh --label "$v" --out-dir "$OUT_BASE" --bin "$dir/recon" \
        --scans "$SCANS" --mesh-repeats "$MESH_REPEATS" --pipeline "$pipe" "${EXTRA[@]}" \
        > "$OUT_BASE/$v.console.log" 2>&1 \
        || { echo "Fallo la corrida $v, ver $OUT_BASE/$v.console.log" >&2; exit 1; }
    grep -E "Ciclo de scans|pico" "$OUT_BASE/$v/run.log" || true
done

echo
if python3 -c "import matplotlib" 2>/dev/null; then
    python3 tools/analyze_profile.py "$OUT_BASE"
else
    python3 tools/analyze_profile.py "$OUT_BASE" --no-plots
fi
echo "== Listo. Tabla comparativa en $OUT_BASE/analysis/summary.md"
