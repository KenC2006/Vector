#!/usr/bin/env bash
# One-shot: convert every .step / .stp under assets/step/ to a co-located
# .binbrep (OCCT native binary B-Rep). Originals are untouched — .binbrep files
# sit alongside as siblings.
#
# Nothing in the app reads .binbrep today (the bake worker that preferred them
# is gone; the runtime loads pre-converted GLBs). Kept for offline CAD work:
# .binbrep parses ~50x faster than STEP and is typically 50-80% smaller.
#
# Re-runs are idempotent: files are skipped when the .binbrep is newer than
# its source. Pass --force to reconvert everything.
#
# Requires the step_to_brep tool from ~/stepreduce-work/step_to_brep/build/.
# Build it from the unify_step / step_to_brep source we keep outside the repo.

set -euo pipefail

TOOL="${HOME}/stepreduce-work/step_to_brep/build/step_to_brep.exe"
DIR="assets/step"
FORCE=0

for arg in "$@"; do
  case "$arg" in
    --force) FORCE=1 ;;
    *) echo "Unknown arg: $arg" >&2; exit 2 ;;
  esac
done

if [[ ! -x "$TOOL" ]]; then
  echo "ERROR: $TOOL not found. Build it from ~/stepreduce-work/step_to_brep/." >&2
  exit 1
fi

if [[ ! -d "$DIR" ]]; then
  echo "ERROR: $DIR not found. Run from the Vector repo root." >&2
  exit 1
fi

# OCCT DLLs live in MSYS2 ucrt64; tool needs them on PATH.
export PATH="/c/msys64/ucrt64/bin:$PATH"

shopt -s nullglob
total=0; converted=0; skipped=0; failed=0
total_step=0; total_brep=0
echo "file,step_bytes,binbrep_bytes,ratio_pct"
for src in "$DIR"/*.step "$DIR"/*.stp; do
  total=$((total+1))
  base="${src%.*}"
  out="${base}.binbrep"

  if [[ $FORCE -eq 0 && -f "$out" && "$out" -nt "$src" ]]; then
    skipped=$((skipped+1))
    continue
  fi

  # step_to_brep writes both text BRep and binary BRep; the text variant is
  # only useful for benchmarking, route it to /tmp and discard.
  scratch="/tmp/step_convert_$$.brep"
  if "$TOOL" "$src" "$scratch" "$out" >/dev/null 2>&1; then
    rm -f "$scratch"
    s=$(stat -c%s "$src")
    b=$(stat -c%s "$out")
    pct=$(awk "BEGIN{printf \"%.1f\", (1-$b/$s)*100}")
    echo "$(basename "$src"),$s,$b,$pct"
    converted=$((converted+1))
    total_step=$((total_step + s))
    total_brep=$((total_brep + b))
  else
    failed=$((failed+1))
    echo "FAILED: $src" >&2
    rm -f "$scratch" "$out"
  fi
done

echo "" >&2
echo "--- summary ---" >&2
echo "files: total=$total converted=$converted skipped=$skipped failed=$failed" >&2
if [[ $converted -gt 0 ]]; then
  pct=$(awk "BEGIN{printf \"%.1f\", (1-$total_brep/$total_step)*100}")
  echo "size:  step=${total_step}B binbrep=${total_brep}B saved=${pct}%" >&2
fi
