#!/usr/bin/env bash
# Generate local API pages; prefer a system Doxygen, then this host's local copy.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p tmp
if command -v doxygen >/dev/null 2>&1; then
  exec doxygen Doxyfile
fi
local_tools="$PWD/tmp/doxygen_local/root"
if [[ -x "$local_tools/usr/bin/doxygen" ]]; then
  export LD_LIBRARY_PATH="$local_tools/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  exec "$local_tools/usr/bin/doxygen" Doxyfile
fi
printf '%s\n' 'Doxygen is unavailable. Install doxygen, then rerun this script.' >&2
exit 1
