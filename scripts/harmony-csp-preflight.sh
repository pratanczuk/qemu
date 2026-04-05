#!/usr/bin/env bash
# Preflight: verify Harmony CSP clone + whether MPLAB-generated makefiles exist.
# See docs/pic32mx-harmony-csp-validation.md

set -euo pipefail
ROOT="${HARMONY_CSP_ROOT:-/data/projects/tmp/harmony_csp}"
UART_X="${ROOT}/csp_apps_pic32mx/apps/uart/uart_echo_blocking/firmware/pic32mx470_curiosity.X"

echo "HARMONY_CSP_ROOT=${ROOT}"
if [[ ! -d "${ROOT}/csp/.git" || ! -d "${ROOT}/csp_apps_pic32mx/.git" ]]; then
  echo "Missing clones. Run:" >&2
  echo "  mkdir -p ${ROOT} && cd ${ROOT}" >&2
  echo "  git clone https://github.com/Microchip-MPLAB-Harmony/csp.git" >&2
  echo "  git clone https://github.com/Microchip-MPLAB-Harmony/csp_apps_pic32mx.git" >&2
  exit 1
fi

if [[ ! -d "$UART_X" ]]; then
  echo "Expected UART sample missing: $UART_X" >&2
  exit 1
fi

VARS="${UART_X}/nbproject/Makefile-variables.mk"
IMPL="${UART_X}/nbproject/Makefile-impl.mk"
if [[ -f "$VARS" && -f "$IMPL" ]]; then
  echo "OK: NetBeans make fragments present — try:" >&2
  echo "  cd ${UART_X} && make -f Makefile CONF=pic32mx470_curiosity build" >&2
  exit 0
fi

echo "NetBeans makefiles not generated yet (typical after fresh git clone):" >&2
echo "  missing: $VARS" >&2
echo "  missing: $IMPL" >&2
echo "Open the .X project in MPLAB X and build once, or export makefiles from the IDE." >&2
exit 2
