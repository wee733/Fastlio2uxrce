#!/usr/bin/env bash
# Human-readable milestones; preserve command output and exit status.
INSTALL_STAGE='Preparing installation'
install_log_init() {
  export GCC_URLS=no GCC_COLORS='' NO_COLOR=1 CLICOLOR=0 COLCON_COLOR_OUTPUT=0
  INSTALL_APP="$1"
  echo "[Install] Preparing $INSTALL_APP"
  trap 'install_log_on_exit $?' EXIT
}
install_step() {
  INSTALL_STAGE="$1"
  echo "[Stage] $INSTALL_STAGE"
}
install_log_on_exit() {
  local code="$1"
  if (( code != 0 )); then
    printf '[Error] %s: %s (exit code %s). Check the output above.\n' "$INSTALL_APP" "$INSTALL_STAGE" "$code" >&2
  fi
}
install_done() {
  echo "[Done] $INSTALL_APP preparation complete; AivudaOS will continue the installation."
}
