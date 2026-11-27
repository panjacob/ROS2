#!/usr/bin/env bash
# Make sure the LifeCam VX-6000 driver (gspca_sn9c20x, USB 045e:00f4) is loaded in WSL.
# Microsoft's WSL kernel lacks it and WSL resets /lib/modules when it restarts, so the module is
# built once per kernel release into ~/.cache/arm_camera/<release>/ and loaded from there (insmod)
# whenever it is missing. Root steps use `sudo -n` or, without a password, Windows' `wsl.exe -u root`.
#   camera_driver.sh             load it (builds first if this kernel has no cached module)
#   camera_driver.sh --rebuild   rebuild, then load
#   WSL_KERNEL_SRC=...           kernel source tree (default ~/wsl-kernel)
set -uo pipefail
MOD=gspca_sn9c20x
RELEASE=$(uname -r)
CACHE=${XDG_CACHE_HOME:-$HOME/.cache}/arm_camera/$RELEASE
KO=$CACHE/$MOD.ko
SRC=${WSL_KERNEL_SRC:-$HOME/wsl-kernel}
MOD_DIR=drivers/media/usb/gspca
WSL_EXE=/mnt/c/Windows/System32/wsl.exe

log() { echo "camera_driver: $*"; }

as_root() {
  if [ "$(id -u)" = 0 ]; then
    "$@"
  elif sudo -n true 2>/dev/null; then
    sudo "$@"
  elif [ -n "${WSL_DISTRO_NAME:-}" ] && [ -x "$WSL_EXE" ]; then
    "$WSL_EXE" -d "$WSL_DISTRO_NAME" -u root -- "$@" </dev/null
  else
    sudo "$@"
  fi
}

vermagic() { /usr/sbin/modinfo -F vermagic "$1" 2>/dev/null | cut -d' ' -f1; }

build() {
  local built="$SRC/$MOD_DIR/$MOD.ko"
  if [ "${1:-}" != --rebuild ] && [ -f "$built" ] && [ "$(vermagic "$built")" = "$RELEASE" ]; then
    install -D -m644 "$built" "$KO" && return 0
  fi
  log "building $MOD for kernel $RELEASE (one-time per WSL kernel update, takes a few minutes)..."
  if ! command -v flex >/dev/null || ! command -v bison >/dev/null || ! command -v pahole >/dev/null; then
    as_root apt-get install -y flex bison dwarves libelf-dev libssl-dev bc build-essential || return 1
  fi
  local tag="linux-msft-wsl-${RELEASE%%-*}"
  if [ ! -d "$SRC/.git" ] || [ "$(git -C "$SRC" describe --tags 2>/dev/null)" != "$tag" ]; then
    rm -rf "$SRC"
    git -c core.autocrlf=false clone -q --depth 1 --branch "$tag" \
      https://github.com/microsoft/WSL2-Linux-Kernel "$SRC" || return 1
  fi
  (
    set -e
    cd "$SRC"
    zcat /proc/config.gz > .config
    scripts/config --module CONFIG_USB_GSPCA_SN9C20X
    make -s olddefconfig
    # Empty LOCALVERSION keeps vermagic identical to the running kernel (no trailing '+').
    make -s -j"$(nproc)" LOCALVERSION= modules_prepare
    [ "$(make -s LOCALVERSION= kernelrelease)" = "$RELEASE" ] || { log "kernel release mismatch"; exit 1; }
    make -s -j"$(nproc)" LOCALVERSION= KBUILD_MODPOST_WARN=1 M="$MOD_DIR" modules 2>&1 \
      | grep -v 'WARNING: modpost' || true
  ) || return 1
  [ "$(vermagic "$built")" = "$RELEASE" ] && install -D -m644 "$built" "$KO"
}

if [ "${1:-}" = --rebuild ]; then
  build --rebuild || { log "build failed"; exit 1; }
fi
[ -d /sys/module/$MOD ] && exit 0
# Native Linux kernels ship the driver; udev loads it on plug-in.
grep -qi microsoft /proc/version || exit 0

if [ ! -f "$KO" ] || [ "$(vermagic "$KO")" != "$RELEASE" ]; then
  build || { log "build failed (rerun: $0 --rebuild)"; exit 1; }
fi
as_root /usr/sbin/modprobe gspca_main || { log "modprobe gspca_main failed"; exit 1; }
as_root /usr/sbin/insmod "$KO"
if [ -d /sys/module/$MOD ]; then
  log "$MOD loaded for $RELEASE"
else
  log "loading $KO failed"; exit 1
fi
