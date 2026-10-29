#!/bin/sh
# GMSL platform assessment: read-only. Usage: sh assess.sh [sudo password]
PW="$1"
S() { if [ "$(id -u)" = 0 ]; then "$@"; else echo "$PW" | sudo -S -p '' "$@"; fi; }
h() { echo; echo "=== $1"; }
K=/lib/modules/$(uname -r)
DT=/proc/device-tree

h identity
echo "model: $(tr -d '\0' < $DT/model 2>/dev/null)"
echo "compatible: $(tr '\0' ' ' < $DT/compatible 2>/dev/null)"
for f in machine family soc_id revision; do [ -r /sys/devices/soc0/$f ] && echo "soc0.$f: $(cat /sys/devices/soc0/$f)"; done
. /etc/os-release 2>/dev/null; echo "os: $PRETTY_NAME"
echo "kernel: $(uname -r) | $(uname -v) | $(uname -m)"
echo "cmdline: $(cat /proc/cmdline)"

h kernel_config
CFG=""; [ -r /proc/config.gz ] && CFG="zcat /proc/config.gz"; [ -z "$CFG" ] && [ -r /boot/config-$(uname -r) ] && CFG="cat /boot/config-$(uname -r)"
if [ -n "$CFG" ]; then echo "source: $CFG"
  $CFG | grep -E '^(# )?CONFIG_(VIDEO_DEV|MEDIA_CONTROLLER|VIDEO_V4L2_SUBDEV_API|V4L2_FWNODE|I2C_ATR|I2C_MUX|I2C_MUX_GPIO|OF|OF_OVERLAY|OF_CONFIGFS|COMMON_CLK|GPIOLIB|MODULES|MODULE_SIG_FORCE|VIDEO_MAXIM_SERDES|VIDEO_MAX9[0-9]+[A-Z]*|VIDEO_IMX219|VIDEO_OV5640|VIDEO_IMX415)[= ]'
else echo "source: none (no /proc/config.gz or /boot/config-*)"
  [ -n "$(find $K -name 'configs.ko*' 2>/dev/null | head -1)" ] && echo "note: the 'configs' module exists; 'sudo modprobe configs' would provide /proc/config.gz (not loaded by this read-only script)"; fi

h gmsl_drivers
for m in max_serdes max96724 max9296a max96717 max96712 max96714 max9286 max9295 max96705 max96706 ds90ub960 ds90ub953 i2c_atr imx219 nv_imx219 ov5640 imx415 videodev v4l2_fwnode; do
  f=$(find $K -name "$m.ko*" -o -name "$(echo $m | tr _ -).ko*" 2>/dev/null | head -1)
  if [ -n "$f" ]; then echo "$m: module ${f#$K/} depends=[$(modinfo -F depends $f 2>/dev/null)]"
  elif grep -qE "/($m|$(echo $m | tr _ -))\.ko" $K/modules.builtin 2>/dev/null; then echo "$m: built-in"; fi
done
echo "loaded: $(lsmod 2>/dev/null | awk 'NR>1{print $1}' | grep -E 'max|serdes|atr|imx|ov5|ds90|videodev|csi|cfe|unicam|camera|vi5|isp' | tr '\n' ' ')"

h kernel_symbols
# names in /proc/kallsyms are readable without root; module symbols only while the module is loaded
for s in v4l2_subdev_enable_streams v4l2_subdev_set_routing __v4l2_subdev_init_finalize i2c_atr_new i2c_mux_add_adapter media_create_pad_link v4l2_fwnode_endpoint_alloc_parse of_overlay_fdt_apply; do
  if grep -qw "$s" /proc/kallsyms 2>/dev/null; then echo "$s: running kernel"; else
    hit=""; for m in videodev v4l2-fwnode v4l2_fwnode i2c-atr i2c_atr i2c-mux i2c_mux mc; do
      f=$(find $K -name "$m.ko*" 2>/dev/null | head -1); [ -z "$f" ] && continue
      case $f in *.zst) c="zstd -dc";; *.xz) c="xz -dc";; *.gz) c="zcat";; *) c="cat";; esac
      $c "$f" 2>/dev/null | grep -aqw "$s" && hit="module $m (not loaded)" && break
    done; echo "$s: ${hit:-NOT FOUND}"; fi
done

h kernel_source
# what Build mode needs: the commit the kernel was built from, the compiler, symbol versioning
echo "release: $(uname -r)"
case "$(uname -r)" in *-g[0-9a-f]*) echo "git hash: $(uname -r | sed -n 's/.*-g\([0-9a-f]\{7,40\}\).*/\1/p')";; *) echo "git hash: none";; esac
echo "built by: $(cut -d' ' -f4- /proc/version | cut -c1-200)"
if [ -n "$CFG" ]; then $CFG | grep -E '^(CONFIG_CC_VERSION_TEXT|CONFIG_GCC_VERSION|CONFIG_CLANG_VERSION|CONFIG_MODVERSIONS|CONFIG_MODULE_SIG|CONFIG_MODULE_SIG_FORCE|CONFIG_LOCALVERSION|CONFIG_LOCALVERSION_AUTO|CONFIG_CC_HAS_COUNTED_BY|CONFIG_LTO_[A-Z_]*|CONFIG_DEBUG_INFO_BTF)=' ; fi
echo "vendor max9*/max2* module names in kernel/ (an ADI module with the same name must not be built): $(find $K/kernel -name 'max9*.ko*' -o -name 'max2*.ko*' 2>/dev/null | sed 's#.*/##; s/\.ko.*//' | sort -u | tr '\n' ' ')"

h build_capability
[ -f $K/build/Makefile ] && echo "kernel headers: $K/build" || echo "kernel headers: none"
for t in gcc make; do command -v $t >/dev/null && echo "$t: $($t --version 2>/dev/null | head -1)" || echo "$t: none"; done
# a vendor module under kernel/ (out-of-tree modules in updates/ can carry another release string)
f=$(find $K/kernel -name '*.ko*' 2>/dev/null | head -1); [ -n "$f" ] && echo "vermagic: $(modinfo -F vermagic $f 2>/dev/null) (from ${f#$K/})"; echo "module signing enforced: $(cat /sys/module/module/parameters/sig_enforce 2>/dev/null || echo unknown)"

h csi_receivers_and_cameras
# device tree nodes that look like camera receivers (nodes with graph endpoints), sensors, serdes
# chips or I2C muxes; with their status, bound driver and number of endpoints
find $DT/ -type f -name compatible 2>/dev/null | while read c; do
  v=$(tr '\0' ' ' < "$c"); n=$(dirname "$c"); p=${n#$DT}
  eps=$(find $n/ -maxdepth 4 -type d -name 'endpoint*' 2>/dev/null | wc -l)
  case "$v" in
    *sony,*|*ovti,*|*onnn,*|*maxim,max9*|*ti,ds90*|*i2c-mux*) ;;
    *) case "$v $p" in *csi*|*mipi*|*cfe*|*unicam*|*capture*|*isp*|*camera*|*vi5*|*vin*|*cam*)
         # disabled receivers in vendor base trees often have empty ports and no endpoints yet
         [ "$eps" -gt 0 ] || { [ -d $n/ports ] || [ -d $n/port ]; } || continue
         case "$v" in *csi*) ;; *dsi*|*display*|*dpu*|*hdmi*|*lvds*|*ldb*) continue;; esac;;
       *) continue;; esac;;
  esac
  st=okay; [ -r $n/status ] && st=$(tr -d '\0' < $n/status)
  drv=""; rn=$(readlink -f $n)
  for d in /sys/bus/*/devices/*; do
    [ "$(readlink -f $d/of_node 2>/dev/null)" = "$rn" ] && [ -e $d/driver ] && drv=$(basename "$(readlink $d/driver)") && break
  done
  echo "$p | $v| status=$st driver=${drv:-none} endpoints=$eps"
done

h receiver_drivers
# for each receiver node (graph endpoints, not a sensor or serdes chip): the drivers whose device
# tree alias matches its compatible strings or their stem (the compatible without its last
# "-suffix", only if the stem still names a function, i.e. "vendor,chip-function"), whether each
# driver uses the V4L2 streams API (yes / no for modules, unknown for built-in drivers), and which
# one is bound. Driver variants with unrelated compatible strings are not found.
smod() { # $1 = module name -> yes / no / unknown
  f=$(find $K -name "$1.ko*" -o -name "$(echo $1 | tr _ -).ko*" 2>/dev/null | head -1); [ -z "$f" ] && echo unknown && return
  case $f in *.zst) c2="zstd -dc";; *.xz) c2="xz -dc";; *.gz) c2="zcat";; *) c2="cat";; esac
  $c2 "$f" 2>/dev/null | grep -aqE 'v4l2_subdev_set_routing|v4l2_subdev_routing_validate|v4l2_subdev_enable_streams' && echo yes || echo no; }
find $DT/ -type f -name compatible 2>/dev/null | while read c; do
  n=$(dirname "$c"); v=$(tr '\0' ' ' < "$c")
  case "$v" in *sony,*|*ovti,*|*onnn,*|*maxim,*|*ti,ds90*|*i2c-mux*|*fixed-clock*) continue;; esac
  case "$v ${n#$DT}" in *csi*|*mipi*|*cfe*|*unicam*|*capture*|*isp*|*camera*|*vi5*|*vin*|*cam*) ;; *) continue;; esac
  [ "$(find $n/ -maxdepth 4 -type d -name 'endpoint*' 2>/dev/null | wc -l)" -gt 0 ] || { [ -d $n/ports ] || [ -d $n/port ]; } || continue
  case "$v" in *csi*) ;; *dsi*|*display*|*dpu*|*hdmi*|*lvds*|*ldb*) continue;; esac
  for stem in $(tr '\0' '\n' < "$c" | while read cs; do [ -z "$cs" ] && continue; echo "$cs"; s=${cs%-*}
      case "${s#*,}" in *-*) echo "$s";; esac; done | sort -u); do
    # exact compatible matches only (a prefix match would turn "syscon" into "syscon-reboot-mode")
    awk -v a="of:N*T*C$stem" '$1 == "alias" && $2 == a {print $2, $3, "module"}' $K/modules.alias 2>/dev/null
    # built-in drivers are not in modules.alias; their aliases are in modules.builtin.modinfo
    [ -r $K/modules.builtin.modinfo ] && tr '\0' '\n' < $K/modules.builtin.modinfo | awk -v a="of:N*T*C$stem" '{ i = index($0, ".alias="); if (i && substr($0, i + 7) == a) print a, substr($0, 1, i - 1), "builtin" }'
  done | sed 's/^of:N\*T\*C//' | sort -u | while read compat mod kind; do
    if [ "$kind" = builtin ]; then echo "${n#$DT}: $compat -> $mod (built in) streams=unknown"
    else echo "${n#$DT}: $compat -> $mod streams=$(smod $mod)"; fi
  done
  rn=$(readlink -f $n); for x in /sys/bus/platform/devices/*; do
    [ "$(readlink -f $x/of_node 2>/dev/null)" = "$rn" ] && [ -e $x/driver ] || continue
    d=$(basename "$(readlink $x/driver)"); m=$(basename "$(readlink $x/driver/module 2>/dev/null)" 2>/dev/null)
    if [ -n "$m" ]; then echo "${n#$DT}: bound driver $d (module $m) streams=$(smod $m)"
    else echo "${n#$DT}: bound driver $d (built in) streams=unknown"; fi; break
  done
done

h shipped_overlays
# boot partitions are not always mounted at /boot (e.g. /run/media/boot-mmcblk0p1)
BOOTDIRS="/boot /lib/firmware $(findmnt -rn -t vfat -o TARGET 2>/dev/null | tr '\n' ' ')"
echo "searched: $BOOTDIRS"
find $BOOTDIRS -name '*.dtbo' 2>/dev/null | grep -iE 'cam|csi|imx|ov[0-9]|gmsl|max9|serdes|mipi' | head -40
echo "total dtbo files: $(find $BOOTDIRS -name '*.dtbo' 2>/dev/null | wc -l)"
# complete board DTBs: camera variants show the receiver wiring (diff them against the base DTB)
find $BOOTDIRS -maxdepth 2 -name '*.dtb' 2>/dev/null | while read f; do echo "dtb: $f $(stat -c %s "$f")"; done | head -40

h media
for m in /dev/media*; do [ -e "$m" ] || continue; echo "-- $m"; media-ctl -d $m -p 2>/dev/null | grep -E '^(driver|model|bus info)|^- entity' ; done
ls /dev/video* /dev/v4l-subdev* 2>/dev/null | tr '\n' ' '; echo

h i2c
if command -v i2cdetect >/dev/null; then i2cdetect -l; else
  echo "(i2cdetect not installed; bus names from sysfs)"
  for b in /sys/bus/i2c/devices/i2c-*; do echo "$(basename $b)	$(cat $b/name 2>/dev/null)"; done | sort -t- -k2 -n; fi

h i2c_devices
# devices Linux knows on every I2C bus, from sysfs and the device tree only (no bus traffic):
# alias pools must avoid them. Bootloader-only devices (e.g. a USB-PD controller) are not listed.
for d in /sys/bus/i2c/devices/[0-9]*-*; do [ -e "$d" ] || continue
  printf "%s name=%s driver=%s\n" "$(basename $d)" "$(cat $d/name 2>/dev/null)" "$(basename "$(readlink $d/driver)" 2>/dev/null)"
done

h device_tree_and_boot
echo "running fdt: $(test -r /sys/firmware/fdt && echo /sys/firmware/fdt || (S test -r /sys/firmware/fdt && echo '/sys/firmware/fdt (root)') || echo none)"
echo "__symbols__: $(ls $DT/__symbols__ 2>/dev/null | wc -l) labels"
echo "boot mount: $(findmnt -no SOURCE,FSTYPE,OPTIONS /boot 2>/dev/null || echo 'not separate')"
echo "vfat mounts (boot partitions): $(findmnt -rn -t vfat -o SOURCE,TARGET 2>/dev/null | tr '\n' ';' || echo none)"
echo "rootfs: $(findmnt -no SOURCE,FSTYPE,OPTIONS / 2>/dev/null)"
echo "efi: $(test -d /sys/firmware/efi && echo yes || echo no)"
for f in /boot/extlinux/extlinux.conf /boot/firmware/config.txt /boot/config.txt /boot/boot.scr /boot/uEnv.txt /boot/firmware/uEnv.txt /boot/grub/grub.cfg /boot/efi/EFI /boot/loader/entries; do [ -e $f ] && echo "found: $f"; done
ls /boot/*.itb /boot/fitImage* /boot/*.fit 2>/dev/null | sed 's/^/FIT: /'
[ -f /boot/extlinux/extlinux.conf ] && grep -nE '^\s*(DEFAULT|TIMEOUT|MENU|LABEL|FDT|FDTDIR|FDTOVERLAYS|OVERLAYS)' /boot/extlinux/extlinux.conf
grep -sE '^\s*(dtoverlay|dtparam|device_tree|camera_auto_detect)' /boot/firmware/config.txt /boot/config.txt
grep -rsE 'devicetree' /boot/loader/entries/ /boot/grub/grub.cfg 2>/dev/null | head -5
[ -r "$DT/chosen/u-boot,version" ] && echo "u-boot version: $(tr -d '\0' < "$DT/chosen/u-boot,version")"
echo "fw_printenv: $(command -v fw_printenv || echo none) fw_env.config: $(test -r /etc/fw_env.config && head -c 120 /etc/fw_env.config | tr '\n' ';' || echo none)"
command -v fw_printenv >/dev/null && S fw_printenv 2>/dev/null | grep -E '^(fdtfile|fdt_file|fdtdir|fdt_addr|bootcmd|altbootcmd|bootlimit|bootcount|upgrade_available|boot_targets|overlays|fdtoverlay)=' | cut -c1-200
echo "runtime overlays: configfs=$(grep -qw configfs /proc/filesystems && echo yes || echo no) dt-overlays-dir=$(test -d /sys/kernel/config/device-tree/overlays && echo yes || echo no) dtoverlay-cmd=$(command -v dtoverlay >/dev/null && echo yes || echo no)"
echo "A/B or update tools: $(for t in nvbootctrl rauc mender swupdate fw_setenv; do command -v $t >/dev/null && printf '%s ' $t; done)"
df -h /boot 2>/dev/null | tail -1

h userland
echo "media-ctl routes: $(media-ctl --help 2>&1 | grep -q -- '--set-routes' && echo yes || (command -v media-ctl >/dev/null && echo no || echo 'no media-ctl'))"
for t in v4l2-ctl i2cdetect dtc fdtoverlay python3 gst-launch-1.0 qv4l2 apt-get dnf opkg; do printf '%s=%s ' $t "$(command -v $t >/dev/null && echo yes || echo no)"; done; echo
echo "desktop: $(pgrep -x 'Xorg|Xwayland|labwc|wayfire|weston|gnome-shell' >/dev/null && echo yes || echo no)"

h recent_kernel_messages
S dmesg 2>/dev/null | grep -iE 'max9|serdes|atr|csi|mipi|imx219|ov5640|camera|vi5|cfe|unicam' | tail -15 | cut -c1-200
echo; echo "=== end"
