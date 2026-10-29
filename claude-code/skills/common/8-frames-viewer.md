# Common 8: frame check and viewer

Checks that frames arrive on every configured video device, then shows the video on the board's
display. Two viewer procedures: §qv4l2 and §GStreamer. **The platform file chooses the order:**
Raspberry Pi tries §qv4l2 first and falls back to §GStreamer (rpi/4 §Viewer); NXP and generic
boards use §GStreamer (nxp/5 §Viewer, generic/1). The video appears only on the board's display
(`DISPLAY=:0` or the Wayland session), never on the host.

Inputs from common/7 (or the platform's own pipeline script): for every camera `<dev_path>` and
its format (width, height, fourcc).

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the frame-check result per video device, the viewer command and its log into `<run_dir>/board/` (§Verbose mode §What to save).

## Frame check

Before launching the viewer, capture a few frames on each configured video device. This works
without a monitor, and tells a pipeline problem apart from a display problem:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "v4l2-ctl -d <dev_path> --get-fmt-video | grep -E 'Width|Pixel Format|Size Image'; timeout 20 v4l2-ctl -d <dev_path> --stream-mmap=4 --stream-count=30 --stream-to=/tmp/gmsl_test.raw; ls -l /tmp/gmsl_test.raw; rm -f /tmp/gmsl_test.raw"
```
v4l2-ctl prints one `<` per frame. **The file size decides**: 30 frames of `Size Image` bytes
(from `--get-fmt-video`) is a pass. Examples: 640x480 `SRGGB10P` (packed RAW10) is
640 * 480 * 10 / 8 = 384000 bytes per frame; 8-bit formats such as `SRGGB8` are `w * h` bytes
per frame; `UYVY` is `w * h * 2`. Receivers can pad lines, so trust `Size Image` over the formula.

- **Dropped buffers** reported by `v4l2-ctl` are normal while the capture starts or when the writer
  is slower than the camera; report the count, do not fail on it.
- Run the capture once more if the first one gets no frames.
- **After a stream start that failed with an error** (`VIDIOC_STREAMON returned -1`), the drivers
  may keep part of the chain enabled: reboot the board (common/5 §Reboot) before the next attempt
  (and re-run the pipeline configuration, e.g. `~/Workspace/media_cfg.sh`), and read the boot's
  kernel log afterwards (`dmesg`, or `journalctl -k -b` if `dmesg` was cleared), since a probe
  failure can then appear.
- **No frames** (the command times out without `<`): check the deserializer and serializer
  `--log-status` packet counters (`v4l2-ctl -d /dev/v4l-subdev<N> --log-status`, then `dmesg |
  tail -40`; see where the counters stop; `video_lock: 0` is normal in tunnel mode), then try
  the other deserializer PHY (common/4 §JSON) if the adapter is unknown.
- With more than one camera: the multi-camera check of common/7 §Check.

Run this check before the viewer starts, because the viewer keeps the video devices open
(`EBUSY`). The file goes to the board's `/tmp` and is deleted by the same command.

## Desktop session

The viewer needs a desktop session on the board. Check that one is running:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "pgrep -x 'Xorg|Xwayland|labwc|wayfire|weston|gnome-shell' >/dev/null && echo desktop || echo none"
```
If it prints `none` (for example a Lite/server image, or no monitor connected at boot), do not
launch a viewer: tell the user that the video can only be shown on a desktop session on the
board, and that the frame check shows whether the video pipeline works.

Find out which display servers run (for §GStreamer's sink):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "ls /run/user/\$(id -u)/wayland-* 2>/dev/null; pgrep -a -x 'Xorg|Xwayland' | head -3"
```
A `wayland-0` socket means a Wayland session; `Xorg` without a Wayland socket means X11 only. The
Wayland socket belongs to the user that runs the desktop: if it is not in the login's
`/run/user/<uid>` (e.g. Weston runs as another user), use that user's `XDG_RUNTIME_DIR`.

For X11 clients (qv4l2, `ximagesink`), detect the XAUTHORITY path dynamically: from the Xwayland
process environment (on some RPi OS versions, e.g. Trixie with labwc, Xwayland uses an unexpected
path), otherwise from the `-auth` argument of the Xorg command line (for example GDM:
`/run/user/<uid>/gdm/Xauthority`), otherwise `~/.Xauthority` (for example the ADI Raspberry Pi
images, which use X11 with startx):
```bash
XAUTH=$(ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "grep -z XAUTHORITY /proc/\$(pgrep Xwayland | head -1)/environ 2>/dev/null | tr -d '\0' | cut -d= -f2-")
[ -z "$XAUTH" ] && XAUTH=$(ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "pgrep -a -x Xorg | grep -o -- '-auth [^ ]*' | head -1 | cut -d' ' -f2")
[ -z "$XAUTH" ] && XAUTH="<board_home>/.Xauthority"
```
(`<board_home>`: the login's home from `<state>`, `echo $HOME` on the board.)

## qv4l2

The Raspberry Pi's first viewer (rpi/4 §Viewer). Other platforms use it only when the user asks
and `qv4l2` is installed: vendor images often lack it and cannot build it (it needs Qt's
`moc`/`uic`/`rcc` of the same Qt version, and Meson); do not try without asking.

Check that it is installed (the platform file's **Tools** installs it, e.g. `apt-get install -y
qv4l2` on Raspberry Pi OS):
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which qv4l2 >/dev/null 2>&1 && echo found || echo missing"
```

Launch it on the board's display, with `$XAUTH` from §Desktop session:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "DISPLAY=:0 XAUTHORITY=$XAUTH XDG_RUNTIME_DIR=/run/user/\$(id -u) nohup qv4l2 > /dev/null 2>&1 &"
```

Verify qv4l2 actually started:
```bash
sleep 2
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "pgrep qv4l2 && echo 'qv4l2 running' || echo 'qv4l2 FAILED to start'"
```
If qv4l2 failed to start, check the XAUTHORITY path and retry once. If it still fails, does not
appear on the display, or the user reports no image, use the platform's fallback (RPi: §GStreamer,
with the reconfiguration of rpi/4 §Viewer). On the ADI RPi images without a monitor, two X servers
can start and block each other (display `:0` does not respond): connect a monitor to HDMI0 and
reboot.

If qv4l2 runs, tell the user: "Press the green Play button in qv4l2 to start the video feed."
qv4l2 opens `/dev/video0` by default; with several cameras or another device, tell the user which
`/dev/videoN` to open (File → Open Device).

## GStreamer

The viewer of NXP and generic boards, and the RPi fallback. It needs GStreamer with `bayer2rgb`
(Bayer sensors) and a desktop session (§Desktop session).

1. **Check the elements:**
   ```bash
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "which gst-launch-1.0; gst-inspect-1.0 bayer2rgb >/dev/null 2>&1 && echo bayer2rgb; gst-inspect-1.0 waylandsink >/dev/null 2>&1 && echo waylandsink; gst-inspect-1.0 ximagesink >/dev/null 2>&1 && echo ximagesink; gst-inspect-1.0 videoconvert >/dev/null 2>&1 && echo videoconvert"
   ```
   If `gst-launch-1.0` or `bayer2rgb` is missing, ask the user (header: "Install"), then install
   `gstreamer1.0-tools gstreamer1.0-plugins-bad` with the board's package manager (needs Internet
   access on the board):
   ```bash
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo '<pw>' | sudo -S apt-get update && echo '<pw>' | sudo -S apt-get install -y gstreamer1.0-tools gstreamer1.0-plugins-bad"
   ```
   (dnf/opkg: the distribution's names for the same packages; `bayer2rgb` is in plugins-bad.)
   Without a package manager or Internet access, skip the viewer and say why; the frame check
   shows that the pipeline works.
2. **Use an 8-bit RAW mode** for it (a copy of the pipeline configuration with the sensor's 8-bit
   code, e.g. `SRGGB8_1X8`, and the matching fourcc, e.g. `RGGB`, written as
   `~/Workspace/media_cfg_view.sh`): GStreamer's 10-bit Bayer formats expect the value in the low
   bits, while receivers such as NXP's ISI store it left-aligned (a washed-out image), and
   `v4l2src` cannot read packed formats such as `SRGGB10P`. Keep the frame-check configuration as
   `media_cfg.sh`. Run the frame check once on the 8-bit mode too.
3. **Start with a small sensor mode** (640x480, or the sensor's 2x2-binned mode): `bayer2rgb`
   demosaics on the CPU, and full-resolution RGB frames can be too slow to display (an i.MX95
   gave ~7 fps at 1080p, jerky, and 45 fps at 640x480).
4. **Measure before showing it to the user:** the camera and conversion alone, then the display
   path:
   ```bash
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "timeout 15 gst-launch-1.0 -v v4l2src device=<dev_path> ! video/x-bayer,format=<rggb|grbg|...>,width=<w>,height=<h> ! bayer2rgb ! video/x-raw,format=BGRx ! fpsdisplaysink video-sink=fakesink text-overlay=false sync=false 2>&1 | grep -o 'average: [0-9.]*' | tail -1"
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "export XDG_RUNTIME_DIR=/run/user/\$(id -u) WAYLAND_DISPLAY=wayland-0; timeout 15 gst-launch-1.0 -v v4l2src device=<dev_path> ! video/x-bayer,format=<rggb|...>,width=<w>,height=<h> ! bayer2rgb ! video/x-raw,format=BGRx ! fpsdisplaysink video-sink=waylandsink text-overlay=false sync=false 2>&1 | grep -o 'average: [0-9.]*' | tail -1"
   ```
   If the display rate is far below the camera rate, use a smaller mode.
5. **Launch it in the background**, with the Wayland environment of the desktop session:
   ```bash
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "export XDG_RUNTIME_DIR=/run/user/\$(id -u) WAYLAND_DISPLAY=wayland-0; nohup gst-launch-1.0 v4l2src device=<dev_path> ! video/x-bayer,format=<rggb|grbg|...>,width=<w>,height=<h> ! bayer2rgb ! video/x-raw,format=BGRx ! waylandsink sync=false > /tmp/gmsl_gst_view.log 2>&1 &"
   ```
   **When only X11 runs** (no Wayland socket, e.g. the ADI RPi images with startx): use
   `ximagesink` with `DISPLAY` and `$XAUTH` from §Desktop session:
   ```bash
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "DISPLAY=:0 XAUTHORITY=$XAUTH nohup gst-launch-1.0 v4l2src device=<dev_path> ! video/x-bayer,format=<rggb|...>,width=<w>,height=<h> ! bayer2rgb ! videoconvert ! ximagesink sync=false > /tmp/gmsl_gst_view.log 2>&1 &"
   ```
   **YUV sensors** (`UYVY`/`YUYV`, e.g. OV5640): no `bayer2rgb`; convert with `videoconvert`:
   `v4l2src device=<dev_path> ! video/x-raw,format=UYVY,width=<w>,height=<h> ! videoconvert ! waylandsink sync=false`
   (or `! videoconvert ! ximagesink sync=false` on X11).
6. **Verify** it runs and look at its log:
   ```bash
   sleep 3
   ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "pgrep -x gst-launch-1.0 && echo 'viewer running' || echo 'viewer FAILED'; tail -5 /tmp/gmsl_gst_view.log"
   ```
   Then ask the user whether the image shows on the board's display.

Do not put `queue` elements between `v4l2src` and `bayer2rgb` (they hold the capture buffers: the
rate dropped from 22 to 7 fps). Stop it with `pkill -x gst-launch-1.0` (`-x`: `pkill -f` with the
pipeline text also matches the shell that runs it). The viewer is left running at the end (the
board is left deployed); the final message says how to stop it.

A PipeWire session manager (`wireplumber`) holding the video device open is normal and does not
block capture.

If no viewer can be started (no desktop session, no GStreamer and no way to install it), do not
launch one: the frame check shows that the pipeline works, and the user can view frames with their
own tools.
