# Common 7: media pipeline with media-ctl

Configures the V4L2 media pipeline by hand with `media-ctl` and `v4l2-ctl` (routes, links, pad
formats, video node format) and writes it to `~/Workspace/media_cfg.sh`. Used by NXP (nxp/5
§Configure pipeline), generic boards, and on the Raspberry Pi for sensors without a `cam.py`
profile (rpi/4 §Configure pipeline). The platform files add their specifics (receiver entities,
crossbar route, pad numbers).

Needs `media-ctl` with routing support (`media-ctl --help` lists `--set-routes`, v4l-utils
1.28+) and `v4l2-ctl` (the platform file's **Tools**). All commands run on the board through the
control socket (common/5 §Control socket); sudo is not needed when the login is in the `video`
group (or is root).

At the end of this file you must know, for every camera, its video device (`<dev_path>`, e.g.
`/dev/video0`) and the configured format (width, height, fourcc, `Size Image`), for common/8.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the topology before and after configuration and a copy of the `media_cfg*.sh` scripts into `<run_dir>/board/` (§Verbose mode §What to save).

## Topology

Find the media device that contains the deserializer (entity `<driver> <bus>-<addr>`, e.g.
`max96716a 2-0028`), and print its topology. The output is long on SoCs with ISPs: save it to a
file and filter the chain's entities:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "for m in /dev/media*; do media-ctl -d \$m -p > /tmp/gmsl_topo_\$(basename \$m).txt 2>&1; grep -l -E 'max9296a|max96716|max96724|max96712|max96792|max96714' /tmp/gmsl_topo_\$(basename \$m).txt; done"
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "grep -nE '^- entity|^\s+pad|->|<-|\[fmt|routes|\[ACTIVE\]' /tmp/gmsl_topo_<mediaN>.txt | head -150"
```
Copy the file to the host if it helps (`scp -o ControlPath=/tmp/gmsl_ssh_sock
<user>@<ip>:/tmp/gmsl_topo_<mediaN>.txt /tmp/`), and delete the board's `/tmp/gmsl_topo_*` files
when the pipeline works (scratch files the skill put in the board's `/tmp`).

The chain is sensor → serializer → deserializer → receiver subdev(s) → video device(s). Note for
each entity its name (quoted in `media-ctl` commands), its pads, and whether it lists routes.

Check the link from the deserializer's source pad `<S>` to the receiver: if it is not
`[ENABLED]`, enable it:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "media-ctl -d <mediaN> -l '\"<des entity>\":<S> -> \"<receiver entity>\":0[1]'"
```
Enable every other disabled link of the chain the same way (e.g. receiver → formatter → ISI).

Receivers that fan one input out to several channels (for example NXP's ISI crossbar, which
routes its sink to all 8 channels by default) need a **single route** to the channel that is
captured, e.g.:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "media-ctl -d <mediaN> -R '\"crossbar\" [2/0 -> 5/0 [1]]'"
```
(the platform file gives the pads; nxp/5 §Configure pipeline for i.MX95).

## Routes

Start with **only the image stream** of each camera (embedded metadata is not needed for video,
and carrying it also needs the receiver's metadata video node configured):
- **each sensor:** set the image format on its source pad (§Formats; this selects the sensor
  mode). If it lists more than one route (image + metadata), request only the image route with
  `-R`, then **check** `media-ctl -p`: a sensor driver can keep all its routes and ignore the
  request without an error. The sensor then still sends the metadata stream (see the metadata
  fallback in §Check);
- **each serializer:** `0/0 -> 1/0 [1]`:
  `media-ctl -d <mediaN> -R '"<ser entity>" [0/0 -> 1/0 [1]]'`;
- **the deserializer:** source pad `<S>` = number of GMSL links of the chip + `phy_idx`
  (e.g. MAX96716A, 2 links, PHY 1: `<S>` = 3; MAX96724, 4 links, PHY 2: `<S>` = 6). Route link
  `<k>` to stream `<k>` of `<S>` (`k/0 -> S/k [1]`), **all routes in one `-R` command** (each `-R`
  replaces the whole routing table), e.g. two cameras:
  `media-ctl -d <mediaN> -R '"<des entity>" [0/0 -> 3/0 [1], 1/0 -> 3/1 [1]]'`;
- **the receiver subdev(s):** if they list routes, route each camera's sink stream to the
  receiver's image source pad/stream for that camera (follow the receiver's own pad layout in
  `media-ctl -p`); if they have no routes, only set the formats.

## Formats

Formats on every pad/stream of the chain: `[fmt:<code>/<w>x<h> field:none]`, with `<code>` the
sensor's image bus code (e.g. `SRGGB10_1X10`; a mode the sensor lists, e.g. `media-ctl -d
<mediaN> --get-v4l2 '"<sensor entity>":0'` and the driver's modes). Write `field:none`
explicitly: a pad left with an unset field ("any") fails the kernel's default link validation
against a sensor that reports `none`. With streams, name the stream: `"<entity>":<pad>/<stream>`:
```bash
media-ctl -d <mediaN> -V '"<sensor entity>":0 [fmt:SRGGB10_1X10/640x480 field:none]'
media-ctl -d <mediaN> -V '"<ser entity>":0/0 [fmt:SRGGB10_1X10/640x480 field:none]'
media-ctl -d <mediaN> -V '"<des entity>":<k>/0 [fmt:SRGGB10_1X10/640x480 field:none]'
media-ctl -d <mediaN> -V '"<receiver entity>":0/<k> [fmt:SRGGB10_1X10/640x480 field:none]'
```
Set the sink side of each routed pair; drivers propagate to the source side of their routes, but
set every pad that `media-ctl -p` still shows different.

## Video node

Each video device:
```bash
v4l2-ctl -d <dev> --list-formats
v4l2-ctl -d <dev> --set-fmt-video=width=<w>,height=<h>,pixelformat=<fourcc>
v4l2-ctl -d <dev> --get-fmt-video
```
with the fourcc from `--list-formats` that matches the bus format (e.g. an unpacked or packed
10-bit Bayer format for `SRGGB10_1X10`: `RG10`, `pRAA`; `RGGB` for `SRGGB8_1X8`; `UYVY` for
`UYVY8_1X16`). Note `Size Image` from `--get-fmt-video` for common/8 §Frame check. The video
device of a camera is the one linked to the receiver output (channel) its route ends at
(`media-ctl -p` shows the `/dev/videoN` of each video entity).

## Check

**Check** with `media-ctl -p` that every pad/stream of the chain shows the same code, size and
field, and that the routes are `[ACTIVE]`:
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "media-ctl -d <mediaN> -p | grep -nE '^- entity|pad[0-9]|fmt:|->|\[ACTIVE\]'"
```

**Metadata fallback:** if streaming then fails because the sensor still sends metadata that
nothing downstream routes (the receiver's link validation may reject it), or the user needs the
metadata: route the image as stream `2k` and the metadata as `2k+1` through the serializer and
deserializer, use the sensor's own metadata bus code from `media-ctl -p` (not always `META_8`),
route it to the receiver's metadata output, and configure that video node (some `v4l2-ctl`
versions set only the fourcc of a metadata format, not its width/height).

**Multi-camera check:** with more than one camera, check after the first frame check (common/8
§Frame check) that every camera arrives on its own video device (`--log-status` of the
deserializer shows the virtual channel per pipe:
`v4l2-ctl -d /dev/v4l-subdev<N> --log-status`, then `dmesg | tail -40`). If cameras are missing
or mixed, try the other stream numbering (`2k` per link) before anything else: how the
deserializer driver assigns virtual channels to streams has only been checked with one camera in
this numbering.

## media_cfg.sh

Write the commands to `~/Workspace/media_cfg.sh`: `#!/bin/sh`, `set -e`, a comment with the
configuration (deserializer, PHY, cameras, sensor mode, date). `chmod +x` it and run it.
```bash
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "echo \$HOME; mkdir -p ~/Workspace"
cat > /tmp/gmsl_media_cfg.sh <<'EOF'
#!/bin/sh
# GMSL pipeline: <deserializer> PHY <phy_idx>, <n> camera(s) <sensor> <w>x<h> <code>
set -e
M=<mediaN>
media-ctl -d $M -l '"<des entity>":<S> -> "<receiver entity>":0[1]'
media-ctl -d $M -R '"<ser entity>" [0/0 -> 1/0 [1]]'
media-ctl -d $M -R '"<des entity>" [0/0 -> <S>/0 [1]]'
media-ctl -d $M -V '"<sensor entity>":0 [fmt:<code>/<w>x<h> field:none]'
# ... every pad/stream of the chain, receiver routes, crossbar route ...
v4l2-ctl -d <dev_path> --set-fmt-video=width=<w>,height=<h>,pixelformat=<fourcc>
EOF
scp -o ControlPath=/tmp/gmsl_ssh_sock /tmp/gmsl_media_cfg.sh <user>@<ip>:Workspace/media_cfg.sh
rm -f /tmp/gmsl_media_cfg.sh
ssh -S /tmp/gmsl_ssh_sock <user>@<ip> -- "chmod +x ~/Workspace/media_cfg.sh && ~/Workspace/media_cfg.sh"
```
`~` is the login's home folder; do not assume it: root's home is `/root` on most images,
including NXP's Yocto images. Check with `echo $HOME` (state `board_home`) and use `~` or
`$HOME`, never `/home/<user>`.

If a command fails, read the error and the last `dmesg` lines, and adjust (receivers differ most
here). If `media-ctl -R` fails with `Device or resource busy` (`EBUSY`) after something has
streamed, reboot (common/5 §Reboot) and configure once (some drivers do not release streams).
Other `EBUSY` errors: another process (a viewer, `v4l2-ctl`, an earlier configuration run) holds
the video devices; close it and retry.

The media configuration is not persistent: run `~/Workspace/media_cfg.sh` again after every
reboot (tell the user in the final message).
