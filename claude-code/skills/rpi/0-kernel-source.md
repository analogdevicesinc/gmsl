# RPi 0: Fetch the kernel source

## Purpose
First step of the Raspberry Pi build path (orchestrator §Flow, "No" answer, RPi 5 / RPi 4), after
common/0 (host tools, sudo password) and the SD card detection of common/11 §Detect.

Auto-detect the latest ADI GMSL branch for Raspberry Pi (used as patch source), detect
the latest stock RPi kernel branch, and clone the stock branch as the build base. This
ensures the resulting kernel maintains the same version as the stock RPi OS image.

## Step 1: Auto-detect the latest ADI GMSL branch

Use the newest ADI GMSL branch: read `<skill dir>/common/4-device-tree.md` and follow its
**Latest ADI GMSL branch** paragraph (§Templates), which compares the newest `gmsl/rpi-*` branch
with ADI's README pointer below; store the result as `adi_gmsl_branch` (if it is already in the
state, use it). ADI's README pointer, for reference:
```bash
curl -sL https://raw.githubusercontent.com/analogdevicesinc/gmsl/main/README.md \
  | grep '\[bi-0\]' \
  | grep -oP 'linux/blob/\K[^/]+/[^/]+(?=/README)' \
  | head -1
```

The `[bi-0]` reference link points to the RPi source code entry. This extracts the branch
name including the slash, e.g., `gmsl/rpi-6.13.y`.

Derive the ADI base branch by stripping the `gmsl/` prefix:
```bash
ADI_BRANCH="<adi_gmsl_branch>"   # the newest ADI GMSL branch (Step 1)
ADI_BASE="${ADI_BRANCH#gmsl/}"  # → rpi-6.13.y
```

## Step 2: Detect the latest stock RPi kernel branch

Query the RPi kernel repository for the latest `rpi-6.x.y` branch:
```bash
STOCK_BRANCH=$(git ls-remote --heads https://github.com/raspberrypi/linux \
  | grep -oP 'refs/heads/\Krpi-6\.\d+\.y' | sort -V | tail -1)
echo "Stock RPi branch: $STOCK_BRANCH"
```

Tell the user:
- "ADI GMSL branch: `<ADI_BRANCH>` (patch source, based on `<ADI_BASE>`)"
- "Stock RPi branch: `<STOCK_BRANCH>` (build base — matches the latest RPi OS image)"

If the stock branch is newer than the ADI base branch (e.g., `rpi-6.18.y` vs `rpi-6.13.y`),
warn: "The stock kernel is newer than the ADI GMSL base. Patches will be ported from
`<ADI_BASE>` to `<STOCK_BRANCH>` — some may need manual adaptation if they don't apply cleanly."

## Step 3: Clone the kernel source

The kernel tree is `<work_dir>/rpi-linux` (`<work_dir>` from `<state>`, default
`~/gmsl-camera-work`), on the **stock** branch. A kept work folder from an earlier run may
already have it (the orchestrator's §Cleanup keeps it for reuse), so check first:
```bash
test -d <work_dir>/rpi-linux/.git && git -C <work_dir>/rpi-linux rev-parse --abbrev-ref HEAD
```
- **Prints `<STOCK_BRANCH>`:** reuse the tree (no download): restore it to the clean branch,
  so rpi/1 applies its changes to an unmodified tree (`git clean` without `-x` keeps the ignored
  build output, which makes the build of rpi/2 faster), and tell the user that the tree is reused:
  ```bash
  git -C <work_dir>/rpi-linux checkout -- . && git -C <work_dir>/rpi-linux clean -fdq
  ```
- **Prints another branch (or the folder exists without `.git`):** tell the user what is there
  (branch, `du -sh <work_dir>/rpi-linux`) and ask (header: "Kernel tree"): **Replace it with
  `<STOCK_BRANCH>` (Recommended)** (deletes the old tree) | **Stop**. Delete it only after the user
  agreed (`rm -rf <work_dir>/rpi-linux`; on `Permission denied`, after `sudo make
  modules_install` in it, repeat with `echo '<pw>' | sudo -S rm -rf <work_dir>/rpi-linux`).
- **No tree:** clone it:
```bash
mkdir -p <work_dir>
git clone --depth 1 --branch <STOCK_BRANCH> https://github.com/raspberrypi/linux <work_dir>/rpi-linux
```

This is a shallow clone to save time and disk space. The full RPi kernel tree is ~1.5 GB
even with `--depth 1`; the build of rpi/2 needs about 4 GB more. Check the free space first
(`df -h <work_dir>`) and tell the user if less than 8 GB are free.

## Step 4: Update state

```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['kernel_src_path'] = '<work_dir>/rpi-linux'
state['kernel_branch'] = '<STOCK_BRANCH>'
state['adi_gmsl_branch'] = '<ADI_BRANCH>'
state['adi_base_branch'] = '<ADI_BASE>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
