# Common 0: Host

Checks the host operating system, installs the host tools, collects the sudo password and installs
usbipd on WSL. The orchestrator runs §Host OS at the start of every run; the other sections only
on the paths that need them:

| Section | Needed on |
|---------|-----------|
| §Host OS (+ §Windows adaptations) | Every run |
| §Quick tool check | The live-board path (orchestrator §Flow, Yes path, before common/1) |
| §Host tools, §Sudo password | The build/flash paths (Raspberry Pi SD card, NXP SD card or eMMC), and Build mode on a live board (ADI drivers built against the board's kernel: nxp/1, common/9, generic best effort) |
| §usbipd | WSL only, when an SD card is flashed (common/11) or an NXP board is flashed over USB with `uuu` from WSL (nxp/4) |

The live-board path (a board that is already running with the ADI drivers) needs no sudo password
on the host and no build tools: only the tools of the installation instructions below (`sshpass`,
`ssh`, `dtc`, `python3` with `jinja2`, `curl`, `git`), checked by §Quick tool check.

## Host OS

Check whether Claude Code is running on Windows. It is Windows if the environment reports
platform `win32`, or if `uname -s` returns a name starting with `MINGW`, `MSYS` or `CYGWIN`.
WSL reports `Linux` and is treated as Linux.
- **WSL** (`/proc/version` contains `microsoft`, or `$WSL_DISTRO_NAME` is set): host = `wsl`.
- **Linux** (`uname -s` is `Linux`, not WSL): host = `linux`.
- **Windows**: host = `windows`.

Store the host type in the state (`host`, see the orchestrator's §State).

If the host is Windows, use AskUserQuestion (header: "Windows") to tell the user that the setup
is intended for Linux or WSL and ask whether they are sure they want to continue on Windows. Say in
the question that native Windows can only set up a board that is already running and on the
network: building a kernel or the ADI drivers and flashing an SD card or eMMC need WSL or Linux.
- **Continue on Windows**: continue with the orchestrator's flow and apply §Windows adaptations.
  If the user then chooses a path that builds or flashes (the orchestrator's "No" answer to Q1,
  or building drivers for a live board: nxp/1, common/9, generic Build), stop that path: tell the user it needs
  WSL or Linux and display §WSL installation instructions.
- **Use WSL (Recommended)**: display §WSL installation instructions and stop.
- **Use Linux**: display §Linux installation instructions and stop.

## Windows adaptations

Only for the live-board path (see §Host OS).
- Python 3 is required on the host (gen_gmsl_dts.py and the SSH automation both need it).
  Check it with `python --version` and `py --version`. Ignore a `python`/`python3` that opens
  the Microsoft Store or prints nothing: that is only the Windows app alias, not Python.
  If Python is missing, use AskUserQuestion (header: "Python") to ask the user:
  - **Install Python (Recommended)**: if `winget` is available, run
    `winget install -e --id Python.Python.3.12`. Otherwise tell the user to download the
    installer from https://www.python.org/downloads/windows/ and to check
    **"Add python.exe to PATH"** during installation, then wait for them to confirm.
    A newly installed Python may not be on the PATH of the current session; if `python`
    is still not found, use the full path
    (`%LOCALAPPDATA%\Programs\Python\Python312\python.exe`) or ask the user to restart
    Claude Code.
  - **Use WSL instead**: display §WSL installation instructions and stop.
  - **Stop**: stop the setup.
- Use `python` (or `py`) wherever the skills say `python3`. gen_gmsl_dts.py also needs the
  `jinja2` module: `python -m pip install jinja2` if it is missing.
- Use `%TEMP%\gen_gmsl_dts` instead of `/tmp/gen_gmsl_dts`, `%TEMP%\gmsl_camera_state.json` instead
  of `/tmp/gmsl_camera_state.json`, and `%TEMP%\gmsl_*` for every other host file the skills put
  in `/tmp/gmsl_*`.
- `sshpass` and SSH control sockets (`-M -S`) are not available on Windows. Run the same remote
  commands with a Python script that uses `paramiko` (install it on the host with
  `python -m pip install paramiko` if missing), and copy files with paramiko's SFTP. Keep one
  paramiko connection open where the skills use the control socket, and reconnect after a reboot
  as common/5 §Reboot does. Never install anything on the board with this method other than what
  the platform file's **Tools** section installs.
- If `dtc` is not available on the host, copy `gmsl-overlay.dts` to the board and compile it
  there with the same `dtc` command (common/4 §Compile), then delete the `.dts` and `.dtbo` from
  the home folder after installing the device tree. The same applies to the `fdtoverlay`
  validation (common/4 §Validate) and the merged DTB (common/4 §Merged DTB): run them on the board
  if it has `fdtoverlay`, in a scratch file in its `/tmp` named `/tmp/gmsl_*`, deleted afterwards.
- Windows-side tools that the WSL instructions call with `.exe` (`powershell.exe`) are called
  without the suffix.

## Quick tool check

The live-board path (orchestrator §Flow, Yes path, before common/1). Native Windows: skip this
section and use the Python, `jinja2` and `paramiko` checks of §Windows adaptations instead.

```bash
for t in ssh sshpass dtc fdtoverlay python3 curl git; do command -v $t >/dev/null 2>&1 || echo "missing: $t"; done
python3 -c 'import jinja2' >/dev/null 2>&1 || echo "missing: python3-jinja2"
```
It must print nothing. Otherwise tell the user what is missing and what it is used for
(`sshpass`: SSH logins with a password; `dtc`/`fdtoverlay`: the device tree; `python3` with
`jinja2`: gen_gmsl_dts.py), and use AskUserQuestion (header: "Host tools"):
- **Install them (Recommended)**: this needs the sudo password (§Sudo password, asked now, also on
  the live-board path), then (Debian/Ubuntu package names: `openssh-client`, `sshpass`,
  `device-tree-compiler`, `python3`, `python3-jinja2`, `curl`, `git`):
  ```bash
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get update
  SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y <packages>
  ```
  (passwordless: `sudo apt-get ...`; without `apt-get`: ask the user to install them with the
  distribution's package manager, as in §Host tools). Run the check again: it must print nothing.
- **I will install them myself**: show the command of §WSL installation instructions step 3 (or
  §Linux installation instructions step 2), wait for the user, and run the check again.
- **Stop**.

## WSL installation instructions
1. Open PowerShell as Administrator and run `wsl --install`, then restart Windows.
2. Open the **Ubuntu** app from the Start menu and create a Linux username and password.
3. In the Ubuntu terminal, install the required tools:
   `sudo apt update && sudo apt install python3 python3-jinja2 curl openssh-client sshpass device-tree-compiler git`
   (the build/flash paths install the rest themselves, see §Host tools).
4. Install Claude Code: `curl -fsSL https://claude.ai/install.sh | bash`
5. Copy the folder that contains this skill's `README.md` into the Ubuntu home folder and `cd`
   into it. Then use one of the options of that `README.md` (§Install):
   - **Option 1 (Recommended, the simplest):** no copying: `cd skills && claude`. If Claude Code
     does not pick up `SKILL.md`, start with "follow SKILL.md".
   - **Option 2, user-wide:** `mkdir -p ~/.claude/skills && cp -r skills ~/.claude/skills/gmsl-camera-pipeline`,
     then run `claude` in any folder.
   - **Option 3, per project:** `mkdir -p <project>/.claude/skills && cp -r skills <project>/.claude/skills/gmsl-camera-pipeline`,
     then run `claude` in `<project>`.
   Options 2 and 3 copy the whole folder, with `common/`, `rpi/`, `nxp/` and `generic/` in it.
6. Ask again for the GMSL camera setup ("set up my GMSL camera").

## Linux installation instructions
1. Install a Linux distribution (for example Ubuntu) on the host PC, following the
   distribution's installation guide.
2. Install the required tools (Debian/Ubuntu):
   `sudo apt update && sudo apt install python3 python3-jinja2 curl openssh-client sshpass device-tree-compiler git`
3. Follow steps 4–6 of §WSL installation instructions in a Linux terminal.

## Host tools

Build/flash paths and Build mode only (see the table at the top). Native Windows: not available
(see §Host OS). Collect the sudo password first (§Sudo password): installing needs it.

Check for each tool and install the missing ones. Batch all missing packages into a single
`apt-get install` command.

| Tool | Package name | Used for |
|------|-------------|----------|
| `aarch64-linux-gnu-gcc` | `gcc-aarch64-linux-gnu` | Cross-compiling the kernel, modules, `i2cburst`, U-Boot envtools |
| `dtc`, `fdtoverlay` | `device-tree-compiler` | Device tree overlay, validation, merged DTB |
| `sshpass` | `sshpass` | SSH logins with a password |
| `curl` | `curl` | Downloads |
| `git` | `git` | Kernel, U-Boot and driver trees |
| `make` | `make` | Builds |
| `bc` | `bc` | Kernel build |
| `flex` | `flex` | Kernel and U-Boot build |
| `bison` | `bison` | Kernel and U-Boot build |
| `xz` | `xz-utils` | Raspberry Pi OS image (`.img.xz`) |
| `zstd` | `zstd` | NXP images (`.wic.zst`), compressed modules (`.ko.zst`) |
| `bzip2` | `bzip2` | NXP images (`.wic.bz2`) |
| `unzip` | `unzip` | NXP release archives (`.zip`) |
| `depmod`, `modinfo` | `kmod` | Installing modules into a mounted image (`depmod -b`), module checks |
| `mkenvimage` | `u-boot-tools` | U-Boot environment image for a fresh NXP medium (common/10 §Offline env image) |
| `python3 -c 'import jinja2'` | `python3-jinja2` | gen_gmsl_dts.py |
| `openssl` | `openssl` | Kernel and U-Boot build (signing tools) |
| OpenSSL headers | `libssl-dev` | Kernel and U-Boot build |

Check each tool:
```bash
MISSING=""
command -v aarch64-linux-gnu-gcc >/dev/null 2>&1 || MISSING="$MISSING gcc-aarch64-linux-gnu"
command -v dtc >/dev/null 2>&1 && command -v fdtoverlay >/dev/null 2>&1 || MISSING="$MISSING device-tree-compiler"
command -v sshpass >/dev/null 2>&1 || MISSING="$MISSING sshpass"
command -v curl >/dev/null 2>&1 || MISSING="$MISSING curl"
command -v git >/dev/null 2>&1 || MISSING="$MISSING git"
command -v make >/dev/null 2>&1 || MISSING="$MISSING make"
command -v bc >/dev/null 2>&1 || MISSING="$MISSING bc"
command -v flex >/dev/null 2>&1 || MISSING="$MISSING flex"
command -v bison >/dev/null 2>&1 || MISSING="$MISSING bison"
command -v xz >/dev/null 2>&1 || MISSING="$MISSING xz-utils"
command -v zstd >/dev/null 2>&1 || MISSING="$MISSING zstd"
command -v bzip2 >/dev/null 2>&1 || MISSING="$MISSING bzip2"
command -v unzip >/dev/null 2>&1 || MISSING="$MISSING unzip"
command -v depmod >/dev/null 2>&1 || [ -x /sbin/depmod ] || MISSING="$MISSING kmod"
command -v mkenvimage >/dev/null 2>&1 || MISSING="$MISSING u-boot-tools"
python3 -c 'import jinja2' >/dev/null 2>&1 || MISSING="$MISSING python3-jinja2"
command -v openssl >/dev/null 2>&1 || MISSING="$MISSING openssl"
dpkg -s libssl-dev >/dev/null 2>&1 || MISSING="$MISSING libssl-dev"
echo "$MISSING"
```

If `$MISSING` is non-empty, install (with an empty password, `sudo apt-get ...` instead):
```bash
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get update
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A apt-get install -y $MISSING
```

If `apt-get` is not available (a native Linux distribution that is not based on Debian or
Ubuntu), do not try another package manager: tell the user which tools are missing
(including the OpenSSL development headers), ask them to install them with their
distribution's package manager, and continue once they confirm.

Verify all tools are now available (run the check again: it must print nothing). If any failed,
report and stop.

The host cross-compiler from the package is not always the right one: kernels built with
`CONFIG_MODVERSIONS=y` need the same compiler major version as the board's kernel
(common/9 §Compiler, nxp/1 §Values). A downloaded toolchain goes into `<work_dir>`, not into the
system.

Also check the free disk space in `<work_dir>` (default `~/gmsl-camera-work`, create it with
`mkdir -p`) before a build: `df -h <work_dir>`. A kernel tree needs about 4 GB, a toolchain about
1 GB, an NXP image several GB (compressed and unpacked). If less than 15 GB are free, tell the
user and ask whether to continue or to choose another work folder (store it as `work_dir`).

Whenever `work_dir` is set or changed, record whether this run creates the folder (the
orchestrator's §Cleanup never deletes a folder the run did not create, and offers only the entries
the run added), before `mkdir -p`:
```bash
python3 -c "
import json, os
state = json.load(open('/tmp/gmsl_camera_state.json'))
d = state['work_dir']
state['work_dir_created'] = not os.path.exists(d)
state['work_dir_preexisting'] = sorted(os.listdir(d)) if os.path.isdir(d) else []
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
(Run it only when `work_dir` changed or the keys are not in the state yet: once the run has
created the folder, `work_dir_created` stays `True`.)

## Sudo password

Build/flash paths and Build mode only (see the table at the top), plus §Quick tool check when the
user chooses to install missing tools. Ask once per run, the first time a step needs it; later
steps use the password file.

**Permission mode:** the sudo commands of this skill (`SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A ...`) can be refused in
Claude Code's auto mode. Before asking, tell the user that the steps from here on need manual
mode (the default permission mode, where they approve each command; Shift+Tab switches) or a
permission rule for these commands. If a sudo command is refused later, stop, tell the user
which command and that it needs manual mode, and continue only after they switched; never
rewrite the command to get around the refusal.

Use AskUserQuestion with 1 question:
- **Sudo Password** (header: "Sudo Password"): say in the question that it is the password of
  the user on this PC (WSL or Linux), not the board's, that it is kept only in the file
  `/tmp/gmsl_pw_sudo` (readable only by the current user), never written into a command, and
  deleted by the cleanup; and that anything typed in the chat stays in Claude Code's session
  history (`~/.claude/projects/`).
  - Option 1: "Passwordless sudo configured" (description: "sudo works without a password on this system")
  - Option 2: Other (user types their password)

Write the typed password to the file **once** (the only command that contains it):
`(umask 077; printf '%s\n' '<typed password>' > /tmp/gmsl_pw_sudo)`. Never put the password, or
any part of it, into another command or message. If passwordless, create an empty file the same
way. Then create the helper that `sudo -A` uses and check it:
```bash
(umask 077; printf '#!/bin/sh\ncat /tmp/gmsl_pw_sudo\n' > /tmp/gmsl_askpass) && chmod 700 /tmp/gmsl_askpass
SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A true && echo "sudo OK"
```
If the check fails, tell the user (wrong password, or the file is missing) and ask again. Store
`sudo_pw_file` = `/tmp/gmsl_pw_sudo` (or `passwordless`) in the state; the state holds no
password.

For all subsequent local sudo commands in this and all other files, use
`SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A <cmd>` (or `... sudo -A sh -c '<cmd>'` for a command
line). Use it for every sudo command, also after long commands such as `dd` or a kernel build:
the cached sudo credentials can expire meanwhile, and a plain `sudo` then fails with "a terminal
is required". Keep `/tmp/gmsl_pw_sudo` and `/tmp/gmsl_askpass` until the end of the run
(§Cleanup deletes them, after the verbose §Package); do not delete the password mid-run.

## usbipd

WSL only (skip when `host` is `linux`: native Linux sees the SD card reader and USB devices
directly). Needed by common/11 (SD card) and by nxp/4 §eMMC with uuu when `uuu` runs inside WSL.

### Check for usbipd
```bash
command -v usbipd.exe >/dev/null 2>&1 && echo "found" || echo "missing"
```

If missing, auto-install:
```bash
winget.exe install --id dorssel.usbipd-win --accept-source-agreements --accept-package-agreements
```

Verify:
```bash
command -v usbipd.exe >/dev/null 2>&1 && echo "ok" || echo "FAILED"
```
(A freshly installed `usbipd.exe` may not be on the PATH of the running WSL session; then also
try `"/mnt/c/Program Files/usbipd-win/usbipd.exe"` before reporting a failure.)

If still missing, tell the user: "usbipd installation failed. Please install it manually from
a Windows terminal: `winget install usbipd`" and use AskUserQuestion to ask if they want
to retry or skip SD card detection.

### Check for wsl --mount support
```bash
wsl.exe --mount 2>&1 | iconv -f utf-16le -t utf-8 | tr -d '\r' | grep -q 'Invalid command line argument: --mount' && echo "wsl --mount: not supported" || echo "wsl --mount: supported"
```
(`wsl.exe` prints UTF-16; `--mount` without arguments prints the usage text when it exists, and
`Invalid command line argument: --mount` when it does not.) If it is supported, built-in SD card
slots can be used. If not, only USB readers will work — inform the user.

### Running Windows commands as administrator

`usbipd.exe bind` and `wsl.exe --mount` / `--unmount` need administrator rights on Windows.
Run them directly first. If they fail with an access-denied or "administrator" error, run them
elevated through PowerShell instead. Windows then shows a UAC prompt: tell the user to approve
it, and wait for the command to finish:
```bash
powershell.exe -NoProfile -Command 'Start-Process <program> -ArgumentList "<arg1>","<arg2>" -Verb RunAs -Wait'
```
Keep this command in single quotes in bash, so the backslashes in `\\.\PHYSICALDRIVE<N>` reach
PowerShell unchanged. If the user declines the UAC prompt, report it and use AskUserQuestion
to ask whether to retry or stop.

All Windows-side commands from WSL use the `.exe` suffix (`usbipd.exe`, `wsl.exe`,
`powershell.exe`); in WSL, use `/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe` if
`powershell.exe` is not on the PATH.
