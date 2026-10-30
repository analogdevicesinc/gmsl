# Common 5: Connect and reboot

The SSH control socket that every later command uses, the reboot that loads a new device tree or
module, and the checks right after it. Placeholders: `<ip>`, `<user>`, `<pw>` from the state
(`board_ip`, `board_user`, `board_pw`).

On Windows there are no control sockets: keep one paramiko connection open instead, and
reconnect it where this file re-opens the socket (common/0 §Windows adaptations).

## Control socket

First close any stale socket and remove the socket file, then open a new one:
```bash
ssh -S /tmp/gmsl_ssh_sock -O exit <user>@<ip> 2>/dev/null
rm -f /tmp/gmsl_ssh_sock
sleep 0.5
sshpass -f /tmp/gmsl_pw_board ssh -M -S /tmp/gmsl_ssh_sock -fN \
  -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 \
  -o ServerAliveInterval=15 -o ServerAliveCountMax=3 <user>@<ip>
ssh -S /tmp/gmsl_ssh_sock -O check <user>@<ip>
```
The check must print `Master running`. If it does not, open the socket again (all the
commands above), up to 3 times, 5 seconds apart.

Do not hide SSH's error output (no `2>/dev/null`) on the connection commands. The host key is not
stored (`UserKnownHostsFile=/dev/null`), so a reflashed board (new host key) connects without a
`REMOTE HOST IDENTIFICATION HAS CHANGED` error; if another command reports it, run
`ssh-keygen -R <ip>` and connect again.
If the connection fails for any other reason, use AskUserQuestion to ask the user to retry,
change the IP, or stop. If the user stops, run the orchestrator's §Cleanup.

From then on, every command on the board goes through the socket:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "<cmd>"
scp -o ControlPath=/tmp/gmsl_ssh_sock -o BatchMode=yes <file> <user>@<ip>:<path>
```
- `BatchMode=yes` makes a command fail at once when the socket is gone (no password prompt, no
  `Host key verification failed` from a fallback connection). If a command prints
  `Control socket connect(...)`, `Permission denied` or `Host key verification failed`, stop and
  open the socket again (this section) before running anything else. Recent OpenSSH versions
  refuse connections for a while after failed logins (`penalty: failed authentication` on the
  board); if connections are reset, wait 30 seconds before trying again.
- Sudo commands on the board: `SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A sh -c '<cmd>'`. Logged in as `root`
  (common on NXP images, which may not have `sudo` at all): run `sh -c '<cmd>'` without the sudo
  part. Not root: the board password and the helper are on the board from common/2 (§Run the
  assessment); put them back after every reboot (the board's `/tmp` is empty then), and delete
  them in the orchestrator's §Cleanup step 3 with the board's `/tmp/gmsl_*`:
  ```bash
  ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "umask 077; cat > /tmp/gmsl_pw_board; printf '#!/bin/sh\ncat /tmp/gmsl_pw_board\n' > /tmp/gmsl_askpass; chmod 700 /tmp/gmsl_askpass" < /tmp/gmsl_pw_board
  ```

## Reboot

If the platform file installed the device tree at runtime (without a reboot, for example a
runtime overlay of generic/1 §Install device tree), skip the reboot and go directly to
§Probe check.

First read the board's current boot ID (`<boot_id>`), a random value the kernel creates at every
boot, to tell the rebooted board apart from the old session later:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "cat /proc/sys/kernel/random/boot_id"
```

Then reboot (as `root`: `reboot` without the sudo part):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A reboot"
ssh -S /tmp/gmsl_ssh_sock -O exit <user>@<ip> 2>/dev/null; rm -f /tmp/gmsl_ssh_sock
```
(The board's `/tmp` is empty after the reboot: put the password helper back (§Control socket) when
the login is not root.)

Then sleep 30, poll with:
```bash
sshpass -f /tmp/gmsl_pw_board ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 <user>@<ip> cat /proc/sys/kernel/random/boot_id
```
every 5 seconds until it succeeds and prints a boot ID different from `<boot_id>` (max 40
attempts). A board that is slow to shut down can still answer with the old boot ID after the
30 seconds: that is not the rebooted board, keep polling. If the board does not come back, tell
the user, with the recovery advice of the platform file's **Troubleshooting** (and the recovery
of the boot change that was made: the platform file's install section, common/10), and ask
whether to wait longer or stop. If the user stops, run the orchestrator's §Cleanup. A board that
comes back at another address (DHCP): see the orchestrator's §Troubleshooting ("The board stops
answering").

After the board is online, sleep 5 more seconds, then re-establish the control socket
(§Control socket). Right after a boot, the SSH server can answer before logins work
(`Permission denied`): that is not a wrong password. Use §Control socket's retries (up to 3
attempts, 5 seconds apart), and wait 30 seconds before a last attempt.

**Verbose runs** (`verbosity` = `verbose`): log as in the orchestrator's §Verbose mode §Logging rules, and save the post-reboot diagnostics (`system`, `dmesg`, `lsmod`, `media`, device tree setting) into `<run_dir>/board/` with a numbered suffix, and a summary in the log (§Verbose mode §What to save).

## Probe check

Check that the devices probed:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "ls /dev/v4l-subdev* /dev/video* /dev/media*"
```
Also check the kernel log of this boot for the GMSL drivers (as a normal user, `dmesg` may need
sudo):
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A dmesg | grep -iE 'max9|max-serdes|max_serdes|<sensor>' | tail -20"
```
- `v4l-subdev*` nodes for the deserializer, the serializers and the sensors, and a media device:
  continue with the platform file.
- No `v4l-subdev*` nodes: run §Camera port check first, before anything else.
- No GMSL driver messages at all: the device tree that was installed is not the one the board
  booted, or the drivers are missing: check the platform file's install section (the boot config
  or `fdtfile` that is actually used) and run common/3 §Driver check again.

For any other failure, see the orchestrator's §Troubleshooting and the platform file's
**Troubleshooting** before continuing.

## Camera port check

If no `v4l-subdev*` nodes are listed, check the camera port:
```bash
ssh -S /tmp/gmsl_ssh_sock -o BatchMode=yes <user>@<ip> -- "SUDO_ASKPASS=/tmp/gmsl_askpass sudo -A dmesg | grep -E '<deserializer_driver> .*(-121|probe)' | tail -3; i2cdetect -y -r <camera_i2c_bus>"
```
(`<camera_i2c_bus>`: the bus number of common/3 §I2C probe, or the one in the platform file's
**Troubleshooting**; `i2cdetect` is in the `i2c-tools` package; as a normal user, run it with
sudo too. On a bus shared with power-critical devices — the assessment's `i2c_devices`, generic
only — do not scan the whole bus: read the deserializer's address only, with
`i2cdetect -y -r <bus> <addr> <addr>`.) If the deserializer probe failed with `-121` and
`i2cdetect` shows no device at any address, nothing is connected to the camera port the device
tree was built for. The most common cause is the other camera connector:
- **Raspberry Pi 5** (the only board here with a second camera port of the same kind, CAM0 and
  CAM1): if the user chose "Not sure" in common/3 §Hardware questions, switch to the other port
  and deploy again from common/4 §JSON without asking (tell the user, and update `hw.connector`
  in the state). Do this only once: if the other port fails the same way, continue below.
- Otherwise use AskUserQuestion (header: "Camera port") to tell the user that nothing answers on
  `<port>` and ask: **Try the other port (Recommended)** (RPi 5 only: deploy again from common/4
  §JSON with the other port) | **I checked the hardware, retry** (reboot and check again) |
  **Stop**. On boards with one camera connector, leave out the first option and say which
  connector the device tree was built for.
- If both ports were tried (or the board has only one), tell the user to check the cable
  orientation and seating, the power supply of the deserializer board, and the EV kit checks of
  common/3 §EV kit checks. Do not suspect CFG levels or I2C addresses while nothing at all
  answers on the bus.

If `i2cdetect` shows devices but the deserializer probe failed with `-121`, see the
orchestrator's §Troubleshooting (wrong I2C address).
