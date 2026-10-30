# Common 1: Find the board

Find the board on the network and log in, so the user does not have to type the IP address or
the credentials. Run by the orchestrator's "Yes" path (also after flashing). At the end, the
state holds `board_ip`, `board_user` and `board_pw` (orchestrator §State).

The images these skills know announce their hostname over mDNS (`<hostname>.local`):
- the ADI GMSL Raspberry Pi images and the SD cards prepared by rpi/3: `analog-rpi-gmsl`, login
  `analog` / `analog`;
- an NXP medium prepared by nxp/0–4: `analog-imx-gmsl`, the hostname nxp/0 §Discovery settings
  set, stored in the state as `expected_hostname`, with the login stored as `board_user` /
  `board_pw`;
- NXP's own Yocto images (not changed by these skills): the Yocto machine name of the board,
  `imx95-19x19-lpddr5-frdm-pro` (the NXP Yocto default for the FRDM-IMX95-PRO; other boards:
  their `MACHINE` name, which nxp/0 §Read the image reads from the image's `/etc/hostname` when an
  image is available), usually with the login `root`.

The board may also run another image (for example a customer's own Raspberry Pi OS or Yocto
build) with a different hostname, login and kernel. mDNS then finds nothing, but §MAC search
still finds boards on the same LAN; the login is then asked in §Ask the user, and the platform
file's driver check (common/3 §Driver check) checks whether the kernel has the GMSL drivers.

Skip §Resolve by name and §MAC search if the user already gave the board's IP address or
hostname (store it as `board_ip`), and go to §Login with it.

## Resolve by name

Hostnames to try, in this order:
1. `expected_hostname` from the state, if set (a medium this run prepared: try it first, and
   only it during the first minute).
2. The hostname of the board type the user named ("my Raspberry Pi" → `analog-rpi-gmsl`, "my
   i.MX95 board" → the NXP names), then the others: `analog-rpi-gmsl`, `analog-imx-gmsl` (set by
   nxp/0), `imx95-19x19-lpddr5-frdm-pro` (NXP Yocto default for the FRDM-IMX95-PRO; for another
   i.MX board, the Yocto `MACHINE` name of its release).

For each hostname:
- **WSL** (`/proc/version` contains `microsoft`, or `$WSL_DISTRO_NAME` is set) and **Windows**:
  WSL cannot resolve `.local` names itself, so ask Windows:
  ```bash
  powershell.exe -NoProfile -Command "Resolve-DnsName <hostname>.local -Type A -ErrorAction SilentlyContinue | Select-Object -ExpandProperty IPAddress"
  ```
  (in WSL, use `/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe` if
  `powershell.exe` is not on the PATH).
- **Linux:** `getent ahostsv4 <hostname>.local | head -1`, or
  `avahi-resolve -4 -n <hostname>.local`.
- Also try `<hostname>-2.local`: mDNS renames duplicate hostnames, so if it resolves, more
  than one board is on the network. Then use AskUserQuestion to let the user pick one.

If nothing resolves, retry every 10 s for up to 2 minutes (a freshly flashed board takes a
while to boot; an NXP board's first boot from a new medium can take longer, as it resizes the
root file system). Images without an mDNS responder (`avahi-daemon`) never resolve: go on with
§MAC search. If nothing has resolved after the first minute, or the user says the board is
already up (for example its desktop shows on the monitor), run §MAC search now instead of
waiting for the rest of the retries: mDNS often does not cross between Wi-Fi and wired clients
of the same router (WSL host on Wi-Fi, FRDM-IMX95-PRO on Ethernet with avahi running:
`analog-imx-gmsl.local` never resolved, while §MAC search found the board on its first read).
Check the host's link type on WSL/Windows with
`powershell.exe -NoProfile -Command 'Get-NetIPAddress -AddressFamily IPv4 | ForEach-Object { "{0} {1}" -f $_.InterfaceAlias, $_.IPAddress }'`
(an alias such as `Wi-Fi`), and skip virtual interfaces (VPN tunnels, ZeroTier `zt*`, Tailscale)
when choosing the subnets to sweep.

## MAC search

If mDNS still fails (some routers do not forward it, for example between Wi-Fi and wired
clients, and some images have no mDNS responder), look for a known board MAC address on the
host's local subnets:

| Board | OUIs (first 3 bytes of the MAC address) |
|-------|------------------------------------------|
| Raspberry Pi 5 | `88:A2:9E`, `2C:CF:67`, `D8:3A:DD` |
| Raspberry Pi 4 | `DC:A6:32`, `E4:5F:01`, `28:CD:C1` |
| Older Raspberry Pi models | `B8:27:EB` |
| NXP (i.MX evaluation boards such as the FRDM-IMX95-PRO) | `00:04:9F` |

Ping every address of each `/24` subnet the host is on (from `ip -4 -o addr show`, ignoring
`127.*`, `169.254.*`, `/32` addresses, addresses on `lo`, such as WSL's internal
`10.255.255.254`, and virtual networks such as ZeroTier `zt*` or Tailscale `tailscale*`) to fill
the neighbour table, then search it:
```bash
for i in $(seq 1 254); do (ping -c1 -W1 <subnet>.$i >/dev/null 2>&1 &); done; sleep 3
```
- **WSL and Windows**: the neighbour table to read is the Windows one:
  ```bash
  powershell.exe -NoProfile -Command 'Get-NetNeighbor -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { $_.LinkLayerAddress -match "^(88-A2-9E|2C-CF-67|D8-3A-DD|DC-A6-32|E4-5F-01|28-CD-C1|B8-27-EB|00-04-9F)" } | ForEach-Object { "{0} {1}" -f $_.IPAddress, $_.LinkLayerAddress }'
  ```
- **Linux**: `ip -4 neigh | grep -iE '88:a2:9e|2c:cf:67|d8:3a:dd|dc:a6:32|e4:5f:01|28:cd:c1|b8:27:eb|00:04:9f'`

Replies to the ping sweep keep arriving for a few seconds, so the first read of the neighbour
table can miss the board. Read the table 3 times, 5 seconds apart, and collect every match. A
board that is switched off can still be listed with the state `Stale`, so a match is only a
candidate, not proof that the board is up. The OUI says which board type a candidate is (try
that type's logins first in §Login). Some NXP boards ship with a MAC address from another range
or a random one: if the user has an NXP board and no `00:04:9F` candidate answers, ask for the
IP address (§Ask the user). Boards with several Ethernet ports (the FRDM-IMX95-PRO) can show two
`00:04:9F` candidates whose MACs differ only in the last byte: that is one board, not two; use
the `Reachable` one, and confirm with §Login (`ip -4 -o addr show` lists the port that is up).

Check each candidate with §Login to find out whether it is a board that answers SSH. Drop a
candidate when SSH fails with `No route to host` or a timeout: that is an old entry for a board
that is now off. If more than one board answers, use AskUserQuestion to let the user pick one.
If no candidate answers, run the ping sweep and the 3 reads again once (the board may still be
booting) before asking in §Ask the user.

- The MAC address search only works on the same LAN: over a VPN or a virtual network (for
  example ZeroTier or Tailscale) mDNS usually does not resolve and the neighbour table shows
  virtual MAC addresses. Then ask for the IP address in §Ask the user.
- If no board is found, ask in §Ask the user. A board connected directly to the PC's Ethernet
  port gets no address (there is no DHCP server, and the board may not fall back to a
  `169.254.*` address): tell the user to connect it to the same router as the PC.

## Login

Log in and identify the board with one command. Credentials to try, in this order, and at most
these (recent OpenSSH versions refuse connections from a host for a while after several failed
logins, `penalty: failed authentication` on the board; if connections are reset, wait 30
seconds before the next attempt):
1. `analog` / `analog` (the ADI images and the RPi cards prepared by rpi/3).
2. The credentials in the state (`board_user` / `board_pw`), if set and different (an NXP medium
   prepared by nxp/0, or a login the user gave earlier in this run). If the state has them and
   the candidate is the board this run prepared (`expected_hostname`, or an NXP OUI after an NXP
   flash), try them first.
3. For an NXP candidate (OUI `00:04:9F` or an NXP hostname) only: one attempt with `root` and an
   empty password (`-o PreferredAuthentications=password,keyboard-interactive` and
   `sshpass -p ''`). Unverified: NXP images may allow root without a password on the console,
   but SSH and `sshpass` usually refuse an empty password. If it fails, tell the user to set a
   root password on the board's serial console (`passwd`) and to enter it (§Ask the user,
   Credentials).
4. Otherwise ask (§Ask the user).

Each candidate password goes into `/tmp/gmsl_pw_board` first (default logins: `(umask 077; printf '%s\n' analog > /tmp/gmsl_pw_board)`; a password the user typed: the same once, the only command that contains it), and `sshpass -f` reads it:
```bash
sshpass -f /tmp/gmsl_pw_board ssh -o StrictHostKeyChecking=no \
  -o ConnectTimeout=10 <user>@<ip> -- \
  "hostname; uname -r; tr -d '\0' < /proc/device-tree/model; echo; tr '\0' ' ' < /proc/device-tree/compatible; echo; test -d ~/Workspace/pyv4l2 && echo pyv4l2; test -d /opt/gen_gmsl_dts && echo gen_gmsl_dts"
```
(on Windows, use common/0 §Windows adaptations instead of `sshpass`).

- If SSH reports `REMOTE HOST IDENTIFICATION HAS CHANGED`, the board's SD card or eMMC was most
  likely reflashed (always the case after this run flashed it): tell the user, run
  `ssh-keygen -R <ip>` (and `ssh-keygen -R <hostname>.local` if a name was used), and log in again.
- `Permission denied` right after a boot can come before logins work (common/5 §Reboot): on a
  board that has just booted, wait 30 seconds and try the same credentials once more before
  trying the next ones.
- If every login is rejected, keep the IP address and ask for the credentials in §Ask the user,
  then run the same command with them.
- Some images allow root logins over SSH only with a key, or not at all (`PermitRootLogin`):
  if `root` is rejected with the right password, tell the user, and ask for another user or for
  them to allow it on the board's console.

When a login works, store it:
```bash
python3 -c "
import json
state = json.load(open('/tmp/gmsl_camera_state.json'))
state['board_ip'] = '<ip>'
state['board_user'] = '<user>'
state['board_pw'] = '<pw>'
json.dump(state, open('/tmp/gmsl_camera_state.json', 'w'), indent=2)
"
```
Keep the output of the identify command for common/2 (hostname, kernel release, model,
compatible, `pyv4l2`, `gen_gmsl_dts`).

## Ask the user

Ask only for what the steps above could not determine, in one AskUserQuestion call:
- **IP Address** (header: "IP Address"), if the board was not found: offer common subnet
  examples (192.168.0.x, 192.168.1.x, 10.0.0.x); the user types the actual IP (or hostname) in
  "Other". Say in the question how to read it on the board: on its serial console or a monitor
  with `ip -4 addr show`, or in the router's list of connected devices.
- **Credentials** (header: "Credentials"), if every login was rejected: "analog / analog"
  (default) | "root (no password)" (NXP Yocto images; only if it was not tried yet: SSH usually
  refuses it, then set a password with `passwd` on the board's console and use Custom) | Custom
  (user types user:pass in "Other").

Then run §Login with the answers. If the board does not answer at the address given, check it
with `ping -c 3 <ip>` and tell the user what failed (no ping: wrong address, board off, or another
network; ping but SSH refused: no SSH server running, or a firewall), and use AskUserQuestion to
let them retry, change the IP, or stop (then run the orchestrator's §Cleanup).
