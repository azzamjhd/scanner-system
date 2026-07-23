# Jetson Nano P3450 PENS WiFi Auto-Login

Goal: run `pens_wifi_login.sh` automatically after boot and whenever the Jetson loses internet through campus WiFi, while preserving Tailscale SSH access.

Target platform:

- Jetson Nano P3450
- JetPack 4.6 / Ubuntu 18.04
- NetworkManager
- Tailscale for SSH access
- PENS `eepiswlan` captive portal

## Important Safety Notes

1. Do not restart NetworkManager remotely unless you have physical access or another working access path.
2. Keep `tailscaled` enabled before testing WiFi automation.
3. The script must check internet through the WiFi interface, not the default route. Otherwise Tailscale, ethernet, or USB tethering may make the script think internet is working while `eepiswlan` is still captive.
4. The script contains a plaintext password. Store it root-owned with `700` permissions.

## 1. Copy the Script to the Jetson

If the script already exists on the Jetson:

```bash
sudo install -m 700 -o root -g root /home/azzam/.hermes/scripts/pens_wifi_login.sh /usr/local/sbin/pens_wifi_login.sh
```

If the script is on another PC:

```bash
scp /home/azzam/.hermes/scripts/pens_wifi_login.sh jetson:/tmp/pens_wifi_login.sh
ssh jetson
sudo install -m 700 -o root -g root /tmp/pens_wifi_login.sh /usr/local/sbin/pens_wifi_login.sh
```

Verify:

```bash
sudo ls -l /usr/local/sbin/pens_wifi_login.sh
```

Expected permission pattern:

```text
-rwx------ root root ... /usr/local/sbin/pens_wifi_login.sh
```

## 2. Find the Jetson WiFi Interface

Run:

```bash
nmcli -t -f DEVICE,TYPE,STATE device status
```

Common names:

```text
wlan0
mlan0
```

Set the interface name used below. Example uses `wlan0`.

Test captive portal status through WiFi only:

```bash
curl -s --max-time 8 --interface wlan0 \
  http://connectivitycheck.gstatic.com/generate_204 \
  -o /dev/null -w "%{http_code}\n"
```

Meaning:

| Code | Meaning |
|---|---|
| `204` | Internet works through WiFi |
| `200` / `302` | Captive portal likely active |
| `000` | WiFi/DNS/routing failure |

## 3. Patch the Script to Check Through WiFi

Edit:

```bash
sudo nano /usr/local/sbin/pens_wifi_login.sh
```

Find this function:

```bash
check_net() {
  curl -s --max-time 8 http://connectivitycheck.gstatic.com/generate_204 \
    -o /dev/null -w "%{http_code}" 2>/dev/null
}
```

Replace it with:

```bash
WIFI_IFACE="${WIFI_IFACE:-wlan0}"

check_net() {
  curl -s --max-time 8 --interface "$WIFI_IFACE" \
    http://connectivitycheck.gstatic.com/generate_204 \
    -o /dev/null -w "%{http_code}" 2>/dev/null
}
```

If your interface is not `wlan0`, either change the default in the script or set it in the systemd service later.

Optional: use a persistent log file instead of `/tmp`.

Change:

```bash
LOG_FILE="/tmp/pens_wifi_login.log"
```

To:

```bash
LOG_FILE="/var/log/pens_wifi_login.log"
```

Create the log file:

```bash
sudo touch /var/log/pens_wifi_login.log
sudo chmod 600 /var/log/pens_wifi_login.log
```

## 4. Verify Tailscale Starts on Boot

Run:

```bash
sudo systemctl enable --now tailscaled
systemctl status tailscaled --no-pager
tailscale status
```

Do not continue remote testing if Tailscale is not working.

## 5. Create the systemd Service

Create `/etc/systemd/system/pens-wifi-login.service`:

```bash
sudo tee /etc/systemd/system/pens-wifi-login.service >/dev/null <<'EOF'
[Unit]
Description=PENS WiFi captive portal auto-login
Wants=network-online.target
After=NetworkManager.service network-online.target tailscaled.service

[Service]
Type=oneshot
Environment=WIFI_IFACE=wlan0
ExecStart=/usr/bin/flock -n /run/pens-wifi-login.lock /usr/local/sbin/pens_wifi_login.sh
TimeoutStartSec=90
EOF
```

If the Jetson WiFi interface is not `wlan0`, edit this line:

```ini
Environment=WIFI_IFACE=wlan0
```

Example for `mlan0`:

```ini
Environment=WIFI_IFACE=mlan0
```

## 6. Create the systemd Timer

Create `/etc/systemd/system/pens-wifi-login.timer`:

```bash
sudo tee /etc/systemd/system/pens-wifi-login.timer >/dev/null <<'EOF'
[Unit]
Description=Run PENS WiFi captive portal auto-login periodically

[Timer]
OnBootSec=45s
OnUnitActiveSec=2min
AccuracySec=20s
Persistent=true
Unit=pens-wifi-login.service

[Install]
WantedBy=timers.target
EOF
```

Enable and start the timer:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pens-wifi-login.timer
```

Run the login service once manually:

```bash
sudo systemctl start pens-wifi-login.service
```

Check status:

```bash
systemctl status pens-wifi-login.service --no-pager
journalctl -u pens-wifi-login.service -n 100 --no-pager
```

Check the script log:

```bash
tail -n 100 /tmp/pens_wifi_login.log
```

If you changed the log path:

```bash
sudo tail -n 100 /var/log/pens_wifi_login.log
```

## 7. Add NetworkManager Dispatcher Hook

The timer recovers every 2 minutes. This dispatcher hook runs the login immediately after WiFi comes up.

Create `/etc/NetworkManager/dispatcher.d/90-pens-wifi-login`:

```bash
sudo tee /etc/NetworkManager/dispatcher.d/90-pens-wifi-login >/dev/null <<'EOF'
#!/bin/bash
IFACE="$1"
STATE="$2"

[ "$STATE" = "up" ] || exit 0

# Only trigger for the WiFi interface.
[ "$IFACE" = "wlan0" ] || exit 0

systemctl start pens-wifi-login.service >/dev/null 2>&1 &
EOF
```

If your WiFi interface is not `wlan0`, edit this line:

```bash
[ "$IFACE" = "wlan0" ] || exit 0
```

Make it executable:

```bash
sudo chmod +x /etc/NetworkManager/dispatcher.d/90-pens-wifi-login
```

Do not restart NetworkManager over SSH unless you have physical fallback.

## 8. Verify Timer and Service

Check timer state:

```bash
systemctl is-enabled pens-wifi-login.timer
systemctl list-timers pens-wifi-login.timer --no-pager
```

Expected:

```text
enabled
pens-wifi-login.timer ... pens-wifi-login.service
```

Check current service logs:

```bash
journalctl -u pens-wifi-login.service -n 100 --no-pager
```

Check logs from current boot:

```bash
journalctl -u pens-wifi-login.service -b --no-pager
```

## 9. Safe Reboot Test

Only run this if Tailscale is confirmed working:

```bash
sudo reboot
```

After reconnecting through Tailscale:

```bash
systemctl status pens-wifi-login.timer --no-pager
journalctl -u pens-wifi-login.service -b --no-pager
tailscale status
```

## 10. Manual Recovery Commands

Run portal login manually:

```bash
sudo systemctl start pens-wifi-login.service
```

Watch service logs:

```bash
journalctl -fu pens-wifi-login.service
```

Watch script log:

```bash
tail -f /tmp/pens_wifi_login.log
```

If using `/var/log`:

```bash
sudo tail -f /var/log/pens_wifi_login.log
```

Check WiFi-only internet status:

```bash
curl -s --max-time 8 --interface wlan0 \
  http://connectivitycheck.gstatic.com/generate_204 \
  -o /dev/null -w "%{http_code}\n"
```

## 11. Troubleshooting

### Script says internet is OK but WiFi is still captive

Cause: check is going through the wrong interface.

Fix: verify `WIFI_IFACE` in the service:

```bash
systemctl cat pens-wifi-login.service
```

Then test manually:

```bash
curl -s --max-time 8 --interface wlan0 \
  http://connectivitycheck.gstatic.com/generate_204 \
  -o /dev/null -w "%{http_code}\n"
```

### Service never runs after boot

Check timer:

```bash
systemctl status pens-wifi-login.timer --no-pager
systemctl list-timers --all | grep pens
```

Reload and re-enable:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pens-wifi-login.timer
```

### Login fails but portal is reachable

Check log:

```bash
tail -n 100 /tmp/pens_wifi_login.log
```

Possible causes:

- Wrong username/password
- Captive portal session limit reached
- Portal URL changed
- WiFi not connected
- DNS picked a bad `iac32.pens.ac.id` portal backend

### Tailscale SSH disappears

Do not restart NetworkManager remotely. Use physical access if possible.

Check after recovery:

```bash
sudo systemctl status tailscaled --no-pager
tailscale status
ip route
nmcli device status
```

## Final Setup Summary

Use both mechanisms:

1. `pens-wifi-login.timer` for periodic recovery every 2 minutes.
2. NetworkManager dispatcher hook for immediate login after WiFi reconnect.
3. `--interface wlan0` in connectivity checks to avoid false positives from Tailscale or other routes.
4. `tailscaled` enabled at boot to preserve SSH access.
