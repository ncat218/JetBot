On the JetBot, use NetworkManager interactively; do not place the password in source:

```bash
nmcli device wifi list
nmcli device wifi connect "<SSID>" password "<PASSWORD>"
nmcli connection show
hostname -I
```

NetworkManager stores a connection profile and reconnects after reboot. Verify after reboot with `nmcli device status`. Use `python3 scripts/test_wifi.py` for read-only status.

