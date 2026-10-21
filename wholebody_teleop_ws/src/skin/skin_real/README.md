# skin real

Wifi/usb connection to skin.
Extracts raw skin information and configuration and publishes two ros topics

### Setup the network in 2.4 GHz mode:

Install the driver
```bash
git clone https://github.com/lwfinger/rtl8852bu.git
cd rtl8852bu
make ARCH=arm64 
sudo make install
```

Check the network adaptors
```bash
ip link show
```

Setup the AP
```bash
sudo nmcli connection add type wifi ifname wlxa0ad9f7a2a93 con-name InternalHotspot autoconnect yes ssid PM01SkinNet mode ap

sudo nmcli connection modify InternalHotspot wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify InternalHotspot wifi-sec.psk "robots4ICS!"

sudo nmcli connection modify InternalHotspot wifi-sec.group ccmp
sudo nmcli connection modify InternalHotspot wifi-sec.pairwise ccmp

sudo nmcli connection modify InternalHotspot ipv4.method shared ipv4.addresses 192.168.7.1/24

sudo nmcli connection modify InternalHotspot 802-11-wireless.band bg

sudo nmcli connection up InternalHotspot

sudo iw dev wlxa0ad9f7a2a93 set power_save off
sudo rfkill block bluetooth
```

### Determine the skin IPs

```bash
ip neigh show dev wlxa0ad9f7a2a93
```