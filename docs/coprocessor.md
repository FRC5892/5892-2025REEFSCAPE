# Coprocessor installation
- [ ] install debian
  - use entire disk
  - install ssh server, no desktop environment
  - leave root password blank
  - set up user account
    - username: photonvision
    - password: FRC5892
- [ ] install photonvision
```bash
sudo apt install -y curl
wget -q https://raw.githubusercontent.com/PhotonVision/photon-image-modifier/main/install.sh -O ./install.sh
chmod +x ./install.sh
sudo ./install.sh
``` 
- [ ] setup Battery Tracking
  - program
    ```bash
    # sudo wget -O /opt/batteryTracking/BatteryTracking.jar https://github.com/FRC5892/BatteryTracking/releases/download/1.0/BatteryTracking-linuxx64.jar
    ```
  - service 
    ```bash
    # wget -O /etc/systemd/system/batteryTracking.service https://raw.githubusercontent.com/FRC5892/BatteryTracking/refs/heads/main/batteryTracking.service
    # systemctl enable batteryTracking
    # systemctl start batteryTracking
    ```
  - change team number in config if needed `sudo nano /etc/systemd/system/batteryTracking.service`
- [ ] setup Read only file system
```bash
# sudo apt install -y overlayroot
```
  - edit `/etc/overlayroot.conf`
    - change `overlayroot=""` to `overlayroot="tmpfs:swap=1,recurse=0"`
  - create `/etc/grub.d/500_usb_selector`
```bash
#!/bin/sh
cat <<'EOF'
search --no-floppy --fs-uuid --set usbswitch 38b90f0c-fa22-4123-aa8c-f9c57e6991c3

if [ "$usbswitch" ] ; then
set default=3
else
set default=0
fi
EOF
```
  - make it executable
```bash
sudo chmod +x /etc/grub.d/500_usb_selector
```
  - update grub
```
- [ ] setup static ip
  - something like this, change ``enp2s0`` to what is currently there 
> [!NOTE]  
> This will remove all internet access, like any other part of the robot. Make sure to install everything else first.
> to resume ssh access, the laptop might need to be plugged into a functional radio
```
auto enp2s0
iface enp2s0 inet static
    address 10.58.92.11
    netmask 255.255.255.0
``` 
# TODO: everything else!
  