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
    - no fully made jar, just copy the files
  - service 
    ```bash
    # wget -O /etc/systemd/system/batteryTracking.service https://raw.githubusercontent.com/FRC5892/BatteryTracking/refs/heads/main/batteryTracking.service
    # systemctl enable batteryTracking
    # systemctl start batteryTracking
    ```
# TODO: everything else!
  