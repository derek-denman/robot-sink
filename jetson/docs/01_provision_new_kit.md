# 01 - Provision a Brand New Kit

This runbook covers both common first-day scenarios:

- A) Kit arrives pre-flashed on NVMe (common for Waveshare kits)
- B) Kit is blank or you want a clean reflash

Hardware assumptions for this doc:

- Waveshare IO-BASE-B family baseboard
- Orin NX 16GB module
- Included 256GB NVMe SSD installed
- Included AW-CB375NF Wi-Fi/BT card installed with antennas attached

Important:

- The Orin NX module itself does not include onboard storage in this kit.
- System boot/rootfs is on NVMe.

## A) Kit Arrives Pre-Flashed

1. Assemble for first power-on.

- Install both Wi-Fi antennas on the AW-CB375NF card before RF use.
- Connect HDMI + keyboard + mouse (or prepare headless ethernet).
- Use a stable PSU that matches the kit input spec (do not undervolt).

2. First power-on.

- Power the kit.
- Wait for first boot wizard or login prompt.

3. Determine whether image is pre-configured.

- If Ubuntu first-run wizard appears, continue with your own user creation.
- If it boots directly to login, the vendor image may already have a user.
- Check any vendor card/sticker/manual for default credentials, then rotate password immediately.

4. Complete first boot setup.

- Set locale/timezone.
- Connect network (ethernet preferred for first updates).
- Confirm NVMe rootfs:

```bash
lsblk
findmnt /
```

5. Enable SSH.

```bash
sudo systemctl enable --now ssh
hostname -I
```

6. Continue with `03_first_boot_checklist.md`.

## B) Not Pre-Flashed (or Reflash Required)

1. Prepare host machine.

- Use Ubuntu 20.04 or 22.04 host (physical machine preferred; VM possible with proper USB passthrough).
- Install NVIDIA SDK Manager.

2. Put Waveshare board into recovery mode.

- Power off the Jetson.
- Short `FC REC` to `GND` on the IO-BASE-B.
- Connect USB-C data cable from Jetson board to host.
- Apply power while recovery short is in place.

3. Flash with SDK Manager.

- In SDK Manager, select Jetson Orin NX module + board profile.
- Select target storage as NVMe.
- Choose JetPack version compatible with your board/module support matrix.

4. Finish flash.

- Wait for flash completion.
- Remove `FC REC` short/jumper.
- Reboot and complete Ubuntu first boot wizard.

5. Verify post-flash basics.

```bash
lsblk
findmnt /
sudo systemctl enable --now ssh
```

6. Continue with `03_first_boot_checklist.md`.
