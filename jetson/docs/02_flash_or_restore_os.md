# 02 - Flash or Restore OS

Use this when:

- You need a clean baseline
- Vendor image is unstable/unknown
- You need a specific JetPack version

## References (Official First)

- NVIDIA SDK Manager: https://developer.nvidia.com/sdk-manager
- SDK Manager docs: https://docs.nvidia.com/sdk-manager/
- Jetson Linux Developer Guide: https://docs.nvidia.com/jetson/

Use Waveshare board-specific guidance only for carrier-board recovery details.

## Host Requirements

- Ubuntu 20.04 or 22.04
- Reliable USB-C data cable
- Stable power supply for Jetson board

## Flash Procedure (SDK Manager)

1. Install SDK Manager on host.
2. Put the board in recovery mode:

- Short `FC REC` + `GND`
- Connect USB-C data to host
- Apply power

3. Open SDK Manager and select:

- Product category: Jetson
- Target: Jetson Orin NX (16GB module)
- JetPack: version compatible with module + IO-BASE-B
- Target storage: NVMe (not eMMC)

4. Start flash and wait for completion.
5. Remove recovery short and reboot Jetson.
6. Complete Ubuntu first boot wizard.

## Restore Notes

- Reflashing overwrites target storage.
- Keep a backup of custom configs before reflashing.
- After restore, rerun this repo's bring-up scripts from `jetson/scripts/`.
