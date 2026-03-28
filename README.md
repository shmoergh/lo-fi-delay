# lo-fi-delay

A Brain module application built with the brain-sdk.

## Build

```bash
./build-firmware.sh
```

## Flash

Flash one of the generated UF2 files to your Brain module by holding the BOOTSEL button while connecting it to your computer, then copy the .uf2 file to the mounted drive:

- `lo-fi-delay-pico.uf2` (RP2040)
- `lo-fi-delay-pico-2.uf2` (RP2350)

## Development

This project includes brain-sdk as a git submodule. To update the SDK:

```bash
cd brain-sdk
git pull origin main
cd ..
git add brain-sdk
git commit -m "Update brain-sdk"
```
