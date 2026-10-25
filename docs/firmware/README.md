# Firmware assets for the OpenDash web flasher

The manifests write one complete factory image at offset `0x0`:

| File | Flash layout |
| --- | --- |
| `OpenDash-v2.2.0-cyd_2usb-factory.bin` | bootloader `0x1000`, partitions `0x8000`, boot_app0 `0xE000`, app `0x10000` |
| `OpenDash-v2.2.0-cyd_ili9341-factory.bin` | same, with the ILI9341 app |

Individual components are kept for transparency and manual flashing:

| File | Offset | Origin |
| --- | ---: | --- |
| `bootloader.bin` | `0x1000` | PlatformIO build for `cyd_2usb`, v2.2.0 |
| `partitions.bin` | `0x8000` | `partitions_ota.csv`, v2.2.0 |
| `boot_app0.bin` | `0xE000` | Arduino-ESP32 framework (initial OTA selection) |
| `OpenDash-v2.2.0-cyd_2usb.bin` | `0x10000` | `pio run -e cyd_2usb`, v2.2.0 |
| `OpenDash-v2.2.0-cyd_ili9341.bin` | `0x10000` | `pio run -e cyd_ili9341`, v2.2.0 |

The `v2.2.0` application images contain the Material You UI
(`src/ui_theme.h`) and the ARGB renderer (`src/led_effects.h`). The local
`firmware.bin` from the `cyd_2usb` build has the same SHA-256 as the published
application image:

```text
87f8195e85ccc7f2375c0ef48e88dc059b770dc0f97196b2d882baac47852b13
```

The factory images were assembled with:

```text
esptool.py --chip esp32 merge_bin \
  -o OpenDash-v2.2.0-cyd_2usb-factory.bin \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 bootloader.bin 0x8000 partitions.bin \
  0xe000 boot_app0.bin 0x10000 OpenDash-v2.2.0-cyd_2usb.bin
```

The second display variant shares the bootloader and partition table; only the
application image differs. SHA-256 sums for all files are in
[`SHA256SUMS.txt`](SHA256SUMS.txt).

The binaries are distributed under the same terms as the upstream project.
