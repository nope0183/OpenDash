# Firmware assets for the OpenDash web flasher

The manifests write one complete factory image at offset `0x0`:

| File | Flash layout |
| --- | --- |
| `OpenDash-v2.2.2-cyd_2usb-factory.bin` | bootloader `0x1000`, partitions `0x8000`, boot_app0 `0xE000`, app `0x10000` |
| `OpenDash-v2.2.2-cyd_ili9341-factory.bin` | same, with the ILI9341 app |

Individual components are kept for transparency and manual flashing:

| File | Offset | Origin |
| --- | ---: | --- |
| `bootloader.bin` | `0x1000` | PlatformIO build for `cyd_2usb`, v2.2.2 |
| `partitions.bin` | `0x8000` | `partitions_ota.csv`, v2.2.2 |
| `boot_app0.bin` | `0xE000` | Arduino-ESP32 framework (initial OTA selection) |
| `OpenDash-v2.2.2-cyd_2usb.bin` | `0x10000` | `pio run -e cyd_2usb`, v2.2.2 |
| `OpenDash-v2.2.2-cyd_ili9341.bin` | `0x10000` | `pio run -e cyd_ili9341`, v2.2.2 |

The `v2.2.2` application images contain the Material You UI
(`src/ui_theme.h`), the ARGB renderer (`src/led_effects.h`) and NeoPixelBus
hardware output for the LED strip (RMT on ESP32-C3). The app image hashes are:

```text
7a65aa66be85af57ea5ab4157ab4b930488a5712db066a1958e6955004e50ad2  OpenDash-v2.2.2-cyd_2usb.bin
57d7f2a053ba531c600fb52444f2cb754e4ec3e61fbee5e998f309c578aac6db  OpenDash-v2.2.2-cyd_ili9341.bin
```

The factory images were assembled with:

```text
esptool.py --chip esp32 merge_bin \
  -o OpenDash-v2.2.2-cyd_2usb-factory.bin \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 bootloader.bin 0x8000 partitions.bin \
  0xe000 boot_app0.bin 0x10000 OpenDash-v2.2.2-cyd_2usb.bin
```

The second display variant shares the bootloader and partition table; only the
application image differs. SHA-256 sums for all files are in
[`SHA256SUMS.txt`](SHA256SUMS.txt).

The binaries are distributed under the same terms as the upstream project.
