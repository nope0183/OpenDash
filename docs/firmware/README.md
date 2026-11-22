# Firmware assets for the OpenDash web flasher

The manifests write one complete factory image at offset `0x0`:

| File | Flash layout |
| --- | --- |
| `OpenDash-v2.2.5-cyd_2usb-factory.bin` | bootloader `0x1000`, partitions `0x8000`, boot_app0 `0xE000`, app `0x10000` |
| `OpenDash-v2.2.5-cyd_ili9341-factory.bin` | same, with the ILI9341 app |

Individual components are kept for transparency and manual flashing:

| File | Offset | Origin |
| --- | ---: | --- |
| `bootloader.bin` | `0x1000` | PlatformIO build for `cyd_2usb`, v2.2.5 |
| `partitions.bin` | `0x8000` | `partitions_ota.csv`, v2.2.5 |
| `boot_app0.bin` | `0xE000` | Arduino-ESP32 framework (initial OTA selection) |
| `OpenDash-v2.2.5-cyd_2usb.bin` | `0x10000` | `pio run -e cyd_2usb`, v2.2.5 |
| `OpenDash-v2.2.5-cyd_ili9341.bin` | `0x10000` | `pio run -e cyd_ili9341`, v2.2.5 |

The `v2.2.5` application images contain the saved-key authentication fix: the
app switches to the stored key after the PING response before sending PAIR.
The images also contain the Material You UI
(`src/ui_theme.h`), the ARGB renderer (`src/led_effects.h`), NeoPixelBus
hardware output for the LED strip, and persistent Ninebot pairing-key reuse.
The app image hashes are:

```text
398b575feb9f97aac5f1fa4b8c7a80e7c96b2f020f0b63a93e03dd9ca25d0122  OpenDash-v2.2.5-cyd_2usb.bin
9cc16281a2b9fcb12a4cbdf3d093bb17571294bd4e95d7c6386d36a7bcddcc80  OpenDash-v2.2.5-cyd_ili9341.bin
```

The factory images were assembled with:

```text
esptool.py --chip esp32 merge_bin \
  -o OpenDash-v2.2.5-cyd_2usb-factory.bin \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 bootloader.bin 0x8000 partitions.bin \
  0xe000 boot_app0.bin 0x10000 OpenDash-v2.2.5-cyd_2usb.bin
```

The second display variant shares the bootloader and partition table; only the
application image differs. SHA-256 sums for all files are in
[`SHA256SUMS.txt`](SHA256SUMS.txt).

The binaries are distributed under the same terms as the upstream project.
