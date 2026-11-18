# PlutoSAT

A QO-100 (Es'hail-2) amateur satellite ground station on a single Zynq-7020 +
AD9363 SDR board: DATV transmit and receive, the narrowband transponder, and
a browser UI, with the signal processing in the FPGA.

The target board is the **fishball-sdr-7020**: xc7z020clg400-2, AD9363,
1 GB DDR3. This repository is a fork of ADI's
[plutosdr-fw](https://github.com/analogdevicesinc/plutosdr-fw); its original
README is kept as [README.plutosdr-fw.md](README.plutosdr-fw.md).

## Features

**DVB-S2 transmit** (`dvbs2/`)
- QPSK 1/4 to 9/10, normal frames, pilots on or off.
- Symbol rates from 33.3 kS/s to 2 MS/s.
- The browser's camera and microphone are encoded with WebCodecs (H.265, H.264 or AV1, with Opus). `plutosatd` muxes them into a constant-rate transport stream that meets TR 101 290 PSI timing.

**DVB-S2 receive** (`dvbs2/`)
- Two receive slots on one RX core. Each slot has its own frequency, symbol rate (33.3 kS/s to 2 MS/s, about 2.27 MBd for both slots together), MODCOD and pilots.
- Frames are found by a frame sync confirmed by PLS decoding, which resists false locks on repeating frames.
- Automatic MODCOD and pilots detection from the PLHEADER.
- LDPC (layered min-sum, shared by both slots through a DDR frame queue) and BCH decoding.
- Low-latency playback in the browser, with audio and video synchronised.

**Narrowband transponder** (`nbiq/`)
- Five independent decimating channels: SSB/CW receivers for several users plus a background decoder.
- FT8/FT4 decoding, both in the daemon and in the browser (WebAssembly).

**Narrowband transmitter** (`iqtx/`)
- Low-rate complex baseband from memory (6 to 300 kS/s), interpolated to 12 MS/s with clean start/stop ramps and a fine NCO: SSB, CW, FT8/FT4 and test tones made in software.
- Shares TX1 with the DVB-S2 transmitter through a glitch-free source switch.

**Spectrum** (`fft/`)
- An FFT in the FPGA (512 to 4096 points, run-time selectable).
- Results reach memory by DMA, at 30 fps and more, for the wideband and narrowband waterfalls.

**Station software** ([`station/`](station), its own repository)
- `plutosatd`, a Rust daemon with a REST/WebSocket API.
- The `plutosat` CLI.
- The web UI, built on [HamUI](https://github.com/M0PZH/HamUI).
- A native iOS client.

**Testing aids**
- A receive noise source (`awgn/`).
- AD9363 digital loopback.
- Command-line tools (`dvbs2-tx`, `dvbs2-rx`, `dvbs2-selftest`, `nbiq`, `iqtx`, …).
- A bit-true Python model for every core, with simulation testbenches.

## Repository layout

| Path | Contents |
|---|---|
| [`dvbs2/`](dvbs2) | DVB-S2 TX and RX cores (RTL, model, sims, C tools); design notes in [`dvbs2/README.md`](dvbs2/README.md) and [`dvbs2/docs/`](dvbs2/docs) |
| [`nbiq/`](nbiq), [`iqtx/`](iqtx), [`fft/`](fft), [`awgn/`](awgn) | Narrowband channels, narrowband transmitter, spectrum FFT, noise source: each with RTL, model, sims, software |
| [`regs/`](regs) | Register maps (TOML, the single source of truth) and the generated headers |
| [`boards/fishball/`](boards/fishball) | Board support: HDL project, device trees, u-boot/Linux configs, buildroot overrides, and deployment to a board running Debian ([`boards/fishball/debian/`](boards/fishball/debian)) |
| [`station/`](station) | Submodule [PlutoSAT-station](https://github.com/M0PZH/PlutoSAT-station): daemon, CLI, web UI, iOS client, [API reference](station/docs/api.md) |
| `hdl/`, `linux/`, `u-boot/`, `buildroot/`, `br2-external/` | Submodules inherited from plutosdr-fw (ADI HDL library, kernel, u-boot, buildroot) |

## Getting started

The board keeps its own Debian 12 system; PlutoSAT goes onto it as Debian
packages (armhf), built on the build host:

| package | contents |
|---|---|
| `plutosat-firmware` | the bitstream, `plutosat-pl-load` and a systemd unit that loads it at every boot; never touches the boot partition |
| `plutosatd` | the daemon, the CLI `plutosat`, the web UI, the systemd service, `/etc/plutosat/plutosatd.toml` (transmitting disabled) |
| `plutosat-tools` | the C tools (`dvbs2-tx`, `dvbs2-rx`, `nbiq`, `iqtx`, `rx-awgn`, `dvbs2-selftest`, ...) |
| `plutosat-boot` | optional and risky: BOOT.bin with the bitstream, the patched device tree, uEnv.txt (see below) |

1. **Build the FPGA design** (Vivado 2026.1):
   ```
   source /opt/Xilinx/2026.1/Vivado/settings64.sh
   ADI_IGNORE_VERSION_CHECK=1 make -C boards/fishball/hdl
   cp boards/fishball/hdl/fishball.runs/impl_1/system_top.bit boards/fishball/hdl/fishball.sdk/system_top.xsa build/
   ```
2. **Build and install the packages** (needs `dpkg-deb`, `fakeroot`, `bootgen`, `dtc`, `arm-linux-gnueabihf-gcc`, `cargo` with the `armv7-unknown-linux-musleabihf` target and `npm`):
   ```
   boards/fishball/debian/deploy.sh --debs --boot root@board   # first time, then reboot the board
   boards/fishball/debian/deploy.sh --debs root@board          # updates
   ```
   `--debs` runs `build-debs.sh` and installs `plutosat-firmware`, `plutosat-tools` and `plutosatd` with apt; by hand: copy `build/debs/plutosat-{firmware,tools}_*.deb` and `plutosatd_*.deb` to the board and `apt install ./*.deb`. The installation prints the admin password once; an existing `/etc/plutosat/plutosatd.toml` is kept as it is.
   `--boot` (once per board) patches the board's own `devicetree.dtb` and `uEnv.txt` (the originals stay as `*.pre-plutosat`): the kernel needs reserved DMA memory and UIO nodes for the cores, and no IIO streaming DMAs. The `plutosat-firmware` package checks the running device tree and warns.
3. **The bitstream** is loaded at every boot by `plutosat-pl-load.service`, whatever BOOT.bin the board has; `plutosat-pl-load` loads it at once. Optionally the `plutosat-boot` package puts it into BOOT.bin (the PL is then ours from power-on), with checks, backups and `plutosat-boot-rollback`; a bad boot file needs someone at the SD card ([`boards/fishball/debian/README.md`](boards/fishball/debian/README.md)).
4. **Open the UI** at `https://board/` (plain `http://board/` redirects there).
   - The daemon serves HTTPS with a self-signed certificate (`tls = "auto"`), because browsers give WebCodecs and the camera only to a secure context.
   - Accept the certificate once, or install it from `https://board/cert.crt`.
5. **To transmit**, set `enabled = true` in `[tx]` (DATV) and / or `[nbtx]` (narrowband) of `/etc/plutosat/plutosatd.toml`, then `systemctl restart plutosatd`. Off, the daemon refuses to arm.

A self-test without any RF: `ssh root@board dvbs2-selftest -p` (stop the service first: `systemctl stop plutosatd`).

## Documentation

- [`AGENTS.md`](AGENTS.md): handover notes for developers and coding agents:
  - build and test commands;
  - the test board and its rules;
  - system facts and lessons learned;
  - open items.
- [`station/docs/api.md`](station/docs/api.md): the station protocol (REST, WebSocket, configuration), with board measurements.
- Component READMEs: [`dvbs2/`](dvbs2/README.md), [`nbiq/`](nbiq/README.md), [`iqtx/`](iqtx/README.md), [`fft/`](fft/README.md), [`awgn/`](awgn/README.md), [`regs/`](regs/README.md), [`boards/fishball/`](boards/fishball/README.md).

## Status

**Working:**
- DVB-S2 TX and two-slot RX, validated in digital loopback and over a cabled RF loop;
- narrowband receive with FT8;
- spectrum;
- the web UI.

**Not yet done:**
- on-air tests via QO-100;
- an IQ transmit path for narrowband SSB/CW;
- 8PSK and higher modulations, and short frames;
- TX power calibration.

See the open items in [`AGENTS.md`](AGENTS.md).

## License

The plutosdr-fw build system and the submodules keep their licenses (see
[`LICENSE.md`](LICENSE.md) and each submodule). The station software is in its
own repository.
