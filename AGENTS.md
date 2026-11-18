# AGENTS.md: working on PlutoSAT

A handover for engineers and coding agents. It records what is not obvious
from the code: how to build and test, how the test board is set up and must
be treated, and what went wrong before. Details live in the component READMEs
(linked below); keep this file short and current.

## What this is

PlutoSAT is a QO-100 ground station on the **fishball-sdr-7020** board
(Zynq-7020 xc7z020clg400-2 + AD9363, 1 GB DDR3). This repo is a fork of ADI's
plutosdr-fw with:

- **PL cores** (SystemVerilog, each with a bit-true Python model):
  - `dvbs2/`: DVB-S2 TX and RX;
  - `nbiq/`: narrowband IQ channels;
  - `iqtx/`: narrowband IQ transmitter (memory -> DAC) and the TX source mux;
  - `fft/`: spectrum FFT with result DMA;
  - `awgn/`: receive noise source for self-tests.
- **station** (submodule, repo `M0PZH/PlutoSAT-station`):
  - the Rust daemon `plutosatd` and CLI `plutosat` (`station/rust`);
  - the web UI (`station/web`, built on HamUI `M0PZH/HamUI`, pinned by commit in `station/web/package.json`);
  - the API reference `station/docs/api.md`;
  - `station/ios/`, a native iOS client added by another contributor (2026-11-05).
- **Board files**: `boards/fishball/` (HDL project, device tree, u-boot/Linux configs, Debian deployment and packages).

Remotes: superproject `git@github.com:M0PZH/PlutoSAT.git` (master), station
`git@github.com:M0PZH/PlutoSAT-station.git` (main). Pushing needs the user's
GitHub SSH access (an agent forwarded from their machine). `~/.ssh/id_rsa` on
the build host is not accepted. If a push fails, commit locally and say so.

## Conventions

- **Language:** everything in the repos (docs, comments, commit messages, TOML docs) is in English. Conversation with the user is in Chinese.
- **Commits:** commit verified work in logical stages and push without being asked. Push the station submodule first, then the superproject pointer. Never commit build outputs. Before committing, check for CJK: `git diff --cached | grep -P '[\x{4e00}-\x{9fff}]'`.
- **Registers:** the TOML files in `regs/` are the single source of truth. After editing one, run `python3 regs/gen.py` and `python3 regs/gen.py --check`. RTL, C tools and Rust all read the generated files. The Rust build reads `regs/` via `PLUTOSAT_REGS`.
- **Core VERSION:** bump it in the core's `*_axi.sv` read mux, in its `regs/*.toml` (value and doc string) and in the sim test that checks it. Software checks minimum versions, e.g. RX ≥ 0x0001_0200, two slots ≥ 0x0002_0000.
- **Files the user picks up by hand** (BOOT.bin, devicetree.dtb, …) go into `build/` (ignored by git, the plutosdr-fw build directory), e.g. `build/fishball-debian-boot/`, never into the repo top directory or a temp directory.

## Build and test

Host: Ubuntu 26.04 (Python 3.14: the old `dvbs2/.venv` is broken; make one with `python3 -m venv --without-pip --system-site-packages V && python3 -m pip --python V/bin/python install pytest cocotb==1.9.2`), 96 cores. Full builds share `flock /home/peter/plutosdr-fw-wt/.vivado-build.lock`. Only Vivado/Vitis **2026.1** is installed
(`/opt/Xilinx/2026.1`), and there is no `xsct`. The disk is nearly full (about 15–20 GB free):

- delete the Vivado build outputs of worktrees once you are done;
- check `df -h /` before starting a full build.

| What | Command |
|---|---|
| Full PL build | `source /opt/Xilinx/2026.1/Vivado/settings64.sh && ADI_IGNORE_VERSION_CHECK=1 make -C boards/fishball/hdl` (a fresh shell has no `vivado` on PATH → Error 127; ADI's scripts want 2025.1 without the env var). Output: `boards/fishball/hdl/fishball.runs/impl_1/system_top.bit`, `fishball.sdk/system_top.xsa`, `timing_impl.log`. About 40 min. Run one at a time. |
| FSBL | `scripts/create_fsbl.sh build/system_top.xsa build/sdk build/sdk/fsbl/Release/fsbl.elf` (sdtgen + empyro + arm-none-eabi-gcc) |
| u-boot | `make -C u-boot ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zynq_fishball_defconfig all` (board files installed by `boards/fishball/install.sh`) |
| Bitstream for the FPGA manager | `bootgen -image bit.bif -arch zynq -process_bitstream bin -w` with `all:{ plutosat.bit }` |
| dvbs2 model tests | `cd dvbs2/model && ../.venv/bin/python -m pytest test_model.py` |
| dvbs2 RX sims (Verilator) | `cd dvbs2/sim/rx && ../../.venv/bin/python -m pytest -q test_rx.py test_rx2.py test_llrq.py` (about 25 min for everything; `-k` to narrow it) |
| Station Rust | `cd station/rust && PLUTOSAT_REGS=$PWD/../../regs PLUTOSAT_FW=$PWD/../.. PLUTOSAT_PYTHON=$PWD/../../dvbs2/.venv/bin/python PLUTOSAT_GOLDEN_REQUIRED=1 cargo test --workspace --release` |
| Board binaries | `cargo build --release --target armv7-unknown-linux-musleabihf -p plutosatd -p plutosat-cli` (static, rust-lld) |
| RADE C sources | `station/rust/crates/plutosat-rade/fetch-sources.sh` once (pinned, checksummed, into the gitignored `third_party/`; needs clang + llvm-ar to build). Without them plutosatd builds without RADE (a cargo warning; the recorder's RADE mode reports itself unavailable) and the RADE tests skip; `PLUTOSAT_RADE_REQUIRED=1` makes both an error. `deploy.sh --station` fetches them and requires them. |
| Web | node is not on PATH in non-interactive shells: `export PATH=/home/peter/.local/share/nvm/v22.11.0/bin:$PATH`; `cd station/web && npm ci && npm test && npm run build` |
| Debian packages | `boards/fishball/debian/build-debs.sh` → `build/debs/`: `plutosat-firmware`, `plutosat-tools`, `plutosatd` and, when `build/system_top.xsa` is the XSA of `build/system_top.bit`, the optional `plutosat-boot` (FSBL built in `/tmp/plutosat-fsbl`, PS setup checked against `ps7_init.ref`; `BOOT_REF=` a known-good BOOT.bin to compare; `UBOOT_ELF=` if `u-boot/` is not checked out). Needs node on PATH (see Web) and the RADE sources. `deploy.sh --debs root@board` builds and installs the first three. Details: `boards/fishball/debian/README.md`. |
| Web e2e | `station/web/test/e2e/{e2e,nb,spectrum,tx,stutter}.mjs` (headless Chromium via playwright-core) against `plutosatd --sim`. Use non-default ports, because other daemons may hold the defaults. |

Notes:

- **Golden tests:** these Rust tests compile the real C tools (`dvbs2/sw/*.c`) and compare them with the Rust ports. With `PLUTOSAT_GOLDEN_REQUIRED=1` a C compile error fails the test instead of skipping it.
- **Verilator flakes:** Verilator 5.020 sometimes aborts under heavy parallel load ("destroy locked Thread Pool"); rerun.
- **`hdl/` in a git worktree:** the ADI `hdl/` tree must be a real copy (`rsync -a --exclude .git`). A symlink breaks the build, because Tcl `file normalize` resolves it and then picks up the main checkout's `dvbs2/`.
- **External code:** the host's auto-mode classifier blocks cloning and building external code; ask the user to run such commands with `!`.

## The test board (pluto2)

`ssh -o HostKeyAlias=pluto2.cbg-1.relu.network root@10.66.3.168` (DHCP on the LAN; it was 10.66.3.169, and the name `pluto2.cbg-1.relu.network` stopped resolving from the build host in Nov 2026 — the host keys are pluto2's), Debian 12 on the SD card, kernel 5.15.

### Hardware setup

- TX1 is cabled to RX1 through a 40 dB attenuator. Any TX attenuation is allowed: at 0 dB about −33 dBm reaches RX1.
- AD9363 RX1 = IIO `voltage0` = plutosatd `rx0`. `rx1` (AD9363 RX2) only sees the TX by crosstalk, about 32 dB lower. A proper rx1 test needs the cable on RX2A, or a splitter.
- On a cable loop, set RX LO = TX LO; otherwise LO-mixing spurs appear at multiples of |RX LO − TX LO|.
- **Avoid 2.4 GHz for cable tests:** a nearby WiFi AP's beacons (every 102.4 ms) leak into the loop and corrupt about 1.4 ms of symbols. Use 2.32 GHz, as `/root/try/try-rf.toml` does.

### Boot chain

As set up on 2026-11-05:

- **BOOT.bin** = FSBL (built from our XSA) + the PlutoSAT bitstream + the fishball u-boot. The PL is ours from power-on.
  - Its PS init (MIO, PLLs, clocks, DDR, peripherals) matches the board's original FSBL register for register.
  - The original image is kept on the card as `BOOT.bin.pre-bitload`.
- **uEnv.txt** = the original plus one `uenvcmd=` line that boots Debian (uImage + devicetree.dtb). The fishball u-boot's own `sdboot` would boot the buildroot .itb instead. See `boards/fishball/debian/README.md`.
- **The fishball u-boot reads its environment from QSPI (0x100000)** whenever the CRC matches. A leftover environment from another firmware once broke the boot (`modeboot` unset, `run` failing at the prompt); the user fixed it at the console.
- **devicetree.dtb**:
  - reserved memory for the DMA buffers and the 2 MB LLR queue;
  - UIO nodes;
  - no IIO streaming DMAs;
  - no `mwipcore@43c00000` (not in our PL).
  
  Every PL node in the tree must exist in the bitstream when the PL is loaded before the kernel.
- **Since 2026-11-18 the boot files come from the `plutosat-boot` package** (`/root/boot-undo.txt` on the board has the latest undo: `plutosat-boot-rollback`, or `BOOT.bin.plutosat-prev` from the card).
- **Backups on the FAT partition:** `*.plutosat-prev`, `*.pre-plutosat-VERSION-DATE`, `*.pre-plutosat`, `*.pre-axiports`, `*.pre-llrq`, `*.pre-bitload`, `*.pre-bootbin`.

### Software on the board

Installed as Debian packages since 2026-11-18 (`deploy.sh --debs`; the files the older `deploy.sh` had put there by hand are in `/root/pre-deb/`):

- **`plutosatd`** (package `plutosatd`; systemd `plutosatd`, enabled): `/usr/bin/plutosatd -c /etc/plutosat/plutosatd.toml`, serving the web UI on `https://board/` (0.0.0.0:443, `tls = "auto"` self-signed, because WebCodecs and the camera need a secure context; port 80 redirects). pluto2's config is the user's own (kept by the package, transmit enabled there); a fresh installation ships with `[tx]` / `[nbtx] enabled = false`.
- **`plutosat-pl-load [FILE]`** (package `plutosat-firmware`, `/usr/sbin`): reprograms the PL at run time from `/lib/firmware`. It stops the plutosatd service around the reload. Default `plutosat.bit.bin` (the package's); other images live alongside it, e.g. `plutosat-rx141.bit.bin`. **`plutosat-pl-load.service`** (enabled) runs it at every boot, before plutosatd (BOOT.bin already has the same bitstream; reloading it is harmless): a bitstream loaded by hand is replaced at the next boot, unless `/etc/default/plutosat-pl-load` sets `PLUTOSAT_BITSTREAM=FILE`. Don't replace `/lib/firmware/plutosat.bit.bin` by hand: the package owns it.
- **C tools in `/usr/bin`** (package `plutosat-tools`, cross-built, dynamically linked for glibc 2.36): `dvbs2-rx`, `dvbs2-tx`, `dvbs2-selftest`, `nbiq`, `iqtx`, `rx-awgn`, `ts-prbs`, `fft-spectrum`. The 1.x versions are in `/usr/local/bin/pre-rx20`.
- **`plutosat-boot`** (optional package, writes BOOT.bin / devicetree.dtb / uEnv.txt with a preflight and `plutosat-boot-rollback`) is **not** installed on pluto2: its boot files are already equivalent (tested there: "nothing to do"). `device-tree-compiler` came with that test.
- **User trial setup** `/root/try/try.sh MIN lb|rf`: plutosatd on 127.0.0.1:18080 in digital loopback or over the cable. It stops the service while it runs and starts it again afterwards. The user also runs their own instance there (`qo100.toml`, 0.0.0.0:443 TLS): don't touch `/root/try` except try.sh/try*.toml.
- **NTP:** systemd-timesyncd. The board has no RTC.

### Rules

These keep the board safe and shared.

- **ssh:** use `ssh -o BatchMode=yes … < /dev/null`.
- **Killing processes:** use `pkill -x NAME`. Never use `pkill -f PATTERN` where the pattern also appears in your own command line. Don't `rm` paths built from shell variables.
- **The lock:** every sequence that touches shared state (AD9363 settings, loopback, PL cores/DMA, a running plutosatd) runs inside `flock -w 900 /run/plutosat-board.lock sh -c '…'`, held under about 10 min. Stop the plutosatd service first if you drive the cores yourself.
- **Leave nothing transmitting:**
  - `plutosat regs dvbs2_tx CTRL 0`;
  - BIST loopback back to 0 (`/sys/kernel/debug/iio/iio:deviceN/loopback`);
  - restore what you changed (RX LO 744.5 MHz, slow_attack, TX attenuation 89.75 dB).
- **Changes that affect booting** (BOOT.bin, uEnv.txt, devicetree.dtb, anything the PL needs at boot) can make the board unreachable; only someone at the SD card can fix that. Before such a change:
  - back up every file;
  - check the DT's PL nodes against the `ad_cpu_interconnect` addresses in `boards/fishball/hdl/system_bd.tcl`;
  - compare the PS init with the known-good FSBL;
  - tell the user how to undo it from the SD card, before rebooting.
  
  On 2026-11-05 a boot with the PL loaded by u-boot and `mwipcore` still in the DT hung, and the card had to be pulled.
- **Who may do what:** only the coordinator reboots, reloads bitstreams or changes boot files, unless a brief explicitly allows it. Subagents get board permissions only as stated in their brief.

## System facts

- **AXI ports:**

  | Port | Use |
  |---|---|
  | M_GP0 | registers |
  | S_GP0 | DVB-S2 RX DMA |
  | S_GP1 | DVB-S2 TX DMA |
  | HP0 | `iqtx` ring reader (1 MB @ 0x3E700000) |
  | HP1 | RX LLR queue |
  | HP2 | nbiq |
  | HP3 | FFT result DMA (`axi_fft_dma` 0x7C4A0000) |

  The HP ports run 64-bit. The ADI FIR and the IIO streaming DMAs are removed.
- **Register bases:**
  - `dvbs2_tx` 0x7C500000, `dvbs2_rx` 0x7C510000, `nbiq` 0x7C520000, `rx_awgn` 0x7C530000, `fft` 0x7C540000, `iqtx` 0x7C550000;
  - TX DMA 0x7C480000, RX DMA 0x7C490000;
  - `axi_ad9361` 0x79020000, I2C 0x41600000, SPI 0x7C430000.
- **Reserved DDR:**

  | Address | Size | Use |
  |---|---|---|
  | 0x3F000000 | 16 MB | TX |
  | 0x3EF00000 | 1 MB | RX DMA |
  | 0x3EB00000 | 4 MB | nbiq |
  | 0x3EA00000 | 1 MB | FFT |
  | 0x3E800000 | 2 MB | RX LLR queue (UIO map3 of `dvbs2-rx`) |
  | 0x3E700000 | 1 MB | iqtx ring (UIO map1 of `iqtx`) |
- **RX core 2.0** (VERSION 0x0002_0000): two DATV slots.
  - **Register map:** slot bank s at 0x100·s keeps the 1.x offsets; the shared bank is at 0x200 (CTRL 0x210 with slot enables [9:8], SNAP, LLRQ_*).
  - **`LLRQ_BASE` (0x2E0) resets to 0**, which means nothing is decoded: software must write the reserved region's address (plutosatd and dvbs2-rx do).
  - **Combined symbol rate** of both slots: at most about 2.27 MBd; software refuses anything above 2.28 MS/s.
  - **History:** 1.4/1.4.1 PLS-confirmed frame sync (fixed the false lock on repeating frames such as a TS of null packets); 1.4.2 LDPC channel LLR clip ±61.
  - Full design notes: `dvbs2/README.md` ("Receiver", "Two receive slots"), `dvbs2/docs/rx2_design.md`.
- **Utilisation and timing** (master 2026-11-18: RX 2.1.2, nbiq 2.1, iqtx): 47.75k LUT (89.8 %, slices 99.99 %), 136.5 of 140 BRAM tiles, 178 DSP; WNS +0.116 ns at 100 MHz. The FPGA is full: anything new must free space first.
  - Fallbacks already used: the slimmer BCH (RX 2.1.2), the nbiq staging buffer in BRAM. Still available: the recovery items in rx2_design.md §8; relaxing the rx_clk constraint (250 MHz, the real l_clk is 48 MHz at 12 MS/s).
  - A false-path constraint for the slot-enable synchroniser (`en_a`) is still to be added.
- **Station protocol 1.6** (`station/docs/api.md`):
  - one `RxDevice` owner demultiplexes the two slots by packet header bit 55;
  - TX defaults `keyframe_frames` 14 and `queue_max_ms` 800, because hardware H.264/H.265 key frames are 8–20 average frames (the user chose this over measuring them; adds about 400 ms for standard receivers);
  - `ldpc_failed` means not converged, not lost (BCH decides);
  - `rx0`/`rx1` are DATV slots; the narrowband source `nb.source` names an AD9363 channel.

## How the work was organised

What worked:

- **Worktrees:** one coordinator, with subagents in their own git worktrees and branches. For the main repo: `git worktree add ../plutosdr-fw-wt/NAME -b wt/NAME master`. For the station: `git -C station worktree add ../plutosdr-fw-wt/station-NAME -b wt/NAME main`, building with `PLUTOSAT_REGS`/`PLUTOSAT_FW` pointing at the matching main-repo tree. The coordinator reviews, runs the tests, merges, pushes and removes the worktree.
- **Briefs** for subagents state:
  - the goal and what is already ruled out;
  - the files to read;
  - the board rules above;
  - "repo text in English";
  - "commit on your branch, don't push".
- **Board measurements before conclusions.** Useful tools:
  - a 50 ms read-only register poller (`/dev/mem` at 0x7C510000);
  - `station/rust/crates/plutosatd/examples/txclient.rs --au-log` for per-access-unit send/receive times;
  - `examples/scanclient.rs` for lock-time statistics;
  - `web/test/e2e/stutter.mjs` for glass-to-screen freezes.
- **Model and RTL stay bit-true.** Reproduce board findings in the model first (e.g. `dvbs2/model/falselock_experiment.py`), then fix both and add a regression test to both.

## Lessons

- A PLS decode of a header is evidence, not truth. False locks can decode the same wrong symbols consistently. Accept only a confident decode of the *configured* code (or a dummy).
- Before blaming the radio for video stutter, check the TX key-frame budget and the mux queue. Hardware encoders' key frames are 8–20× the average frame.
- At high SNR the LLR gain saturates every |LLR| at 127, above any check message (≤ 62), so a full-scale wrong parity bit never converged; the decoder now clips the channel LLRs to ±61.
- The board's wall clock was wrong until NTP was installed. Use the monotonic stamps (`clock.mono_us` in `/api/info`) for latency.
- `plutosat tx` rejects 333000 S/s (not 12e6/integer); use 333333.

## Status and open items (2026-11-18)

**On master and on the board** (debs 0.39+183, BOOT.bin from `plutosat-boot`):

- RX 2.1.2: two slots, 8PSK and short frames, the pilots-off carrier fix of 2.1.1 (no PLL drive from the headers, lock detector + frequency search), the slimmer BCH;
- nbiq 2.1: eight channels (7 users around the overview on channel 4), FIR taps allocated by `allocate()` (190 taps for 7×48k + 600k), ring base + offset;
- iqtx and narrowband transmit (SSB, CW, FT8/FT4, RADE, tones), board-validated over the cable;
- station protocol 1.15: TX key-frame buffer auto/manual, MODCOD auto by PLS hints, narrowband TX, beacon lock, the background recorder, 8PSK/short, eight nbiq channels, TX1 to the idle attenuation at start;
- the Debian packages (`plutosatd`, `plutosat-firmware`, `plutosat-tools`, `plutosat-boot`).

**Open:**

- **MODCOD auto for 8PSK and short frames:** with a QPSK-normal hypothesis the frame sync tags data as headers and the PLS "confident" test passes on them (metric 160–285 vs about 1950 for a real header), so the hints name random codes. Fixed MODCOD works. A fix (RX 2.1.3) is in progress.
- **Not tested on air:** QO-100 has not been received or transmitted on air.
- **Not tested on the board:** two different simultaneous DVB-S2 signals (simulation only); a proper rx1 RF input; the beacon lock's F1A tone direction.
- **Not started:** the buildroot firmware image with RX, FFT, plutosatd and web; RADE EOO callsign.
- **Dropped:** PL zoom (the 48 kHz zoom FFT stays on the ARM: resolution and rate are bound by the data, and it would starve the shared PL FFT).
- **Known limits:**
  - with sound on, the browser plays about 1.0–1.35 s behind (the audio buffer covers arrival jitter);
  - constellation updates drop to about 2.5 Hz per slot when both slots are viewed;
  - a PL reload under a running plutosatd is not detected (`plutosat-pl-load` stops the service for that reason);
  - the RADE recorder costs about 0.36 of a core per channel;
  - the station's beacon e2e test sometimes fails under a fully parallel `cargo test` (passes alone).
- **Housekeeping:**
  - the 33 kS/s test stream should be re-encoded at ≤ 16.6 kbit/s;
  - the planning document (the user's Claude Doc "software and system plan", in Chinese) is behind this file.
