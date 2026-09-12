# 3DS-SDR

**3DS-SDR** turns a **New Nintendo 3DS** into a handheld analog SDR scanner and spectrum display using an RTL-SDR receiver connected over the network with `rtl_tcp`.

The RTL-SDR is the RF front end. The 3DS handles the scanner logic, FM demodulation/DSP, audio, squelch, memories, spectrum scope, waterfall, controls, and UI locally.

> Current public beta: **v0.1.0-beta**
>
> Based on internal development build **v21.13d**.

## Development note

**This entire project was vibe-coded with ChatGPT.** I started this project with no prior coding/programming knowledge. I have directed the design and features, tested the software repeatedly on real New Nintendo 3DS hardware, reported bugs and RF/audio behavior, and iterated with ChatGPT to generate and modify the code.

That means this is an experimental, enthusiast-built project rather than software written through a traditional development background. I am publishing the source openly so others can inspect it, test it, point out mistakes, improve the DSP, and contribute fixes.

## Architecture

```text
RTL-SDR
   |
   v
rtl_tcp host (OpenWrt / Linux / PC)
   |
   |  240 ksps unsigned 8-bit I/Q over TCP
   v
Wi-Fi / LAN
   |
   v
New Nintendo 3DS
   |- scanner / VFO
   |- NFM + WBFM demodulation
   |- audio DSP
   |- squelch
   |- spectrum scope + waterfall
   |- memory banks
   `- dual-screen UI + controls
```

The default `rtl_tcp` port is **1234**. On startup the 3DS presents a server-IP editor and does not connect until you confirm the address. The last confirmed server IP is remembered.

## Features

- Conventional analog scanner with banks and channel memories
- VFO mode
- NFM voice receive
- WBFM broadcast receive
- 256-bin spectrum scope
- Full-screen rolling waterfall using the same FFT/smoothing data as the scope
- Squelch with global and per-channel settings
- DELAY, CARRIER, and TIME scanner resume modes
- Explicit SCAN / HOLD / HOLD RX operating states
- Temporary channel avoid and monitor controls
- Channel/bank editing and persistent configuration
- Duplicate-memory cleanup
- Master PLAY/STOP that pauses DSP/scanning while keeping `rtl_tcp` drained live
- Startup `rtl_tcp` server selection with remembered IPv4 address
- Battery indicator
- IQ/audio FIFO, application LOAD, and allocator-memory diagnostics
- Automatic `rtl_tcp` reconnect/recovery

## Requirements

### 3DS

- New Nintendo 3DS / New Nintendo 3DS XL / New Nintendo 2DS XL
- Homebrew-capable system capable of launching `.3dsx` applications

### SDR side

- RTL-SDR compatible with `rtl_tcp`
- A host that can run `rtl_tcp` (for example OpenWrt, Linux, or a PC)
- The 3DS and `rtl_tcp` server must be mutually reachable over IP

This project was developed with an RTL-SDR attached to an Archer C7 running OpenWrt, but the 3DS client only depends on the standard `rtl_tcp` protocol.

## Running `rtl_tcp`

A typical server command is:

```sh
rtl_tcp -a 0.0.0.0 -p 1234
```

The 3DS sends the sample-rate, frequency, and tuner commands after connecting.

**Security note:** `rtl_tcp` does not provide authentication or encryption. Run it only on a trusted LAN or behind an appropriate private network/VPN; do not expose it directly to the public Internet.

## Build

Prebuilt releases are provided when available. You can also build the current source yourself with devkitPro/devkitARM and libctru.

With a working devkitPro 3DS environment:

```sh
make clean
make
```

Successful compilation creates:

```text
3ds-sdr.3dsx
3ds-sdr.elf
```

Copy `3ds-sdr.3dsx` to a folder on the SD card, for example:

```text
sdmc:/3ds/3ds-sdr/3ds-sdr.3dsx
```

The application stores its runtime configuration in:

```text
sdmc:/3ds/3ds-sdr/scanner-v21.cfg
sdmc:/3ds/3ds-sdr/server.cfg
```

`server.cfg` stores only the last confirmed `rtl_tcp` server address. Scanner configuration remains separate for compatibility with existing installations.

## Startup

1. Start `rtl_tcp` on the SDR host.
2. Launch 3DS-SDR.
3. The remembered server IP is preloaded (first-run fallback: `10.0.0.15`).
4. Adjust the IPv4 address with the D-pad if necessary.
5. Press **A** to connect.

No `rtl_tcp` connection is initiated until the server address is confirmed.

## Controls

### Normal scanner view

| Control | Action |
|---|---|
| **A** | SCAN / HOLD |
| **B** | Master PLAY / STOP |
| **X** | Normal → Scope → Waterfall → Normal |
| **Y** | Menu |
| **D-pad Up/Down** | Previous / next bank |
| **D-pad Left/Right** | Previous / next channel |
| **R** | Temporary avoid |
| **L (hold)** | Temporary monitor |
| **ZL / ZR** | Squelch down / up |
| **SELECT** | Diagnostics |
| **START** | Exit |

### VFO

D-pad Left/Right tunes by the selected step. D-pad Up/Down changes the step size.

## Receiver notes

### NFM

The current NFM path uses:

```text
240 ksps complex IQ
  -> 63-tap complex channel LPF
  -> 5:1 decimation to 48 ksps
  -> phase-difference FM discriminator
  -> 2nd-order ~300 Hz voice HPF
  -> 75 us de-emphasis
  -> ~4.2 kHz voice LPF
  -> ~+4.1 dB makeup gain
  -> soft limiter
  -> 48 kHz audio
```

NFM is usable and intelligible but remains an area of active tuning. Reports comparing the same transmission against a conventional receiver are welcome.

### Scope and waterfall

The scope and waterfall share the same 256-point FFT and smoothed magnitude data. Switching display modes does not change receiver bandwidth or create a second DSP path.

## Configuration and memories

The scanner supports enabled/disabled banks, enabled/disabled channels, temporary avoids, per-channel mode, and optional per-channel squelch overrides. Scan eligibility requires:

```text
bank enabled
AND channel enabled
AND not temporarily avoided
```

Duplicate cleanup compares **frequency + mode**, preserves the first memory, removes later duplicates, compacts the bank, clamps affected indices, and saves the configuration.

## Current limitations

- Receive-only. The 3DS does not transmit RF.
- Analog NFM/WBFM only; no digital voice decoding or trunk tracking.
- `rtl_tcp` is required; the RTL-SDR is not connected directly to the 3DS.
- NFM audio is still being refined and may not yet match a dedicated handheld receiver in clarity.
- Battery level exposed by the 3DS API is coarse rather than a precise percentage.
- This is an independent homebrew project and is not affiliated with Nintendo, Realtek, or the `rtl-sdr` project.

## Legal / responsible use

3DS-SDR is a receive-only general-purpose radio tool. Radio-monitoring laws and restrictions vary by jurisdiction and by service. You are responsible for using the software and connected receiver lawfully. Do not rely on this project for safety-critical or emergency communications.

## Contributing

Bug reports, RF/DSP measurements, reproducible audio comparisons, UI feedback, and small focused patches are welcome. See [CONTRIBUTING.md](CONTRIBUTING.md).

When reporting an RF/audio issue, it is especially useful to include:

- mode (NFM/WBFM)
- frequency/service type
- approximate signal level shown by 3DS-SDR
- whether a conventional receiver heard the same transmission cleanly
- steps to reproduce
- build/version

## License

3DS-SDR is released under the [MIT License](LICENSE).

The project uses devkitPro/libctru when built; those projects retain their own licenses and are not bundled in this source repository.

## Release history

See [CHANGELOG.md](CHANGELOG.md).
