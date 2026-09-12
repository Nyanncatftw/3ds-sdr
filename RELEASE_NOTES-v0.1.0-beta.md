# 3DS-SDR v0.1.0-beta — first public beta

This is the first public beta release of 3DS-SDR. It is based on internal development build **v21.13d**.

## Highlights

- New Nintendo 3DS acts as the scanner/DSP/audio/UI client for a networked RTL-SDR.
- Conventional scanner banks and channel memories with explicit SCAN/HOLD behavior.
- NFM and WBFM receive.
- Spectrum scope and full-screen waterfall sharing one FFT/smoothing path.
- Startup `rtl_tcp` server selection with remembered IP address.
- Master PLAY/STOP, battery status, FIFO/load/memory diagnostics.
- Automatic reconnect/recovery and nonblocking `rtl_tcp` command transport.
- Includes the duplicate-channel loader fix from internal build v21.13d.

## Important beta note

NFM is functional and intelligible but is still being tuned for clarity and level relative to dedicated handheld receivers. Feedback with side-by-side receiver comparisons is especially useful.

## Building

This source release does not contain a newly compiled `.3dsx`. Build with a current devkitPro/devkitARM + libctru environment using `make`.

## Upgrade/configuration

Existing scanner configuration remains at:

`sdmc:/3ds/3ds-sdr/scanner-v21.cfg`

The remembered `rtl_tcp` server is stored separately at:

`sdmc:/3ds/3ds-sdr/server.cfg`

## License

MIT. See `LICENSE`.
