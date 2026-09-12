# Reddit post draft

## Suggested title — 3DS/homebrew audience

**I turned a New Nintendo 3DS into a networked RTL-SDR scanner — NFM/WFM, scanning, DSP, scope + waterfall run on the 3DS**

## Suggested title — amateur radio / SDR audience

**3DS-SDR: a New Nintendo 3DS conventional scanner/DSP client for rtl_tcp (NFM/WFM, squelch, memories, FFT + waterfall)**

## Body

I've been building a receive-only SDR/scanner homebrew app for the **New Nintendo 3DS**.

The architecture is:

```text
RTL-SDR -> Archer C7/OpenWrt running rtl_tcp -> Wi-Fi -> New 3DS
```

The important part is that the 3DS is **not just a remote display**. The New 3DS handles the scanner logic, NFM/WFM demodulation and audio DSP, squelch, channel banks/memories, scope/waterfall, controls, and audio locally. The OpenWrt/RTL-SDR side is basically the networked RF front end.

Current features include:

- analog NFM and WBFM
- conventional scanning with banks/memories
- explicit SCAN / HOLD / HOLD RX states
- DELAY / CARRIER / TIME resume modes
- global/per-channel squelch
- VFO
- temporary avoid / monitor
- 256-bin scope
- full-screen waterfall using the same FFT/smoothing path
- startup rtl_tcp server selection and reconnect recovery
- battery, IQ/audio FIFO, main-loop load, and memory diagnostics
- persistent scanner configuration

The first public beta is **v0.1.0-beta** (based on internal build **v21.13d**) and the source is MIT licensed.

NFM is now understandable and usable, but I'm still refining its audio quality against conventional handheld receivers, so feedback from SDR/radio people would be particularly useful.

I built this because the New 3DS is actually a pretty interesting little radio control head: dual screens, physical controls, Wi-Fi, audio output, and enough CPU to do the scanner/DSP work locally.

If anyone tests it, I'd especially like feedback on NFM audio, scan behavior, and performance on real hardware.

**GitHub:** https://github.com/Nyanncatftw/3ds-sdr

*Receive only. Use radio-monitoring software in accordance with the laws that apply where you live.*
