# Architecture notes

3DS-SDR deliberately keeps the RF front end and user-facing receiver separate.

```text
RTL-SDR + rtl_tcp host
        |
        | TCP I/Q (240 ksps, unsigned 8-bit I/Q)
        v
New Nintendo 3DS
        |
        +-- raw-IQ FIFO / pacing
        +-- tuner control over rtl_tcp
        +-- NFM/WBFM demodulation
        +-- scanner state machine / memories / squelch
        +-- FFT smoothing -> scope / waterfall
        +-- audio FIFO / NDSP output
        `-- framebuffer UI / controls
```

## Transport

The socket is nonblocking. rtl_tcp tuner commands use a reliable short-send loop rather than assuming one `send()` call always writes the complete five-byte command. Reconnect logic restores the selected server/session state.

When master STOP is active, the TCP stream continues to be drained so stale IQ does not accumulate. IQ is discarded rather than admitted to DSP until PLAY resumes.

## Scanner

HOLD is an explicit mode and must not advance automatically. Manual bank/channel selection remains available while held. Scan eligibility is shared between traversal and diagnostics.

## Display analysis

The spectrum and waterfall share one 256-point FFT and the same smoothed magnitude array. Waterfall rendering stores successive display rows; it does not create a separate analysis/DSP path or change RF span.

## Configuration

Scanner configuration and server selection are intentionally separate:

- `scanner-v21.cfg` — banks, channels, scanner/VFO settings
- `server.cfg` — last confirmed rtl_tcp IPv4 address

This keeps the existing scanner configuration format backward-compatible.
