# Changelog

## v0.1.0-beta scanner hotfix — 2026-09-13

- Replaced the fragile first public scanner acquisition behavior with the hardware-tested NFM scan path from the dev11 line.
- The live NFM scan detector no longer adaptively subtracts a centered carrier as DC, preventing a continuously keyed channel from opening briefly and then disappearing.
- The live NFM detector remains authoritative through CHECK / RECEIVE / DELAY and uses close hysteresis/debounce so CARRIER mode stays on an active transmission and resumes after carrier drop.
- NFM now uses a recency-first IQ FIFO: about 50 ms startup, 100 ms target, and a 200 ms hard queue ceiling. When necessary, the oldest stale IQ is discarded instead of preserving seconds of obsolete RF history. WFM keeps its deeper continuity-first buffering.
- Scan acquisition keeps detector history across SETTLE instead of resetting again immediately before CHECK.
- Scanner retunes drain already-queued socket IQ and apply a 250 ms NFM retune quarantine before acquisition. This intentionally slows scanning, but prevents delayed samples from the previous channel being attributed to the frequency currently shown on screen.
- Removed the separate **Scope on Hit** behavior. RADIO / SCOPE / WATERFALL is now the persistent display selection; idle scanning keeps the radio/status screens visible, and the selected visualization appears on RECEIVE or HOLD.
- The existing scanner config remains backward-compatible; the old Scope on Hit field is accepted and ignored.
- NFM demodulation/voice filtering and WFM DSP are otherwise unchanged.
- The repository prebuilt binary is refreshed with the user-compiled, hardware-tested hotfix build.

## v0.1.0-beta — first public release

- First public beta of 3DS-SDR.
- Based on internal development build **v21.13d**.
- Public releases use semantic versioning; the historical `v21.x` entries below preserve the pre-public development lineage.

This file preserves the development history leading to the current public beta. Earlier entries may use internal development-version terminology.

# 3DS-SDR v21.13d

### v21.13d
- Fix config-load duplication of hard-coded first-run memories by zero-initializing each bank before loading its saved channels.
- Existing first-run defaults remain unchanged when no scanner config exists.

### v21.13c

- NFM voice conditioning now uses a 2nd-order ~300 Hz Butterworth high-pass in place of the previous one-pole ~250 Hz stage, improving rejection of discriminator/DC rumble and sub-audible signalling.
- Added ~+4.1 dB (x1.60) NFM post-filter makeup gain before the existing soft limiter; IQ filtering, decimation, discriminator scaling, de-emphasis and LPF remain unchanged.
- Top radio status line shows live main-loop LOAD plus allocator memory as both used and free RAM (standard heap + linear heap, MiB) alongside the existing IQ/AUDIO FIFO percentages.
- Scanner, squelch, rtl_tcp, waterfall/scope, startup server selection and config formats are unchanged.

### v21.13a

- Added startup rtl_tcp server selection before any connection attempt.
- Last confirmed server IP is remembered in `sdmc:/3ds/3ds-sdr/server.cfg` and preloaded on the next launch.
- Server entry uses a D-pad digit-adjuster UI; A confirms/connects and START exits.
- Port remains fixed at 1234.
- Automatic reconnects use the selected server instead of reverting to the compiled default.
- Existing `scanner-v21.cfg` format and compatibility are unchanged.

### v21.13

- New-channel creation now flows directly from the confirmed channel-name keyboard into the existing Edit Memory Frequency editor for that newly added channel.
- Renaming an existing channel is unchanged and still returns normally to the channel menu.
- No scanner, DSP, squelch, rtl_tcp, config-format, or existing frequency-editor behavior was changed.

### v21.12

- Waterfall now fills the complete selected 3DS screen.
- Waterfall history increased from 180 to 240 FFT rows so full-height display uses real successive spectrum rows rather than vertically stretching history.
- RF span, FFT size, smoothing and DSP pipeline are unchanged.

### v21.11

Feature release: X now cycles **normal -> scope -> waterfall -> normal**. The waterfall reuses the existing 256-point FFT and smoothed spectrum data used by the scope; it adds only a rolling display-history renderer. The master PLAY icon is also corrected to a true right-pointing green triangle. Existing scanner, rtl_tcp, NFM/WFM DSP, squelch, battery, config format, and transport behavior are unchanged.

### v21.10g
- Added B master PLAY/STOP in normal radio view. STOP keeps rtl_tcp connected and drained, but pauses IQ DSP, audio, and scanner timing; PLAY resumes from fresh live IQ.
- Added top-banner framebuffer transport icon: stop square while running, play triangle while paused.
- Added top-right PTMU battery indicator with five coarse bars: green at high/full levels, yellow in the middle, red at the last/critical level, and orange while charging.

v21.10g retains the v21.10d/e NFM squelch and narrowband-audio fixes and adds the master receiver transport and battery banner indicators.

- NFM squelch metrics are now taken from raw 240 ksps IQ before the existing 63-tap NFM channel FIR.
- NFM detector coefficients/time constants were rescaled for 240 ksps; the NFM demod/audio DSP chain itself is unchanged.
- Main status shows effective SQL and marks a channel override with `OVR`.
- Diagnostics show global SQL, effective SQL, override state, squelch state, and the actual audio-gate state.
- rtl_tcp remains 10.0.0.15:1234 and scanner config remains scanner-v21.cfg.

# 3DS-SDR v21.10c

v21.10c is a surgical scanner-memory/diagnostics update over v21.10b.

- Eligibility diagnostics now show eligible/total memories plus eligible/current-bank memories.
- Eligibility counting calls the same `scanner_channel_eligible()` predicate used by scanner traversal.
- Duplicate cleanup can target the current bank or all banks.
- Cleanup considers frequency + mode duplicates, preserves the first entry, compacts arrays, clamps live/edit cursors, and saves only when removals occur.
- The existing add-channel exact-duplicate rejection behavior is unchanged.
- Config format/path, DSP, rtl_tcp, scanner state machine, renderer architecture, and controls are unchanged.

# 3DS-SDR v21.10b

## NFM simplex audio overhaul

v21.10b replaces the old lightweight NFM receive path with a proper channelized voice chain while leaving the WFM broadcast path unchanged.

NFM now uses:

- 240 ksps raw complex IQ input, same rtl_tcp rate as before
- 63-tap complex low-pass channel filter (~12 kHz cutoff)
- filtered 5:1 decimation to 48 ksps complex IQ
- phase-difference FM discriminator at 48 ksps
- ~250 Hz speech high-pass / discriminator DC cleanup
- 75 us de-emphasis to reduce harsh pre-emphasized highs and hiss
- ~4.2 kHz final voice low-pass
- mild soft limiting for discriminator spikes

The old NFM path used a five-sample boxcar before demodulation and then sent essentially raw discriminator audio to the speaker. The new chain is intended to make simplex and land-mobile voice cleaner, less hissy, and more scanner-like while improving adjacent-channel rejection.

WFM processing, scanner behavior, HOLD logic, memory handling, duplicate-channel fixes, UI, and rtl_tcp behavior are unchanged from v21.10a.

**Build note:** devkitARM is required to compile.

## Duplicate-channel / keyboard-return fix

- Channel creation is now transactional: cancelling the software keyboard does not create a channel.
- After the software keyboard closes, all controls are ignored until every key has been released once. This prevents the A-button confirmation from retriggering Add Channel / Save VFO.
- A successful Add Channel / Save VFO moves the menu cursor away from the add action.
- Exact duplicate memories are rejected while still allowing same-frequency channels that intentionally have different names/settings.
- SYSTEM now includes **Remove Exact Duplicates** to safely compact existing matching frequency/mode/name/settings channel memories within each bank.

All v21.10 radio, scanner, squelch, DSP, UI, and rtl_tcp behavior is otherwise retained.

## Top-screen NFM/WFM indicator

The top status screen now shows the active demodulation mode directly on the
frequency line, without entering settings.

Examples:

    103.300000 MHz  WFM
    146.520000 MHz  NFM

This applies both to normal memory channels and VFO mode.

No scanner, HOLD, squelch, DSP, audio, rtl_tcp, reconnect, memory, or control
behavior was changed from v21.8.

## Top-screen status layout

This revision changes only the UI hierarchy.

The top screen now emphasizes what the scanner is doing rather than the center
frequency.

A centered banner appears at the top:

    ------ SCANNING ------
    ------ RECEIVE -------
    -------- HOLD --------
    ------ HOLD RX -------
    ------- VFO HOLD -----
    -------- VFO RX ------

Below the banner are:

- bank number/name,
- channel number/name,
- current channel frequency,
- squelch state + SQL,
- RF / noise / margin,
- scan resume mode,
- hit/status information.

The frequency remains visible on the top screen but is intentionally smaller.

The bottom screen keeps the large frequency display and controls, so the
frequency is still easy to read at a glance.

No scanner, squelch, DSP, audio, reconnect, rtl_tcp, VFO, or memory behavior was
changed from v21.7.

## Hard HOLD mode

HOLD is now an explicit operating mode instead of being represented only by a
false `scanning` flag.

Pressing A while scanning now immediately enters `SCANNER_RUN_HOLD` and:

- cancels SETTLE / CHECK / RECEIVE / DELAY scan state,
- prevents every automatic `scanner_next()` call from changing channels,
- remains centered on the current channel indefinitely,
- continues updating squelch on the held channel,
- allows normal squelched audio or Hold Audio MONITOR behavior,
- still allows manual bank/channel changes while held.

Pressing A again explicitly enters `SCANNER_RUN_SCAN`.

`scanner_next()` itself now refuses to move when HOLD is active. This is a
second guard in addition to the scanner state machine, so a stale CHECK/DELAY
state cannot move the receiver after HOLD has been selected.

Manual channel/bank changes while held retune to the selected memory but leave
the scanner in HOLD rather than creating a pending SETTLE scan state.

## UI state

The operating state is no longer hidden when squelch opens:

- `HOLD` = held, squelch closed
- `HOLD RX` = held, squelch open
- `SCAN` = scanning, no active signal
- `RECEIVE` = scanning and stopped on an active signal
- `VFO HOLD` / `VFO RX` = VFO equivalents

All v21.6 channel-selective squelch work and the previous dynamic scan,
rtl_tcp command, reconnect, memory-editor, and scope changes are retained.

## Channel-selective squelch redesign

The old squelch detector used broadband raw-IQ power across the full rtl_tcp
sample bandwidth. That meant an off-center signal or general RF energy could
open squelch even when the selected channel itself was idle.

v21.6 moves squelch measurement into the paced DSP path.

### New detector

For each DSP complex sample the squelch detector now:

- removes slow I/Q DC so the RTL center spike does not masquerade as a carrier,
- applies a complex low-pass centered on the tuned channel,
- measures selected center-channel energy,
- measures rejected/adjacent energy,
- forms a gain-independent center-vs-adjacent margin in dB.

The display fields now mean:

- RF = center-channel energy
- NF = rejected/adjacent energy
- M = RF - NF channel-quality margin

This makes the detector far less sensitive to AGC level and signals elsewhere
inside the 240 kHz rtl_tcp span.

### SQL calibration

SQL 0..100 now maps approximately to:

    -1.5 dB .. +18 dB required channel-quality margin

with 2.5 dB hysteresis.

Therefore SQL 100 is intentionally very strict and should no longer remain open
on a visually idle/off-center channel merely because broadband RF power is high.

### Retune behavior

The squelch metric is reset on every actual retune, along with the existing
IQ/audio/DSP pipeline reset. It waits for a short batch of fresh center-channel
samples before making a new open/closed decision.

All v21.5b scan fixes, v21.5a reliable rtl_tcp command handling, reconnect
recovery, independent memory editing, and bottom-only frequency editing are
retained.

## Dynamic bank/channel scan fix

The scan-next engine has been rewritten to walk the actual current memory
database instead of relying on the older cursor/wrap logic.

Each scan advance now considers every physical memory slot across every bank and
accepts a channel only when:

- its bank is enabled,
- the channel is scan-enabled,
- the channel is not temporarily avoided.

Newly added banks are enabled by default.
Newly added channels are scan-enabled by default.

Therefore banks/channels added during the current session are immediately part
of the scan set; restarting the application is not required.

The radio status/debug screen now shows `ELIG n`, the number of channels the
scanner currently considers eligible. This makes scan-list problems visible
without guessing.

All v21.5a reliable rtl_tcp command handling and v21.5 reconnect/editor changes
are retained.

## Critical rtl_tcp command fix

The rtl_tcp socket is nonblocking. Earlier builds incorrectly assumed a single
`send()` would always transmit the complete 5-byte rtl_tcp control command.

That can fail transiently or partially. When it did, the 3DS changed local DSP
state and reset its buffers, but the rtl_tcp server could remain centered on the
old frequency. This matches the observed symptom of hearing a change while the
server remained at 103.3 MHz.

v21.5a now:
- loops until all 5 control bytes are sent,
- handles partial TCP writes,
- retries EAGAIN / EWOULDBLOCK / EINPROGRESS / EALREADY,
- uses a 150 ms command deadline,
- records failed command attempts internally.

This fix applies to frequency, sample-rate and gain commands because all of them
use the same `send_rtl_cmd()` transport.

All v21.5 reconnect recovery, independent memory editing, and bottom-only
frequency editing are retained.

## Transport recovery

`ECONNRESET` (errno 104), EOF, `ENOTCONN`, `EPIPE` and `ETIMEDOUT` no longer
immediately terminate the scanner.

The receiver now attempts to:
1. close the stale rtl_tcp socket,
2. reconnect to 10.0.0.15:1234,
3. restore the current live center frequency/mode/sample rate/gain,
4. reset IQ/audio/DSP/FFT state,
5. resume normal reception.

The 3DS software keyboard also deliberately replaces the rtl_tcp connection
when it closes. The keyboard may block and miss audio; stale/catch-up audio is
discarded.

Transient nonblocking states remain nonfatal:
- EAGAIN
- EWOULDBLOCK
- EINPROGRESS
- EALREADY

## Frequency editor display

The top screen no longer turns into a duplicate frequency editor.

While editing a stored memory:
- TOP = live RF/scanner status and actual receiver center frequency
- BOTTOM = digit frequency editor for the stored memory

While editing the VFO:
- TOP = live RF/scanner status
- BOTTOM = VFO digit editor
- A applies the VFO frequency to the receiver

The lower editor uses readable 3x 5x7 digits and shows the edit bank/channel
context near the bottom.

## Retained from v21.4

- independent edit bank/channel cursor
- stored channel editing does not retune the SDR
- nonblocking digit editor
- clean retune pipeline resets
- 45 ms stale-IQ drain window
- VFO save to selected edit bank
- HOLD Audio SQUELCHED/MONITOR
- scanner modes Delay / Carrier / Time
- selectable scope screen and overlay

### v21.13c diagnostics
- Renamed the top-screen CPU metric to `LOAD` because it represents measured main-loop busy time, not total system CPU utilization.
- `MEM` now reports both allocator-used and allocator-free memory (`MEM UxxM FyyM`), combining the standard malloc heap with the linear heap. This avoids the misleading OS application-region free counter, which can read 0 after libctru reserves the heaps at startup.
