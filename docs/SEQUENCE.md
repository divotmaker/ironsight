# Connection Lifecycle and Message Sequencing

Companion to [WIRE.md](WIRE.md), which covers framing, field encodings, and the
message catalog. This document covers *when* to send *what*: connection lifecycle,
handshake ordering, shot flow, and re-arm logic.

All message types, payload formats, and field encodings referenced here are
defined in WIRE.md. Bus addresses: APP=0x10, DSP=0x40, AVR=0x30, PI=0x12.

---

## 1. Overview

The Mevo+ exposes three TCP services:

| Port | Protocol | Purpose |
| ---- | -------- | ------- |
| 5100 | Binary (this spec) | All radar/config/shot data |
| 1258 | Newline-delimited JSON | Camera trigger/status (GVP) |
| 8080 | HTTP MJPEG | Camera video stream |

Only port 5100 is required for shot data.

**Strictly sequential**: APP always waits for a response before sending the next
message. There is no pipelining. Typical APP reaction time after receiving a
response is <1 ms.

**Total handshake time**: ~1.2 s without retries, ~3.8 s with.

---

## 2. Connection Handshake

Six phases, executed in order. Every exchange within a phase is also sequential.

### 2.1 Phase 1 — DSP Sync (~114 ms)

| Dir | Type | Payload | Typical &Delta; | Notes |
| --- | ---- | ------- | --------------- | ----- |
| APP&rarr;DSP | 0xAA | `[01 01]` (2B) | &mdash; | Status poll |
| DSP&rarr;APP | 0xAA | 129B | 19 ms | Device state (battery, power) |
| APP&rarr;DSP | 0x48 | empty | <1 ms | DSP hardware query |
| DSP&rarr;APP | 0xC8 | 3B: `[02 80 0E]` | 13 ms | dspType=0x80, pcb=14 |
| APP&rarr;DSP | 0x67 | empty | <1 ms | Device info request |
| DSP&rarr;APP | 0xE7 | 76B ASCII | 19 ms | Model, firmware, serial |
| APP&rarr;DSP | 0xFD | `[01 00]` | <1 ms | Product info (&times;3 sub-queries: 0x00, 0x08, 0x09) |
| DSP&rarr;APP | 0xFD | 34B ASCII &times;3 | 10 ms avg | Pi ID, camera model |
| APP&rarr;DSP | 0x21 | empty | <1 ms | TParameters config query |
| DSP&rarr;APP | 0xA0 | 69B | 30 ms | 34 radar parameters |

DSP response latency: 10–30 ms.

### 2.2 Phase 2 — AVR Sync (~144 ms)

| Dir | Type | Payload | Typical &Delta; | Notes |
| --- | ---- | ------- | --------------- | ----- |
| APP&rarr;AVR | 0xAA | `[01 01]` (2B) &times;2 | &mdash; | Status poll (2 rounds) |
| AVR&rarr;APP | 0xAA | 25B &times;2 | 5–9 ms | Tilt, roll, temperature |
| APP&rarr;AVR | 0x67 | empty &times;2 | <1 ms | Device info (&times;2) |
| AVR&rarr;APP | 0xE7 | 75B ASCII &times;2 | 27 ms | Firmware, build date |
| APP&rarr;AVR | 0xBE | `[03 00 00 0C]`, `[03 00 00 0D]` | <1 ms | Param reads (HW/FW version) |
| AVR&rarr;APP | 0xBF | 7B &times;2 | 5–7 ms | Param values |
| APP&rarr;AVR | 0x21 | empty | <1 ms | TParameters config query |
| AVR&rarr;APP | 0xA0 | 69B | 6 ms | 34 radar params (26 non-zero) |
| APP&rarr;AVR | 0xD2 | 10B (sub-cmd 0x03) | <1 ms | Factory cal request |
| AVR&rarr;APP | 0xD3 | 176B | 6 ms | Factory cal constants |
| APP&rarr;AVR | 0xD0 | `[02 00 08]` | <1 ms | IF cal params request |
| AVR&rarr;APP | 0xE3 | text | varies | "DSP: CameraParam Read PASS" |
| AVR&rarr;APP | 0x95 | 3B | varies | ConfigAck (acks 0xD0) |
| AVR&rarr;APP | 0xD1 | 242B | 6 ms | IF cal response (constant) |
| APP&rarr;AVR | 0x23 | empty | <1 ms | AVR config query |
| AVR&rarr;APP | 0xA2 | 17B | 6 ms | Gain factors, config bytes |
| APP&rarr;AVR | 0xBE | `[03 00 00 64]` | <1 ms | Final param read |
| APP&rarr;AVR | 0x9B | 9B | <1 ms | Time sync (current epoch) |
| AVR&rarr;APP | 0x9B | 9B | 32 ms | Echo with AVR tail bytes |

AVR response latency: 2–32 ms.

**Intermediate responses**: The D0 (CalParamReq) exchange produces an 0xE3 Text
and 0x95 ConfigAck before the actual 0xD1 response. The D2 (CalDataReq) may also
produce an intermediate ConfigAck. Skip Text and ConfigAck messages when waiting
for D3/D1 responses.

### 2.3 Phase 3 — PI Sync (~396 ms)

PI is the **slowest component**. First STATUS response takes ~119 ms.

| Dir | Type | Payload | Typical &Delta; | Notes |
| --- | ---- | ------- | --------------- | ----- |
| APP&rarr;PI | 0xAA | `[01 03]` (2B) | &mdash; | Status poll |
| PI&rarr;APP | 0xAA | 17B | **119 ms** | First PI response (slow) |
| APP&rarr;PI | 0x67 | empty | <1 ms | Device info |
| PI&rarr;APP | 0xE7 | 75B ASCII | 24 ms | Pi firmware date |
| APP&rarr;PI | 0xBE | `[03 00 00 0A]` | 4–6 ms | Param read (cap flag 0x0A) |
| APP&rarr;PI | 0x83 | `[02 01 05]` &times;2 | <1 ms | Camera config request (&times;2, keep last) |
| PI&rarr;APP | 0x82 | 52B &times;2 | varies | Current camera config |
| APP&rarr;PI | 0xDE | `[01 00]` / `[01 08]` | <1 ms | Net config (SSID, password) |
| PI&rarr;APP | 0xDE | 54B &times;2 | 34–40 ms | SSID + password |
| APP&rarr;PI | 0xBE | &times;4 (0x01, 0x07, 0x08, 0x09) | 5–39 ms | Param reads (cap flags) |
| APP&rarr;PI | 0x90 | &times;12 chunks | 2–14 ms | Sensor activation (optional) |
| PI&rarr;APP | 0x89 | certificate | varies | Licensing response (optional) |
| APP&rarr;PI | 0xBE | `[03 00 00 06]` | varies | Param read (cap flag 0x06) |
| APP&rarr;PI | 0x87 | &times;8 pages | 2–13 ms | WiFi scan (optional) |
| APP&rarr;PI | 0xBE | &times;4 (0x0B, 0x03, 0x04, 0x05) | 37–39 ms | Final param reads |

PI response latency: 2–119 ms.

**Optional messages**: Sensor activation (0x90/0x89) and WiFi scan (0x87) are not
required for operation. The device arms without them. See Section 7 for a minimal
handshake.

**Note**: PI param reads use IDs 0x0A, 0x01, 0x07, 0x08, 0x09, 0x06, 0x0B, 0x03,
0x04, 0x05 (10 read params). ID 0x02 is a PI *write* param (keepalive interval),
not a read param — do not send it as a ParamReadReq.

### 2.4 Phase 4 — Post-Sync Configuration (~100 ms + retries)

Rapid-fire AVR config: BF param writes, mode set, radar calibration. Each
BF&rarr;95 exchange takes ~5 ms. **Every command is followed by B0 `[01 00]`
config commit**, not just at the start/end.

| Dir | Type | Payload | Notes |
| --- | ---- | ------- | ----- |
| | | **&times;N per BF param:** | |
| APP&rarr;AVR | 0xBF | param write | Ball type, tee height, track%, etc. |
| AVR&rarr;APP | 0x95 | `[02 30 3F]` | CONFIG_ACK for BF |
| APP&rarr;AVR | 0xB0 | `[01 00]` | Config commit |
| AVR&rarr;APP | 0x95 | `[02 30 30]` | CONFIG_ACK for B0 |
| | | **Then mode + cal:** | |
| APP&rarr;AVR | 0xA5 | `[02 00 XX]` | Set DetectionMode (XX = commsIndex) |
| AVR&rarr;APP | 0xA5 | `[02 00 XX]` | Echo/ack |
| APP&rarr;AVR | 0xB0 | `[01 00]` | Config commit |
| AVR&rarr;APP | 0x95 | `[02 30 30]` | CONFIG_ACK |
| APP&rarr;AVR | 0xA4 | `[06 RR RR 00 HH 00 00]` | Radar cal (range mm, height mm) |
| AVR&rarr;APP | 0xA4 | echo | Echo/ack |
| APP&rarr;AVR | 0xB0 | `[01 00]` | Config commit |
| AVR&rarr;APP | 0x95 | `[02 30 30]` | CONFIG_ACK |

**Retry behavior**: The AVR may not fully configure on the first attempt. Retry with
exponential backoff: 200 ms &rarr; 300 ms &rarr; 600 ms. In practice the initial config
usually succeeds; retry if no response within ~200 ms.

### 2.5 Phase 5 — PI Post-Config (~250 ms)

Final camera and PI configuration after AVR config completes.

| Dir | Type | Payload | Notes |
| --- | ---- | ------- | ----- |
| APP&rarr;PI | 0xBE | param read | Capability check |
| APP&rarr;PI | 0x82 | 52B | Camera config push |
| PI&rarr;APP | 0x95 | `[02 12 02]` | Acks 0x82 cam config |
| APP&rarr;PI | 0x83 | `[02 01 05]` | Camera config request |
| PI&rarr;APP | 0x82 | 52B | Current config readback |
| APP&rarr;PI | 0x81 | `[01 01]` | Start camera |
| PI&rarr;APP | 0x95 | `[02 12 01]` | Acks 0x81 cam state |
| APP&rarr;PI | 0x82 | 52B | Final camera config |
| APP&rarr;PI | 0x83 | `[02 01 05]` | Config request |
| PI&rarr;APP | 0x82 | 52B | Readback |
| APP&rarr;PI | 0x81 | `[01 01]` | Confirm camera start |
| PI&rarr;APP | 0x95 | `[02 12 01]` | Acks 0x81 |
| APP&rarr;PI | 0xBF | PI param (0x0002 = 10) | PI keepalive/timeout interval |
| PI&rarr;APP | 0x95 | `[02 12 3F]` | Acks 0xBF param write |

### 2.6 Phase 6 — ARM (~170 ms)

| Dir | Type | Payload | Notes |
| --- | ---- | ------- | ----- |
| APP&rarr;DSP | 0xAA | `[01 01]` | Final DSP status poll |
| DSP&rarr;APP | 0xAA | 129B | Status response |
| APP&rarr;AVR | **0xB0** | **`[01 01]`** | **ARM command** |
| AVR&rarr;APP | 0x95 | `[02 30 30]` | CONFIG_ACK (arm ack) |
| APP&rarr;PI | 0xAA | `[01 03]` | PI status poll |
| PI&rarr;APP | 0xAA | 17–20B | Status response |
| DSP&rarr;APP | 0xE3 | "System State 6" | +84 ms after arm |
| AVR&rarr;APP | 0xE3 | "ARMED DetectionMode=XX" | **+170 ms after arm** |
| AVR&rarr;APP | 0xE3 | "GainInit=0, ..." | Radar parameters |
| AVR&rarr;APP | 0xE3 | "ClubTrigger Disabled" | Club trigger state |
| AVR&rarr;APP | 0xE3 | "Arm Epoch ... delay = N us" | Arm timing |

Wait for the 0xE3 text containing "ARMED" as confirmation the device is ready.

### 2.7 Timing Summary

| Phase | Duration | Key Latency |
| ----- | -------- | ----------- |
| 1. DSP sync | ~114 ms | 10–30 ms/exchange |
| 2. AVR sync | ~144 ms | 2–32 ms/exchange |
| 3. PI sync | ~396 ms | 2–119 ms (first STATUS slowest) |
| 4. Post-sync config | ~100 ms | ~5 ms/exchange |
| 5. PI post-config | ~250 ms | 20–50 ms/exchange |
| 6. ARM &rarr; ARMED | ~170 ms | Async text responses |
| **Total (no retries)** | **~1.2 s** | |
| **Total (with retries)** | **~3.8 s** | Observed with config retry backoff |

---

## 3. Keepalive

After ARM, poll all three nodes sequentially every ~1 second:

```
APP→DSP  0xAA  [01 01]  →  DSP→APP  0xAA  (129B)   Δ=14–16 ms
APP→AVR  0xAA  [01 01]  →  AVR→APP  0xAA  (25B)    Δ=2–3 ms
APP→PI   0xAA  [01 03]  →  PI→APP   0xAA  (17–20B) Δ=1–2 ms
                                              Total: ~20 ms I/O
... ~980 ms idle ...
(repeat)
```

Poll order is always DSP &rarr; AVR &rarr; PI. Each poll waits for its response
before the next is sent. The AVR response byte [1] = 0x01 confirms armed state.

**Note**: The DSP also sends unsolicited 0xAA status messages outside of
keepalive polling. These may arrive during any exchange and should be filtered
by bus address (see &sect;7.3).

---

## 4. Shot Sequence

A single shot produces the following message flow, organized in three phases.
All shot data flows AVR &rarr; APP unless noted otherwise.

### 4.1 Phase 1 — Result Delivery (device-pushed)

The device sends all shot results without APP requests:

```
AVR→APP  0xE5   "BALL TRIGGER: N ms back, at Epoch ..."    shot detected
AVR→APP  0xE9   TRACKING_STATUS (82B)                       trigger phase (×2)
AVR→APP  0xE3   text logs (launch conditions)               debug info
AVR→APP  0xE8   FLIGHT_RESULT_V1 (94B)                      early launch data
AVR→APP  0xEC   PRC_DATA ×N                                 initial ball tracking burst
AVR→APP  0xE5   "Clubimpact at Epoch ..."                   club impact detected
AVR→APP  0xE9   TRACKING_STATUS (82B)                       club-found phase (×1)
AVR→APP  0xD4   FLIGHT_RESULT (158B)                        ★ main flight result
AVR→APP  0xED   CLUB_RESULT (167–172B)                      ★ club head data
AVR→APP  0xD9   SPEED_PROFILE (172B)                        club speed curve
AVR→APP  0xEF   SPIN_RESULT (138B)                          ★ spin analysis
AVR→APP  0xEC   PRC_DATA ×N                                 remaining tracking pages
PI→APP   0x84   CAM_IMAGE_AVAIL (67B)                       camera image notification
AVR→APP  0xE5   "PROCESSED"                                 ← end of Phase 1
```

The three messages marked ★ contain the core shot data: ball speed, launch angles,
carry distance, club speed, attack angle, face angle, spin rate, and spin axis.

**Message ordering is not strict.** The sequence above is the *typical* order
observed in pcap, but late-arriving messages (EC, EE, EF, D9, 0x84) may still be
in flight when "PROCESSED" arrives. A robust client must drain all messages and
not assume Phase 1 delivery is complete before "PROCESSED".

**D9 stub form.** The device occasionally sends a D9 with only 2 bytes of payload
(no speed data). This is valid — decode as an empty SpeedProfile.

### 4.2 Post-Shot Complete Cycle

The entire post-shot flow — from "PROCESSED" through re-arm — should be driven
as **one continuous operation**. If the client pauses or fails to re-arm, the
device stops responding to all messages (including keepalive).

**Minimal post-shot flow:**

```
                                                        ┌─ drain all messages
On E5 "PROCESSED":                                      │  (EC, EE, E9, text,
  APP→AVR  0x69  ×2         ShotDataAck (fire-and-forget)│   etc.) while waiting
  ... drain ...             consume all remaining data ──┘   for IDLE
On E5 "IDLE":
  APP→AVR  0x21             ConfigQuery
  AVR→APP  B1 + A0          ModeAck + ConfigResp (either order)
  APP→AVR  0x6D             ShotResultReq (optional)
  AVR→APP  0xED             duplicate ClubResult (discard)
  APP→AVR  0xB0  [01 01]   ★ RE-ARM
  AVR→APP  0x95             ConfigAck
  AVR→APP  0xE3             "ARMED DetectionMode=XX"
```

**Key behaviors observed in live testing:**

- **ShotDataAck (0x69)**: Send both immediately after "PROCESSED". Do not wait
  for E9 retransmissions between them — E9 responses may be delayed or
  interleaved with other late shot data.
- **Drain phase**: After sending both 0x69s, consume all messages until "IDLE"
  arrives. Everything received (E9, EC, EE, Text, ConfigAck, CamImageAvail)
  can be safely discarded.
- **B1 + A0 ordering**: After ConfigQuery (0x21), ModeAck (B1) and ConfigResp
  (A0) arrive in **either order**. Wait for both.
- **ShotResultReq (0x6D)**: Best-effort — triggers a duplicate ED. If the device
  doesn't respond, proceed to arm anyway.
- **Device hangs without re-arm**: If the client does not complete the re-arm
  sequence after IDLE, the device enters a dormant state and stops responding
  to all messages, including keepalive polls.

### 4.3 Full Post-Shot Sequence (reference)

The native app performs additional optional steps (pcap reference):

```
APP→AVR  0x69   (empty) ×2             ShotDataAck
AVR→APP  0xE9   TRACKING_STATUS ×2     retransmissions
APP→AVR  0xEC   request (4B) ×N        PRC re-fetch (optional)
AVR→APP  0xEC   response (244B) ×N     identical data
DSP→APP  0xE3   "System State 5"       DSP state transition
AVR→APP  0xE5   "IDLE"                 ← triggers rearm
APP→AVR  0x21   CONFIG_QUERY
AVR→APP  0xB1 + 0xA0                   (either order)
APP→AVR  0x6D   SHOT_RESULT_REQ
AVR→APP  0xED   CLUB_RESULT (duplicate)
APP→AVR  0xEE   CLUB_PRC request ×N    bulk club PRC fetch (optional)
AVR→APP  0xEE   CLUB_PRC response ×N
APP→AVR  0xD2   CAL_DATA_REQ (0x07)    post-shot param dump (optional)
AVR→APP  0xD3   CAL_DATA_RESP ×4
APP→AVR  0xD0   CAL_PARAM_REQ          IF cal re-read (optional)
AVR→APP  0xD1   CAL_PARAM_RESP
APP→AVR  0xB0   [01 01]               ★ RE-ARM
AVR→APP  0x95   CONFIG_ACK
AVR→APP  0xE3   "ARMED DetectionMode=XX"
```

The EE CLUB_PRC fetch, D2/D0 re-reads, and EC re-fetch are all optional.

---

## 5. Mode Change Sequence

Changing detection mode (e.g., Normal &rarr; Putting) triggers a full disarm/re-arm
cycle. The current mode's commsIndex is set via 0xA5 (see WIRE.md &sect;7 for values).

```
AVR→APP  0xE3   "ARMED CANCELLED"              disarm notification
AVR→APP  0xE3   "ADC errors = ..."             diagnostics
DSP→APP  0xE3   "System State 5"               state transition
AVR→APP  0xB1   [02 00 00]                     mode reset ack
APP→AVR  0xBF   ×N                             parameter reads/writes
APP→AVR  0xB0   [01 00]  +  0x95 ack  ×N       config exchanges
APP→AVR  0xA5   [02 00 XX]                     set DetectionMode=XX
AVR→APP  0xA5   [02 00 XX]                     echo/ack
APP→AVR  0xB0   [01 01]                        arm command
DSP→APP  0xE3   "System State 6"               arming
AVR→APP  0xE3   "ARMED DetectionMode=XX"       ← armed confirmation
AVR→APP  0xE3   "GainInit=..., MaxVel=..."     radar params for new mode
```

---

## 6. Settings Change Sequence

Changing a setting (ball type, tee height, distances) uses the same
disarm/re-arm cycle as a mode change, but **without** the 0xA5 mode set
(the detection mode stays the same).

```
AVR→APP  0xE3   "ARMED CANCELLED"              disarm
AVR→APP  0xB1   [02 00 00]                     reset ack
APP→AVR  0xBF   param writes ×N                new settings values
AVR→APP  0x95   CONFIG_ACK per write            ack each write
APP→AVR  0xA4   [06 RR RR 00 HH 00 00]        radar cal update
AVR→APP  0xA4   echo                            ack
APP→AVR  0xB0   [01 00]  +  0x95 ack            config commit
APP→AVR  0xB0   [01 01]                         re-arm
AVR→APP  0xE3   "ARMED DetectionMode=XX"        ← armed with new settings
```

Key writable parameters (see WIRE.md &sect;6.3 for encoding):

| Param | Type | Description |
| ----- | ---- | ----------- |
| 0x0006 | INT24 | Ball type (0=RCT, 1=Standard) |
| 0x000F | FLOAT40 | Minimum track percentage (0.6–1.0) |
| 0x0026 | FLOAT40 | Driver tee height (meters) |

Radar calibration (0xA4) bytes 1-2 carry sensor-to-tee distance in mm; byte 4
carries `floor(surfaceHeight_inches * 25.4)`.

---

## 7. Client Implementation Notes

### 7.1 Minimal Handshake

A minimal client can skip several optional exchanges:

| Phase | Required | Optional / Skippable |
| ----- | -------- | -------------------- |
| 1. DSP sync | All | &mdash; |
| 2. AVR sync | All | &mdash; |
| 3. PI sync | STATUS, 0x67, 0xBE reads, 0x83 ×2, 0xDE ×2 | 0x90 sensor act, 0x87 WiFi scan |
| 4. Post-sync config | BF writes + B0 each, A5 mode + B0, A4 cal + B0 | &mdash; |
| 5. PI post-config | 0x82 cam config, 0x81 cam start | &mdash; |
| 6. ARM | B0 `[01 01]` + wait for "ARMED" | &mdash; |

**Bus filtering**: When expecting a response from a specific bus (DSP/AVR/PI),
skip messages from other buses. The PI may send unsolicited messages (e.g.
CamState) that arrive during AVR configuration. Also skip 0xE3 Text debug
messages at all times.

### 7.2 Minimal Post-Shot Flow

After receiving shot results, the minimum required to keep the device cycling.
**This must be driven as one continuous operation** — see &sect;4.2 for details.

1. On E5 "PROCESSED": send 0x69 (SHOT_DATA_ACK) &times;2 (fire-and-forget)
2. Drain all messages until E5 "IDLE" (discard everything)
3. Send 0x21 (CONFIG_QUERY), collect B1 + A0 (either order, best-effort)
4. Send 0x6D (SHOT_RESULT_REQ), drain to ED (best-effort, skip on timeout)
5. Re-arm: send B0 `[01 01]`, wait for 0x95 ConfigAck
6. Wait for E3 "ARMED"

If any step times out, retry with `...` logging. If the client fails to reach
step 5 (re-arm), the device enters a dormant state and stops responding to all
messages including keepalive — a reconnect is required to recover.

Skip EC PRC re-fetch and EE CLUB_PRC fetch unless trajectory visualization is
needed. The duplicate ED from 0x6D can be discarded.

### 7.3 Timeouts

| Situation | Recommended timeout |
| --------- | ------------------- |
| All exchanges (unified) | **1000 ms** |

Live testing showed that 500 ms is too tight for some post-shot exchanges,
particularly after "PROCESSED" when the device is flushing results. A unified
1 s timeout simplifies the implementation and accommodates all observed
latencies with margin.

**Retry strategy**: On timeout, log the error and retry the same operation.
This is preferable to increasing the timeout further, since most exchanges
complete in &lt;50 ms and a 1 s pause already indicates something unusual.
Continuous timeouts (every retry fails) indicate the device is in a dormant
state — see &sect;7.2.

**Unsolicited DSP status**: The DSP sends 0xAA status messages (129B)
asynchronously — they are not always in response to a poll. These can arrive
while waiting for an AVR or PI response, consuming the receive window and
causing a timeout on the expected reply. Bus filtering (skip messages from
the wrong source) handles this naturally; without it, two back-to-back DSP
status messages can eat two full timeout cycles.

**Reconnection resilience**: On reconnect, the device may be in an armed or
partially-configured state from the previous session. The AVR may send
unsolicited ModeAck (0xB1) or return Unknown (0x94) for CalParamReq (0xD0).
Skip these gracefully — they do not affect operation.

### 7.4 Core Shot Data

The three messages that carry the primary shot result:

| Message | Key fields |
| ------- | ---------- |
| 0xD4 FLIGHT_RESULT | LaunchSpeed, LaunchElevation (VLA), LaunchAzimuth (HLA), CarryDistance |
| 0xED CLUB_RESULT | PreClubSpeed (club head speed), AttackAngle, FaceAngle, DynamicLoft, StrikeDirection (path) |
| 0xEF SPIN_RESULT | PMSpinFinal (total spin), SpinAxis (/10 = degrees) |

0xE8 FLIGHT_RESULT_V1 can serve as a partial/early result if latency matters.
