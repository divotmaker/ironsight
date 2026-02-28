# Wire Protocol Reference

Interoperability specification for the FlightScope Mevo+ binary protocol
on TCP port 5100. All multi-byte integers are big-endian unless noted.

---

## 1. Frame Format

Every message is wrapped in a frame delimited by `0xF0` (start) and `0xF1` (end).
These marker bytes never appear inside the frame; a byte-stuffing scheme (Section 1.1)
ensures this.

```
┌──────┬──────────────────────────────────────────────────────┬──────┐
│ 0xF0 │  STUFFED( DEST  SRC  TYPE  PAYLOAD...  CS_HI CS_LO )│ 0xF1 │
│start │  byte-stuffed interior                               │ end  │
└──────┴──────────────────────────────────────────────────────┴──────┘
```

After unstuffing, the interior has the following logical structure:

| Field   | Size      | Description             |
| ------- | --------- | ----------------------- |
| DEST    | 1 byte    | Destination bus address |
| SRC     | 1 byte    | Source bus address      |
| TYPE    | 1 byte    | Message type identifier |
| PAYLOAD | 0–N bytes | Type-specific data      |
| CS_HI   | 1 byte    | Checksum high byte      |
| CS_LO   | 1 byte    | Checksum low byte       |

Minimum wire frame: **7 bytes** (F0 + DEST + SRC + TYPE + CS_HI + CS_LO + F1,
with no payload and no escaping needed).

### 1.1 Byte Stuffing

Four byte values cannot appear literally inside a frame. Each is escaped with a
two-byte sequence using `0xFD` as the escape prefix:

| Original | Wire encoding | Why escaped          |
| -------- | ------------- | -------------------- |
| `0xF0`   | `0xFD 0x01`   | Frame start marker   |
| `0xF1`   | `0xFD 0x02`   | Frame end marker     |
| `0xFD`   | `0xFD 0x03`   | Escape prefix itself |
| `0xFA`   | `0xFD 0x04`   | Reserved             |

Any `0xFD` in the interior is always an escape prefix. The byte following it
selects the decoded value (01 &rarr; F0, 02 &rarr; F1, 03 &rarr; FD, 04 &rarr; FA).

Stuffing applies to **all** interior bytes: DEST, SRC, TYPE, payload, and the
checksum bytes themselves.

### 1.2 Worked Example

Send a STATUS poll from APP (0x10) to DSP (0x40), type 0xAA, payload `[01 01]`.

**Logical frame** (unstuffed):

```
DEST=40  SRC=10  TYPE=AA  PAYLOAD=01 01  CS_HI  CS_LO
```

No bytes require stuffing (none are F0/F1/FA/FD), so the raw wire bytes for
the checksummed region are `40 10 AA 01 01`.

Checksum = `0x40 + 0x10 + 0xAA + 0x01 + 0x01 = 0x00FC`.

CS_HI = `0x00`, CS_LO = `0xFC`. Neither requires stuffing.

**Wire frame**: `F0 40 10 AA 01 01 00 FC F1` (9 bytes).

---

## 2. Checksum

A 16-bit sum of the **raw (stuffed) wire bytes** from DEST through the last
payload byte. The checksum value itself is then byte-stuffed before being placed
on the wire.

```
fn compute_checksum(stuffed_interior: &[u8], payload_end: usize) -> u16 {
    // stuffed_interior is everything between F0 and F1
    // payload_end is the byte offset where the checksum begins
    let mut sum: u16 = 0;
    for &b in &stuffed_interior[..payload_end] {
        sum = sum.wrapping_add(b as u16);
    }
    sum
}

fn verify_frame(wire: &[u8]) -> bool {
    let interior = &wire[1..wire.len() - 1]; // strip F0 and F1

    // Unstuff to find where the checksum starts
    let mut unstuffed: Vec<(u8, usize)> = Vec::new(); // (value, wire_offset)
    let mut i = 0;
    while i < interior.len() {
        if interior[i] == 0xFD && i + 1 < interior.len() {
            let decoded = match interior[i + 1] {
                0x01 => 0xF0,
                0x02 => 0xF1,
                0x03 => 0xFD,
                0x04 => 0xFA,
                _ => return false, // invalid escape
            };
            unstuffed.push((decoded, i));
            i += 2;
        } else {
            unstuffed.push((interior[i], i));
            i += 1;
        }
    }

    // Last 2 unstuffed values are the checksum (big-endian)
    let n = unstuffed.len();
    let cs_received = ((unstuffed[n - 2].0 as u16) << 8)
                    | (unstuffed[n - 1].0 as u16);

    // Sum raw wire bytes before the checksum
    let data_end = unstuffed[n - 2].1;
    let cs_computed: u16 = interior[..data_end]
        .iter()
        .map(|&b| b as u16)
        .sum::<u16>();

    cs_received == cs_computed
}
```

---

## 3. Bus Addresses

| Address | Node | Description                              |
| ------- | ---- | ---------------------------------------- |
| `0x10`  | APP  | Client application (phone, PC, our tool) |
| `0x12`  | PI   | Raspberry Pi camera processor            |
| `0x30`  | AVR  | AVR microcontroller (radar I/O, battery) |
| `0x40`  | DSP  | Digital signal processor (radar core)    |

Traffic examples:

- `DEST=0x40, SRC=0x10` &mdash; APP &rarr; DSP
- `DEST=0x10, SRC=0x30` &mdash; AVR &rarr; APP

All traffic in our use case flows between APP and the three device nodes.
The device nodes do not address each other through the TCP link.

---

## 4. Field Encodings

All multi-byte fields are **big-endian**. Signed types use two's complement with
sign extension.

### INT16 &mdash; 2 bytes

```
value = (byte[0] << 8) | byte[1]
if value >= 0x8000: value -= 0x10000   // sign-extend
```

### INT24 &mdash; 3 bytes

```
value = (byte[0] << 16) | (byte[1] << 8) | byte[2]
if value >= 0x800000: value -= 0x1000000   // sign-extend
```

### INT32 &mdash; 4 bytes

Standard big-endian signed 32-bit integer.

### FLOAT40 &mdash; 5 bytes

Custom 40-bit floating point.

**Wire layout**: `[exp_hi, exp_lo, mant_hi, mant_mid, mant_lo]`

**Decode**:

```
exponent = SignExtend16( (byte[0] << 8) | byte[1] )       // signed 16-bit
mantissa = SignExtend24( (byte[2] << 16) | (byte[3] << 8) | byte[4] )  // signed 24-bit
value    = mantissa * 2^(exponent - 23)
```

Equivalently: `value = ldexp(mantissa, exponent - 23)`.

**Encode**:

```
(frac, exp) = frexp(value)     // frac in [0.5, 1.0)
mantissa    = (int)(frac * 2^23)
bytes       = [exp >> 8, exp & 0xFF, (mantissa >> 16) & 0xFF,
               (mantissa >> 8) & 0xFF, mantissa & 0xFF]
```

**Properties**:

- 23-bit mantissa precision (same as IEEE float32 significand)
- 16-bit signed exponent range (far wider than float32's 8-bit)
- Zero is encoded as five zero bytes

**Examples**:

| Value  | Encoded (hex)    | Exponent | Mantissa |
| ------ | ---------------- | -------- | -------- |
| 0.0    | `00 00 00 00 00` | 0        | 0        |
| 1.0    | `00 01 40 00 00` | 1        | 4194304  |
| 12.5   | `00 04 64 00 00` | 4        | 6553600  |
| -2.3   | `00 02 b6 66 67` | 2        | -4823449 |
| 100.0  | `00 07 64 00 00` | 7        | 6553600  |
| 0.0254 | `ff fb 68 09 e2` | -5       | 6818274  |

---

## 5. Request/Response Convention

For a request with type `0xNN` (where `NN < 0x80`), the response type is
generally `0xNN | 0x80`:

- `0x48` &rarr; `0xC8`
- `0x67` &rarr; `0xE7`

**Variant**: Some types clear the LSB before the OR: `(type & ~0x01) | 0x80`:

- `0x21` &rarr; `0xA0`
- `0x23` &rarr; `0xA2`

---

## 6. Message Catalog

Payload byte offsets below are within the **unstuffed payload** (i.e., after
stripping DEST, SRC, and TYPE). Offset `[0]` is the first payload byte.

### 6.1 Shot Result Messages

All flow AVR &rarr; APP (DEST=0x10, SRC=0x30).

#### 0xD4 &mdash; FLIGHT_RESULT (158 bytes)

The primary ball flight result. 1 per shot. Contains 52 INT24 fields: 36 linear
scalars followed by a polynomial scale factor and 15 polynomial coefficients.

```
Offset   Width  Scale    Field
──────── ────── ──────── ────────────────────────────
[0]      UINT8  —        Length (always 0x9C = 156)
[1-3]    INT24  /1       Total (shot counter)
[4-6]    INT24  /1000    TrackTime (s)
[7-9]    INT24  /1000    StartPosition[0] (m)
[10-12]  INT24  /1000    StartPosition[1] (m)
[13-15]  INT24  /1000    StartPosition[2] (m)
[16-18]  INT24  /1000    LaunchSpeed (m/s)
[19-21]  INT24  /1000    LaunchAzimuth (deg, neg = right)
[22-24]  INT24  /1000    LaunchElevation (deg)
[25-27]  INT24  /1000    CarryDistance (m)
[28-30]  INT24  /1000    FlightTime (s)
[31-33]  INT24  /1000    MaxHeight (m)
[34-36]  INT24  /1000    LandingPosition[0] (m)
[37-39]  INT24  /1000    LandingPosition[1] (m)
[40-42]  INT24  /1000    LandingPosition[2] (m)
[43-45]  INT24  /1       BackspinRPM
[46-48]  INT24  /1       SidespinRPM
[49-51]  INT24  /1       RiflespinRPM
[52-54]  INT24  /1       LandingSpinRPM[0]
[55-57]  INT24  /1       LandingSpinRPM[1]
[58-60]  INT24  /1       LandingSpinRPM[2]
[61-63]  INT24  /1000    LandingVelocity[0] (m/s)
[64-66]  INT24  /1000    LandingVelocity[1] (m/s)
[67-69]  INT24  /1000    LandingVelocity[2] (m/s)
[70-72]  INT24  /1000    TotalDistance (m) [†]
[73-75]  INT24  /1000    RollDistance (m) [†]
[76-78]  INT24  /1000    FinalPosition[0] (m) [†]
[79-81]  INT24  /1000    FinalPosition[1] (m) [†]
[82-84]  INT24  /1000    FinalPosition[2] (m) [†]
[85-87]  INT24  /1000    ClubheadSpeed (m/s)
[88-90]  INT24  /1000    ClubStrikeDirection (deg)
[91-93]  INT24  /1000    ClubAttackAngle (deg)
[94-96]  INT24  /1000    ClubheadSpeedPost (m/s)
[97-99]  INT24  /1000    ClubSwingPlaneTilt (deg)
[100-102] INT24 /1000    ClubSwingPlaneRotation (deg)
[103-105] INT24 /1000    ClubEffectiveLoft (deg)
[106-108] INT24 /1000    ClubFaceAngle (deg)
[109-111] INT24 /1       PolyScaleFactor
[112-126] 5×INT24 /poly  PolyX[0..4]
[127-141] 5×INT24 /poly  PolyY[0..4]
[142-156] 5×INT24 /poly  PolyZ[0..4]
```

**[†] = Not populated by DSP.** TotalDistance contains diagnostic values
(negative in 4/10 shots). RollDistance is always zero. FinalPosition is always (0,0,0).
Roll and total distance must be computed client-side from landing velocity,
landing spin, and surface type.

Polynomial coefficients are each divided by PolyScaleFactor [109-111].
The three arrays are stored contiguously: X[0..4], then Y[0..4], then Z[0..4].

**Trajectory model**: `pos(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4`
for each axis. DSP coordinates: X = forward (range), Y = vertical, Z = lateral.

**Total spin** (not directly on wire): `sqrt(BackspinRPM^2 + SidespinRPM^2)`.

**Spin axis**: See SPIN_RESULT (0xEF) field SpikeSpin, which encodes the spin axis
angle in tenths of degrees. Backspin and sidespin are the trigonometric decomposition:
`BackspinRPM = TotalSpin * cos(axis)`, `SidespinRPM = TotalSpin * sin(axis)`.

**Shot 1 caveat**: The first shot in a session may have uninitialized data in
fields after byte 27 (flight model not yet computed). Shot 2 onward has full data.

#### 0xE8 &mdash; FLIGHT_RESULT_V1 (94 bytes)

Early/partial flight result, sent before 0xD4. 1 per shot. Contains launch
conditions and a 5th-order trajectory polynomial.

```
Offset  Width  Scale       Field
─────── ────── ─────────── ──────────────────
[0]     UINT8  —           Length (0x5D = 93)
[1-3]   INT24  /1          Total (shot counter)
[4-6]   INT24  /1000       ClubVelocity (m/s)
[7-9]   INT24  /1000       BallVelocity (m/s)
[10-12] INT24  /1000       FlightTime (s)
[13-15] INT24  /1000       Distance (m)
[16-18] INT24  /1000       Height (m)
[19-21] INT24  /1000       Lateral (m)
[22-24] INT24  /1000       Elevation (deg, VLA)
[25-27] INT24  /1000       Azimuth (deg, HLA)
[28-30] INT24  /1000       TrackedTime (s)
[31-33] INT24  /1000000    Drag
[34-36] INT24  /1          Backspin (RPM)
[37-39] INT24  /1          Sidespin (RPM)
[40-42] INT24  /1000       Acceleration
[43-45] INT24  /1000       ClubStrikeDirection (deg)
[46-48] INT24  /1 (min 1)  PolyScale
[49-63] 5×INT24 /PolyScale X polynomial [0..4]
[64-78] 5×INT24 /PolyScale Y polynomial [0..4]
[79-93] 5×INT24 /PolyScale Z polynomial [0..4]
```

BallVelocity equals the magnitude of the velocity vector `(x[1], y[1], z[1])`.
ClubVelocity, Drag, ClubStrikeDirection, and Sidespin are typically zero in this
message (club data comes in 0xED, spin data in 0xEF).

#### 0xED &mdash; CLUB_RESULT (167-172 bytes)

Club head measurements. Appears **twice per shot** (duplicate transmission,
byte-identical). Contains 16 scalar fields, a polynomial scale factor, 36
polynomial coefficients, and 3 timing fields.

```
Offset    Width   Scale  Field
───────── ─────── ────── ────────────────────────
[0]       UINT8   —      Length prefix
[1]       UINT8   —      NumClubPRCPoints
[2-4]     INT24   raw    Flags
[5-7]     INT24   /100   PreClubSpeed (m/s)
[8-10]    INT24   /100   PostClubSpeed (m/s)
[11-13]   INT24   /100   StrikeDirection (deg)
[14-16]   INT24   /100   AttackAngle (deg)
[17-19]   INT24   /100   FaceAngle (deg)
[20-22]   INT24   /100   DynamicLoft (deg)
[23-25]   INT24   /1000  SmashFactor
[26-28]   INT24   /1000  DispersionCorrection
[29-31]   INT24   /100   SwingPlaneHorizontal (deg)
[32-34]   INT24   /100   SwingPlaneVertical (deg)
[35-37]   INT24   /100   ClubAzimuth (deg)
[38-40]   INT24   /100   ClubElevation (deg)
[41-43]   INT24   /1000  ClubOffset (m)
[44-46]   INT24   /1000  ClubHeight (m)
[47-49]   INT24   raw    PolyScale (min 1)
[50-157]  36×INT24 /poly Polynomial coefficients (12 arrays × 3)
[158-160] INT24   /100   PreImpactTime (ms)
[161-163] INT24   /100   PostImpactTime (ms)
[164-166] INT24   /100   ClubToBallTime (ms)
```

**Polynomial layout** &mdash; 12 arrays of 3 coefficients each (9 bytes per array),
stored consecutively:

| Offset    | Array         | Description                      |
| --------- | ------------- | -------------------------------- |
| [50-58]   | Pre_v[0,1,2]  | Pre-impact velocity              |
| [59-67]   | Pst_v[0,1,2]  | Post-impact velocity             |
| [68-76]   | Pre_x[0,1,2]  | Pre-impact position X (forward)  |
| [77-85]   | Pst_x[0,1,2]  | Post-impact position X           |
| [86-94]   | Pre_y[0,1,2]  | Pre-impact position Y (vertical) |
| [95-103]  | Pst_y[0,1,2]  | Post-impact position Y           |
| [104-112] | Pre_z[0,1,2]  | Pre-impact position Z (lateral)  |
| [113-121] | Pst_z[0,1,2]  | Post-impact position Z           |
| [122-130] | Pre_YX[0,1,2] | Pre-impact Y/X ratio             |
| [131-139] | Pst_YX[0,1,2] | Post-impact Y/X ratio            |
| [140-148] | Pre_ZX[0,1,2] | Pre-impact Z/X ratio             |
| [149-157] | Pst_ZX[0,1,2] | Post-impact Z/X ratio            |

#### 0xEF &mdash; SPIN_RESULT (138 bytes)

Spin measurement data. 1 per shot. Contains per-antenna signal data and spin
estimates from multiple algorithms. The version/length byte [0] determines
format; all observed data uses version 0x89 (137 data bytes).

```
Offset     Width   Scale  Field
────────── ─────── ────── ──────────────────────────
[0]        UINT8   —      Length/version (0x89 = 137)
[1-105]    —       —      Antenna arrays (see below)
[106-107]  INT16   /1     PMSpinRaw (RPM)
[108-109]  INT16   /1     PMSpinFinal (RPM) — total spin
[110-111]  INT16   /1     PMSpinConfidence (0-100)
[112-113]  INT16   /1     LiftSpin (RPM)
[114-115]  INT16   /1     SpinValidateExpected (RPM)
[116-117]  INT16   /1     SpinValidateLowLimit (RPM)
[118-119]  INT16   /1     SpinValidateHighLimit (RPM)
[120-121]  INT16   —      SpinValidateScaling
[122]      UINT8   —      SpinMethod (algorithm selector)
[123-125]  INT24   raw    SpinFlags
[126-127]  INT16   /1     LaunchSpin (RPM)
[128-129]  INT16   /1     AMSpin (RPM)
[130-131]  INT16   /1     PMSpin (RPM)
[132-133]  INT16   /10    SpinAxis (deg, spin axis angle)
[134-135]  INT16   /1     AODSpin (RPM)
[136-137]  INT16   /1     PLLSpin (RPM)
```

**Antenna arrays** [1-105] &mdash; 5 antenna groups &times; 3 range bins &times; 7 bytes:

Each 7-byte element:

```
[+0..1]  INT16  /1     SpinRPM
[+2..4]  INT24  /1000  Peak (signal strength)
[+5..6]  INT16  /1     SNR (signal-to-noise)
```

5 &times; 3 &times; 7 = 105 bytes. Mostly zeros in practice.

**Key fields**: PMSpinFinal [108-109] is the authoritative total spin. SpinAxis
[132-133] divided by 10 gives the spin axis angle in degrees. The sign of SpinAxis
is negated relative to typical app display convention.

**Spin decomposition**:

```
Backspin  = PMSpinFinal * cos(SpinAxis / 10)
Sidespin  = PMSpinFinal * sin(SpinAxis / 10)
```

#### 0xD9 &mdash; SPEED_PROFILE (172 bytes)

Club head speed profile sampled at ~853 &mu;s intervals (~1.17 kHz). 1 per shot.

```
Offset  Width   Scale   Field
─────── ─────── ─────── ──────────────────────────
[0]     UINT8   —       Length (0xAB = 171)
[1]     UINT8   —       Flags/version (always 0x01)
[2]     UINT8   —       NumPrePoints (36-45)
[3]     UINT8   —       NumPostPoints (18-38)
[4-6]   INT24   raw     ScaleFactor (always 100)
[7-11]  FLOAT40 —       TimeInterval (s, ~0.000853)
[12+]   INT16[] /scale  Speed samples (2 bytes each)
```

Each INT16 speed sample divided by ScaleFactor [4-6] gives velocity in m/s.
Samples are zero-padded to 80 entries (172B total = 12 header + 80 &times; 2).
The pre/post-impact boundary is at index NumPrePoints.

**Stub form**: The device occasionally sends a 2-byte D9 payload (length byte
only, no speed data). This is valid — treat as an empty profile with zero
samples.

#### 0xE9 &mdash; TRACKING_STATUS (82 bytes)

Radar tracking metadata. 5 per shot in three phases: trigger (1), club-found (1),
processed (3, including retransmissions).

```
Offset  Width   Field
─────── ─────── ──────────────────────────────
[0]     UINT8   Length (0x51 = 81)
[1]     UINT8   State/flags
[2]     UINT8   Flags
[3-5]   3B      Reserved
[6-15]  10B     Device identity (constant)
[16-19] 4B      Mode/config
[20-21] 2B      Reserved
[22-24] UINT24  PreTrigBufStart (sample index)
[25-27] UINT24  ClubImpactIdx (FFFFFF = not yet found)
[28-30] UINT24  TriggerIdx (sample index)
[31]    UINT8   Reserved
[32-34] UINT24  RadarCal1 (constant)
[35-37] UINT24  RadarCal2 (= RadarCal1)
[38-39] UINT16  RadarCalAVR
[40-46] 7B      Reserved
[47]    UINT8   ProcessingIteration (0-2)
[48]    UINT8   ResultQuality
[49-50] 2B      Reserved
[51]    UINT8   DetectionSubtype
[52-53] 2B      Reserved
[54]    UINT8   PRCTrackingCount
[55]    UINT8   Reserved
[56-57] UINT16  RadarMeasurement
[58]    UINT8   Reserved
[59]    UINT8   TriggerFlags
[60-61] 2B      Reserved
[62-63] UINT16  EventCounter
[64-66] 3B      Reserved
[67-69] SINT24  RadarBaseline
[70-72] SINT24  TrackMeasure1
[73-75] SINT24  TrackMeasure2
[76-78] SINT24  TrackMeasure3
[79]    UINT8   Reserved
[80-81] UINT16  TrackMeasure4
```

Radar sample indices [22-30] index into a circular buffer of size 2^18 (262144).
`TriggerIdx - PreTrigBufStart = 8192` (normally).

Fields [47-48], [54], [70-81] are only populated in the processed phase (zeros
in trigger and club-found phases).

#### 0xEC &mdash; PRC_DATA (variable, 60-byte sub-records)

Raw ball radar tracking points streamed during flight. **Not** the computed
trajectory &mdash; trajectory polynomials are computed client-side from these points.

Each frame has a 4-byte header followed by N sub-records of 60 bytes:

```
[0]     UINT8   header_byte: (header_byte - 3) / 60 = N sub-records
[1-2]   INT16   frame sequence number (increments by N per frame)
[3]     UINT8   sub_count (= N)
```

Version detection: `(header_byte - 3) % 60 == 0` &rarr; version 4 (60B stride).
Older versions use stride 26 (v3) or 23 (v2), but only v4 has been observed.

**Sub-record layout** (60 bytes, version 4):

| Offset | Size | Type  | Scale               | Field     |
| ------ | ---- | ----- | ------------------- | --------- |
| 0      | 2    | INT16 | raw                 | index     |
| 2      | 2    | INT16 | raw                 | peak      |
| 4      | 3    | INT24 | raw                 | SNR       |
| 7      | 2    | INT16 | raw                 | BufIdx    |
| 9      | 1    | BYTE  | raw                 | flags     |
| 10     | 3    | INT24 | raw                 | Time      |
| 13     | 3    | INT24 | /100000             | n         |
| 16     | 2    | INT16 | /100                | Az (deg)  |
| 18     | 2    | INT16 | /100                | El (deg)  |
| 20     | 3    | INT24 | /100                | Vel (m/s) |
| 23     | 3    | INT24 | /1000               | Dist (m)  |
| 26     | 3    | INT24 | raw                 | SyncIdx   |
| 29     | 3    | INT24 | raw                 | SyncBuf   |
| 32     | 2    | INT16 | /100                | Az1 (deg) |
| 34     | 2    | INT16 | /100                | Az2 (deg) |
| 36     | 2    | INT16 | /100                | Az3 (deg) |
| 38     | 2    | INT16 | /100                | El1 (deg) |
| 40     | 2    | INT16 | /100                | El2 (deg) |
| 42     | 3    | INT24 | &times;(10000/2^23) | Pk0       |
| 45     | 3    | INT24 | &times;(10000/2^23) | Pk1       |
| 48     | 3    | INT24 | &times;(10000/2^23) | Pk2       |
| 51     | 3    | INT24 | &times;(10000/2^23) | Pk3       |
| 54     | 3    | INT24 | &times;(10000/2^23) | Pk4       |
| 57     | 3    | INT24 | &times;(10000/2^23) | Pk5       |

Time counter resolution: ~26.7 &mu;s per count. Typical point spacing: 32 counts
(~853 &mu;s) at close range, 128 counts (~3.4 ms) at far range. 46-112 points per shot.

Vel is the radial (line-of-sight) velocity, typically 1.6-3% below the true
launch speed from 0xD4 due to the radial vs total velocity difference.

**Retransmission**: After the initial AVR-pushed burst, the APP re-fetches all pages
via 4-byte EC requests (`[03 00 XX 08]`) and the AVR responds frame-by-frame.

#### 0xEE &mdash; CLUB_PRC (76-byte sub-records, paginated)

Raw club head radar tracking points. Paginated bulk fetch:

- **Request** (APP &rarr; AVR, 77B): `[stride=0x4C, start_index(2), ...]`
- **Response** (AVR &rarr; APP): `[data_len(1), sub_records...]`
  - `data_len` = 0xE4 (228 = 3&times;76) for full pages, 0x98 (152 = 2&times;76) for final
  - Records fetched 3 at a time, start_index increments by 3

**Sub-record layout** (76 bytes):

| Offset | Size | Type    | Scale               | Field      |
| ------ | ---- | ------- | ------------------- | ---------- |
| 0      | 2    | INT16   | raw                 | index      |
| 2      | 2    | INT16   | raw                 | bufOfs     |
| 4      | 2    | INT16   | raw                 | peak       |
| 6      | 3    | INT24   | raw                 | SNR        |
| 9      | 2    | INT16   | raw                 | BufIdx     |
| 11     | 3    | INT24   | raw                 | Time       |
| 14     | 3    | INT24   | /100000             | n          |
| 17     | 2    | INT16   | /100                | Az (deg)   |
| 19     | 2    | INT16   | /100                | El (deg)   |
| 21     | 3    | INT24   | /100                | Vel (m/s)  |
| 24     | 3    | INT24   | /100                | Vel2 (m/s) |
| 27     | 3    | INT24   | /1000               | Dist (m)   |
| 30     | 3    | INT24   | /1000               | f30        |
| 33     | 3    | INT24   | /1000               | f33        |
| 36     | 2    | &mdash; | &mdash;             | (gap)      |
| 38     | 1    | BYTE    | raw                 | version    |
| 39     | 3    | INT24   | raw                 | f39        |
| 42     | 3    | INT24   | raw                 | f42        |
| 45     | 3    | INT24   | /1000               | f45        |
| 48     | 2    | INT16   | /100                | Az1 (deg)  |
| 50     | 2    | INT16   | /100                | Az2 (deg)  |
| 52     | 2    | INT16   | /100                | Az3 (deg)  |
| 54     | 2    | INT16   | /100                | El1 (deg)  |
| 56     | 2    | INT16   | /100                | El2 (deg)  |
| 58     | 3    | INT24   | &times;(10000/2^23) | Pk0        |
| 61     | 3    | INT24   | &times;(10000/2^23) | Pk1        |
| 64     | 3    | INT24   | &times;(10000/2^23) | Pk2        |
| 67     | 3    | INT24   | &times;(10000/2^23) | Pk3        |
| 70     | 3    | INT24   | &times;(10000/2^23) | Pk4        |
| 73     | 3    | INT24   | &times;(10000/2^23) | Pk5        |

`bufOfs` is a signed offset from the trigger point. Negative = before impact
(downswing, -1600 to 0), positive = after impact (follow-through, 0 to +2100).

#### 0xE5 &mdash; SHOT_TEXT (variable, 6-72 bytes)

ASCII text messages indicating shot processing state:

- `"BALL TRIGGER: N ms back, at Epoch ..."` &mdash; shot detected
- `"Clubimpact at Epoch ..."` &mdash; club impact timing
- `"\nPROCESSED\0"` &mdash; processing complete
- `"\x05IDLE\0"` &mdash; system idle, ready for next shot

### 6.2 Status and Keepalive

#### 0xAA &mdash; STATUS (bidirectional)

Polled every ~1 s to each node. Request payload is always 2 bytes: `[01 01]`
for DSP/AVR, `[01 03]` for PI. Response format depends on source.

**AVR response** (25 bytes, SRC=0x30):

| Bytes   | Type    | Field                                         |
| ------- | ------- | --------------------------------------------- |
| [0]     | UINT8   | Version (0x18 = 24)                           |
| [1]     | UINT8   | State (0=idle, 1=armed, 2=arming, 3=tracking) |
| [2]     | UINT8   | HardwareID high byte                          |
| [3-4]   | INT16   | (reserved)                                    |
| [5]     | UINT8   | HardwareID low byte                           |
| [6-7]   | 2B      | (reserved)                                    |
| [8-10]  | INT24   | FullAppID                                     |
| [10-14] | FLOAT40 | Temperature (&deg;C)                          |
| [15-19] | FLOAT40 | Tilt (deg)                                    |
| [20-24] | FLOAT40 | Roll (deg, sign negated)                      |

**DSP response** (129 bytes, SRC=0x40):

Contains battery and power information. Key fields:

| Bytes   | Type  | Field                           |
| ------- | ----- | ------------------------------- |
| [0]     | UINT8 | Version (0x80 = 128)            |
| [1]     | UINT8 | State                           |
| [4-5]   | INT16 | InputVoltageUSB (mV, ~4900)     |
| [8-9]   | INT16 | SystemVoltage (mV, ~3300)       |
| [18-19] | INT16 | BatteryCurrent (mA)             |
| [40-41] | INT16 | Temperature/100 (&deg;C)        |
| [53-54] | INT16 | BatteryVoltage (mV)             |
| [57-58] | INT16 | BatteryVoltage2 (mV)            |
| [61-62] | INT16 | PowerLevel (high byte = 0-100%) |
| [63]    | UINT8 | ExternalPowerConnected (bool)   |

### 6.3 Configuration

#### 0xA5 &mdash; MODE_SET (3 bytes)

Set detection mode. APP &rarr; AVR. Payload: `[02 00 XX]` where `XX` is the
commsIndex (see Section 7). AVR echoes the same payload back as acknowledgment.

#### 0xB0 &mdash; CONFIG (2 bytes)

AVR configuration control. APP &rarr; AVR.

- `[01 00]` &mdash; config exchange (sent before parameter writes)
- `[01 01]` &mdash; **arm trigger** (starts radar measurement)

Each is acknowledged with a 0x95 CONFIG_ACK.

#### 0xB1 &mdash; MODE_ACK (3 bytes)

Mode reset acknowledgment. AVR &rarr; APP. Always `[02 00 00]`. Appears once per
mode change or settings change cycle.

#### 0xBE &mdash; PARAM_READ_REQ (4 bytes)

Parameter read request. APP &rarr; AVR or APP &rarr; PI.

Format: `[03 00 00 XX]` where `XX` is the parameter ID (low byte of a 16-bit
param ID; high byte is always 0x00 in observed traffic).

#### 0xBF &mdash; PARAM_VALUE (7-9 bytes)

Dual-use: parameter read response (device &rarr; APP) and parameter write (APP &rarr; device).
Writes are acknowledged with 0x95 CONFIG_ACK.

Format:

```
INT24 values:  [06 00 00 param_id value_hi value_mid value_lo]      (7B)
FLOAT40 values: [08 00 00 param_id exp_hi exp_lo mant_hi mant_mid mant_lo]  (9B)
```

Key writable parameters (APP &rarr; AVR):

| Param ID | Type    | Description                                |
| -------- | ------- | ------------------------------------------ |
| 0x0006   | INT24   | Ball type (0 = RCT, 1 = Standard)          |
| 0x000F   | FLOAT40 | Outdoor minimum track percentage (0.6-1.0) |
| 0x0026   | FLOAT40 | Driver tee height (meters)                 |
| 0x0007   | INT24   | Radar config (mode-dependent)              |
| 0x0008   | INT24   | Surface firmness index                     |
| 0x0016   | INT24   | Mode sub-index                             |
| 0x0025   | INT24   | Config flags                               |

#### 0xA4 &mdash; RADAR_CAL (7 bytes)

Radar calibration. Bidirectional: APP sends, AVR echoes back.

Format: `[06 range_hi range_lo 00 height_mm 00 00]`

- Bytes 1-2: sensor-to-tee distance (UINT16, millimeters)
- Byte 4: `floor(surfaceHeight_inches * 25.4)` (mm, truncated)

#### 0x95 &mdash; CONFIG_ACK (3 bytes)

Generic command acknowledgment. Device &rarr; APP only.

Format: `[02 bus_addr acked_cmd]`

- `bus_addr`: responding subsystem (0x30 = AVR, 0x12 = PI)
- `acked_cmd`: low 7 bits of the command being acknowledged (`cmd_type & 0x7F`)

| Ack byte | Acknowledges       |
| -------- | ------------------ |
| `0x3F`   | 0xBF param write   |
| `0x30`   | 0xB0 config/arm    |
| `0x52`   | 0xD2 cal data      |
| `0x01`   | 0x81 camera state  |
| `0x02`   | 0x82 camera config |

### 6.4 Device Info

#### 0x48 / 0xC8 &mdash; DSP_QUERY / DSP_QUERY_RESP

Request: empty (0 bytes), APP &rarr; DSP.
Response: 3 bytes, DSP &rarr; APP.

```
[0]  version   (0x02)
[1]  dspType   (0x80 = 128, Mevo+ radar DSP)
[2]  pcb       (0x0E = 14, PCB revision)
```

Constant across sessions (same device).

#### 0x67 / 0xE7 &mdash; DEV_INFO_REQ / DEV_INFO_RESP

Request: empty, APP &rarr; DSP/AVR/PI.
Response: 75-76 bytes of ASCII strings: model name, firmware version, serial
number, build date/time.

#### 0xFD &mdash; PROD_INFO

Request: 2 bytes `[01 XX]` with sub-query (0x00, 0x08, 0x09).
Response: 34 bytes ASCII. Contains Pi hardware ID, camera model.

#### 0xDE &mdash; NET_CONFIG

Request: 2 bytes `[01 XX]` (0x00 = SSID, 0x08 = password).
Response: 54 bytes ASCII. Network SSID and password.

### 6.5 Calibration

#### 0x21 / 0xA0 &mdash; CONFIG_QUERY / CONFIG_RESP (TParameters)

Request: empty, APP &rarr; DSP and APP &rarr; AVR.
Response: 69 bytes = `[size(1), 34 * INT16_BE(68)]`.

34 named radar configuration parameters. All constant for a given device.
Key parameters:

| Index | Name        | AVR Value | Description                       |
| ----- | ----------- | --------- | --------------------------------- |
| 0     | MaxVel      | 120       | Max trackable velocity (m/s)      |
| 3     | AntennaFreq | 24140     | Radar frequency (MHz = 24.14 GHz) |
| 14    | AntennaX    | 2400      | Distance behind tee (mm)          |
| 28    | PreTrigger  | 8192      | Pre-trigger buffer (samples)      |
| 29    | PostTrigger | 4         | PRC format version                |

#### 0x23 / 0xA2 &mdash; AVR_CONFIG_QUERY / AVR_CONFIG_RESP

Request: empty, APP &rarr; AVR.
Response: 17 bytes, constant. Contains version, gain factors (1000 = unity),
and configuration bytes.

```
[0]     size = 0x10 (16)
[1]     version = 0x01
[2]     0xE1
[3-4]   0x03E8 = 1000 (gain factor)
[5]     0x00
[6]     0x0C = 12
[7-8]   0x03E8 = 1000 (gain factor)
[9]     0x00
[10]    0x3D = 61
[11-16] zeros
```

#### 0xD0 / 0xD1 &mdash; CAL_PARAM_REQ / CAL_PARAM_RESP

Request: `[02 00 08]` (3 bytes), APP &rarr; AVR.
Response: 242 bytes, constant (factory-programmed).

Contains calibrator name (16 bytes, truncated), calibration date (19 bytes),
and INT16 gain/offset arrays (8 channels at unity = 1000).

#### 0xD2 / 0xD3 &mdash; CAL_DATA_REQ / CAL_DATA_RESP

Two sub-commands with distinct formats:

**Sub-cmd 0x03** (handshake): Factory calibration info.
Request: `[09 00 00 03 00 00 00 00 00 A5]` (10 bytes).
Response: 175 bytes with calibrator name, date, and 18 FLOAT40 factory constants
(angular offsets, radar geometry).

**Sub-cmd 0x07** (post-shot): Full device parameter dump.
Request: `[09 00 00 07 00 00 00 00 00 00]` (10 bytes).
Response: Paginated across 4 frames (30 + 29 + 7 entries + terminator).

Each entry is a TLV:

```
Type 0x01: [01 00 param_hi param_lo INT24(3B)]     — 7 bytes total
Type 0x02: [02 00 param_hi param_lo FLOAT40(5B)]   — 9 bytes total
```

66 total parameters. Includes readback of writable config (ball type, tee height,
track percentage) and live sensor readings (temperature, drift).

#### 0x9B &mdash; TIME_SYNC (9 bytes)

Bidirectional time synchronization. APP sends local epoch time; AVR echoes it.

```
[0]     0x08 (remaining size)
[1]     0x00
[2-5]   Epoch timestamp (UINT32 BE, seconds since 1970-01-01 UTC)
[6]     Session byte (same both directions)
[7-8]   Direction-specific tail bytes
```

For client implementation: send current epoch at bytes [2-5]; bytes [6-8] can be
arbitrary. Receive and discard the AVR response.

### 6.6 Camera

All camera messages flow between APP (0x10) and PI (0x12).

#### 0x81 &mdash; CAM_STATE (2 bytes)

Camera start/stop. Payload: `[01 XX]`.

| Direction     | XX   | Meaning                   |
| ------------- | ---- | ------------------------- |
| APP &rarr; PI | 0x00 | Stop camera               |
| APP &rarr; PI | 0x01 | Start camera (basic mode) |
| APP &rarr; PI | 0x03 | Start camera + streaming  |
| PI &rarr; APP | 0x00 | Camera off                |
| PI &rarr; APP | 0x01 | Camera on                 |

Timeout: 3 s, 6 retries.

#### 0x82 &mdash; CAM_CONFIG (52 bytes)

Camera configuration exchange. S51 format (payload[0] = 0x33).
Bidirectional: PI sends current config in response to 0x83; APP sends to push.

```
Offset  Size  Type      Field
──────  ────  ────────  ──────────────────────────────────────
[0]     1     UINT8     Size (0x33 = 51, S51 marker)
[1]     1     bool      dynamic_config
[2-3]   2     INT16     resolution_width (1024 or 1640)
[4-5]   2     INT16     resolution_height (768 or 1232)
[6-7]   2     INT16     rotation
[8-9]   2     INT16     ev (exposure value)
[10]    1     UINT8     quality (20 or 80)
[11]    1     UINT8     framerate (10 or 20)
[12]    1     UINT8     streaming_framerate (1 or 20)
[13-14] 2     INT16     ringbuffer_pretime_ms (1000 or 2000)
[15-16] 2     INT16     ringbuffer_posttime_ms (1500 or 4000)
[17]    1     UINT8     rawCameraMode
[18]    1     bool      fusionCameraMode
[19-23] 5     FLOAT40   rawShutterSpeedMax
[24-25] 2     INT16     rawEvRoiX (-1)
[26-27] 2     INT16     rawEvRoiY (-1)
[28-29] 2     INT16     rawEvRoiWidth (-1)
[30-31] 2     INT16     rawEvRoiHeight (-1)
[32-33] 2     INT16     rawXOffset (-1)
[34]    1     bool      rawBin44
[35-36] 2     INT16     rawLivePreviewWriteInterval_ms (-1)
[37-38] 2     INT16     rawYOffset (-1)
[39-40] 2     INT16     bufferSubSamplingPreTriggerDiv (-1)
[41-42] 2     INT16     bufferSubSamplingPostTriggerDiv (-1)
[43-47] 5     FLOAT40   bufferSubSamplingSwitchTimeOffset (-1.0)
[48-49] 2     INT16     bufferSubSamplingTotalBufferSize (-1)
[50-51] 2     INT16     bufferSubSamplingPreTriggerBufferSize (-1)
```

#### 0x83 &mdash; CAM_CONFIG_REQ (3 bytes)

Camera config request. APP &rarr; PI. Always `[02 01 05]`. Each elicits one 0x82
response.

#### 0x84 &mdash; CAM_IMAGE_AVAIL (67 bytes)

Per-shot camera image notification. PI &rarr; APP. 1 per shot.

Long form (payload[0] = 0x42):

```
[0]     UINT8   Size (0x42 = 66)
[1]     UINT8   streamingFlags (bit 0: streaming available)
[2]     UINT8   fusionFlags (bit 0: fusion, bit 1: video)
[3-34]  32B     streamingTimestamp (null-padded ISO: YYYY.MM.DDTHH.MM.SS.mmm)
[35-66] 32B     fusionTimestamp (same format)
```

Short form (payload[0] = 0x01): 2 bytes, `[01 flags]`. No timestamps.

### 6.7 Licensing

#### 0x90 / 0x89 &mdash; SENSOR_ACT / SENSOR_ACT_RESP

Client-side licensing exchange. APP &rarr; PI sends activation data in 12 chunks
(1 init `[01 01]` + 11 data chunks, 843 bytes total). PI responds with a stored
device certificate.

The device arms without this exchange &mdash; it is not required for operation.
Zero-filled data is accepted. The certificate is parsed client-side only for
premium feature gating.

### 6.8 Shot Acknowledgment Flow

Two zero-payload control messages drive post-shot data retrieval.
Both APP &rarr; AVR only.

#### 0x69 &mdash; SHOT_DATA_ACK (0 bytes)

Sent 2&times; per shot after E5 "PROCESSED". Each elicits one E9 TRACKING_STATUS
retransmission. After the second ack, APP initiates PRC_DATA bulk re-fetch.

#### 0x6D &mdash; SHOT_RESULT_REQ (0 bytes)

Sent 1&times; per shot after E5 "IDLE" and config re-query. Triggers a duplicate
0xED CLUB_RESULT (byte-identical to the first) and initiates 0xEE CLUB_PRC
paginated bulk fetch.

### 6.9 Text and Debug

#### 0xE3 &mdash; TEXT (variable)

ASCII debug/log messages from device subsystems. Examples:

- `"ARMED DetectionMode=9"`
- `"System State 6"`
- `"DSP: CameraParam Read PASS (165)"`
- `"ARMED CANCELLED"`

#### 0x87 &mdash; WIFI_SCAN (variable)

Paginated SSID list. PI &rarr; APP. Contains nearby WiFi network names.
Request format: `[03 00 XX 02]` where `XX` is the page offset.

---

## 7. Detection Modes

The commsIndex value is sent via 0xA5 `[02 00 XX]` to set the detection mode.

| commsIndex | Mode                  | Description         | Notes                                           |
| ---------- | --------------------- | ------------------- | ----------------------------------------------- |
| 1          | Indoor                | Indoor              | Never observed in pcap                          |
| **2**      | **LongIndoor**        | **Long Indoor**     | Observed in pcap (transient, never armed alone) |
| **3**      | **ShortIndoor**       | **Putting**         | MaxVel=30, Fs=9868.4, GainInit=1                |
| 4          | ClubSwing             | Club Swing Only     | No ball tracking. Never observed in pcap        |
| **5**      | **SimulatorChipping** | **Chipping**        | MaxVel=120, Fs=37500.0                          |
| 6          | SimulatorPutting      | Sim Putting         | Never observed in pcap                          |
| **9**      | **Outdoor**           | **Normal/Outdoor**  | MaxVel=120, Fs=37500.0, club trigger off        |
| 13         | RawSampling           | Environmental       | Never observed in pcap                          |
| 14         | Putting               | Putting (dedicated) | Never observed in pcap                          |
| 15         | ShortGameChipIn       | Chipping Indoor     | Never observed in pcap                          |
| 16         | ShortGameChipOut      | Chipping Outdoor    | Never observed in pcap                          |

Bold entries have been observed in pcap captures. Modes 3, 5, and 9 are the
primary operating modes. Mode 2 (LongIndoor) only appears as a transient
intermediate step during configuration &mdash; it is set and then immediately
overwritten by the final target mode before arming.

---

## 8. Coordinate System

### Wire Coordinates (DSP)

Wire values use DSP coordinates:

- **X** = forward (range, toward target)
- **Y** = vertical (height)
- **Z** = lateral (left/right)

### Units

All wire values use metric:

- Speeds: encoded as millidegrees or centidegrees divided to m/s (see per-message scale factors)
- Angles: millidegrees in D4/E8 (&div;1000), centidegrees in ED (&div;100)
- Distances: millimeters (&div;1000 for meters)
- Spin: RPM (direct)

### DSP-to-PC Transform

The internal flight model uses a rotated coordinate system:

```
PC_X = -DSP_Z    (negate lateral axis)
PC_Y =  DSP_X    (forward becomes Y)
PC_Z =  DSP_Y    (vertical becomes Z)
```

Applied to all Vector3 fields (positions, velocities, spin vectors) and polynomial
arrays. Several scalar angular fields are also negated: launch azimuth, sidespin,
club strike direction, swing plane rotation, club face angle, spin axis.

**For display purposes, wire values can be used directly.** The DSP-to-PC
transform is only relevant for internal trajectory computation.
