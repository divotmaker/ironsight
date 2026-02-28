# Camera Protocol Reference

The Mevo+ Raspberry Pi camera processor (GVP — Golf Video Processor) exposes
two additional TCP services alongside the binary protocol on port 5100.

---

## 1. Port 1258 — GVP JSON Protocol

Null-terminated JSON messages (`\x00` delimiter) over TCP. Used for camera
configuration, shot triggering, image tracking, and results.

### 1.1 Transport

- Each message is a UTF-8 JSON object terminated by `\x00`
- Multiple messages may arrive in a single TCP segment
- Structured messages have `"type"` and `"version"` fields at the top level
- Connection is established by the APP after the binary handshake completes

### 1.2 Message Types

Nine message types observed across pcap captures:

| Type | Version | Direction | Purpose |
| ---- | ------- | --------- | ------- |
| `CONFIG_REQUEST` | 1 | APP &rarr; GVP | Query current camera config |
| `CONFIG` | 5 | Both | Camera config exchange |
| `STATUS` | 1 | GVP &rarr; APP | Buffer status updates |
| `TRIGGER` | 6 | APP &rarr; GVP | Notify camera of shot trigger |
| `LOG` | 1 | GVP &rarr; APP | Debug log messages |
| `MT_GOLF_EXPECTED_CLUB_TRACK` | 1 | APP &rarr; GVP | Radar-derived club trajectory hint |
| `MT_GOLF_EXPECTED_TRACK` | 1 | APP &rarr; GVP | Radar-derived ball trajectory hint |
| `RESULT` | 1 | GVP &rarr; APP | BallTrackerResult: raw pixel tracking output |
| `MT_VIDEO_AVAILABLE` | 1 | GVP &rarr; APP | Video file saved notification |

### 1.3 CONFIG_REQUEST

Sent by APP to request the current camera configuration. GVP responds with a
CONFIG message followed by a STATUS message.

```json
{
  "type": "CONFIG_REQUEST",
  "version": 1
}
```

### 1.4 CONFIG

Bidirectional. GVP sends current config in response to CONFIG_REQUEST; APP sends
to push updated settings. APP re-sends CONFIG several times (~200 ms apart)
during the handshake, and once more per shot after TRIGGER.

```json
{
  "type": "CONFIG",
  "version": 5,
  "bufferConfiguration": {
    "bufferSizePreTrigger": 10,
    "bufferSizePostTrigger": 200
  },
  "cameraCalibration": {
    "cx": 0, "cy": 0,
    "fx": 0, "fy": 0,
    "width": 0, "height": 0,
    "position": [0, 0, 0],
    "rotation": [0, 0, 0],
    "distCoeffs": [0, 0, 0, 0, 0, 0, 0, 0]
  },
  "cameraConfiguration": {
    "ROI_x": 0, "ROI_y": 0,
    "ROI_width": 1024, "ROI_height": 768,
    "ROI_maxWidth": 0, "ROI_maxHeight": 0,
    "isFreeRun": true,
    "rotationDegCW": 0
  },
  "livePreviewProcessingConfiguration": {
    "ROI_center_u": 0, "ROI_center_v": 0,
    "ROI_width": 320, "ROI_height": 160,
    "enabled": false,
    "rotationDegCW": 0
  },
  "frameNumberInfoEnabled": true,
  "loggingEnabled": true,
  "saveVideosEnabled": true
}
```

The `cameraCalibration` and `cameraConfiguration` values shown are defaults.
When the APP pushes config, it typically zeroes out most fields (GVP uses its
own stored calibration).

The `ROI_width`/`ROI_height` in `cameraConfiguration` reflect the capture
resolution. Two configurations have been observed:

| Mode | Width | Height | Notes |
| ---- | ----- | ------ | ----- |
| Standard | 1024 | 768 | Default |
| Fusion | 1640 | 1232 | High-res, enables face impact fusion |

These correspond to the binary 0x82 CAM_CONFIG resolution fields on port 5100
(see WIRE.md).

### 1.5 STATUS

Buffer status updates. Sent by GVP after CONFIG_REQUEST and during shot
processing.

```json
{
  "type": "STATUS",
  "version": 1,
  "bufferStatus": [
    {
      "bufferIndex": 0,
      "status": "IDLE"
    }
  ]
}
```

Status transitions per shot:

```
IDLE → TRIGGERED → CONVERTING → PROCESSING → SAVING → IDLE
```

- **IDLE**: Ready for next trigger
- **TRIGGERED**: Camera buffer capture in progress
- **CONVERTING**: Decoding captured frames (histogram, preload, decode)
- **PROCESSING**: SimpleObjectTracker running (club then ball)
- **SAVING**: Encoding and saving video (h264 → ffmpeg → copy)
- **IDLE**: Processing complete, ready again

### 1.6 TRIGGER

Sent by APP to GVP when the binary protocol reports a ball trigger (0xE5
"BALL TRIGGER" on port 5100). Contains a GUID that ties the camera data to the
shot.

```json
{
  "type": "TRIGGER",
  "version": 6,
  "guid": "{aa26908f-2ef3-45f6-9fad-b3ae7db97313}",
  "epochTime": 1771904973.258451,
  "frameNumber": 0,
  "savePath": "{aa26908f-2ef3-45f6-9fad-b3ae7db97313}",
  "reprocessPath": "",
  "skipTracking": false,
  "triggerTimeOffset": 0
}
```

| Field | Type | Description |
| ----- | ---- | ----------- |
| `guid` | string | UUID for this shot (used in binary E9 DTO client-side) |
| `epochTime` | float | Trigger time as Unix epoch (fractional seconds) |
| `frameNumber` | int | Always 0 in observed traffic |
| `savePath` | string | On-device storage path (= guid) |
| `reprocessPath` | string | Empty in normal operation |
| `skipTracking` | bool | Always false in normal operation |
| `triggerTimeOffset` | float | Always 0 in observed traffic |

### 1.7 LOG

Debug/informational messages from the GVP processor.

```json
{
  "type": "LOG",
  "version": 1,
  "level": 1,
  "message": "[GVP][2026-02-24 05:48:41.199][407.441/743.766MB][Comms] Updating dynamic config"
}
```

`level` values observed: 1 (info), 2 (debug/encoding). The `message` field
contains timestamped log text with memory usage and subsystem tags.

### 1.8 MT_GOLF_EXPECTED_CLUB_TRACK

Sent by APP to GVP after radar launch data is available. Contains a 4th-order
polynomial describing the expected club head trajectory in camera pixel space,
derived from radar EE CLUB_PRC tracking points.

```json
{
  "type": "MT_GOLF_EXPECTED_CLUB_TRACK",
  "version": 1,
  "guid": "{4a3e95df-79cd-4fb0-a7f2-87af7f653298}",
  "duration": 0.1,
  "startTime": 0.014213,
  "polyU": [308.79, 0.0129, -0.119, -15.205, 293.455],
  "polyV": [237.32, 0.0072, 0.300, -8.510, 28.224],
  "polyRadius": [1, 16.332, 2.713, 0, 0]
}
```

| Field | Type | Description |
| ----- | ---- | ----------- |
| `guid` | string | Shot GUID (matches TRIGGER) |
| `duration` | float | Time window for polynomial evaluation (seconds) |
| `startTime` | float | Start time offset from trigger (seconds) |
| `polyU` | float[5] | 4th-order polynomial coefficients for u (horizontal pixel) |
| `polyV` | float[5] | 4th-order polynomial coefficients for v (vertical pixel) |
| `polyRadius` | float[5] | 4th-order polynomial for expected object radius (pixels) |

The polynomial `p(t) = c[0] + c[1]*t + c[2]*t^2 + c[3]*t^3 + c[4]*t^4`
predicts the expected position in camera pixel coordinates. The GVP's
SimpleObjectTracker uses this as a search hint to find the club head in each
frame.

The APP computes these polynomials from the radar tracking data (EE CLUB_PRC
points converted to pixel coordinates via camera geometry).

### 1.9 MT_GOLF_EXPECTED_TRACK

Same structure as MT_GOLF_EXPECTED_CLUB_TRACK, but for the ball trajectory.
Derived from radar EC PRC_DATA tracking points.

```json
{
  "type": "MT_GOLF_EXPECTED_TRACK",
  "version": 1,
  "guid": "{4a3e95df-79cd-4fb0-a7f2-87af7f653298}",
  "duration": 0.1,
  "startTime": 0.014213,
  "polyU": [304.23, 1146.59, -7727.58, 40503.22, -107020.24],
  "polyV": [348.22, -4328.94, 29239.04, -151572.94, 393951.34],
  "polyRadius": [1, 18.163, 2.713, 0, 0]
}
```

Ball polynomial coefficients are much larger than club coefficients because the
ball moves faster and farther in pixel space over the same time interval.

### 1.10 MT_VIDEO_AVAILABLE

Sent by GVP after the shot video has been encoded and saved.

```json
{
  "type": "MT_VIDEO_AVAILABLE",
  "version": 1,
  "guid": "{4a3e95df-79cd-4fb0-a7f2-87af7f653298}",
  "absolutePath": "/home/ftp/backup/{4a3e95df-...}/video.mp4",
  "relativePath": "/{4a3e95df-...}/video.mp4"
}
```

### 1.11 RESULT (BallTrackerResult)

Sent by GVP after completing image tracking for both club and ball. Contains
raw pixel tracking data from the SimpleObjectTracker (~3200 bytes).

```json
{
  "type": "RESULT",
  "version": 1,
  "guid": "{4a3e95df-79cd-4fb0-a7f2-87af7f653298}",
  "cameraCalibration": {
    "cx": 0, "cy": 0, "fx": 0, "fy": 0,
    "width": 0, "height": 0,
    "position": [0, 0, 0], "rotation": [0, 0, 0],
    "distCoeffs": [0, 0, 0, 0, 0, 0, 0, 0]
  },
  "tracks": [
    {
      "trackId": 1,
      "frameNumber": [13, 15, 17, 18, ...],
      "timestamp": [1771985894.233, ...],
      "u": [218.20, 248.65, 269.63, ...],
      "v": [124.90, 198.90, 256.08, ...],
      "radius": [56.15, 55.13, 56.67, ...],
      "circularityFactor": [59.80, 53.35, 43.48, ...],
      "shutterTime_ms": [1, 1, 1, ...]
    },
    ...
  ]
}
```

**Track fields:**

| Field | Type | Description |
| ----- | ---- | ----------- |
| `trackId` | int | Object identifier (see table below) |
| `frameNumber` | int[] | Camera frame indices where object was detected |
| `timestamp` | float[] | Unix epoch timestamps per frame (fractional seconds) |
| `u` | float[] | Horizontal pixel coordinates (sub-pixel precision) |
| `v` | float[] | Vertical pixel coordinates (sub-pixel precision) |
| `radius` | float[] | Detected object radius in pixels |
| `circularityFactor` | float[] | Shape metric (high = circular, low = elongated) |
| `shutterTime_ms` | int[] | Shutter time per frame (always 1 ms observed) |

**Track IDs:**

| trackId | Object | Typical points | Radius (px) | Notes |
| ------- | ------ | -------------- | ----------- | ----- |
| 1 | Club head | 14-15 | 30-56 | Pre- and post-impact frames |
| 0 | Ball | 10 | 8-16 | From impact through early flight |
| 2 | Reference | 1 | ~12 | Ball position at trigger frame (cf=31) |
| 3 | Reference | 1 | ~0.05 | Near-zero marker (purpose TBD) |
| 4 | Reference | 1 | 1.0 | Unit reference at trigger epoch (cf=31) |

All tracks use the same coordinate system (camera sensor pixels). The
`cameraCalibration` in the RESULT is always all-zeros in observed traffic — the
GVP does not provide its own calibration; the client is expected to use
calibration data from the binary protocol (0xD1 CAM_PARAM_RESP).

**Face impact is NOT in this message.** The RESULT contains only raw pixel
coordinates. Face impact location is computed client-side by fusing these
camera tracking points with radar data (EC PRC_DATA, EE CLUB_PRC).

### 1.12 Connection Sequence

```
APP→GVP   CONFIG_REQUEST
GVP→APP   STATUS          {"status": "IDLE"}
GVP→APP   CONFIG          (current camera config, version 5)
APP→GVP   CONFIG          (updated settings)
GVP→APP   LOG ×3          (config change confirmations)
APP→GVP   CONFIG          (re-sent ~200ms later, ×2-3 during handshake)
```

### 1.13 Per-Shot Sequence

```
                                         Binary protocol (port 5100)
                                         ─────────────────────────────
                                         AVR→APP  0xE5  "BALL TRIGGER"
APP→GVP   TRIGGER         {guid, epochTime, ...}
APP→GVP   CONFIG          (re-sent with current settings)
GVP→APP   LOG             "GVP version 1.20" / "Image processing version 20"
GVP→APP   LOG             "Triggered, waiting for files..."
GVP→APP   STATUS          {"status": "TRIGGERED"}
GVP→APP   LOG             "Files available!"
GVP→APP   STATUS          {"status": "CONVERTING"}
GVP→APP   LOG ×N          (histogram, probing, preloading, decoding)
GVP→APP   LOG             "performProcess() - finished"
                                         AVR→APP  0xE8  LAUNCH_RESULT (radar)
                                         AVR→APP  0xEC  PRC_DATA (radar)
APP→GVP   MT_GOLF_EXPECTED_CLUB_TRACK    (polynomial from radar EE data)
GVP→APP   LOG             "Setting expected club track..."
GVP→APP   LOG             "Started processing club."
GVP→APP   STATUS          {"status": "PROCESSING"}
GVP→APP   LOG             "Tracking club..."
                                         AVR→APP  0xEC  PRC_DATA (more radar)
APP→GVP   MT_GOLF_EXPECTED_TRACK         (polynomial from radar EC data)
GVP→APP   LOG             "Processing took N ms"
GVP→APP   LOG             "Found N club points"
GVP→APP   LOG             "Setting expected track..."
GVP→APP   LOG             "Started processing ball."
GVP→APP   LOG             "Tracking ball..."
GVP→APP   LOG             "Found N ball points"
GVP→APP   LOG             "Sending result BallTrackerResult"
GVP→APP   RESULT          (BallTrackerResult: ~3200B raw pixel tracks)
GVP→APP   STATUS          {"status": "SAVING"}
GVP→APP   LOG ×N          (h264 encoding, ffmpeg, copy)
GVP→APP   MT_VIDEO_AVAILABLE  {absolutePath, relativePath, guid}
GVP→APP   LOG             "Free RAM: N MB" / storage stats
GVP→APP   STATUS          {"status": "IDLE"}
```

Timing (observed from face_impact_shot_id_4.pcapng, shot 2):
- TRIGGER to CONVERTING: ~600ms (waiting for camera files)
- CONVERTING to club track sent: ~290ms (image decode)
- Club tracking: ~350ms
- Ball tracking: ~160ms
- Video encoding: ~450ms
- Total TRIGGER to IDLE: ~1.2s

---

## 2. Port 8080 — MJPEG Stream

Standard HTTP MJPEG stream from the Raspberry Pi camera module (RasPiCam V2).

- Content-Type: `multipart/x-mixed-replace` with MIME boundary markers
- Frame rate and resolution controlled by CONFIG messages on port 1258 and
  binary 0x82 CAM_CONFIG on port 5100
- No custom framing or encoding beyond standard MJPEG

This stream is purely visual and not required for shot data. It provides a
live camera feed for alignment and face impact visualization in apps that
support it.

---

## 3. Client Implementation Notes

### Required for shot data: None

Neither port 1258 nor port 8080 is required for basic shot data integration.
All shot results (ball speed, launch angles, spin, club data) flow exclusively
through the binary protocol on port 5100.

### Face impact location

Face impact location is **computed client-side**. It is NOT present in any
wire protocol message.

The computation fuses three data sources:
1. **Radar tracking data** from port 5100 (EC PRC_DATA ball points, EE CLUB_PRC
   club head points)
2. **Camera tracking data** from port 1258 RESULT (~3200B raw pixel tracks for
   club and ball)
3. **Camera calibration** from port 5100 0xD1 CAM_PARAM_RESP (factory-programmed
   calibration constants)

Prerequisites for face impact:
- Camera running in Fusion mode (1640x1232 resolution)
- Adequate lighting (GVP needs to reliably track club head across 14+ frames)
- ProPackage license
- User calibration offsets (lateral, height)

The output is a 2D impact point (lateral, vertical) on the club face plus
an availability flag.

### Shot GUIDs

The TRIGGER message's `guid` field is used client-side to populate the E9
TRACKING_STATUS DTO's `guid` field. If your application needs shot correlation
IDs, generate and send TRIGGER messages.

### Minimal camera setup

If port 1258 is not needed, you can still start the camera via the binary
protocol alone (0x81 CAM_STATE `[01 01]` on port 5100) as part of the normal
handshake. The GVP will run but no JSON coordination occurs.
