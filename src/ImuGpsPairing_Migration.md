# ImuGpsPairing Migration Checklist

Use this checklist to port GPS/IMU sample-pairing logic into another firmware.

## 1) Copy module files

- Copy `ImuGpsPairing.h`
- Copy `ImuGpsPairing.cpp`

## 2) Add include and instances

In your main file (or a central globals file):

```cpp
#include "ImuGpsPairing.h"

// Tune timeouts per sensor type and project behavior.
ImuGpsPairing tm171Pairing(40, 45, false);      // initialPeriodMs, timeoutMs, filterStaleOnTimeout
ImuGpsPairing bnoPairing(GYRO_LOOP_TIME, 30, true);

int32_t tm171GpsDeltaMs = 0;
int32_t bnoGpsDeltaMs = 0;
```

## 3) Hook IMU sample arrival

Call `noteImuSample(millis())` when a valid sample payload is decoded.

### TM171 payload decode hook

```cpp
// Inside valid TM171 RPY packet handling:
tm171Pairing.noteImuSample(millis());
```

### BNO payload decode hook

```cpp
// Inside successful bno08x.dataAvailable() branch:
bnoPairing.noteImuSample(millis());
```

## 4) Hook GGA arrival

In your GGA handler, schedule pairing and allow immediate send if sample is already close.

### TM171 in GGA handler

```cpp
uint32_t ggaNow = millis();
tm171Pairing.onGga(ggaNow);

if (tm171Pairing.tryImmediatePair(ggaNow, tm171GpsDeltaMs))
{
  imuHandler();
  BuildNmea();
}
```

### BNO in GGA handler

```cpp
uint32_t ggaNow = millis();
bnoPairing.onGga(ggaNow);

if (bnoPairing.tryImmediatePair(ggaNow, bnoGpsDeltaMs))
{
  imuHandler();
  BuildNmea();
}
```

## 5) Hook loop pending evaluation

In your main loop, evaluate pending send requests and commit stats only for valid timing samples.

### TM171 loop hook

```cpp
ImuGpsPairing::Decision tm171Decision = tm171Pairing.evaluatePending(millis());
if (tm171Decision.shouldSend)
{
  imuHandler();

  if (tm171Decision.validForStats)
  {
    tm171GpsDeltaMs = tm171Decision.deltaMs;
    tm171Pairing.recordDelta(tm171GpsDeltaMs);
  }

  BuildNmea();
}
```

### BNO loop hook

```cpp
ImuGpsPairing::Decision bnoDecision = bnoPairing.evaluatePending(millis());
if (bnoDecision.shouldSend)
{
  imuHandler();

  if (bnoDecision.validForStats)
  {
    bnoGpsDeltaMs = bnoDecision.deltaMs;
    bnoPairing.recordDelta(bnoGpsDeltaMs);
  }

  BuildNmea();
}
```

## 6) Optional debug output

```cpp
const ImuGpsPairing::Stats &s = tm171Pairing.stats();
int32_t avgMs = (s.count > 0) ? (int32_t)(s.sumMs / (int64_t)s.count) : 0;

Serial.print("TM171/GGA dt=");
Serial.print(tm171GpsDeltaMs);
Serial.print("ms, min=");
Serial.print(s.minMs);
Serial.print("ms, max=");
Serial.print(s.maxMs);
Serial.print("ms, avg=");
Serial.print(avgMs);
Serial.print("ms, n=");
Serial.print(s.count);
Serial.print(", stale=");
Serial.print(s.staleCount);
Serial.print(", estPeriod=");
Serial.print(tm171Pairing.estimatedPeriod());
Serial.println("ms");
```

Note: `staleCount` is most useful when `filterStaleOnTimeout=true` (used for BNO in this repo).

## 7) Recommended defaults

- TM171:
  - Initial period: `40` ms
  - Timeout: `45` ms
  - `filterStaleOnTimeout=false`
- BNO:
  - Initial period: `GYRO_LOOP_TIME`
  - Timeout: `30` ms
  - `filterStaleOnTimeout=true`

## 8) Validation checklist

- Build succeeds.
- No duplicate Panda sends per GGA.
- `estPeriod` converges near expected IMU period.
- `dt` distribution shrinks as IMU output rate increases.
- `staleCount` remains flat during steady-state operation.
