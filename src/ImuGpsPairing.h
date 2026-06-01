#ifndef IMU_GPS_PAIRING_H
#define IMU_GPS_PAIRING_H

#include <Arduino.h>

class ImuGpsPairing
{
public:
  struct Stats
  {
    int32_t minMs = 0;
    int32_t maxMs = 0;
    int64_t sumMs = 0;
    uint32_t count = 0;
    uint32_t staleCount = 0;
    bool initialized = false;
  };

  struct Decision
  {
    bool shouldSend = false;
    bool validForStats = false;
    int32_t deltaMs = 0;
  };

  ImuGpsPairing(uint32_t initialPeriodMs, uint16_t timeoutMs, bool filterStaleOnTimeout);

  void noteImuSample(uint32_t nowMs);
  void onGga(uint32_t nowMs);
  bool tryImmediatePair(uint32_t nowMs, int32_t &deltaMs);
  Decision evaluatePending(uint32_t nowMs);
  void recordDelta(int32_t deltaMs);

  uint32_t estimatedPeriod() const;
  const Stats &stats() const;

private:
  uint32_t m_lastSampleMs = 0;
  uint32_t m_estimatedPeriodMs = 0;
  uint32_t m_sampleCounter = 0;
  uint32_t m_counterAtGga = 0;
  uint32_t m_ggaArrivalMs = 0;
  bool m_pendingBuild = false;

  uint16_t m_timeoutMs = 0;
  bool m_filterStaleOnTimeout = false;

  Stats m_stats;
};

#endif
