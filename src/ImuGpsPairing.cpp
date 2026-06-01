#include "ImuGpsPairing.h"

ImuGpsPairing::ImuGpsPairing(uint32_t initialPeriodMs, uint16_t timeoutMs, bool filterStaleOnTimeout)
    : m_estimatedPeriodMs(initialPeriodMs),
      m_timeoutMs(timeoutMs),
      m_filterStaleOnTimeout(filterStaleOnTimeout)
{
}

void ImuGpsPairing::noteImuSample(uint32_t nowMs)
{
  if (m_lastSampleMs != 0)
  {
    uint32_t dt = nowMs - m_lastSampleMs;
    // Smooth period estimate to reduce jitter impact.
    m_estimatedPeriodMs = ((m_estimatedPeriodMs * 3) + dt) / 4;
  }

  m_lastSampleMs = nowMs;
  m_sampleCounter++;
}

void ImuGpsPairing::onGga(uint32_t nowMs)
{
  m_ggaArrivalMs = nowMs;
  m_counterAtGga = m_sampleCounter;
  m_pendingBuild = true;
}

bool ImuGpsPairing::tryImmediatePair(uint32_t nowMs, int32_t &deltaMs)
{
  if (m_lastSampleMs == 0)
    return false;

  uint32_t imuAgeMs = nowMs - m_lastSampleMs;
  uint32_t halfPeriod = m_estimatedPeriodMs / 2;

  if (imuAgeMs > halfPeriod)
    return false;

  deltaMs = (int32_t)m_lastSampleMs - (int32_t)nowMs;
  m_pendingBuild = false;
  return true;
}

ImuGpsPairing::Decision ImuGpsPairing::evaluatePending(uint32_t nowMs)
{
  Decision decision;

  if (!m_pendingBuild)
    return decision;

  bool gotNewSampleAfterGga = (m_sampleCounter != m_counterAtGga);
  bool timedOutWaiting = ((nowMs - m_ggaArrivalMs) >= m_timeoutMs);

  if (!(gotNewSampleAfterGga || timedOutWaiting))
    return decision;

  decision.shouldSend = true;

  bool hasValidSample = (m_lastSampleMs != 0);
  uint32_t sampleAgeMs = hasValidSample ? (nowMs - m_lastSampleMs) : 0;

  bool canUseForStats = hasValidSample &&
                        (gotNewSampleAfterGga ||
                         !m_filterStaleOnTimeout ||
                         (sampleAgeMs <= (m_estimatedPeriodMs + m_timeoutMs)));

  if (canUseForStats)
  {
    decision.validForStats = true;
    decision.deltaMs = (int32_t)m_lastSampleMs - (int32_t)m_ggaArrivalMs;
  }
  else
  {
    m_stats.staleCount++;
  }

  m_pendingBuild = false;
  return decision;
}

void ImuGpsPairing::recordDelta(int32_t deltaMs)
{
  if (!m_stats.initialized)
  {
    m_stats.minMs = deltaMs;
    m_stats.maxMs = deltaMs;
    m_stats.sumMs = deltaMs;
    m_stats.count = 1;
    m_stats.initialized = true;
    return;
  }

  if (deltaMs < m_stats.minMs)
    m_stats.minMs = deltaMs;
  if (deltaMs > m_stats.maxMs)
    m_stats.maxMs = deltaMs;

  m_stats.sumMs += deltaMs;
  m_stats.count++;
}

uint32_t ImuGpsPairing::estimatedPeriod() const
{
  return m_estimatedPeriodMs;
}

const ImuGpsPairing::Stats &ImuGpsPairing::stats() const
{
  return m_stats;
}
