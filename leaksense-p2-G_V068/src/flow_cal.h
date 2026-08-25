/*
 * flow_cal.h  -  Flow calibration polynomial, in one testable place (V062).
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The Hz -> GPM calibration used to live as a static function inside
 * leaksense.cpp, which the host tests cannot link (leaksense.cpp pulls in
 * setup()/loop() and the whole Particle application). Appendix G asks for two
 * things that both need this maths reachable on its own:
 *
 *   - G.3.1 : report the calibration's usable frequency range at session start,
 *             computed from the live coefficients rather than a stale comment.
 *   - G.6.3 : a host regression test that pins the point where the polynomial
 *             collapses to 0, so a coefficient edit that silently moves the
 *             usable range is caught.
 *
 * So the polynomial is a plain inline here, freqToGpm() in leaksense.cpp just
 * calls it, and the host test includes this header directly. Nothing about the
 * numbers changes - this is a move, not a retune. The open questions about the
 * coefficients themselves (Appendix G.1.1 collapse, G.1.8 FLOW_C4) are
 * DELIBERATELY left untouched here; this file only makes them observable.
 */

#pragma once
#include "app_config.h"

// Hz -> GPM. Pure function of the sensor frequency and the user's calibration
// scale. Identical maths to the pre-V062 freqToGpm(); see Appendix G.1.1 for the
// known high-frequency collapse this intentionally preserves (a value >0 turning
// negative is clamped to 0, which is what makes ~109 Hz and above read as 0 GPM).
static inline float flowFreqToGpm(float freq, float scale) {
  if (freq <= 0.0f) return 0.0f;                          // No frequency -> no flow.
  float f  = freq / (1.0f + (FLOW_C5 * freq + FLOW_C6));  // High-frequency correction.
  float g0 = FLOW_C0 * f;                                 // Rough GPM.
  float g  = g0 - (FLOW_C1 * g0 * g0 + FLOW_C2 * g0 + FLOW_C3);  // Calibration polynomial.
  g *= scale;                                             // User calibration scale.
  return (g < 0.0f) ? 0.0f : g;                           // Never negative (Appendix G.1.1 collapse -> 0).
}

// Result of scanning the calibration for its usable range (Appendix G.3.1).
struct FlowValidRange {
  float validMaxHz;    // Highest frequency that still returns >0 GPM before the collapse.
  float peakGpm;       // Largest GPM the polynomial produces anywhere.
  float peakFreqHz;    // Frequency at which that peak occurs.
  bool  collapses;     // true if the polynomial falls back to 0 at some finite frequency.
};

// Scan the polynomial from just above 0 Hz upward and report where it peaks and
// where it collapses to 0. Scale-independent for the collapse point (scaling a
// positive value cannot change its sign), so 'scale' only affects peakGpm.
//
// 'maxScanHz' bounds the search; the default comfortably covers a 0xFFFF sample
// at the bench capture period (65535 / 5.29 s ~= 12388 Hz). 'stepHz' is the scan
// resolution (0.1 Hz is enough to place the collapse to within a tenth of a Hz).
static inline FlowValidRange flowComputeValidRange(float scale,
                                                   float maxScanHz = 13000.0f,
                                                   float stepHz    = 0.1f) {
  FlowValidRange r;
  r.validMaxHz = 0.0f;
  r.peakGpm    = 0.0f;
  r.peakFreqHz = 0.0f;
  r.collapses  = false;
  if (stepHz < 0.001f) stepHz = 0.1f;

  bool  everPositive = false;   // have we seen any flow at all yet
  float lastPositive = 0.0f;    // last frequency that still produced >0 GPM
  for (float freq = stepHz; freq <= maxScanHz; freq += stepHz) {
    float g = flowFreqToGpm(freq, scale);
    if (g > 0.0f) {
      everPositive = true;
      lastPositive = freq;
      if (g > r.peakGpm) { r.peakGpm = g; r.peakFreqHz = freq; }
    } else if (everPositive && !r.collapses) {
      // First zero AFTER the polynomial had been producing flow: the collapse.
      r.collapses  = true;
      r.validMaxHz = lastPositive;
    }
  }
  if (!r.collapses) r.validMaxHz = lastPositive;   // monotone in-range across the whole scan
  return r;
}
