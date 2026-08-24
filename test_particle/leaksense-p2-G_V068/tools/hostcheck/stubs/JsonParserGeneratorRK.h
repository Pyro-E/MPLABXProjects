/*
 * JsonParserGeneratorRK.h  -  HOST-ONLY STUB for `g++ -fsyntax-only`.
 *
 * Declares just the JsonWriter surface leaksense.cpp uses. The real vendored
 * library in lib/JsonParserGeneratorRK/ is what the Particle build compiles.
 */

#pragma once

#include "Particle.h"

class JsonWriter {
public:
  void insertKeyValue(const char *, const char *) {}
  void insertKeyValue(const char *, int) {}
  void insertKeyValue(const char *, float) {}
  void insertKeyValue(const char *, double) {}
  void insertKeyValue(const char *, bool) {}
  void insertKeyArray(const char *) {}
  void insertArrayValue(float) {}
  void insertArrayValue(double) {}
  void insertArrayValue(int) {}
  void finishObjectOrArray() {}
  const char *getBuffer() const { return ""; }
  // V068: the size/precision surface the contract publish path relies on.
  // setFloatPlaces() is what stops "%f" from printing six decimals per bucket,
  // and isTruncated() is what makes an oversized payload say so.
  void   setFloatPlaces(int) {}
  bool   isTruncated() const { return false; }
  size_t getOffset() const { return 0; }
  size_t getBufferLen() const { return 0; }
};

template <size_t N>
class JsonWriterStatic : public JsonWriter {};

class JsonWriterAutoObject {
public:
  explicit JsonWriterAutoObject(JsonWriter *) {}
};
