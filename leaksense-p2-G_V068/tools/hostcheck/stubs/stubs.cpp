/* Definitions for the extern symbols declared in the Particle stub header.
 * Only needed when a host test is LINKED (check.sh only parses). */
#include "Particle.h"
Stream Serial, Serial1, Serial2, Serial3;
Logger Log;
TimeClass Time;
bool     TimeClass::s_valid = false;   // unsynced by default (Appendix H.3.3 host default)
uint32_t TimeClass::s_now   = 0u;
ParticleClass Particle;
NetworkClass WiFi, Cellular;
SystemClass System;
EEPROMClass EEPROM;
