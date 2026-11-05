// SkiUpdate.h
// Firmware update received from the radio controller (TetraskiController,
// radio-controller/src/SkiUpdateService.ino is the other end).
#ifndef SKI_UPDATE_H
#define SKI_UPDATE_H

#include <Arduino.h>

#define SKI_CMD_ENTER_UPDATE  0x01
#define SKI_CMD_EXIT_UPDATE   0x05
#define SKI_ACK               0x06

// Puts this firmware's MAJOR.MINOR.PATCH in the image as text. The radio
// refuses to relay an image that does not contain its own version string.
// Use once, in main.cpp: SKI_UPDATE_VERSION_TAG(VERSION_MAJOR, VERSION_MINOR, VERSION_REV);
#define SKI_UPDATE_STR2(x) #x
#define SKI_UPDATE_STR(x) SKI_UPDATE_STR2(x)
#define SKI_UPDATE_VERSION_TAG(major, minor, rev) \
  extern const char skiVersionTag[]; \
  const char skiVersionTag[] = SKI_UPDATE_STR(major) "." SKI_UPDATE_STR(minor) "." SKI_UPDATE_STR(rev)

// Call after the radio sent SKI_CMD_ENTER_UPDATE. Blocks while in update
// mode, so nothing else (actuators included) runs until it returns. Returns
// on EXIT or after 10 s without bytes from the radio. A COMMIT that passes
// every check rewrites program flash and reboots, and does not return.
void skiUpdateRun(HardwareSerial& link);

#endif
