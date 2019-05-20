#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include "IRremote.h"

enum RemoteKey {
  kKey0,
  kKey1,
  kKey2,
  kKey3,
  kKey4,
  kKey5,
  kKey6,
  kKey7,
  kKey8,
  kKey9,

  kKeyChMinus,
  kKeyCh,
  kKeyChPlus,
  kKeyPrev,
  kKeyNext,
  kKeyPause,
  kKeyMinus,
  kKeyPlus,
  kKeyEq,
  kKey100Plus,
  kKey200Plus,

  kKeyMax
};

class ControlObserver {
 public:
  virtual void OnRemoteKey(RemoteKey key) = 0;
  ~ControlObserver() {}
};

class RemoteControl {
 public:
  RemoteControl(int pin) : ir_recv_(pin) {}
  void Initialize();
  void ReadAndDispatch(ControlObserver* observer);

 private:
  IRrecv ir_recv_;
};

#endif  // _REMOTE_CONTROL_H