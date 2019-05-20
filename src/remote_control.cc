#include "remote_control.h"

#include "IRremote.h"

void RemoteControl::Initialize() {
	ir_recv_.enableIRIn();
}

void RemoteControl::ReadAndDispatch(ControlObserver* observer) {
  decode_results results;
  if (!ir_recv_.decode(&results))
    return;

  RemoteKey k = kKeyMax;

  switch (results.value) {
    case 0xFFA25D: k = kKeyChMinus; break;
    case 0xFF629D: k = kKeyCh; break;
    case 0xFFE21D: k = kKeyChPlus; break;
    case 0xFF22DD: k = kKeyPrev; break;
    case 0xFF02FD: k = kKeyNext; break;
    case 0xFFC23D: k = kKeyPause; break;
    case 0xFFE01F: k = kKeyMinus; break;
    case 0xFFA857: k = kKeyPlus; break;
    case 0xFF906F: k = kKeyEq; break;
    case 0xFF6897: k = kKey0; break;
    case 0xFF9867: k = kKey100Plus; break;
    case 0xFFB04F: k = kKey200Plus; break;
    case 0xFF30CF: k = kKey1; break;
    case 0xFF18E7: k = kKey2; break;
    case 0xFF7A85: k = kKey3; break;
    case 0xFF10EF: k = kKey4; break;
    case 0xFF38C7: k = kKey5; break;
    case 0xFF5AA5: k = kKey6; break;
    case 0xFF42BD: k = kKey7; break;
    case 0xFF4AB5: k = kKey8; break;
    case 0xFF52AD: k = kKey9; break;
  }

  if (k != kKeyMax)
    observer->OnRemoteKey(k);

  ir_recv_.resume();
}