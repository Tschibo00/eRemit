#include "DigiEnc.h"

bool DigiEnc::process() {
  bool retVal=false;
  bool _a=digitalRead(_pinA);
  bool _b=digitalRead(_pinB);
  // clockwise
  if (_lastA&&_lastB&&_a&&!_b)  _valQuad++;
  if (!_lastA&&_lastB&&_a&&_b)  _valQuad++;
  if (!_lastA&&!_lastB&&!_a&&_b)  _valQuad++;
  if (_lastA&&!_lastB&&!_a&&!_b)  _valQuad++;
  // counter-clockwise
  if (_lastA&&_lastB&&!_a&&_b)  _valQuad--;
  if (!_lastA&&_lastB&&!_a&&!_b)  _valQuad--;
  if (!_lastA&&!_lastB&&_a&&!_b)  _valQuad--;
  if (_lastA&&!_lastB&&_a&&_b)  _valQuad--;

  _lastA=_a;
  _lastB=_b;

  if ((_a&&_b)||((!_a)&&(!_b))) {
    int32_t _stepSize;
    if (_valQuad!=0)
      _deltaLastUpdate=millis()-_lastUpdate;
    if ((_deltaLastUpdate>=8)||(!_dynamic))
      _stepSize=1;
    else
      _stepSize=8-_deltaLastUpdate;
    if (_valQuad<0){
      val-=_stepSize;
      retVal=true;
    }
    if (_valQuad>0){
      val+=_stepSize;
      retVal=true;
    }
    if (val>_max){
      if (_wrapping)
        val=_min;
      else
        val=_max;
    }
    if (val<_min){
      if (_wrapping)    // wrapping needs to be adapted if step is larger than 1
        val=_max;
      else
        val=_min;
    }
    _valQuad=0;
    _lastUpdate=millis();
  }
  return retVal;
}
