#ifndef DIGIENC
#define DIGIENC

#include <Arduino.h>

class DigiEnc {
  private:
    uint8_t _pinA;
    uint8_t _pinB;
    int32_t _min;
    int32_t _max;
    bool _wrapping=false;
    bool _dynamic=true;

    bool _lastA=true;
    bool _lastB=true;
    int8_t _valQuad=0;
    unsigned long _lastUpdate=0;
    unsigned long _deltaLastUpdate=0;

  public:
    int32_t val=0;
  
    DigiEnc(uint8_t _pinA=26, uint8_t _pinB=27, int32_t _min=-128, int32_t _max=127, bool _wrapping=false, bool _dynamic=true){
      this->_pinA=_pinA;
      this->_pinB=_pinB;
      this->_min=_min;
      this->_max=_max;
      this->_wrapping=_wrapping;
      this->_dynamic=_dynamic;
      pinMode(_pinA, INPUT_PULLUP);
      pinMode(_pinB, INPUT_PULLUP);
    }

    void process();     // should be called at least 250 times/second to ensure a errorfree processing even for fast turning

};

#endif
