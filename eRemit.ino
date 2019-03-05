#include "DigiEnc.h"
#include <avr/pgmspace.h>
#include "font.h"

#define FASTLED_ALLOW_INTERRUPTS 1
#include <FastLED.h>
//FASTLED_USING_NAMESPACE
#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#if defined(__AVR_ATmega328P__)
// Timer2 is the same on the mega328 and mega168
#define __AVR_ATmega168__
#endif

#define DATA_PIN    5
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    7*11
#define BRIGHTNESS          255

CRGB leds[NUM_LEDS];
CRGB screen[NUM_LEDS];
DigiEnc *enc;
volatile bool playing=false;

void setup() {
  cli();//disable interrupts
  //set timer1 interrupt at 8064hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1 = 0;//initialize counter value to 0
  OCR1A=30; // (16.000.000/64/8064)-1 = 31-1
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);      // Set CS10 and CS11 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt
  sei();//enable interrupts
  
  // setup timer4 fast PWM
  TCCR4A=0;
  TCCR4B=1;   // use 375khz pwm
  TCCR4C=0;
  TCCR4D=0;
  PLLFRQ=(PLLFRQ&0xCF)|0x10; // use 96mhz pll
  OCR4C=255;
  DDRD|=1<<7;    // Set Output Mode D7
  TCCR4C|=0x09; // Activate channel

  // setup display
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(UncorrectedColor); //TypicalLEDStrip
  FastLED.setBrightness(BRIGHTNESS);

  // setup data enc button  
  pinMode(2, INPUT_PULLUP);

  // setup the encoder
  enc=new DigiEnc(9,4,5,255,false,true);
}

ISR(TIMER1_COMPA_vect){//timer 1 interrupt
  if (!playing){
    OCR4D=0;  // Set PWM value on D6
  }else{
    OCR4D=0;  // Set PWM value on D6
  }
}

void transformPicture(){
  for (uint8_t y=0;y<7;y+=2)
    for (uint8_t x=0;x<11;x++){
      leds[y*11+x]=screen[y*11+x];
      leds[y*11+11+x]=screen[y*11+21-x];
    }
}

void drawLogo(const uint16_t *logo, CRGB color){
  uint16_t c;
  for (uint8_t y=0;y<7;y++){
    c=logo[y];
    for (uint8_t x=0;x<11;x++){
      if (c&0x8000)
        screen[y*11+x]=color;
      else
        screen[y*11+x]=CRGB::Black;
    }
  }
}

void drawDigit(const uint8_t *font, uint8_t pos, uint8_t width, uint8_t number, CRGB color){
  uint8_t c;
  for (uint8_t y=0;y<7;y++){
    c=font[7*number];
    for (uint8_t x=0;x<width;x++){
      if (c&128)
        screen[y*11+x+pos]=color;
      else
        screen[y*11+x+pos]=CRGB::Black;
    }
  }
}

void drawNumber(int number, CRGB color){
  uint8_t digits;
  uint8_t d0, d1, d2;

  if (number<0) number=0;
  if (number>999) number=999;
  digits=3;
  d0=number/100;
  d1=(number%100)/10;
  d2=number%100;
  if (d0==0) digits=2;

  //clear screen
  for (int i=0;i<77;i++)
    screen[i]=CRGB::Black;

  if (digits==2){
    drawDigit(font57, 0, 5, d1, color);
    drawDigit(font57, 6, 5, d2, color);
  }else{
    drawDigit(font37, 0, 3, d0, color);
    drawDigit(font37, 4, 3, d1, color);
    drawDigit(font37, 8, 3, d2, color);
  }
}

void drawTime(int minutes, CRGB color){
  int d0, d1;

  if (minutes<0) minutes=0;
  if (minutes>599) minutes=599;
  d0=minutes/60;
  d1=minutes%60;
  drawNumber(d0*100+d1, color);
}

void loop() {
  enc->process();

//  drawRandom();
//  drawSingle();
  int m=millis()%512;
  m=enc->val;
  switch(m){
    case 0: drawTime((millis()/1000)%600, CRGB::White); break;
    case 1: drawLogo(logos, CRGB::Green); break;
    case 2: drawLogo(logos+7, CRGB::Green); break;
    case 3: drawLogo(logos+14, CRGB::Green); break;
    default:  drawNumber(m, CRGB::Magenta);
  }
  transformPicture();
  FastLED.show();  
}
