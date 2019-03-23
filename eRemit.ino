#include "DigiEnc.h"
#include <avr/pgmspace.h>
#include <math.h>
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

CRGB leds[NUM_LEDS];
CRGB screen[NUM_LEDS];
DigiEnc *encTimer,*encMenu,*encAlarm,*encLight,*encColor;
uint8_t brightness;
uint8_t flashspeed;
volatile bool playing;
bool buttonPressLocked;
uint32_t secLeft;        // this is the timer's time
bool timerPaused;    // true=>pause, false=running
#define STATE_TIME  0
#define STATE_MENU  2
#define STATE_MENU_ALARM_SET  6
#define STATE_MENU_LIGHT_SET  7
#define STATE_MENU_COLOR_SET  8
uint8_t displayState;
uint8_t toneVal=0;
uint32_t toneDelay=0;
uint32_t beepDelay=3000;

void setup() {
  brightness=80;
  flashspeed=0;
  playing=false;
  buttonPressLocked=false;
  secLeft=0;
  timerPaused=false;
  displayState=STATE_TIME;

  cli();//disable interrupts

  //set timer1 interrupt at 8064hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1 = 0;//initialize counter value to 0
  OCR1A=30; // (16.000.000/64/8064)-1 = 31-1
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10);      // Set CS10 and CS11 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A);      // enable timer compare interrupt

  // set timer3 interrupt at 1hz => clock
  TCCR3A = 0;
  TCCR3B = 0;
  // Set compare match register to desired timer count - 25ms 40Hz (16000000/1024/40 = 390.63)
  OCR3A = 15624;
  // Turn on CTC mode WGM32 set CS10 and CS12 bits for Timer3 prescaler of 1024
  TCCR3B = bit (WGM32) | bit (CS32) | bit (CS30);
  // Enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A);
  
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
  FastLED.setBrightness(brightness);

  // setup data enc button  
  pinMode(2, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // setup the encoder in multiple instances
  encTimer=new DigiEnc(9,4,-1,599,false,true);
  encMenu=new DigiEnc(9,4,0,3,true,false);
  encAlarm=new DigiEnc(9,4,0,15,false,false);
  encLight=new DigiEnc(9,4,1,255,false,true);
  encColor=new DigiEnc(9,4,0,7,false,true);
  encLight->val=brightness;

  Serial.begin(115200);
}

ISR(TIMER1_COMPA_vect){//timer 1 interrupt playing samples (8064/sec)
/*  if (!playing){
    OCR4D=0;  // Set PWM value on D6
  }else{
    OCR4D=0;  // Set PWM value on D6
  }*/


  // beeper stuff
  playing=false;
  if (toneDelay&512){
    if ((toneDelay&4095)>1024){
      toneVal+=64;
      playing=true;
    }
  }
  if (toneDelay>0)
    toneDelay--;

  if (playing)
    OCR4D=toneVal;
  else
    OCR4D=0;
}

ISR(TIMER3_COMPA_vect){ // Timer3 interrupt running the time (1/sec)
  if (!timerPaused){
    if (secLeft==1){
      toneDelay=beepDelay;
    }
    if (secLeft>0)
      secLeft--;
  }
}

// only returns true once as long as the button is pressed
bool getButtonClick(){
  bool buttonPressed=!digitalRead(2);
  if (buttonPressLocked){
    if (!buttonPressed)
      buttonPressLocked=false;
    return false;
  }else{
    if (buttonPressed){
      buttonPressLocked=true;
      return true;
    } else
      return false;
  }
}

/* ****************** Display stuff ******************** */
void transformPicture(){
  for (uint8_t y=0;y<6;y+=2)
    for (uint8_t x=0;x<11;x++){
      leds[y*11+x]=screen[y*11+x];
      leds[y*11+11+x]=screen[y*11+21-x];
    }
  for (uint8_t x=0;x<11;x++)
    leds[66+x]=screen[66+x];
}

void setBrightness(){
  if ((flashspeed==0)||(displayState!=STATE_TIME)){
    FastLED.setBrightness(brightness);
  }else{
    FastLED.setBrightness(((uint8_t)((sin(0.001f*millis()*flashspeed)+1.0)/2.0*((float)brightness))));
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
      c=c<<1;
    }
  }
}

void drawDigit(const uint8_t *font, uint8_t pos, uint8_t width, uint8_t number, CRGB color){
  uint8_t c;
  for (uint8_t y=0;y<7;y++){
    c=font[7*number+y];
    for (uint8_t x=0;x<width;x++){
      if (c&128)
        screen[y*11+x+pos]=color;
      else
        screen[y*11+x+pos]=CRGB::Black;
      c=c<<1;
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
  d2=number%10;
  if (d0==0) digits=2;

  //clear screen
  for (int i=0;i<77;i++)
    screen[i]=CRGB::Black;

  if (digits==2){
    if (d1>0)
      drawDigit(font57, 0, 5, d1, color);
    drawDigit(font57, 6, 5, d2, color);
  }else{
    drawDigit(font37, 0, 3, d0, color);
    drawDigit(font37, 4, 3, d1, color);
    drawDigit(font37, 8, 3, d2, color);
    screen[25]=CRGB(color.g/4,color.g/4,color.g/4);
    screen[47]=CRGB(color.g/4,color.g/4,color.g/4);
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

void drawTime(){
  CRGB c;
  if (timerPaused){
    flashspeed=3;
    c=CRGB(255,255,255);
  }else{
    flashspeed=0;
    if (secLeft>315)
      c=CRGB(0,255,0);
    else {
      if (secLeft<60){
        flashspeed=10;
        c=CRGB(255,0,0);
      }else
        c=CRGB(315-secLeft,secLeft-60,0);
    }
  }
  if (secLeft<60){
    drawTime(secLeft,c);
  }else{
    drawTime(secLeft/60,c);
  }
}

// **************** MAIN CONTROL LOOP ****************
void loop() {
  switch(displayState){
    case STATE_MENU:
      encMenu->process();
      switch(encMenu->val){
        case 0:
          drawLogo(logos+7, CRGB::White);
          if (getButtonClick()) displayState=STATE_MENU_ALARM_SET;
          break;
        case 1:
          drawLogo(logos+14, CRGB::White);
          if (getButtonClick()) displayState=STATE_MENU_LIGHT_SET;
          break;
        case 2:
          drawLogo(logos, CRGB::White);
          if (getButtonClick()) displayState=STATE_MENU_COLOR_SET;
          break;
        case 3:
          // draw battery status
          drawLogo(logos+21, CRGB::White);
          uint16_t bat1,bat2;
          bat1=analogRead(A0);
          bat2=analogRead(A1);
          if (bat1<675) bat1=0; else {if (bat1>859) bat1=184; else bat1-=675;}
          if (bat2<675) bat2=0; else {if (bat2>859) bat2=184; else bat2-=675;}
          bat2=bat2*2-bat1;     // bat1 0..184 equals 3,3v..4,2v of battery 1, bat2 similar for battery 2
          if (bat2>184) bat2=184;
          uint16_t i;
          for (i=1;i<(bat1/21);i++) screen[11+i]=CRGB::Green;
          for (i=1;i<(bat2/21);i++) screen[55+i]=CRGB::Green;
          if (getButtonClick())
            displayState=STATE_TIME;
          break;
      }
      break;
      
    case STATE_MENU_ALARM_SET:
      if (encAlarm->process()){
        beepDelay=encAlarm->val*1000;
        toneDelay=beepDelay;          // play the beep
      }
      if (getButtonClick())
        displayState=STATE_TIME;
      break;
    case STATE_MENU_LIGHT_SET:
      encLight->process();
      brightness=encLight->val;
      drawLogo(logos+14, CRGB::White);
      if (getButtonClick())
        displayState=STATE_TIME;
      break;
    case STATE_MENU_COLOR_SET:
//      drawLogo(logos, CRGB::White);
      if (getButtonClick())
        displayState=STATE_TIME;
      break;
      
    default:
      encTimer->val=secLeft/60;
      if ((secLeft/30)==1) encTimer->val=0;
      if ((secLeft/30)==0) encTimer->val=-1;
      if (encTimer->process()){    // change the current set time when encoder is used
        secLeft=encTimer->val*60;
        if (encTimer->val==0) secLeft=30;
        if (encTimer->val==-1) secLeft=0;
      }
      if (getButtonClick()){       // pause/unpause with button
        if (secLeft==0)
          displayState=STATE_MENU;
        else
          timerPaused=!timerPaused;
      }
      drawTime();
  }

  transformPicture();
  setBrightness();
  if (!playing)       // only update display if no tone is playing to not break the interrupt
    FastLED.show();  
}
