#include "config.h"


void LEDsetup() {
  pinMode(PIN_PWM_RGB_B,OUTPUT);
  pinMode(PIN_PWM_RGB_R,OUTPUT);
  pinMode(PIN_PWM_RGB_G,OUTPUT);

  for(int i=0;i<5;++i) {
    LEDsetColor(255,255,255);
    delay(50);
    LEDsetColor(0,0,0);
    delay(50);
  }
}


void LEDtest() {
  for(int i=0;i<256;++i) {
    wheel(i);
    delay(5);
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
void wheel(byte WheelPos) {
  if(WheelPos < 85) {
    LEDsetColor(255 - WheelPos * 3,0,WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    LEDsetColor(0,WheelPos * 3,255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    LEDsetColor(WheelPos * 3,255 - WheelPos * 3,0);
  }
}


void LEDsetColor(int r,int g,int b) {
  analogWrite(PIN_PWM_RGB_R,r);
  analogWrite(PIN_PWM_RGB_G,g);
  analogWrite(PIN_PWM_RGB_B,b);
}