/******************************************************************
File:             voicePlayback.ino
Description:      Audio source Playback
Note:             
******************************************************************/
#include <BMV31K304.h>

//BMV31K304 myBMV31K304(10,&SPI,9);   //Create an object BMduino UNO
BMV31K304 myBMV31K304(29,&SPI1,22);   //Create an object,BMduino UNO
//BMV31K304 myBMV31K304(4,&SPI2,9);  //Create an object,BMduino UNO

#define DEFAULT_VOLUME 6      //default volume
#define VOICE_TOTAL_NUMBER 10 //This example tests 10 voices

uint8_t voiceNum = 0;
void setup() {
  myBMV31K304.begin();//Initialize 
  
  myBMV31K304.setVolume(DEFAULT_VOLUME);//Initialize the default volume
  
  for(voiceNum = 0;voiceNum < VOICE_TOTAL_NUMBER;voiceNum++)//Play audio source
  {
    myBMV31K304.setLED(BMV31K304_LED_ON);//LED on
    myBMV31K304.playVoice(voiceNum); 
    while(myBMV31K304.isPlaying() == 1); 
    myBMV31K304.setLED(BMV31K304_LED_OFF);//LED off
    delay(100);
  }
}

void loop() {
}
