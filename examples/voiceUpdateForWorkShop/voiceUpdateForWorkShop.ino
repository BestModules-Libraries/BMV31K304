/******************************************************************
File:             voiceUpdateForWorkShop.ino
Description:      Audio source update
Note:             
******************************************************************/
#include <BMV31K304.h>

//BMV31K304 myBMV31K304(10,&SPI,9);   //Create an object BMduino UNO
BMV31K304 myBMV31K304(29,&SPI1,22);   //Create an object,BMduino UNO
//BMV31K304 myBMV31K304(4,&SPI2,9);  //Create an object,BMduino UNO

#define DEFAULT_VOLUME 6  //default volume

void setup() {
  myBMV31K304.begin();//Initialize  
  myBMV31K304.initAudioUpdate();//Initialize online update of audio source
}

void loop() {
  if(myBMV31K304.isUpdateBegin() == BMV31K304_UPDATE_BEGIN)//detect update signal
  {
    myBMV31K304.executeUpdate(1);//Execute audio source updates
  }
}
