/*********************************************************************************************
File:             BMV31K304.cpp
Author:           BEST MODULES CORP.
Description:      single wire communicates with BMV31K304 and controls audio playback
History：    V1.0.1   -- 2024-07-19
**********************************************************************************************/
#include "BMV31K304.h"

#define PAUSE_PLAY    	0XF1	//Pause playing the current voice and sentence command
#define CONTINUE_PLAY   0XF2	//Continue playing the paused voice and sentence command
#define LOOP_PLAY    	0XF4	//Loop playback for the current voice and sentence command
#define STOP_PLAY     	0XF8	//Stop playing the current voice and sentence command

#define SPI_FLASH_PAGESIZE 256

#define CE         0x60  // Chip Erase instruction 
#define PP         0x02  // Page Program instruction 
#define READ       0x03  // Read from Memory instruction  
#define WREN       0x06  // Write enable instruction 
#define RDSR       0x05  // Read Status Register instruction 
#define	SFDP	     0x5a	 // Read SFDP.

#define WIP_FLAG   0x01  // Write In Progress (WIP) flag 
#define WEL_FLAG   0x02 // Write Enable Latch

#define DUMMY_BYTE 0xff

/*CRC8：x8+x5+x4+1，MSB*/
static const uint8_t crc_table[] =
{
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d,
    0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8,
    0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb,
    0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13,
    0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95,
    0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6,
    0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17,
    0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2,
    0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a,
    0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef,
    0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac
};

/************************************************************************* 
Description:  Constructor
parameter:    cs1_ledPin:Chip selection pin/LED control pin, default to 29
              *spiClass:SPI communication interface, default to SPI1
              powerPin:Power pin, default to 22 pins        
Return:         
Others:         
*************************************************************************/
BMV31K304::BMV31K304(uint8_t cs1_ledPin,SPIClass *spiClass,uint8_t powerPin)
{
	_flashAddr = 0;

  _sel = cs1_ledPin; 
  _power = powerPin;
  if(spiClass != NULL)
  {
    if(spiClass == &SPI)
    {    
      _spi = &SPI;   
      _icpck = 13;
      _icpda = 11;
      _data = 12;
    }
    if(spiClass == &SPI1)
    {   
      _spi = &SPI1;
      _icpck = 27;
      _icpda = 28;
      _data = 26;
    }
    if(spiClass == &SPI2)
    {
      _spi = &SPI2;
      _icpck = 5;
      _icpda = 6;
      _data = 7;
    }
  }
}

/************************************************************************* 
Description:Initialize 
parameter:  void             
Return:     void       
Others:         
*************************************************************************/
void BMV31K304::begin(void)
{
  pinMode(_power, OUTPUT);
  digitalWrite(_power, HIGH);  
  pinMode(_icpda, OUTPUT);//DATA
  digitalWrite(_icpda, HIGH);
  pinMode(_sel, OUTPUT);//DATA
  digitalWrite(_sel, HIGH);
  pinMode(_data, OUTPUT);//DATA
  digitalWrite(_data, HIGH);
  pinMode(_icpck, INPUT);
     
  delay(1000);//There's a delay here to get the BMV31K302SPI ready
}

/************************************************************************* 
Description:Set the volume
parameter:  volume：0~11(0:minimum volume（mute）;11:maximum volume)   
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::setVolume(uint8_t volume)
{
	writeCmd(0xe1 + volume);
}
/************************************************************************* 
Description:Play voice.
parameter:  num：The number of the voice being played
            loop：default 0.(1：Loops the current voice，0：Play it only once)         
Return:     void         
Others:         
*************************************************************************/
void BMV31K304::playVoice(uint8_t num, uint8_t loop)
{
  if(num < 128)
  {
    writeCmd(0xfa, num);
  }
  else
  {
    writeCmd(0xfb, num % 128);
  }
	
	if(loop)
	{
		writeCmd(0xf4);
	}
}

/************************************************************************* 
Description:  Play sentence.
parameter:    num：Number of the sentence being played
              loop：default 0.(1：Loops the current sentence，0：Play it only once)                  
Return:       void       
Others:         
*************************************************************************/
void BMV31K304::playSentence(uint8_t num, uint8_t loop)
{
	writeCmd(num);
	if(loop)
	{
		writeCmd(0xf4);
	}
}

/************************************************************************* 
Description:Stop playing the current voice and sentence.
parameter:  void                  
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::playStop(void)
{
	writeCmd(STOP_PLAY);
}

/************************************************************************* 
Description:Pause playing the current voice and sentence.
parameter:  void             
Return:     void       
Others:         
*************************************************************************/
void BMV31K304::playPause(void)
{
	writeCmd(PAUSE_PLAY);
}

/************************************************************************* 
Description:Continue playing the paused voice and sentence.
parameter:  void               
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::playContinue(void)
{
	writeCmd(CONTINUE_PLAY);
}

/************************************************************************* 
Description:Loop playback the current voice/sentence
parameter:  void               
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::playRepeat(void)
{
  writeCmd(LOOP_PLAY);
}

/************************************************************************* 
Description:Get the play status
parameter:  void                 
Return:     false:Not in the play
            true:In the play
Others:         
*************************************************************************/
bool BMV31K304::isPlaying(void)
{
	if(0 == digitalRead(_icpck))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/************************************************************************* 
Description:Set the onboard LED on or off
parameter:  status: 0:LED off; 1:LED on         
Return:     void      
Others:         
*************************************************************************/
void BMV31K304::setLED(uint8_t status)
{
	digitalWrite(_sel, !status);
}

/************************************************************************* 
Description:Update your audio source with Ardunio
parameter:  baudrate：Updated baud rate       
Return:     void        
Others:         
*************************************************************************/
void BMV31K304::initAudioUpdate(unsigned long baudrate)
{
  pinMode(_data, OUTPUT);
  digitalWrite(_data, HIGH);
	SerialUSB.begin(baudrate);
}

/************************************************************************* 
Description:Get the update sound source signal
parameter:  void      
Return:     true：execute update; false：not execute update
Others:         
*************************************************************************/
bool BMV31K304::isUpdateBegin(void)
{
  if (SerialUSB.available())
  {
    return true;
  }
  else
  {
    return false;
  }        
}

/************************************************************************* 
Description:Update the audio source
parameter:  mode:0:BMduino Voice Widget  1:Holtek Voice MCU Workshop     
Return:     true:Update completed; false:Update failed 
Others:         
*************************************************************************/
bool BMV31K304::executeUpdate(uint8_t mode)
{
  if(mode == 0)
  {
    return executeUpdateWidget();
  }
  else if(mode == 1)
  {
    return executeUpdateWorkshop();
  }
  return false;
}

/************************************************************************* 
Description:Update the audio source for BMduino voice Widget
parameter:  void      
Return:     true:Update completed; false:Update failed 
Others:         
*************************************************************************/
bool BMV31K304::executeUpdateWidget(void)
{
    static int8_t dataLength = 0;
    uint32_t delayCount = 0;
    _EraseCnt=0;
    while(1)
    {
        if(SerialUSB.available())
        {
            delayCount = 0;
            SerialUSB.readBytes(rxBuffer, 3);   
            if ((0xAA == rxBuffer[0]) && (0x23 == rxBuffer[1]))
            {
                dataLength = rxBuffer[2];
                SerialUSB.readBytes(rxBuffer + 3, dataLength + 2);                 
                if (rxBuffer[dataLength + 3] == checkCRC8(rxBuffer + 2, dataLength + 1))
                {
                    if (6 == dataLength)
                    {
                        dataLength =0;
                         delayCount =0;
                        if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'S') && (rxBuffer[7] == 'P') && (rxBuffer[8] == 'I'))
                        {
                            if (false == switchSPIMode())
                            {
                                
                                //SerialUSB.write(0xe3);
                                deviceIDBuf[0]=0xe3;
                               // SerialUSB.write(deviceIDBuf, 4);                                
                                SerialUSB.write(deviceIDBuf, 1); 
                                digitalWrite(_power, LOW);
                                delay(500);
                                digitalWrite(_power, HIGH);    

                                _flashAddr = 0;
                                pinMode(_data, OUTPUT);
                                digitalWrite(_data, HIGH);
                               // pinMode(STATUS_PIN, INPUT);
                                pinMode(_icpda, OUTPUT);
                                digitalWrite(_icpda, HIGH);
                                pinMode(_icpck, INPUT);
                            }
                            else
                            {
                                //SerialUSB.write(0x3e);//ACK
                                deviceIDBuf[0]=0x3e;
                                SerialUSB.write(deviceIDBuf, 1);   
                               // SerialUSB.write(deviceIDBuf, 6);                                
                            }
                            
                        }
                        else if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'O') && (rxBuffer[7] == 'R') && (rxBuffer[8] == 'D'))
                        {
                            SerialUSB.write(0x3e);//ACK

//                          reset();
//                          pinMode(_power, OUTPUT);
//                          digitalWrite(_power, LOW);	
//
//  	                      pinMode(_data, OUTPUT);//_data
//	                 	      digitalWrite(_data, HIGH);
// 	                        pinMode(_icpck, INPUT);
//  	                      pinMode(STATUS_PIN, INPUT);
//                          delay(10);
//                          return 1;


//                          digitalWrite(_power, LOW); 
//                          delay(500);
//                          digitalWrite(_power, HIGH);             
                            _flashAddr = 0;
                            _spi->end();
                            pinMode(_power, OUTPUT);
                            pinMode(_data, OUTPUT);
                            pinMode(_icpda, OUTPUT);
                            pinMode(_icpck, OUTPUT);
                            pinMode(_sel, OUTPUT);
                            digitalWrite(_power, LOW);  
                            digitalWrite(_data, LOW);
                            digitalWrite(_icpda, LOW);
                            digitalWrite(_icpck, LOW);
                            digitalWrite(_sel, LOW);
                            delay(500);
                            pinMode(_power, OUTPUT);
                            digitalWrite(_power, HIGH);  
                            pinMode(_data, OUTPUT);
                            digitalWrite(_data, HIGH);
                            //pinMode(STATUS_PIN, INPUT);
                            pinMode(_icpda, OUTPUT);
                            digitalWrite(_icpda, HIGH);
                            pinMode(_icpck, INPUT);
                            delay(10);
                            return true;
                        }
                    }
                    else if (5 == dataLength)
                    {
                        dataLength =0;
                         delayCount =0;
                        if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'C') && (rxBuffer[7] == 'E'))
                        {
                          _EraseCnt++;
                          if(_EraseCnt<2)
                          {
                            SPIFlashChipErase();
                           // SerialUSB.write(0x3e);//ACK
                          }
                          else {_EraseCnt=0;}

                            delayCount = 0;
                            SerialUSB.write(0x3e);//ACK
                        }
					    else 
                        if ((rxBuffer[3] == 'R') && (rxBuffer[4] == 'e') && (rxBuffer[5] == 's')
                        && (rxBuffer[6] == 'e') && (rxBuffer[7] == 't'))
                        {
							          SerialUSB.write(0x3e);//ACK


                     reset();
                     pinMode(_power, OUTPUT);
                     digitalWrite(_power, LOW);	
	                  //pinMode(LED_PIN, OUTPUT);
		                //digitalWrite(LED_PIN, HIGH);
  	                pinMode(_data, OUTPUT);//_data
	                 	digitalWrite(_data, HIGH);
 	                  pinMode(_icpck, INPUT);
  	               // pinMode(STATUS_PIN, INPUT);

                        }
                    }
                    else if (4 == dataLength)
                    {
                        dataLength =0;
                        delayCount =0;
                        if ((rxBuffer[3] == 'A') && (rxBuffer[4] == 'C') && (rxBuffer[5] == 'O')
                        && (rxBuffer[6] == 'M'))
                        {
                            SerialUSB.write(0x3e);//ACK
                        }
                        else
                            SerialUSB.write(0xe3);//NACK

                    }                               
                }
                else
                {
                    dataLength =0;
                    SerialUSB.write(0xe3);//NACK
                }

            }
            else
            {
                recAudioData();
            }
        }                 
        delayCount++;
        delayMicroseconds(50);//waiting for receive data 
        if(delayCount>=2000)
        {
            delayCount=0;
            return false;//timeout is 50us*2000=100ms,nothing for receive
        }
    }
}

/************************************************************************* 
Description:Update the audio source for Holtek Voice MCU Workshop
parameter:  void   
Return:     true:Update completed; false:Update failed 
Others:         
*************************************************************************/
bool BMV31K304::executeUpdateWorkshop(void)
{
    static int8_t dataLength = 0;
    uint32_t delayCount = 0;
    _EraseCnt=0;
    while(1)
    {
        if(SerialUSB.available())
        {
            delayCount = 0;
            SerialUSB.readBytes(rxBuffer, 3);   
            if ((0xAA == rxBuffer[0]) && (0x23 == rxBuffer[1]))
            {
                dataLength = rxBuffer[2];
                SerialUSB.readBytes(rxBuffer + 3, dataLength + 2);                 
                if (rxBuffer[dataLength + 3] == checkCRC8(rxBuffer + 2, dataLength + 1))
                {
                    if (6 == dataLength)
                    {
                        dataLength =0;
                         delayCount =0;
                        if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'S') && (rxBuffer[7] == 'P') && (rxBuffer[8] == 'I'))
                        {
                            if (false == switchSPIMode())
                            {
                                
                                //SerialUSB.write(0xe3);
                                deviceIDBuf[0]=0xe3;
                                SerialUSB.write(deviceIDBuf, 4);                                
                                digitalWrite(_power, LOW);
                                delay(500);
                                digitalWrite(_power, HIGH);    

                                _flashAddr = 0;
                                pinMode(_data, OUTPUT);
                                digitalWrite(_data, HIGH);
                               // pinMode(STATUS_PIN, INPUT);
                                pinMode(_icpda, OUTPUT);
                                digitalWrite(_icpda, HIGH);
                                pinMode(_icpck, INPUT);
                            }
                            else
                            {
                                //SerialUSB.write(0x3e);//ACK
                                deviceIDBuf[0]=0x3e;
                                SerialUSB.write(deviceIDBuf, 4);                                
                            }
                            
                        }
                        else if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'O') && (rxBuffer[7] == 'R') && (rxBuffer[8] == 'D'))
                        {
                            SerialUSB.write(0x3e);//ACK

                            digitalWrite(_power, LOW);
                            delay(500);
                            digitalWrite(_power, HIGH);                
                            _flashAddr = 0;
                            _spi->end();
                            pinMode(_data, OUTPUT);
                            digitalWrite(_data, HIGH);
                           // pinMode(STATUS_PIN, INPUT);
                            pinMode(_icpda, OUTPUT);
                            digitalWrite(_icpda, HIGH);
                            pinMode(_icpck, INPUT);
                            delay(10);
                            return true;
                        }
                    }
                    else if (5 == dataLength)
                    {
                        dataLength =0;
                         delayCount =0;
                        if ((rxBuffer[3] == 'C') && (rxBuffer[4] == 'O') && (rxBuffer[5] == 'M')
                        && (rxBuffer[6] == 'C') && (rxBuffer[7] == 'E'))
                        {
                          _EraseCnt++;
                          if(_EraseCnt<2)
                          {
                            SPIFlashChipErase();
                          }
                          else {_EraseCnt=0;}

                            delayCount = 0;
                            SerialUSB.write(0x3e);//ACK
                        }
              else 
                        if ((rxBuffer[3] == 'R') && (rxBuffer[4] == 'e') && (rxBuffer[5] == 's')
                        && (rxBuffer[6] == 'e') && (rxBuffer[7] == 't'))
                        {
                        SerialUSB.write(0x3e);//ACK


                     reset();
                     pinMode(_power, OUTPUT);
                     digitalWrite(_power, LOW);  
                    //pinMode(LED_PIN, OUTPUT);
                    //digitalWrite(LED_PIN, HIGH);
                    pinMode(_data, OUTPUT);//DATA
                    digitalWrite(_data, HIGH);
                    pinMode(_icpck, INPUT);
                  //  pinMode(STATUS_PIN, INPUT);
                        }
                    }
                    else if (4 == dataLength)
                    {
                        dataLength =0;
                        delayCount =0;
                        if ((rxBuffer[3] == 'A') && (rxBuffer[4] == 'C') && (rxBuffer[5] == 'O')
                        && (rxBuffer[6] == 'M'))
                        {
                            SerialUSB.write(0x3e);//ACK
                        }
                        else
                            SerialUSB.write(0xe3);//NACK

                    }                               
                }
                else
                {
                    dataLength =0;
                    SerialUSB.write(0xe3);//NACK
                }

            }
            else
            {
                recAudioData();
            }
        }
                          
        delayCount++;
        delayMicroseconds(50);//waiting for receive data 
        if(delayCount>=2000)
        {
            delayCount=0;
            return false;//timeout is 50us*2000=100ms,nothing for receive
        }
    }
}

/************************************************************************* 
Description:Reset 
parameter:  void                
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::reset(void)
{
  digitalWrite(_power, LOW);
  delay(500);
  digitalWrite(_power, HIGH);
}

/************************************************************************* 
Description:Set the power up or down of the BMV31K304
parameter:  tatus: 0:power down; 1:power up
Return:     void        
Others:         
*************************************************************************/
void BMV31K304::setPower(uint8_t status)
{
  digitalWrite(_power, status);
}

/************************************************************************* 
Description:check IC
parameter:  void             
Return:     0:fail 1:succes       
Others:         
*************************************************************************/
uint8_t  BMV31K304::CheckIC(void)
{
  uint16_t i;
  pinMode(_power, OUTPUT);
  pinMode(_sel, INPUT_PULLDOWN);
  pinMode(_data, INPUT_PULLDOWN);
  pinMode(_icpck, INPUT_PULLDOWN);
  pinMode(_icpda, INPUT_PULLDOWN);   


  //digitalWrite(_power, LOW);  
  //reset();
  digitalWrite(_power, HIGH);
  delay(500);
  digitalWrite(_power, LOW);
  delay(50);
  //pinMode(_icpda, INPUT);
  //digitalWrite(_icpda, HIGH);
  //for(i=0;i<254;i++);
  //delay(10);
  if(( digitalRead(_sel)==0)&&( digitalRead(_data)==0) )
  {
    i=1;
    reset();
    pinMode(_sel, OUTPUT);
    pinMode(_data, OUTPUT);
    pinMode(_icpck, OUTPUT);
    pinMode(_icpda, INPUT);         
    return i;
  }
//    if( digitalRead(_icpck)==0) i= 0;
//    else{
//      i= 1;
//      reset();
//      pinMode(_sel, OUTPUT);
//      pinMode(_data, OUTPUT);
//      pinMode(_icpck, OUTPUT);
//      pinMode(_icpda, INPUT);   
//      return i;
//    } 
  i= 1; 
      
  if(( digitalRead(_sel)==1)&&( digitalRead(_data)==1)&&( digitalRead(_icpck)==1) ) i= 0;
  reset();
  pinMode(_sel, OUTPUT);
  pinMode(_data, OUTPUT);
  pinMode(_icpck, OUTPUT);
  pinMode(_icpda, INPUT);         
  return i;
}

/************************************************************************* 
Description:Switch SPI Mode
parameter:  void      
Return:     true:Switch successfully
            false:Fail to switch
Others:         
*************************************************************************/
bool BMV31K304::switchSPIMode(void)
{
  static uint8_t correctFlag = 0;
  static uint8_t retransmissionTimes = 0;
  for(correctFlag=0;correctFlag<8;correctFlag++) rxBuffer[correctFlag]=0;
  if (false == programEntry(0x02))
  {
    return false;
  }
  correctFlag=0;
  sendAddr(0x0020);
  sendData(0x0000);
  sendData(0x0000);
  sendData(0x0007);
  sendData(0x0000);    


//////////////////////////////

  _spi->begin();
  pinMode(_sel, OUTPUT);
  digitalWrite(_sel, HIGH);
  delay(10);
  SPIFlashRead0x9F(deviceSFDPBuf,3);
  deviceIDBuf[1]=deviceSFDPBuf[0];
  deviceIDBuf[2]=deviceSFDPBuf[1];
  deviceIDBuf[3]=deviceSFDPBuf[2];
  return true;
  do{     
    SPIFlashReadSFDP(deviceSFDPBuf,0,4);
    //SPIFlashRead0x90(deviceSFDPBuf,2);
    rxBuffer[1]=deviceSFDPBuf[0];
    rxBuffer[2]=deviceSFDPBuf[1];
    rxBuffer[3]=deviceSFDPBuf[2];
    rxBuffer[4]=deviceSFDPBuf[3];
    if ((0x53 == deviceSFDPBuf[0]) && (0x46 == deviceSFDPBuf[1]) 
      && (0x44 == deviceSFDPBuf[2]) && (0x50 == deviceSFDPBuf[3]))      
    {
      correctFlag = 1;
    }
    retransmissionTimes++;
    if (3 == retransmissionTimes)
    {
      retransmissionTimes = 0;
      return false;
    }
  }while(0 == correctFlag);
  correctFlag = 0;
  retransmissionTimes = 0;
  return true;
}

/************************************************************************* 
Description:Sends playback control commands
parameter:  cmd：playback control commands
            data : 0x00~0x7f is select the voice 0~127 to play if cmd is 0xfa
            0x00~0x7f is select the voice 128~255 to play if cmd is 0xfb       
Return:     void      
Others:         
*************************************************************************/
void BMV31K304::writeCmd(uint8_t cmd, uint8_t data)
{
  delayMicroseconds(5000);
  uint8_t i, temp;
  temp = 0x01;
    
  if(0xff != data)
  {
        //start signal
    digitalWrite(_data, LOW);
    delay(5);

    for (i = 0; i < 8; i ++)
    {
      if (1 == (cmd & temp))
      {
        // out bit high
        digitalWrite(_data, HIGH);
        delayMicroseconds(1200);
        digitalWrite(_data, LOW);
        delayMicroseconds(400);
      }
      else
      {
        // out bit low
        digitalWrite(_data, HIGH);
        delayMicroseconds(400);
        digitalWrite(_data, LOW);
        delayMicroseconds(1200);
      }
            cmd >>= 1;
    }
    digitalWrite(_data, HIGH);
    delay(5);
    //start signal
    digitalWrite(_data, LOW);
    delay(5);

    for (i = 0; i < 8; i ++)
    {
      if (1 == (data & temp))
      {
        // out bit high
        digitalWrite(_data, HIGH);
        delayMicroseconds(1200);
        digitalWrite(_data, LOW);
        delayMicroseconds(400);
      }
      else
      {
        // out bit low
        digitalWrite(_data, HIGH);
        delayMicroseconds(400);
        digitalWrite(_data, LOW);
        delayMicroseconds(1200);
      }
      data >>= 1;
    }
    digitalWrite(_data, HIGH);
    delay(5);
  }
  else
  {
    //start signal
    digitalWrite(_data, LOW);
    delay(5);
    for (i = 0; i < 8; i ++)
    {
      if (1 == (cmd & temp))
      {
        // out bit high
        digitalWrite(_data, HIGH);
        delayMicroseconds(1200);
        digitalWrite(_data, LOW);
        delayMicroseconds(400);
      }
      else
      {
        // out bit low
        digitalWrite(_data, HIGH);
        delayMicroseconds(400);
        digitalWrite(_data, LOW);
        delayMicroseconds(1200);
      }
      cmd >>= 1;
    }
    digitalWrite(_data, HIGH);
    delay(5);        
  }
}

/************************************************************************* 
Description:Enter update mode
parameter:  mode:set mode     
Return:     true:Enter mode successfully
            false:Failed to enter mode
Others:         
*************************************************************************/
bool BMV31K304::programEntry(uint16_t mode)
{
  static uint8_t retransmissionTimes = 0;
	
  digitalWrite(_power, LOW);
  //pinMode(STATUS_PIN, OUTPUT);
  //digitalWrite(STATUS_PIN, LOW);
  pinMode(_data, OUTPUT);
  digitalWrite(_data, LOW);
  pinMode(_sel, OUTPUT);
  digitalWrite(_sel, LOW);
  pinMode(_icpck, OUTPUT);
  digitalWrite(_icpck, LOW);
  pinMode(_icpda, OUTPUT);
  digitalWrite(_icpda, LOW);
    
  delay(10);
  //pinMode(STATUS_PIN, OUTPUT);
  //digitalWrite(STATUS_PIN, LOW);
  pinMode(_icpck, OUTPUT);
  digitalWrite(_icpck, LOW);
  pinMode(_icpda, OUTPUT);
  digitalWrite(_icpda, LOW);
  delay(5);
  digitalWrite(_icpck, LOW);
  // pinMode(STATUS_PIN, INPUT);
  delay(1);
  digitalWrite(_power, HIGH);
  digitalWrite(_icpck, HIGH);
  delay(2);
  digitalWrite(_icpda, HIGH);
  do{
    /*READY*/
    digitalWrite(_icpck, LOW);
    delayMicroseconds(160);//tready:150us~

    /*MATCH*/
    digitalWrite(_icpck, HIGH);
    delayMicroseconds(84);//tmatch:60us~
    /*Match Pattern and set mode:0100 1010 1xxx*/
    matchPattern(mode);
    retransmissionTimes++;
    if(5 == retransmissionTimes)
    {
      retransmissionTimes = 0;
      return false;
    }
  }while(mode != ack());
  dummyClocks();
  retransmissionTimes = 0;
  return true;
}

/************************************************************************* 
Description:ack of mode
parameter:  void       
Return:     mode data
Others:         
*************************************************************************/
uint16_t BMV31K304::ack(void)
{
  /*MSB*/
  static uint8_t i;
  uint16_t ackData = 0;
  pinMode(_icpda, INPUT);
  digitalWrite(_icpck, LOW);
  for (i = 0; i < 3; i++)
  {
    digitalWrite(_icpck, HIGH);
    digitalWrite(_icpck, LOW);
    if (HIGH == digitalRead(_icpda))
    {
      ackData |= (0x04 >> i);
    }
    else
    {
      ackData &= ~(0x04 >> i);
    }
    delayMicroseconds(5);
  } 
  digitalWrite(_icpck, HIGH);
  pinMode(_icpda, OUTPUT);
  return ackData;
}

/************************************************************************* 
Description:Send the dummy Clocks
parameter:  void       
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::dummyClocks(void)
{
  static uint16_t i;
  for (i = 0; i < 512; i++)
  {
    digitalWrite(_icpck, LOW);
    delayMicroseconds(1);
    digitalWrite(_icpck, HIGH);
    delayMicroseconds(1);    
  }
}

/************************************************************************* 
Description:Send data bit in high
parameter:  void   
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::programDataOut1(void)
{
  digitalWrite(_icpda, HIGH);
  delayMicroseconds(1);
  digitalWrite(_icpck, LOW);  
  delayMicroseconds(1);//tckl:1~15us
  digitalWrite(_icpck, HIGH);
}

/************************************************************************* 
Description:Send data bit in low
parameter:  void       
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::programDataOut0(void)
{
  digitalWrite(_icpda, LOW);
  delayMicroseconds(1);
  digitalWrite(_icpck, LOW);
  delayMicroseconds(1);//tckl:1~15us
  digitalWrite(_icpck, HIGH);
}

/************************************************************************* 
Description:Send address bit in high
parameter:  void    
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::programAddrOut1(void)
{
  /*at entry mode :tckl+tckh < 15us*/
  digitalWrite(_icpda, HIGH);
  digitalWrite(_icpck, LOW);  
  delayMicroseconds(1);//tckl:1~15us
  digitalWrite(_icpck, HIGH);
  delayMicroseconds(4);//tckh:1~15us
}

/************************************************************************* 
Description:Send address bit in low
parameter:  void     
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::programAddrOut0(void)
{
  digitalWrite(_icpda, LOW);
  digitalWrite(_icpck, LOW);
  delayMicroseconds(1);//tckl:1~15us
  digitalWrite(_icpck, HIGH);
  delayMicroseconds(4);//tckh:1~15us
}

/************************************************************************* 
Description:Pattern(mode) matching
parameter:  mode:0x02       
Return:     void       
Others:         
*************************************************************************/
void BMV31K304::matchPattern(uint16_t mode)
{
  uint16_t i, temp, pattern, mData;
  pattern = 0x4A8;//0100 1010 1000:low 3 bits are mode; high 9 bits are fixed
  mData = (pattern | mode) << 4;
	temp = 0x8000;//MSB

	for (i = 0; i < 12; i++)
	{
		if(mData&temp)
			programDataOut1();
		else
			programDataOut0();	
		mData <<= 1;
	}
  digitalWrite(_icpda, HIGH);
}

/************************************************************************* 
Description:Send the address
parameter:  addr;send addr       
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::sendAddr(uint16_t addr)
{
  pinMode(_icpda, OUTPUT);
  digitalWrite(_icpda, HIGH);
    /*LSB*/
	uint16_t i, temp;
	temp = 0x0001;//LSB	
	for (i = 0; i < 12; i++)
	{
		if (addr & temp)
			programAddrOut1();
		else
			programAddrOut0();		
		addr >>= 1;	
	}
}

/************************************************************************* 
Description:Send the data
parameter:  data:Data sent to the BMV31K304 at a fixed address
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::sendData(uint16_t data)
{
  pinMode(_icpda, OUTPUT);
	uint16_t i, temp;
	temp = 0x0001;//LSB

	for (i = 0; i < 14; i++)
	{
		if (data & temp)
			programDataOut1();
		else
			programDataOut0();	
		data >>= 1;		
	}
  delayMicroseconds(1);
  digitalWrite(_icpck, LOW);
  delayMicroseconds(1);
  digitalWrite(_icpck, HIGH);
  delayMicroseconds(2000);
	digitalWrite(_icpck, LOW);
  delayMicroseconds(1);
  digitalWrite(_icpck, HIGH);
  delayMicroseconds(5);
}

/************************************************************************* 
Description:Read the data
parameter:  void
Return:     data:Data  
Others:         
*************************************************************************/
uint16_t BMV31K304::readData(void)
{
    /*LSB*/
	uint8_t i;
  uint16_t rxData = 0;
  pinMode(_icpda, INPUT);
  digitalWrite(_icpck, LOW);    	
  for (i = 0; i < 14; i++)
  {
    digitalWrite(_icpck, LOW);
    if (HIGH == digitalRead(_icpda))
    {
      rxData |= (0x01 << i);
    }
    else
    {
      rxData &= ~(0x01 << i);
    }
    digitalWrite(_icpck, HIGH);
    delayMicroseconds(2);
  }
  digitalWrite(_icpck, HIGH);//15th
  delayMicroseconds(2);
  digitalWrite(_icpck, LOW);
  delayMicroseconds(1);
  digitalWrite(_icpck, HIGH);//16th
  delayMicroseconds(2000);
  digitalWrite(_icpck, LOW);
  delayMicroseconds(1);
  digitalWrite(_icpck, HIGH);
  return rxData;
}

/************************************************************************* 
Description:Data check
parameter:  *ptr:The array to check
            len:Length of data to be check       
Return:     1:correct
            0:error
Others:         
*************************************************************************/
uint8_t BMV31K304::checkCRC8(uint8_t *ptr, uint8_t len) 
{
  uint8_t  crc = 0x00;

  while (len--)
  {
    crc = crc_table[crc ^ *ptr++];
  }
  return (crc);
}

/************************************************************************* 
Description:Receive audio data update from upper computer into BMV31K304
parameter:  void       
Return:     1:Correct reception; 0:Error of reception
Others:         
*************************************************************************/
void BMV31K304::recAudioData(void)
{
  static int8_t dataLength = 0;
  static uint8_t remainder = 0;
  static uint32_t sumDataCnt = 0;
  if ((0x55 == rxBuffer[0]) && (0x23 == rxBuffer[1]))
  {
    rxBuffer[0]=rxBuffer[1]=0;
    dataLength = rxBuffer[2];
    SerialUSB.readBytes(rxBuffer + 3, dataLength + 2);  
    if(rxBuffer[dataLength + 3] == checkCRC8(rxBuffer + 2, dataLength + 1))
    {
      sumDataCnt += dataLength;
      remainder = sumDataCnt % 64;
      if (remainder <= 59)
      {
        SPIFlashPageWrite(rxBuffer + 3, _flashAddr, dataLength - remainder);
        SPIFlashPageWrite(rxBuffer + 3 + dataLength - remainder, _flashAddr + dataLength - remainder, remainder);         
      }
      else
      {
        SPIFlashPageWrite(rxBuffer + 3, _flashAddr, dataLength);
      }
                
      _flashAddr += dataLength;
      SerialUSB.write(0x3e);//ACK
    }
    else
    {
      SerialUSB.write(0xe3);//NACK
      return;
    }
  }
}

/************************************************************************* 
Description:Enables the write access to the FLASH.
parameter:  void      
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::SPIFlashWriteEnable(void)
{
  /* Select the FLASH: Chip Select low */
  digitalWrite(_sel, LOW);

  /* Send instruction */
  _spi->transfer(WREN);

  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel, HIGH);
}

/************************************************************************* 
Description:Polls the status of the Write In Progress (WIP) flag in 
            the FLASH's status register and loop until write  opertaion has completed.
parameter:  void                       
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::SPIFlashWaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;
  /* Select the FLASH: Chip Select low */
  digitalWrite(_sel, LOW);	
  /* Send "Read Status Register" instruction */
  _spi->transfer(RDSR);
  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH 
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = _spi->transfer(DUMMY_BYTE);

  } while((FLASH_Status & WIP_FLAG) == 1); /* Write in progress */
    /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel, HIGH);	
}

/************************************************************************* 
Description:Erases the entire FLASH.
parameter:  void      
Return:     void    
Others:         
*************************************************************************/
void BMV31K304::SPIFlashChipErase(void)
{
  //int i;
  /* Send write enable instruction */
  SPIFlashWriteEnable();
  /* Bulk Erase */ 
  /* Select the FLASH: Chip Select low */
  digitalWrite(_sel, LOW);
  /* Send Chip Erase instruction  */
  _spi->transfer(CE);
  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel, HIGH);	
  delay(200);
  /* Wait the end of Flash writing */
  SPIFlashWaitForWriteEnd();
}

/************************************************************************* 
Description:Writes more than one byte to the FLASH with a single WRITE cycle(Page WRITE sequence). 
            The number of byte can't exceed the FLASH page size.
parameter:  pBuffer : pointer to the buffer  containing the data to be written to the FLASH.
            writeAddr : FLASH's internal address to write to.
            numByteToWrite : number of bytes to write to the FLASH, must be equal or less 
            than "SPI_FLASH_PAGESIZE" value.       
Return:     void        
Others:         
*************************************************************************/
void BMV31K304::SPIFlashPageWrite(uint8_t* pBuffer, uint32_t writeAddr, uint16_t numByteToWrite)
{
  /* Enable the write access to the FLA
  SH */
  SPIFlashWriteEnable();
  /* Select the FLASH: Chip Select low */
  digitalWrite(_sel, LOW);
  /* Send "Write to Memory " instruction */
  _spi->transfer(PP);
  /* Send writeAddr high nibble address byte to write to */
  _spi->transfer((writeAddr & 0xFF0000) >> 16);
  /* Send writeAddr medium nibble address byte to write to */
  _spi->transfer((writeAddr & 0xFF00) >> 8);  
  /* Send writeAddr low nibble address byte to write to */
  _spi->transfer(writeAddr & 0xFF);
  
  /* while there is data to be written on the FLASH */
  while(numByteToWrite--) 
  {
    /* Send the current byte */
    _spi->transfer(*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++; 
  }
  
  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel, HIGH);	
  /* Wait the end of Flash writing */
  SPIFlashWaitForWriteEnd();
}

/************************************************************************* 
Description:Read SFDP.
parameter:  pBuffer : pointer to the buffer that receives the data read from the FLASH.
            ReadAddr : FLASH's internal address to read from.
            NumByteToRead : number of bytes to read from the FLASH.        
Return:     void        
Others:         
*************************************************************************/
void BMV31K304::SPIFlashReadSFDP(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  digitalWrite(_sel,LOW);	
  /* Send "Read from Memory " instruction */
  _spi->transfer(SFDP);
  /* Send ReadAddr high nibble address byte to read from */
  _spi->transfer((ReadAddr & 0xFF0000) >> 16);
    /* Send ReadAddr medium nibble address byte to read from */
  _spi->transfer((ReadAddr& 0xFF00) >> 8);
    /* Send ReadAddr low nibble address byte to read from */
  _spi->transfer(ReadAddr & 0xFF);
	/* Send 1 byte dummy clock */
	_spi->transfer(DUMMY_BYTE);
  //SPI_FIFOReset(SPIx, SPI_FIFO_RX);
  while(NumByteToRead--) /* while there is data to be read */
  {
		/* Read a byte from the FLASH */
		*pBuffer = _spi->transfer(DUMMY_BYTE);
		/* Point to the next location where the byte read will be saved */
		pBuffer++;
  }
  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel,HIGH);	
}

/************************************************************************* 
Description:Read 0x90
parameter:  pBuffer : pointer to the buffer that receives the data read from the FLASH.
            NumByteToRead : number of bytes to read from the FLASH.        
Return:     void       
Others:         
*************************************************************************/
void BMV31K304::SPIFlashRead0x90(uint8_t* pBuffer,  uint16_t NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  delay(100);
  digitalWrite(_sel,LOW);	

  /* Send "Read from Memory " instruction */
  _spi->transfer(0x90);

	/* Send 1 byte dummy clock */
	_spi->transfer(DUMMY_BYTE);
	/* Send 1 byte dummy clock */
	_spi->transfer(DUMMY_BYTE);	
  /* Send 1 byte dummy clock */
	_spi->transfer(DUMMY_BYTE);

  //SPI_FIFOReset(SPIx, SPI_FIFO_RX);

  while(NumByteToRead--) /* while there is data to be read */
  {
		/* Read a byte from the FLASH */
		*pBuffer = _spi->transfer(DUMMY_BYTE);
		/* Point to the next location where the byte read will be saved */
		pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel,HIGH);	
}

/************************************************************************* 
Description:Read 0x9f
parameter:  pBuffer : pointer to the buffer that receives the data read from the FLASH.
            NumByteToRead : number of bytes to read from the FLASH.        
Return:     void        
Others:         
*************************************************************************/
void BMV31K304::SPIFlashRead0x9F(uint8_t* pBuffer,  uint16_t NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  delay(100);
  digitalWrite(_sel,LOW);	

  /* Send "Read from Memory " instruction */
  _spi->transfer(0x9F);

  while(NumByteToRead--) /* while there is data to be read */
  {
		/* Read a byte from the FLASH */
		*pBuffer = _spi->transfer(DUMMY_BYTE);
		/* Point to the next location where the byte read will be saved */
		pBuffer++;
  }
  /* Deselect the FLASH: Chip Select high */
  digitalWrite(_sel,HIGH);	
}
