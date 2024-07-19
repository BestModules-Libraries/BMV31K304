/*************************************************************************
File:         BMV31K304.h
Author:       BEST MODULES CORP.
Description:  Define classes and required variables
History：  V1.0.1   -- 2024-07-19
**************************************************************************/
#ifndef _BMV31K304_H
#define _BMV31K304_H

#include <SPI.h>
#include <Arduino.h>
#include <stdio.h>
#include <math.h>
/*************************playback control command***************************************************************************************
 * Play voice                                00H~7FH ——> when the 0xfa command is used,00H:is voice 0； from 0 to 127;
                                                         when the 0xfa cammand is used ,00H:is voice 128;from 128 to 255.
 * Play sentence                             80H~DFH ——> 80H:is sentence 0；from 0 to 95, there are 96 sentences.
 * Volume selection                          E1H~ECH ——> E1H:is the minimum volume（mute）；There are 12 levels of volume adjustment.
 * Pause voice/sentence                      F1H     ——> Pause playing the current voice and sentence.
 * Play after pause                          F2H      ——> Continue playing the paused voice and sentence.
 * Loop playback the current voice/sentence  F4H      ——> Loop playback for the current voice and sentence.
 * Stop playing the current voice/sentence   F8H      ——> Stop playing the current voice and sentence.
**************************************************************************************************************************************/
#define BMV31K304_LED_ON	 	    1
#define BMV31K304_LED_OFF    	  0
#define BMV31K304_BUSY	 	 	    1
#define BMV31K304_NOBUSY     	  0
#define BMV31K304_POWER_ENABLE	1	 	 
#define BMV31K304_POWER_DISABLE 0
#define BMV31K304_UPDATE_BEGIN  1
#define BMV31K304_NO_KEY		    0
#define BMV31K304_VOLUME_MAX    11
#define BMV31K304_VOLUME_MIN	  0

class BMV31K304
{
public:
	BMV31K304(uint8_t cs1_ledPin = 29,SPIClass *spiClass = &SPI1,uint8_t powerPin = 22);
	void begin(void);
	void setVolume(uint8_t volume = 8);
	void playVoice(uint8_t num, uint8_t loop = 0);
	void playSentence(uint8_t num, uint8_t loop = 0);
	void playStop(void);
	void playPause(void);
	void playContinue(void);
	void playRepeat(void);
	bool isPlaying(void);
	void setLED(uint8_t status);
  
	void initAudioUpdate(unsigned long baudrate = 256000);
	bool isUpdateBegin(void);
  bool executeUpdate(uint8_t mode);
private:
  bool executeUpdateWidget(void);
  bool executeUpdateWorkshop(void);
  void reset(void);
  void setPower(uint8_t status);
  uint8_t CheckIC(void);
  bool switchSPIMode(void);  
  void writeCmd(uint8_t cmd, uint8_t data = 0xff);
	//--------------------program voice source--------------------------
  bool programEntry(uint16_t mode);
  uint16_t ack(void);
  void dummyClocks(void);
  void programDataOut1(void);
  void programDataOut0(void);
  void programAddrOut1(void);
  void programAddrOut0(void);
  void matchPattern(uint16_t mode);
  void sendAddr(uint16_t addr);
  void sendData(uint16_t data);
  uint16_t readData(void);

  uint8_t checkCRC8(uint8_t *ptr, uint8_t len); 
  void recAudioData(void);
  void SPIFlashWriteEnable(void);
  void SPIFlashWaitForWriteEnd(void);
  void SPIFlashChipErase(void);
  void SPIFlashPageWrite(uint8_t* pBuffer, uint32_t writeAddr, uint16_t numByteToWrite);
  void SPIFlashReadSFDP(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
  void SPIFlashRead0x90(uint8_t* pBuffer,  uint16_t NumByteToRead);
  void SPIFlashRead0x9F(uint8_t* pBuffer,  uint16_t NumByteToRead);
  
  uint8_t   deviceIDBuf[4];
  uint8_t   deviceSFDPBuf[3];
  uint8_t   rxBuffer[64];
  uint32_t  _flashAddr;
  uint8_t   _EraseCnt;

  SPIClass *_spi = NULL;
  uint8_t _power = 22;
  uint8_t _sel = 29;
  uint8_t _icpck = 27;
  uint8_t _icpda = 28;
  uint8_t _data = 26;
};
#endif
