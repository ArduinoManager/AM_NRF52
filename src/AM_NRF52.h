/*
   AMController libraries, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Author: Fabrizio Boco - fabboco@gmail.com

   Version: 1.1.0

   All rights reserved

*/
#ifndef AMCONTROLLERRF52_H
#define AMCONTROLLERRF52_H

#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <string.h>

#define ALARMS_SUPPORT          // uncomment to enable support for Alarm Widget - LEFT THIS ALONE AT THE MOMENT
//#define SD_SUPPORT              // uncomment to enable support for SD Widget  - LEFT THIS ALONE AT THE MOMENT
//#define SDLOGGEDATAGRAPH_SUPPORT    // uncomment to enable support for Logged Data Widget - LEFT THIS ALONE AT THE MOMENT
//#define DEBUG           // uncomment to enable debugging - You should not need it !

#if defined(ALARMS_SUPPORT)

#include "utility/Alarm.h"
#include "utility/FileManager.h"

#define MAX_ALARMS             5      // Maximum number of Alarms Widgets
#define ALARM_CHECK_INTERVAL  60      // [s]

#endif

#define VARIABLELEN       		14
#define VALUELEN          		14
#define WRITE_DELAY						10		  // Delay after notification

class AMController {

  private:

  	bool      	_var;
  	char    		_variable[VARIABLELEN + 1];
  	char      	_value[VALUELEN + 1];
  	uint8_t   	_idx = 0;  	
    String			_remainBuffer;
    bool      	_connected;    
    bool				_sync;
    
    BLEService 	arduinoManagerService;
    BLEBas			batteryLevelService;

#ifdef SD_SUPPORT
    SDLib::File      _root;
    SDLib::File      _entry;
#endif

#ifdef ALARMS_SUPPORT
    char 						_alarmId[8];
    bool 						_alarmRepeat;
    String          _alarmFile;
    unsigned long   _lastAlarmCheck;
    unsigned long   _startTime;
    unsigned long   _tmpTime;    
#endif

    /**
      Pointer to the function where to put code in place of loop()
    **/
    void (*_doWork)(void);

    /*
      Pointer to the function where Switches, Knobs and Leds are syncronized
    */
    void (*_doSync)(void);

    /*
      Pointer to the function where incoming messages are processed

      variable

      value

    */
    void (*_processIncomingMessages)(char *variable, char *value);

    /*
      Pointer to the function where outgoing messages are processed
    */
    void (*_processOutgoingMessages)(void);

    /*
      Pointer to the function called when a device connects to Arduino

    */
    void (*_deviceConnected)(void);

    /*
      Pointer to the function called when a device disconnects from Arduino

    */
    void (*_deviceDisconnected)(void);

#ifdef SD_SUPPORT
	void processSD(char *variable, char *value);
#endif	

#ifdef ALARMS_SUPPORT
		void processAlarms(char *variable, char *value);
    /*
      Pointer to the function where alerts are processed
    */
    void (*_processAlarms)(char *alarm);
#endif

    void readVariable(void);
    void setupArduinoManagerService(void);
    void startAdv(void);


#ifdef ALARMS_SUPPORT

    void breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day);

    void syncTime();
    void readTime();

    void inizializeAlarms();
    void checkAndFireAlarms();
    void createUpdateAlarm(char *id, unsigned long time, bool repeat);
    void removeAlarm(char *id);
		
#endif

  public:

    BLECharacteristic arduinoManagerCharacteristic;

    AMController(
      void (*doWork)(void),
      void (*doSync)(void),
      void (*processIncomingMessages)(char *variable, char *value),
      void (*processOutgoingMessages)(void),
      void (*deviceConnected)(void),
      void (*deviceDisconnected)(void));

#if defined(ALARMS_SUPPORT)

    AMController(
      void (*doWork)(void),
      void (*doSync)(void),
      void (*processIncomingMessages)(char *variable, char *value),
      void (*processOutgoingMessages)(void),
      void (*processAlarms)(char *alarm),
      void (*deviceConnected)(void),
      void (*deviceDisconnected)(void));
#endif

    void begin();    
/**
This function has to be called at least once in the loop 
to properly start the advertisement data with the desired device name
**/    
    void setDeviceName(const char *deviceName);  

    void loop();
    void loop(unsigned long delay);
    void writeMessage(const char *variable, int value);
    void writeMessage(const char *variable, float value);
    void writeTripleMessage(const char *variable, float vX, float vY, float vZ);
    void writeTxtMessage(const char *variable, const char *value);
    
    void updateBatteryLevel(uint8_t level); // A value between 0% and 100%

    void log(const char *msg);
    void log(int msg);

    void logLn(const char *msg);
    void logLn(int msg);
    void logLn(long msg);
    void logLn(unsigned long msg);

    void temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms);

#ifdef ALARMS_SUPPORT
    unsigned long now();
#ifdef DEBUG
    void dumpAlarms();
    void printTime(unsigned long time);
#endif
#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT

    void sdLogLabels(const char *variable, const char *label1);
    void sdLogLabels(const char *variable, const char *label1, const char *label2);
    void sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3);
    void sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4);
    void sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4, const char *label5);

    void sdLog(const char *variable, unsigned long time, float v1);
    void sdLog(const char *variable, unsigned long time, float v1, float v2);
    void sdLog(const char *variable, unsigned long time, float v1, float v2, float v3);
    void sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4);
    void sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4, float v5);

    void sdSendLogData(const char *variable);

    uint16_t sdFileSize(const char *variable);
    void sdPurgeLogData(const char *variable);
#endif

		void writeBuffer(uint8_t *buffer, int l);
    void processIncomingData(char *data, uint16_t len);
    void notifyConnected(void);
    void notifyDisconnected(void);
};

#endif

