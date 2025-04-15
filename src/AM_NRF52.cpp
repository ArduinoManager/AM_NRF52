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

   All rights reserved

*/
#include "AM_NRF52.h"
#include <string.h>


#ifdef DEBUG
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
static  const uint8_t monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; // API starts months from 1, this array starts from 0
#endif

#if defined(ALARMS_SUPPORT)

bool check(uint8_t *pRecord, void *pData) {
  Alarm a;
  memcpy(&a, pRecord, sizeof(a));
  if (strcmp(a.id, (char *)pData) == 0)
    return true;
  return false;
}

#endif

void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value);
void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
//char *dtostrf (double val, signed char width, unsigned char prec, char *sout);

AMController *myGlobal;

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)
) {
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;
  _sync = false;  
  _var = true;
	_remainBuffer[0]='\0';

  myGlobal = this;
}

#if defined(ALARMS_SUPPORT)

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(void),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*processAlarms)(char *alarm),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)
) : AMController(doWork, doSync, processIncomingMessages, processOutgoingMessages, deviceConnected, deviceDisconnected) {
  _alarmFile = "ALARMS.TXT";
  _processAlarms = processAlarms;
  _startTime = 0;
  _lastAlarmCheck = 0;
  //inizializeAlarms();
}
#endif

void AMController::begin() {

  // Initialise the Bluefruit module
#ifdef DEBUG    
  Serial.println("Initialise the Bluefruit nRF52 module");
#endif

  Bluefruit.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(&connect_callback);
  Bluefruit.Periph.setDisconnectCallback(&disconnect_callback);
  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(500);

  Bluefruit.setName("AManager");
  
  // Setup the Arduino Manager Service
#ifdef DEBUG  
  Serial.println("Configuring the Arduino Manager Service");
#endif  
  setupArduinoManagerService();

#ifdef DEBUG  
  Serial.println("Setting up the advertising payload(s)");
#endif  
	// Setup the advertising packet(s)
  startAdv();
}

/**
This function has to be called at least once in the loop 
to properly start the advertisement data with the desired device name
**/
void AMController::setDeviceName(const char *deviceName) {

#ifdef DEBUG    
  Serial.print("Setting new name "); Serial.println(deviceName);
#endif
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();     
	Bluefruit.setName(deviceName);

  // Setup the advertising packet(s)
#ifdef DEBUG  	
	Bluefruit.printInfo();
  Serial.println("Setting up the advertising payload(s)");
#endif  

	startAdv();
}

void AMController::startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include ArduinoManager Service UUID
  Bluefruit.Advertising.addService(arduinoManagerService);

  // Include Name
  Bluefruit.Advertising.addName();
  //Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
  
#ifdef DEBUG  
  Serial.println("startAdv Completed");
#endif  
}

void AMController::setupArduinoManagerService(void) {

  arduinoManagerService = BLEService(0x108D);
  arduinoManagerService.begin();

  arduinoManagerCharacteristic = BLECharacteristic(0x0001);
  arduinoManagerCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY | CHR_PROPS_WRITE_WO_RESP);
  arduinoManagerCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  arduinoManagerCharacteristic.setFixedLen(20);
  arduinoManagerCharacteristic.setCccdWriteCallback(cccd_callback);
  arduinoManagerCharacteristic.setWriteCallback(write_callback);

  arduinoManagerCharacteristic.begin();

  uint8_t data[20];
  
  for(int i=0; i<20; i++) 
  	data[i] = 0x00;
  	
  arduinoManagerCharacteristic.notify(data, 20);                   // Use .notify instead of .write!
}

void AMController::loop() {
  this->loop(0);
}

void AMController::loop(unsigned long _delay) {

	if (_sync) {
		_sync = false;
		_doSync();
	}
	
  _doWork();

  if (_connected)
    _processOutgoingMessages();
    
#ifdef ALARMS_SUPPORT

  if (_processAlarms != NULL) {
    unsigned long now = this->now();
// printTime(now);
// Serial.println();
    if ( (now - _lastAlarmCheck) > ALARM_CHECK_INTERVAL) {
      _lastAlarmCheck = now;
      this->checkAndFireAlarms();
    }
  }

#endif    
  
  delay(_delay);
}


void AMController::processIncomingData(char *data, uint16_t len) {
	
	_remainBuffer += String(data).substring(0,len);
	
#ifdef DEBUG		
	//Serial.print("buffer >"); Serial.print(_remainBuffer); Serial.println("<");
#endif
	
	int poundPos = _remainBuffer.indexOf('#');
	if (poundPos != -1) {
		String chunk = _remainBuffer.substring(0,poundPos);
		
#ifdef DEBUG		
		//Serial.print("\tchunk >"); Serial.print(chunk); Serial.println("<");
#endif
		
		int equalPos = chunk.indexOf('=');
		
		if (equalPos != -1) {
		
			strncpy(_variable,chunk.substring(0,equalPos).c_str(), VARIABLELEN);
			strncpy(_value,chunk.substring(equalPos+1).c_str(), VALUELEN);
		
#ifdef DEBUG		
			Serial.print("\tvariable >"); Serial.print(_variable); Serial.println("<");
			Serial.print("\tvalue >"); Serial.print(_value); Serial.println("<");
#endif
		
			if (strlen(_value) > 0 && strcmp(_variable, "Sync") == 0) {
	          // Process sync messages for the variable _value
    	      _sync=true;
      }		
			else {
				if ( strcmp(_variable, "$Time$") == 0 || strcmp(_variable, "$AlarmId$") == 0 || strcmp(_variable, "$AlarmT$") == 0 || strcmp(_variable, "$AlarmR$") == 0 ) {
#ifdef ALARMS_SUPPORT					
					this->processAlarms(_variable, _value);
#endif						
				}
				else {
					if (strcmp(_variable, "SD") == 0) {
#ifdef SD_SUPPORT
						this->processSD(_variable, _value);
#endif						
					}
					else {
						//
						// Processing other variables						
						//
						_processIncomingMessages(_variable, _value);
					}
				}		
			}
		}
		
		_remainBuffer = _remainBuffer.substring(poundPos+1);
#ifdef DEBUG		
		//Serial.print("\tbuffer >"); Serial.print(_remainBuffer); Serial.println("<");
#endif		
	}
	
}

#ifdef ALARMS_SUPPORT
void AMController::processAlarms(char *variable, char *value) {

  if (strcmp(_variable, "$Time$") == 0 && strlen(value) > 0) {

  	_startTime = atol(_value) - millis() / 1000;
#ifdef DEBUG
		Serial.print("Time Synchronized ");
		this->printTime(_startTime);
		Serial.println();
#endif
		return;
  }

	if (strcmp(variable, "$AlarmId$") == 0 && strlen(value) > 0) {

		strcpy(_alarmId, value);
	} else if (strcmp(variable, "$AlarmT$") == 0 && strlen(value) > 0) {

		_tmpTime = atol(_value);
	}
	else if (strcmp(variable, "$AlarmR$") == 0 && strlen(value) > 0) {
		if (_tmpTime == 0)
			this->removeAlarm(_alarmId);
		else
			this->createUpdateAlarm(_alarmId, _tmpTime, atoi(value));
	}
}
#endif

#ifdef SD_SUPPORT
void AMController::processSD(char *variable, char *value) {

	if (strcmp(_variable, "$SDDL$") == 0 && strlen(_variable) > 0) {
#ifdef DEBUG
		Serial.println("List of Files");
#endif
  	_root = SD.open("/", FILE_READ);

  	if (!_root) {
  		Serial.println("Cannot open root dir");
  		return;
  	}

  	_root.rewindDirectory();
  	_entry = _root.openNextFile();

  	while (_entry) {

  		if (!_entry.isDirectory()) {
  	
    		this->writeTxtMessage("SD", _entry.name());
#ifdef DEBUG
      	Serial.println(_entry.name());
#endif
			}

    	_entry.close();
    	_entry = _root.openNextFile();
   }

  	_root.close();
    arduinoManagerCharacteristic.notify("SD=$EFL$#", strlen("SD=$EFL$#"));

#ifdef DEBUG
    Serial.println("File list sent");
#endif
  } 
  else if (strlen(_variable) > 0 && strcmp(_variable, "$SDDL$") == 0) {

#ifdef DEBUG
  	Serial.print("File: "); Serial.println(_value);
#endif

    _entry = SD.open(_value, FILE_READ);

    if (_entry) {

#ifdef DEBUG
    	Serial.println("File Opened");
#endif
      unsigned long n = 0;
      uint8_t buffer[64];

      arduinoManagerCharacteristic.notify("SD=$C$#", strlen("SD=$C$#"));
      delay(3000);

      while (_entry.available()) {
      	n = _entry.read(buffer, sizeof(buffer));
        arduinoManagerCharacteristic.notify(buffer, n * sizeof(uint8_t));
      }
      _entry.close();
      arduinoManagerCharacteristic.notify("SD=$E$#", strlen("SD=$E$#"));

#ifdef DEBUG
      Serial.println("End Sent");
#endif
    }
  }
}
#endif

void AMController::writeMessage(const char *variable, int value) {
char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%d#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  delay(WRITE_DELAY);
}

void AMController::writeMessage(const char *variable, float value) {
	char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%.4f#", variable, value);
  if (strlen(buffer)==21) {
    snprintf(buffer, 128, "%s=%.4f#####", variable, value);
  }
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  delay(WRITE_DELAY);
}

void AMController::writeTripleMessage(const char *variable, float vX, float vY, float vZ) {
	char buffer[VARIABLELEN + VALUELEN + 3];

	if (!_connected) {
    return;
  }
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%.2f:%.2f:%.2f#", variable, vX, vY, vZ);
  writeBuffer((uint8_t *)&buffer, strlen(buffer)*sizeof(char));
  delay(WRITE_DELAY);
}

void AMController::writeTxtMessage(const char *variable, const char *value) {
 char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%s#", variable, value);
  Serial.print("-");Serial.print(buffer); Serial.println("-"); 
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  delay(WRITE_DELAY);
}

/**
	Can send a buffer longer than 20 bytes
**/
void AMController::writeBuffer(uint8_t *buffer, int l) {

  if (!_connected) {
    return;
  }

  uint16_t mtu = Bluefruit.Connection(0)->getMtu();
  //Serial.print("MTU Size: "); Serial.println(mtu);
  uint8_t buffer1[mtu+1];


  uint8_t idx = 0;

  while (idx < l) {

    uint8_t this_block_size = min(mtu, l - idx);
    memset(&buffer1, '\0', mtu);
    memcpy(&buffer1, buffer + idx, this_block_size);
    buffer1[mtu] = '\0';

#ifdef DEBUG
    //Serial.print("\tSending >"); Serial.print((char *)buffer1); Serial.print("< "); Serial.print(strlen((char *)buffer1)); Serial.println();
#endif

Serial.print("\t["); Serial.print(mtu); Serial.print("] ");
Serial.print("\tSending >"); Serial.print((char *)buffer1); Serial.print("< "); Serial.print(strlen((char *)buffer1)); Serial.println();

		arduinoManagerCharacteristic.notify(&buffer1, mtu);
    delay(WRITE_DELAY);

    idx += this_block_size;
  }

  delay(WRITE_DELAY);
}

void AMController::log(const char *msg) {

  this->writeTxtMessage("$D$", msg);
}

void AMController::log(int msg) {

  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$D$", buffer);
}


void AMController::logLn(const char *msg) {

  this->writeTxtMessage("$DLN$", msg);
}

void AMController::logLn(int msg) {

  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(long msg) {

  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(unsigned long msg) {

  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {

  boolean previousValue = digitalRead(pin);

  digitalWrite(pin, value);
  delay(ms);
  digitalWrite(pin, previousValue);
}


#ifdef ALARMS_SUPPORT

#ifdef DEBUG

void AMController::breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day) {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from 1970 !!!

  unsigned long year;
  uint8_t month, monthLength;
  unsigned long days;

  *seconds = time % 60;
  time /= 60; // now it is minutes
  *minutes = time % 60;
  time /= 60; // now it is hours
  *hours = time % 24;
  time /= 24; // now it is days
  *Wday = ((time + 4) % 7) + 1;  // Sunday is day 1

  year = 0;
  days = 0;
  while ((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  *Year = year + 1970; // year is offset from 1970

  days -= LEAP_YEAR(year) ? 366 : 365;
  time -= days; // now it is days in this year, starting at 0

  days = 0;
  month = 0;
  monthLength = 0;
  for (month = 0; month < 12; month++) {
    if (month == 1) { // february
      if (LEAP_YEAR(year)) {
        monthLength = 29;
      }
      else {
        monthLength = 28;
      }
    }
    else {
      monthLength = monthDays[month];
    }

    if (time >= monthLength) {
      time -= monthLength;
    }
    else {
      break;
    }
  }
  *Month = month + 1;  // jan is month 1
  *Day = time + 1;     // day of month
}

void AMController::printTime(unsigned long time) {

  int seconds;
  int minutes;
  int hours;
  int Wday;
  long Year;
  int Month;
  int Day;

  this->breakTime(time, &seconds, &minutes, &hours, &Wday, &Year, &Month, &Day);

  Serial.print(Day);
  Serial.print("/");
  Serial.print(Month);
  Serial.print("/");
  Serial.print(Year);
  Serial.print(" ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.print(seconds);
}
#endif

unsigned long AMController::now() {
  if (_startTime == 0) {
  	// Time never synchronized 
  	return 0;
  }
  unsigned long now = _startTime + millis() / 1000;
  return now;
}

void AMController::createUpdateAlarm(char *id, unsigned long time, bool repeat) {

#ifdef DEBUG
    Serial.println("createUpdateAlarm");
    Serial.print(id); Serial.print(" "); Serial.print(time); Serial.print(" "); Serial.println(repeat);
#endif
  FileManager fileManager;
  Alarm     	a;
  int     		pos;
  pos = fileManager.find(_alarmFile, (uint8_t*)&a, sizeof(a), &check, id);

  if (pos > -1) {

    a.time = time;
    a.repeat = repeat;
    fileManager.update(_alarmFile, pos, (uint8_t *)&a, sizeof(a));
#ifdef DEBUG
    dumpAlarms();
#endif
    return;
  }

  strcpy(a.id, id);
  a.time = time;
  a.repeat = repeat;
  fileManager.append(_alarmFile, (uint8_t *)&a, sizeof(a));
#ifdef DEBUG
  dumpAlarms();
#endif
}

void AMController::removeAlarm(char *id) {

  FileManager fileManager;
  Alarm     a;
  int     pos;
  pos = fileManager.find(_alarmFile, (uint8_t*)&a, sizeof(a), &check, id);

  if (pos > -1) {
    fileManager.remove(_alarmFile, pos, sizeof(a));
  }
#ifdef DEBUG
  dumpAlarms();
#endif
}


#ifdef DEBUG
void AMController::dumpAlarms() {

  Serial.println("\t----Dump Alarms -----");

  FileManager fileManager;

  for (int i = 0; i < MAX_ALARMS; i++) {

    Alarm a;

    if (!fileManager.read(_alarmFile, i, (uint8_t *)&a, sizeof(a)))
      return;

    Serial.print("\tId: "); Serial.print(a.id);
    Serial.print(" time: "); printTime(a.time);
    Serial.print(" Repeat: "); Serial.println(a.repeat);
  }
}
#endif

void AMController::checkAndFireAlarms() {

  FileManager fileManager;
  unsigned long now = this->now();

#ifdef DEBUG
  Serial.print("checkAndFireAlarms ");
  this->printTime(now);
  Serial.println();
  this->dumpAlarms();
#endif

  for (int i = 0; i < MAX_ALARMS; i++) {

    Alarm a;

    if (!fileManager.read(_alarmFile, i, (uint8_t *)&a, sizeof(a)))
      return;

    if (a.time <= now) {

#ifdef DEBUG
      Serial.print("Firing "); Serial.println(a.id);
#endif
      // First character of id is A and has to be removed
      _processAlarms(a.id);

      if (a.repeat) {
        // Alarm rescheduled and updated
        a.time += 86400; // Scheduled again tomorrow
        fileManager.update(_alarmFile, i, (uint8_t *)&a, sizeof(a));
#ifdef DEBUG
        Serial.print("Alarm rescheduled at ");
        this->printTime(a.time);
        Serial.println();
#endif
      }
      else {
        // Alarm removed
        fileManager.remove(_alarmFile, i, sizeof(a));
#ifdef DEBUG
        this->dumpAlarms();
#endif
      }

    }
  }
}

#endif

////////////////////////////////////////////////////

void AMController::notifyConnected() {

  _connected = true;

  if (_deviceConnected != NULL)
    _deviceConnected();
}

void AMController::notifyDisconnected() {

  _connected = false;

  if (_deviceDisconnected != NULL)
    _deviceDisconnected();
}


///////////////////////////////////////////////////////////

void connect_callback(uint16_t conn_handle) {

#ifdef DEBUG
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);
#endif

  myGlobal->notifyConnected();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {

  (void) conn_handle;
  (void) reason;

  myGlobal->notifyDisconnected();
#ifdef DEBUG  
  Serial.println("**** Advertising ****");
#endif  
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) {

  // Display the raw request packet
  Serial.print("CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  Serial.print(cccd_value);
  Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr->uuid == myGlobal->arduinoManagerCharacteristic.uuid) {
    if (chr->notifyEnabled()) {
      Serial.println("'Notify' enabled");
    } else {
      Serial.println("'Notify' disabled");
    }
  }
  if (chr->uuid == myGlobal->arduinoManagerCharacteristic.uuid) {
    if (chr->indicateEnabled()) {
      Serial.println("'Indicate' enabled");
    } else {
      Serial.println("'Indicate' disabled");
    }
  }
}

void write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {

#ifdef DEBUG
  //   Serial.print("write_callback Len "); Serial.println(len);
  //
  //   int idx = 0;
  //   for (int i = 0; i < len; i++) {
  //     Serial.print(data[i], HEX); Serial.print(" - "); Serial.println((char)data[i]);
  //   }
#endif

  myGlobal->processIncomingData((char *)data, len);
}

// char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
// 
//   uint32_t iPart = (uint32_t)val;
//   sprintf(sout, "%d", iPart);
// 
//   if (prec > 0) {
//     uint8_t pos = strlen(sout);
//     sout[pos++] = '.';
//     uint32_t dPart = (uint32_t)((val - (double)iPart) * pow(10, prec));
// 
//     for (uint8_t i = (prec - 1); i > 0; i--) {
//       size_t pow10 = pow(10, i);
//       if (dPart < pow10) {
//         sout[pos++] = '0';
//       }
//       else {
//         sout[pos++] = '0' + dPart / pow10;
//         dPart = dPart % pow10;
//       }
//     }
// 
//     sout[pos++] = '0' + dPart;
//     sout[pos] = '\0';
//   }
// 
//   return sout;
// }


