/*
   Test Arduino Manager for iPad / iPhone / Mac

   A simple test program to show the Arduino Manager
   features.

   Author: Fabrizio Boco - fabboco@gmail.com

   Version: 1.0

   05/29/2021

   All rights reserved

*/

/*
   AMController libraries, example sketches (The Software) and the related documentation (The Documentation) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under authors copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an AS IS basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "AM_NRF52.h"

#if defined(ALARMS_SUPPORT)
AMController amController(&doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &processAlarms, &deviceConnected, &deviceDisconnected);
#else
AMController amController(&doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &deviceConnected, &deviceDisconnected);
#endif

#define YELLOWLEDPIN 16
#define REDLEDPIN 15
#define TEMPERATUREPIN A0
#define POTENTIOMETERPIN A1


float temperature;
bool yellowLed = HIGH;
uint16_t pot;
uint16_t ledIntensity = 0;

long int lastTempMeasurementMillis;

void setup() {

  Serial.begin(115200);
  Serial.println("AM_NRF52Example");
  Serial.println("-------------------------");

  amController.begin();

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(REDLEDPIN, OUTPUT);

  pinMode(YELLOWLEDPIN, OUTPUT);
  digitalWrite(YELLOWLEDPIN, yellowLed);


  analogWrite(REDLEDPIN, ledIntensity);

  Serial.println("**** Advertising ****");
  Serial.println("Ready");
}


void loop() {
  amController.loop();
}

/**
  This function is called periodically and its equivalent to the standard loop() function
*/
void doWork() {

  if (millis() - lastTempMeasurementMillis > 2500) {
    lastTempMeasurementMillis = millis();
    float voltage;
    voltage = getVoltage(TEMPERATUREPIN);  //getting the voltage reading from the temperature sensor
    temperature = (voltage - 0.5) * 100.0;
  }

  digitalWrite(YELLOWLEDPIN, yellowLed);
  pot = analogRead(POTENTIOMETERPIN);
}

/**
  This function is called when the ios device connects and needs to initialize the position of switches and knobs
*/
void doSync() {
  //Serial.print("Sync ");
  amController.writeMessage("S1", digitalRead(YELLOWLEDPIN));
  amController.writeMessage("Knob1", ledIntensity);
}

/**
  This function is called when a new message is received from the iOS device
*/
void processIncomingMessages(char *variable, char *value) {
  //Serial.print(variable); Serial.print(" "); Serial.println(value);

  if (strcmp(variable, "S1") == 0) {
    yellowLed = atoi(value);
  }

  if (strcmp(variable, "Push1") == 0) {
    if (strcmp(value, "1") == 0)
      digitalWrite(LED_BUILTIN, HIGH);

    if (strcmp(value, "0") == 0)
      digitalWrite(LED_BUILTIN, LOW);
  }

  if (strcmp(variable, "Knob1") == 0) {
    ledIntensity = atoi(value);
    analogWrite(REDLEDPIN, map(ledIntensity, 0, 1023, 0, 255));
  }
}

/**
  This function is called periodically and messages can be sent to the iOS device
*/
void processOutgoingMessages() {

  amController.writeMessage("Led", yellowLed);
  amController.writeMessage("T", temperature);
  amController.writeMessage("Pot", pot);
}

#if defined(ALARMS_SUPPORT)
/**


  This function is called when a Alarm is fired

*/
void processAlarms(char *alarm) {
  Serial.print(alarm); Serial.println(" fired");
}
#endif

/**
  This function is called when the iOS device connects
*/
void deviceConnected() {

  Serial.println("Device connected");
}

/**
  This function is called when the iOS device disconnects
*/
void deviceDisconnected() {
  Serial.println("Device disconnected");
}

/**
  Additional functions
**/
float getVoltage(int pin) {
  return (analogRead(pin) * 3.3F / 1024.0F);
}