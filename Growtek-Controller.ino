/**
  ____    _    ____  ____  _____ _   _    ____ ___  _   _ _____ ____   ___  _     _     _____ ____
 / ___|  / \  |  _ \|  _ \| ____| \ | |  / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \
| |  _  / _ \ | |_) | | | |  _| |  \| | | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |
| |_| |/ ___ \|  _ <| |_| | |___| |\  | | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <
 \____/_/   \_\_| \_\____/|_____|_| \_|  \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\

************************************************************************************
**                                  Notes                                         **
************************************************************************************
Version 1.0
 • Included Relay (4 Channel) & humidity/temperature Sensor(s) sketch - Tested & Verified 06/10/17
    - Solenoid on Channel 1
    - Fans to cool control box on Channel 2
    - Channel 3 & 4 are open for future expansion (However they are enabled in the sketch)
 • Included Soil Mositure Sensor(s) - Untested 06/12/17
     - Both use analog ports (A0 & A1) available to expand upto 15
 • Started assembling box and power supply 12v --> 5v & 12v available 06/17/17
 • Included Water Flow Pulse Sensor - 06/18/17

 Version 1.1
  •

  Future Plans
  • Add a screen with information from the system
  • Add LED notification Lights to the system for easy visual interruptation
  • Add more soil sensors
  • Add UV Sensor
  • Clean up control box and make it look better

***********************************************************************************
**                              Connections                                      **
***********************************************************************************
      ------------------------------------------------------------
      | Arduino | NRF24 | Flow Meter | Relay | DHT | Soil Sensor |
      ------------------------------------------------------------
      |  GND    |  GND  |     GND    |  GND  | GND |     GND     |
      |  5V     |   -   |     5V     |       |     |             |
      |  3.3V   |  VCC  |            |       |     |             |
      |  D1     |       |            |       |     |             |
      |  D2     |       |            |       |     |             |
      |  D3     |       |            |       |     |             |
      |  D4     |       |            |       |     |             |
      |  D5     |       |            |       |     |             |
      |  D6     |       |            |       |     |             |
      |  D7     |       |            |       |     |             |
      |  D8     |       |            |       |     |             |
      |  D9     |       |            |       |     |             |
      |  A0     |       |            |       |     |             |
      |  A1     |       |            |       |     |             |
      |         |       |            |       |     |             |
      ------------------------------------------------------------
*/

// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Lets get that radio working with the Mega
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53

// Enable repeater functionality for this node
#define MY_REPEATER_FEATURE

// Pull in those external libraries
#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

// Pin Numbers that the sensors are on and Define some items.
#define DIGITAL_INPUT_SENSOR 2 // The digital input you attached your sensor.  (Only 2 and 3 generates interrupt!)
#define RELAY_1  4  // Arduino Digital I/O pin number for first relay (second on pin+1 etc)
// #define Relay uses Digital I/O pins 4-7
#define DHT1_DATA_PIN 8 // Temperature and Humidity sensor.
#define DHT2_DATA_PIN 9 // Temperature and Humidity sensor.
int mositure_sensor_pin1 = A0;
int mositure_sensor_pin2 = A1;

// Relay Information
#define NUMBER_OF_RELAYS 4 // Total number of attached relays
#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

// Set this offset if the sensor has a permanent small offset to the real temperatures
#define SENSOR_TEMP_OFFSET 0

// Sleep time between sensor updates (in milliseconds)
// Must be >1000ms for DHT22 and >2000ms for DHT11
static const uint64_t UPDATE_INTERVAL = 60000;

// Force sending an update of the temperature after n sensor reads, so a controller showing the
// timestamp of the last update doesn't show something like 3 hours in the unlikely case, that
// the value didn't change since;
// i.e. the sensor would force sending an update every UPDATE_INTERVAL*FORCE_UPDATE_N_READS [ms]
static const uint8_t FORCE_UPDATE_N_READS = 10;

// child ID for the sensors
#define CHILD_ID_HUM1 5 // ID of the DHT humidity sensor child
// #define CHILD_ID_RELAY 1 // ID of the relay (It is expressed in another area of the code)
#define CHILD_ID_TEMP1 6 // ID of the DHT temperature sensor child
#define CHILD_ID_HUM2 7 // ID of the DHT humidity sensor child
#define CHILD_ID_TEMP2 8 // ID of the DHT temperature sensor child
#define CHILD_ID_SOIL1 9 // ID of the first soil monitor
#define CHILD_ID_SOIL2 10 // ID of the second soil monitor
#define CHILD_ID_FLOW 11 // ID of the water flow sensor

float lastTemp1;
float lastHum1;
float lastTemp2;
float lastHum2;
uint8_t nNoUpdatesTemp1;
uint8_t nNoUpdatesHum1;
uint8_t nNoUpdatesTemp2;
uint8_t nNoUpdatesHum2;
bool metric = true;

// Water flow sensor information
//#define PULSE_FACTOR 12500                    // Nummber of blinks per m3 of your meter (One rotation/liter)
#define PULSE_FACTOR 288000
#define SLEEP_MODE false                        // flowvalue can only be reported when sleep mode is false.
#define MAX_FLOW 25                             // Max flow (l/min) value to report. This filetrs outliers.
#define INTERRUPT DIGITAL_INPUT_SENSOR-2        // Usually the interrupt = pin -2 (on uno/nano anyway)
unsigned long SEND_FREQUENCY = 10000;           // Minimum time between send (in miliseconds). We don't want to spam the gateway.

double ppl = ((double)PULSE_FACTOR)/1000;       // Pulses per liter

volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
bool pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double oldflow = 0;
double volume;
double oldvolume;
unsigned long lastSend;
unsigned long lastPulse;
unsigned long currentTime;
// End water flow information

// Here we are setting up some water thresholds that we will
// use later. Note that you will need to change these to match
// your soil type and environment. It doesn't do much for me because I'm using domoticz
int thresholdUp = 400;
int thresholdDown = 075;

 // Message from the Sensors
MyMessage msgHum1(CHILD_ID_HUM1, V_HUM);
MyMessage msgTemp1(CHILD_ID_TEMP1, V_TEMP);
MyMessage msgHum2(CHILD_ID_HUM2, V_HUM);
MyMessage msgTemp2(CHILD_ID_TEMP2, V_TEMP);
MyMessage msgSoil1(CHILD_ID_SOIL1, V_LEVEL);
MyMessage msgSoil2(CHILD_ID_SOIL2, V_LEVEL);
DHT dht1;
DHT dht2;
MyMessage flowMsg(CHILD_ID_FLOW, V_FLOW);
MyMessage volumeMsg(CHILD_ID_FLOW, V_VOLUME);
MyMessage pcMsg(CHILD_ID_FLOW, V_VAR1);

void before()
        //
        //  Start of the Relay before all other sections
        //
{
    for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
        // Then set relay pins in output mode
        pinMode(pin, OUTPUT);
        // Set relay to last known state (using eeprom storage)
        digitalWrite(pin, loadState(sensor)?RELAY_ON:RELAY_OFF);
    }
}
        //
        // End of the Relay before all other sections
        //

void setup()
{
  //
  // Start of the DHT setup
  //
  dht1.setup(DHT1_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht1.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht1.getMinimumSamplingPeriod());
  //
  // End of the DHT Setup
  //

  //
  // Start of the DHT setup
  //
  dht2.setup(DHT2_DATA_PIN); // set data pin of DHT sensor
  if (UPDATE_INTERVAL <= dht2.getMinimumSamplingPeriod()) {
    Serial.println("Warning: UPDATE_INTERVAL is smaller than supported by the sensor!");
  }
  // Sleep for the time of the minimum sampling period to give the sensor time to power up
  // (otherwise, timeout errors might occure for the first reading)
  sleep(dht2.getMinimumSamplingPeriod());
  //
  // End of the DHT Setup
  //

  //
  // Begin Water Flow Setup
  //
  //begin(incomingMessage, AUTO, true);
  //begin(incomingMessage, AUTO, false, AUTO, RF24_PA_LOW);
  //begin(incomingMessage, AUTO, false, AUTO);
  //  send(pcMsg.set(0));
  //  send(volumeMsg.set(0.000, 3));
  //Water meter setup
  //Serial.print("PPL:");
  //Serial.print(ppl);

  // Fetch last known pulse count value from gw
  request(CHILD_ID_FLOW, V_VAR1);
  //pulseCount = oldPulseCount = 0;

  //Serial.print("Last pulse count from gw:");
  //Serial.println(pulseCount);
  attachInterrupt(INTERRUPT, onPulse, RISING);
  lastSend = millis();
  //
  // End Water Flow Setup
  //
}


void presentation()
 {
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Garden Controller", "1.0");

    // Relay Presentation/Register all sensors to Gateway
    for (int sensor=1, pin=RELAY_1; sensor<=NUMBER_OF_RELAYS; sensor++, pin++) {
        // Register all sensors to gw (they will be created as child devices)
        present(sensor, S_BINARY);
    }
    // DHT Presentation/Register all sensors to Gateway (they will be created as child devices)
    present(CHILD_ID_HUM1, S_HUM);
    present(CHILD_ID_TEMP1, S_TEMP);
    present(CHILD_ID_HUM2, S_HUM);
    present(CHILD_ID_TEMP2, S_TEMP);

    metric = getControllerConfig().isMetric;

   // Soil moisture sensor Presentation/Register this device as Moisture sensor
    present(CHILD_ID_SOIL1, S_MOISTURE);
    present(CHILD_ID_SOIL2, S_MOISTURE);

   // Register this device as Waterflow sensor
    present(CHILD_ID_FLOW, S_WATER);
  }

void loop()
 {
  //
  // Start the DHT Sensor loop
  //
  // Force reading sensor, so it works also after sleep()
  dht1.readSensor(true);

  // Get temperature from DHT library
  float temperature1 = dht1.getTemperature();
  if (isnan(temperature1)) {
    Serial.println("Failed to read the temperature from DHT");
  } else if (temperature1 != lastTemp1 || nNoUpdatesTemp1 == FORCE_UPDATE_N_READS) {
    // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
    lastTemp1 = temperature1;
    if (!metric) {
      temperature1 = dht1.toFahrenheit(temperature1);
    }
    // Reset no updates counter
    nNoUpdatesTemp1 = 0;
    temperature1 += SENSOR_TEMP_OFFSET;
    send(msgTemp1.set(temperature1, 1));

    #ifdef MY_DEBUG
    Serial.print("Temperature: ");
    Serial.println(temperature1);
    #endif
  } else {
    // Increase no update counter if the temperature stayed the same
    nNoUpdatesTemp1++;
  }

  // Get humidity from DHT library
  float humidity1 = dht1.getHumidity();
  if (isnan(humidity1)) {
    Serial.println("Failed to read the humidity from DHT");
  } else if (humidity1 != lastHum1 || nNoUpdatesHum1 == FORCE_UPDATE_N_READS) {
    // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
    lastHum1 = humidity1;
    // Reset no updates counter
    nNoUpdatesHum1 = 0;
    send(msgHum1.set(humidity1, 1));

    #ifdef MY_DEBUG
    Serial.print("Humidity: ");
    Serial.println(humidity1);
    #endif
  } else {
    // Increase no update counter if the humidity stayed the same
    nNoUpdatesHum1++;
  }

 //
 // End the DHT Sensor Loop
 //

 //
 // Start the DHT Sensor loop
 //
 // Force reading sensor, so it works also after sleep()
 dht2.readSensor(true);

 // Get temperature from DHT library
 float temperature2 = dht2.getTemperature();
 if (isnan(temperature2)) {
   Serial.println("Failed to read the temperature from DHT");
 } else if (temperature2 != lastTemp2 || nNoUpdatesTemp2 == FORCE_UPDATE_N_READS) {
   // Only send temperature if it changed since the last measurement or if we didn't send an update for n times
   lastTemp2 = temperature2;
   if (!metric) {
     temperature2 = dht2.toFahrenheit(temperature2);
   }
   // Reset no updates counter
   nNoUpdatesTemp2 = 0;
   temperature2 += SENSOR_TEMP_OFFSET;
   send(msgTemp2.set(temperature2, 2));

   #ifdef MY_DEBUG
   Serial.print("Temperature: ");
   Serial.println(temperature2);
   #endif
 } else {
   // Increase no update counter if the temperature stayed the same
   nNoUpdatesTemp2++;
 }

 // Get humidity from DHT library
 float humidity2 = dht2.getHumidity();
 if (isnan(humidity2)) {
   Serial.println("Failed to read the humidity from DHT");
 } else if (humidity2 != lastHum2 || nNoUpdatesHum2 == FORCE_UPDATE_N_READS) {
   // Only send humidity if it changed since the last measurement or if we didn't send an update for n times
   lastHum2 = humidity2;
   // Reset no updates counter
   nNoUpdatesHum2 = 0;
   send(msgHum2.set(humidity2, 2));

   #ifdef MY_DEBUG
   Serial.print("Humidity: ");
   Serial.println(humidity2);
   #endif
 } else {
   // Increase no update counter if the humidity stayed the same
   nNoUpdatesHum2++;
 }
   //
   // End the DHT Sensor Loop
   //

   //
   // Start Water Flow Loop
   //
   currentTime = millis();
   bool sendTime = currentTime - lastSend > SEND_FREQUENCY;
   if (pcReceived && (SLEEP_MODE || sendTime)) {
     // New flow value has been calculated
     if (!SLEEP_MODE && flow != oldflow) {
       // Check that we dont get unresonable large flow value.
       // could hapen when long wraps or false interrupt triggered
       if (flow<((unsigned long)MAX_FLOW)) {
         send(flowMsg.set(flow, 2));        // Send flow value to gw
       }

       //Serial.print("l/min: ");
       //Serial.println(flow);
       oldflow = flow;
     }

     // No Pulse count in 2min
     if(currentTime - lastPulse > 20000){
       flow = 0;
     }


     // Pulse count has changed
     if (pulseCount != oldPulseCount) {
       send(pcMsg.set(pulseCount));                  // Send  volumevalue to gw VAR1
       double volume = ((double)pulseCount/((double)PULSE_FACTOR)*264.172);
       //double volume = ((double)pulseCount/((double)PULSE_FACTOR));
       oldPulseCount = pulseCount;
       Serial.print("Pulse count:");
       Serial.println(pulseCount);
       if (volume != oldvolume) {
         send(volumeMsg.set(volume, 3));               // Send volume value to gw
         //Serial.print("m3: ");
         //Serial.println(volume);
         oldvolume = volume;
       }
     }
     lastSend = currentTime;
   }
   else if (sendTime) {
     // No count received. Try requesting it again
     request(CHILD_ID_FLOW, V_VAR1);
     lastSend=currentTime;
   }
   //
   // End Water Flow Lop
   //

  // Sleep for a while to save energy
  sleep(UPDATE_INTERVAL);

  //
  // Start Soil Mositure Loop
  //
    int sensorValue1;
sensorValue1 = analogRead(mositure_sensor_pin1);
    int sensorValue2;
sensorValue2 = analogRead(mositure_sensor_pin2);

  //send back the values
    send(msgSoil1.set(sensorValue1));
    send(msgSoil2.set(sensorValue2));

  //
  // End Soil mositure loop
  //
}

void receive(const MyMessage &message)
 {
    // We only expect one type of message from controller. But we better check anyway.
    if (message.type==V_STATUS) {
        // Change relay state
        digitalWrite(message.sensor-1+RELAY_1, message.getBool()?RELAY_ON:RELAY_OFF);
        // Store state in eeprom
        saveState(message.sensor, message.getBool());
        // Write some debug info
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getBool());
    }
 }

 void onPulse()
 {
   if (!SLEEP_MODE) {
     unsigned long newBlink = micros();
     unsigned long interval = newBlink-lastBlink;
     lastPulse = millis();
     if (interval < 2080) {       // Sometimes we get interrupt on RISING,  500000 = 0.5sek debounce ( max 120 l/min)  WAS 2080
       return;
     }

     flow = ((60000000.0 /interval) / ppl)*.264172;
     //flow = ((60000000.0 /interval) / ppl);
     // Serial.print("interval:");
     // Serial.println(interval);
     lastBlink = newBlink;
      // Serial.println(flow, 4);
   }
   pulseCount++;

 }

 void incomingMessage(const MyMessage &message) {
   if (message.type==V_VAR1) {
     pulseCount = oldPulseCount = message.getLong();
     Serial.print("Received last pulse count from gw:");
     Serial.println(pulseCount);
     pcReceived = true;
   }

 }
