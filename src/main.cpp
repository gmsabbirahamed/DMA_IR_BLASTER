#include <WiFiManager.h> // For managing Wi-Fi credentials
#include <PubSubClient.h> // For MQTT
#include <IRremoteESP8266.h>
#include <WiFi.h>
#include <Wire.h>
#include <FastLED.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>

#include <assert.h>
#include <IRac.h>
#include <IRtext.h>

#include "Config.h"


// Define the number of LEDs and the data pin
#define NUM_LEDS 1
#define DATA_PIN 4

#define SHT3X_I2C_ADDR 0x44  // SHT3x I2C address
#define TEMP_HUM_CMD 0x2400  // High repeatability measurement command
float temperature = 0.0, humidity = 0.0;

// Create an array of LEDs
CRGB leds[NUM_LEDS];

// IR setup &&&& PIN
//const uint16_t kRecvPin = 36;
const uint16_t kIrLedPin = 27; // GPIO for IR LED
const int LED_PIN = 2; // LED status indicator
const int RESET_BUTTON_PIN = 35; // Wi-Fi reset button
int count = 0;//     loop count

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
const uint8_t kTolerancePercentage = 25; // Default tolerance of 25%

IRsend irsend(kIrLedPin);
//IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
//decode_results results;

// Create objects for each protocol
IRCoolixAC coolixAC(kIrLedPin);
IRGoodweatherAc goodweatherAC(kIrLedPin); // Goodweather AC object
IRMitsubishiAC mitsubishiAC(kIrLedPin);
IRLgAc lgAC(kIrLedPin);
IRTcl112Ac tcl112ACS(kIrLedPin);
// Add additional objects as needed

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// WiFiManager Object
WiFiManager wifiManager;

// Timing Variables
unsigned long buttonPressedTime = 0;
unsigned long longPressTime = 2000;  // 2 seconds
bool isButtonPressed = false;

unsigned long lastMsg = 0;

bool isPwCmdValid = false;
bool isTempCmdValid = false;
bool isModeCmdValid = false;
bool isFanCmdValid = false;
bool isProtocolCmdValid = false;

// MQTT settings   &&&   Topic
const char* mqtt_broker = "broker2.dma-bd.com";
const int mqtt_port = 1883;
const char* mqtt_user = "broker2";
const char* mqtt_pass = "Secret!@#$1234";

#define WORK_PACKAGE                                "1209"
#define DEVICE_TYPE                                 "00"
#define DEVICE_CODE_UPLOAD_DATE                     "240912"
#define DEVICE_SERIAL_ID                            "0000"

#define DEVICE_ID                                   WORK_PACKAGE DEVICE_TYPE DEVICE_CODE_UPLOAD_DATE DEVICE_SERIAL_ID
//#define SUB_TOPIC_B                               ("A/C/" DEVICE_ID "/D")

// MQTT topics
#define AIRFLOW_SENSOR_TOPIC                        ("DMA/AC/" DEVICE_ID "/AIRFLOW")         //PUBLISH
#define AMBIENT_SENSOR_TOPIC                        ("DMA/AC/" DEVICE_ID "/AMBIENT")         //PUBLISH
#define STATUS_TOPIC                                ("DMA/AC/" DEVICE_ID "/STATUS")          //PUBLISH
#define REMOTE_TOPIC                                ("DMA/AC/" DEVICE_ID "/REMOTE")          //PUBLISH
#define CONTROL_TOPIC                               ("DMA/AC/" DEVICE_ID "/CONTROL")         //SUBSCR

const char* ac_protocol;

struct state {
  uint8_t temperature = 24;
  uint8_t fan = 0;
  uint8_t operation = 0;
  uint8_t choose_protocol = 0;
  bool powerStatus;
};

state acState;

// All Functions.......................
void blinkLED();
void connectToWiFi();
void startWiFiConfigMode();
void reconnectMQTT();
void cllback(char* topic, byte* payload, unsigned int length);
void publishState();
void controlAC();

void handleButtonPress();

// Function to calculate the CRC for the SHT3x data (polynomial 0x31)
uint8_t calculateCRC(uint8_t data[], uint8_t length) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

// Function to read temperature and humidity from SHT3x sensor
bool ReadTempSensor(float &temperature, float &humidity) {
  Wire.beginTransmission(SHT3X_I2C_ADDR);
  Wire.write(TEMP_HUM_CMD >> 8);  // Send MSB
  Wire.write(TEMP_HUM_CMD & 0xFF);  // Send LSB
  Wire.endTransmission();

  delay(15);  // Wait for the measurement (typical 15ms)

  // Read 6 bytes (temperature MSB, temperature LSB, CRC, humidity MSB, humidity LSB, CRC)
  Wire.requestFrom(SHT3X_I2C_ADDR, 6);

  if (Wire.available() == 6) {
    uint8_t tempData[2], humidityData[2], tempCRC, humidityCRC;

    tempData[0] = Wire.read();  // Temperature MSB
    tempData[1] = Wire.read();  // Temperature LSB
    tempCRC = Wire.read();      // CRC for temperature

    humidityData[0] = Wire.read();  // Humidity MSB
    humidityData[1] = Wire.read();  // Humidity LSB
    humidityCRC = Wire.read();      // CRC for humidity

    // Verify CRC for temperature
    if (calculateCRC(tempData, 2) != tempCRC) {
      Serial.println("Temperature CRC check failed!");
      return false;
    }

    // Verify CRC for humidity
    if (calculateCRC(humidityData, 2) != humidityCRC) {
      Serial.println("Humidity CRC check failed!");
      return false;
    }

    // Convert raw temperature to Celsius
    uint16_t tempRaw = (tempData[0] << 8) | tempData[1];
    temperature = -45.0 + 175.0 * ((float)tempRaw / 65535.0);

    // Convert raw humidity to percentage
    uint16_t humidityRaw = (humidityData[0] << 8) | humidityData[1];
    humidity = 100.0 * ((float)humidityRaw / 65535.0);

    return true;
  }
  return false;  // Return false if no data available

}

// Handle button press for Wi-Fi reset
void handleButtonPress() {
    if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (!isButtonPressed) {
      buttonPressedTime = millis();
      isButtonPressed = true;
    }

    // If button is held for more than 5 seconds, start WiFiManager AP
    if (isButtonPressed && (millis() - buttonPressedTime >= longPressTime)) {
      Serial.println("Long press detected. Entering WiFi configuration mode...");
      startWiFiConfigMode();
      isButtonPressed = false;  // Reset button press state
    }
  } else {
    isButtonPressed = false;
  }
}

/********************************************************************************* */
    //     ******    WiFi  --- MQTT  ----   publish  ----   controlAC    ******
/********************************************************************************* */
// Connect to Wi-Fi using saved credentials
void connectToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to Wi-Fi...");
    WiFi.begin();
    int counter = 0;
    while (WiFi.status() != WL_CONNECTED && counter < 30) {  // Wait for max 15 seconds
      delay(500);
      Serial.print(".");
      counter++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to connect to Wi-Fi.");
    }
  }
}
// Function to start Wi-Fi configuration mode (AP mode)
void startWiFiConfigMode() {
  digitalWrite(LED_PIN, LOW);  // Turn off LED during configuration mode
  wifiManager.resetSettings(); // Reset saved credentials (optional)
  
  // Enter AP mode with default settings or custom parameters
  if (!wifiManager.startConfigPortal("DMA IR Blaster")) {           ///////////////////////-----AP_Name----//////////////////////////
    Serial.println("Failed to enter configuration mode.");
  } else {
    Serial.println("Configuration mode started.");
  }

  // After config, reconnect to the Wi-Fi network
  connectToWiFi();
}


// Reconnect to the MQTT broker
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
    else if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Connected to MQTT broker");
      client.subscribe(CONTROL_TOPIC);
    } else {
      Serial.print("  Failed, rc=");
      Serial.println(client.state());
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

// MQTT message callback handler
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  
  // Convert payload to a string
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Split the message by spaces and store each part
  String parts[5];  // Assuming 5 parts max
  int partIndex = 0;
  int lastIndex = 0;

  // Loop to split the message by spaces
  for (int i = 0; i < message.length() && partIndex < 5; i++) {
    if (message[i] == ' ') {
      parts[partIndex] = message.substring(lastIndex, i);
      partIndex++;
      lastIndex = i + 1;
    }
  }
  // Store the last part (if any remaining)
  if (lastIndex < message.length()) {
    parts[partIndex] = message.substring(lastIndex);
  }

  // Ensure all parts are valid before processing
  if (partIndex >= 4) {  // Check if there are at least 5 parts
    String powerCmd = parts[0];
    String tempCmd = parts[1];
    String modeCmd = parts[2];
    String fanCmd = parts[3];
    String protocolCmd = parts[4];


    // Example: print each part
    for (int i = 0; i <= partIndex; i++) {
      Serial.print("Part ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(parts[i]);
    }

    if(powerCmd == "on" || powerCmd == "off") {
      isPwCmdValid = true;
      Serial.println(powerCmd);
    } else {
      isPwCmdValid = false;
      Serial.println("Invalid Power Command: on/off");
    }

    if(tempCmd == "16" || tempCmd == "17" || tempCmd == "18" || tempCmd == "19" || tempCmd == "20" || tempCmd == "21" || tempCmd == "22" || tempCmd == "23" || tempCmd == "24" || tempCmd == "25" || tempCmd == "26" || tempCmd == "27" || tempCmd == "28" || tempCmd == "29" || tempCmd == "30" || tempCmd == "31") {
      isTempCmdValid = true;
      Serial.println(tempCmd);
    } else {
      isTempCmdValid = false;
      Serial.println("Invalid Temperature: 16 to 26");
    }

    if(modeCmd == "auto" || modeCmd == "cool" || modeCmd == "heat" || modeCmd == "fan") {
      isModeCmdValid = true;
      Serial.println(modeCmd);
    } else {
      isModeCmdValid = false;
      Serial.println("Invalid Mode: auto/cool/heat/fan");
    }
    if(fanCmd == "auto" || fanCmd == "min" || fanCmd == "med" || fanCmd == "max") {
      isFanCmdValid = true;
      Serial.println(fanCmd);
    } else {
      isFanCmdValid = false;
      Serial.println("Invalid Fan: auto/min/med/max");
    }
    if(protocolCmd == "coolix" || protocolCmd == "goodweather" || protocolCmd == "mitsubishi" || protocolCmd == "lg" || protocolCmd == "tcl112") {
      isProtocolCmdValid = true;
      Serial.println(protocolCmd);
    } else {
      isProtocolCmdValid = false;
      Serial.println("Invalid Protocol: coolix/goodweather/tcl112...");
    }

    /////////////////////////
    if(isPwCmdValid && isTempCmdValid && isModeCmdValid && isFanCmdValid && isProtocolCmdValid) {
      // Use the parts[] array for further logic, e.g., controlling AC
      // Parse power command (on/off)
      acState.powerStatus = (powerCmd == "on") ? true : false;
      acState.temperature = tempCmd.toInt();

      // Parse mode (cool/heat/fan/auto)
      if (modeCmd == "cool") {
        acState.operation = 1;
      } else if (modeCmd == "heat") {
        acState.operation = 3;
      } else if (modeCmd == "fan") {
        acState.operation = 4;
      } else if (modeCmd == "auto") {
        acState.operation = 0;
      }

      // Parse fan speed (auto/min/med/max)
      if (fanCmd == "auto") {
        acState.fan = 0;
      } else if (fanCmd == "min") {
        acState.fan = 1;
      } else if (fanCmd == "med") {
        acState.fan = 2;
      } else if (fanCmd == "max") {
        acState.fan = 3;
      }

      // Parse protocol command (coolix/goodweather/mitsubishi/lg/tcl112)
      if (protocolCmd == "coolix") {
        acState.choose_protocol = 0;
      } else if (protocolCmd == "goodweather") {
        acState.choose_protocol = 1;
      } else if (protocolCmd == "mitsubishi") {
        acState.choose_protocol = 2;
      } else if (protocolCmd == "lg") {
        acState.choose_protocol = 3;
      } else if (protocolCmd == "tcl112") {
        acState.choose_protocol = 4;
      }
      controlAC();  // Control the AC based on the parsed commands
    } else {
      Serial.println("Command Error:");
    }
    
  } else {
    Serial.println("Error: Invalid message format or insufficient parts.");
  }
}


/////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////


void controlAC() {

  // 0. coolix Protocol
  if (acState.choose_protocol == 0) {
    if (acState.powerStatus) {
      coolixAC.on();
      
      coolixAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: coolixAC.setMode(kCoolixAuto); break;
        case 1: coolixAC.setMode(kCoolixCool); break;
        case 3: coolixAC.setMode(kCoolixHeat); break;
        case 4: coolixAC.setMode(kCoolixFan); break;
      }
      switch (acState.fan) {
        case 0: coolixAC.setFan(kCoolixFanAuto); break;
        case 1: coolixAC.setFan(kCoolixFanMin); break;
        case 2: coolixAC.setFan(kCoolixFanMed); break;
        case 3: coolixAC.setFan(kCoolixFanMax); break;
      }
    } else {
      coolixAC.off();
    }
    coolixAC.send();
    publishState();
  }
  // 1. goodweather Protocol
  else if (acState.choose_protocol == 1) {
    if (acState.powerStatus) {
      goodweatherAC.on();
      
      goodweatherAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: goodweatherAC.setMode(kGoodweatherAuto); break;
        case 1: goodweatherAC.setMode(kGoodweatherCool); break;
        case 3: goodweatherAC.setMode(kGoodweatherHeat); break;
        case 4: goodweatherAC.setMode(kGoodweatherFan); break;
      }
      switch (acState.fan) {
        case 0: goodweatherAC.setFan(kGoodweatherFanAuto); break;
        case 1: goodweatherAC.setFan(kGoodweatherFanLow); break;
        case 2: goodweatherAC.setFan(kGoodweatherFanMed); break;
        case 3: goodweatherAC.setFan(kGoodweatherFanHigh); break;
      }
    } else {
      goodweatherAC.off();
    }
    goodweatherAC.send();
    publishState();
  }

  // 2. Mitsubishi Protocol
  else if (acState.choose_protocol == 2) {
    if (acState.powerStatus) {
      mitsubishiAC.on();
      mitsubishiAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: mitsubishiAC.setMode(kMitsubishiAcAuto); break;
        case 1: mitsubishiAC.setMode(kMitsubishiAcCool); break;
        case 3: mitsubishiAC.setMode(kMitsubishiAcHeat); break;
        case 4: mitsubishiAC.setMode(kMitsubishiAcFan); break;
      }
      switch (acState.fan) {
        case 0: mitsubishiAC.setFan(kMitsubishiAcFanAuto); break;
        case 1: mitsubishiAC.setFan(kMitsubishiAcFanQuiet); break;
        case 2: mitsubishiAC.setFan(kMitsubishiAcFanAuto); break;
        case 3: mitsubishiAC.setFan(kMitsubishiAcFanMax); break;
      }
    } else {
      mitsubishiAC.off();
    }
    mitsubishiAC.send();
    publishState();
  }
  // 3. LG Protocol
  else if (acState.choose_protocol == 3) {
    if (acState.powerStatus) {
      lgAC.on();
      lgAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: lgAC.setMode(kLgAcAuto); break;
        case 1: lgAC.setMode(kLgAcCool); break;
        case 3: lgAC.setMode(kLgAcHeat); break;
        case 4: lgAC.setMode(kLgAcFan); break;
      }
      switch (acState.fan) {
        case 0: lgAC.setFan(kLgAcFanAuto); break;
        case 1: lgAC.setFan(kLgAcFanLow); break;
        case 2: lgAC.setFan(kLgAcFanMedium); break;
        case 3: lgAC.setFan(kLgAcFanMax); break;
      }
    } else {
      lgAC.off();
    }
    lgAC.send();
    publishState();
  }

  // 4. tcl Protocol
  else if (acState.choose_protocol == 4) {
    if (acState.powerStatus) {
      tcl112ACS.on();
      tcl112ACS.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: tcl112ACS.setMode(kTcl112AcAuto); break;
        case 1: tcl112ACS.setMode(kTcl112AcCool); break;
        case 3: tcl112ACS.setMode(kTcl112AcHeat); break;
        case 4: tcl112ACS.setMode(kTcl112AcFan); break;
      }
      switch (acState.fan) {
        case 0: tcl112ACS.setFan(kTcl112AcFanAuto); break;
        case 1: tcl112ACS.setFan(kTcl112AcFanLow); break;
        case 2: tcl112ACS.setFan(kTcl112AcFanMed); break;
        case 3: tcl112ACS.setFan(kTcl112AcFanHigh); break;
      }
    } else {
      tcl112ACS.off();
    }
    tcl112ACS.send();
    publishState();
  }


/////////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************/
/////////////////////////////////////////////////////////////////////////////////////

}


void publishState() {
  String powerCmd = acState.powerStatus ? "on" : "off";
  String modeCmd;
  String fanCmd;

  switch (acState.operation) {
    case 0: modeCmd = "auto"; break;
    case 1: modeCmd = "cool"; break;
    case 3: modeCmd = "heat"; break;
    case 4: modeCmd = "fan"; break;
  }
  switch (acState.fan) {
    case 0: fanCmd = "auto"; break;
    case 1: fanCmd = "min"; break;
    case 2: fanCmd = "med"; break;
    case 3: fanCmd = "max"; break;
  }

  String message = powerCmd + " " + String(acState.temperature) + " " + modeCmd + " " + fanCmd;
  client.publish(STATUS_TOPIC, message.c_str());
}

/*************************************  ---  End  --  ************************************ */
    //     ******    WiFi  --- MQTT  ----   publish  ----   controlAC    ******
/********************************************************************************* */


/************************************************************************************* */
/**********----  -----   -------   -----   main    ------   -------   -------------***** */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(50); // Wait for serial connection
  Wire.begin();  // Initialize I2C
  Serial.println("--sabbir--");
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  delay(255);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  //irrecv.setTolerance(kTolerancePercentage);
  //irrecv.enableIRIn();  // Start the IR receiver

//     ------------------------    wifi reset    ---------------------------
  unsigned long startTime = millis();  // Store the starting time
  startTime = millis();  // Initialize startTime
  Serial.println("You can reset Wi-Fi");
  while (millis() - startTime <= 7000) {  // 7 second without delay
    handleButtonPress();  // Correctly call the function
  }
  Serial.println("End Wi-Fi reset");
//     ------------------------ end   wifi reset    ---------------------------

  connectToWiFi();     // Attempt to connect to saved Wi-Fi credentials

  coolixAC.begin();
  goodweatherAC.begin();
  mitsubishiAC.begin();
  tcl112ACS.begin();
  lgAC.begin();
  irsend.begin();


 //connecting to a mqtt broker
 client.setServer(mqtt_broker, mqtt_port);
 client.setCallback(callback);
 reconnectMQTT();

/*
   // Check if the sensor is connected
  Wire.beginTransmission(SHT3X_I2C_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("SHT3x not detected!");
    while (1);  // Halt if not found
  }*/

}
//************************************************************************** */
//*************************************************************************** */
void loop() {
  
  if (WiFi.status() != WL_CONNECTED) {// If Wi-Fi is not connected, attempt to reconnect
    connectToWiFi();
  }
  if (!client.connected()) {
    reconnectMQTT();
  }
  // Read temperature and humidity and send data every 5 seconds using the function
  unsigned long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    if (ReadTempSensor(temperature, humidity)) {
    const char* sensor_1 = String("Temperature: " + String(temperature) + ", Humidity: " + String(humidity)).c_str();
    Serial.print(sensor_1);
    if (client.connected()) {
      client.publish(AIRFLOW_SENSOR_TOPIC, sensor_1);
      Serial.println(sensor_1);
      } else {
      client.publish(AIRFLOW_SENSOR_TOPIC, "error");
      Serial.println("Failed to read from sensor.");
      }
    }
  }
  client.loop(); // Handle incoming MQTT messages
  delay(10); // Publish data every .1 seconds
}

/**********----  -----   -------   ----- end  main    ------   -------   -------------***** */
/************************************************************************************* */
