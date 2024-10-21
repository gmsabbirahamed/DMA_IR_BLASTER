#include <WiFiManager.h> // For managing Wi-Fi credentials
#include <PubSubClient.h> // For MQTT
#include <IRremoteESP8266.h>
#include <WiFi.h>
#include <Wire.h>
#include <FastLED.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <ir_Airton.h>
#include <ir_Airwell.h>
#include <ir_Amcor.h>
#include <ir_Argo.h>
#include <ir_Bosch.h>
#include <ir_Carrier.h>
#include <ir_Coolix.h>
#include <ir_Corona.h>
#include <ir_Daikin.h>
#include <ir_Daikin.h>
#include <ir_Ecoclim.h>
#include <ir_Goodweather.h>
#include <ir_Gree.h>
#include <ir_Kelon.h>
#include <ir_Kelvinator.h>
#include <ir_Lg.h>
#include <ir_Magiquest.h>
#include <ir_Midea.h>
#include <ir_Mirage.h>
#include <ir_Mitsubishi.h>
#include <ir_Nec.h>
#include <ir_Neoclima.h>
#include <ir_Panasonic.h>
#include <ir_Rhoss.h>
#include <ir_Samsung.h>
#include <ir_Sanyo.h>
#include <ir_Sharp.h>
#include <ir_Tcl.h>
#include <ir_Teco.h>
#include <ir_Transcold.h>
#include <ir_Trotec.h>
#include <ir_Truma.h>
#include <ir_Voltas.h>
#include <ir_York.h>


// Define the number of LEDs and the data pin
#define NUM_LEDS 1
#define DATA_PIN 4

//#define SHT3X_I2C_ADDR 0x44  // SHT3x I2C address
//#define TEMP_HUM_CMD 0x2400  // High repeatability measurement command
//float temperature = 0.0, humidity = 0.0;

//#define id 1209002409120000

// Create an array of LEDs
CRGB leds[NUM_LEDS];

// IR setup &&&& PIN
//const uint16_t kRecvPin = 14;
const uint16_t kIrLedPin = 27; // GPIO for IR LED
const int LED_PIN = 2; // LED status indicator
const int RESET_BUTTON_PIN = 35; // Wi-Fi reset button
int count = 0;//     loop count

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
const uint8_t kTolerancePercentage = 25; // Default tolerance of 25%

IRsend irsend(kIrLedPin);
//IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;

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

// MQTT settings
const char* mqtt_broker = "broker2.dma-bd.com";
const int mqtt_port = 1883;
const char* mqtt_user = "broker2";
const char* mqtt_pass = "Secret!@#$1234";

// MQTT topics
//const char* temp_topic =      "DMA/ac/id/sensor_1/temp";    //publish
const char* hum_topic =       "DMA/ac/idd/sensor_1/hum";     //publish
//const char* temp_topic =      "DMA/ac/idd/sensor_1";    //publish
const char* remote_topic =    "DMA/ac/idd/remote";           //publish
const char* control_topic =   "DMA/ac/idd/control";          //subscribe
const char* ac_stat_topic =   "DMA/ac/idd/status";          //publish

const char* ac_remot_topic =   "DMA/ac/idd/remote";          //publish

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
/*
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

}*/




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
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(500);
      Serial.print(".");
      leds[0] = CRGB::Black;
      FastLED.show();

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
      client.subscribe(control_topic);
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
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message received: ");
  Serial.println(message);

  // Split the message by spaces
  int firstSpace = message.indexOf(' ');
  int secondSpace = message.indexOf(' ', firstSpace + 1);
  int thirdSpace = message.indexOf(' ', secondSpace + 1);
  int fourthSpace = message.indexOf(' ', thirdSpace + 1);

  // Parse power command (on/off)
  String powerCmd = message.substring(0, firstSpace);
  acState.powerStatus = (powerCmd == "on") ? true : false;

  // Parse temperature
  String tempCmd = message.substring(firstSpace + 1, secondSpace);
  acState.temperature = tempCmd.toInt();

  // Parse mode (cool/heat/fan/auto)
  String modeCmd = message.substring(secondSpace + 1, thirdSpace);
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
  String fanCmd = message.substring(thirdSpace + 1);
  if (fanCmd == "auto") {
    acState.fan = 0;
  } else if (fanCmd == "min") {
    acState.fan = 1;
  } else if (fanCmd == "med") {
    acState.fan = 2;
  } else if (fanCmd == "max") {
    acState.fan = 3;
  }

    // Parse fan speed (auto/min/med/max)
  String protocolCmd = message.substring(fourthSpace + 1);
  if (protocolCmd == "coolix") {
    acState.choose_protocol = 0;
  } else if (protocolCmd == "goodweather") {
    acState.choose_protocol = 1;
  } else if (protocolCmd == "mitsubishi") {
    acState.choose_protocol = 2;
  } else if (protocolCmd == "lg") {
    acState.choose_protocol = 3;
  }


  else if (protocolCmd == "tcl112") {
    acState.choose_protocol = 4;
  }

  controlAC();  // Control the AC based on the parsed commands
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


/*
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
  if (acState.choose_protocol == 1) {
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
/*
  else if (acState.choose_protocol == 2) {
    if (acState.powerStatus) {
      mitsubishiAC.on();
      mitsubishiAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: mitsubishiAC.setMode(kMitsubishiHeavyAuto); break;
        case 1: mitsubishiAC.setMode(kMitsubishiHeavyCool); break;
        case 3: mitsubishiAC.setMode(kMitsubishiHeavyHeat); break;
        case 4: mitsubishiAC.setMode(kMitsubishiHeavyFan); break;
      }
      switch (acState.fan) {
        case 0: mitsubishiAC.setFan(kMitsubishiHeavyFanAuto); break;
        case 1: mitsubishiAC.setFan(kMitsubishiHeavyFanLow); break;
        case 2: mitsubishiAC.setFan(kMitsubishiHeavyFanMed); break;
        case 3: mitsubishiAC.setFan(kMitsubishiHeavyFanHigh); break;
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
        case 0: lgAC.setMode(kLGAuto); break;
        case 1: lgAC.setMode(kLGCool); break;
        case 3: lgAC.setMode(kLGHeat); break;
        case 4: lgAC.setMode(kLGFan); break;
      }
      switch (acState.fan) {
        case 0: lgAC.setFan(kLGFanAuto); break;
        case 1: lgAC.setFan(kLGFanLow); break;
        case 2: lgAC.setFan(kLGFanMed); break;
        case 3: lgAC.setFan(kLGFanHigh); break;
      }
    } else {
      lgAC.off();
    }
    lgAC.send();
    publishState();
  }
  // 4. Fujitsu Protocol
  else if (acState.choose_protocol == 4) {
    if (acState.powerStatus) {
      fujitsuAC.on();
      fujitsuAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: fujitsuAC.setMode(kFujitsuAuto); break;
        case 1: fujitsuAC.setMode(kFujitsuCool); break;
        case 3: fujitsuAC.setMode(kFujitsuHeat); break;
        case 4: fujitsuAC.setMode(kFujitsuFan); break;
      }
      switch (acState.fan) {
        case 0: fujitsuAC.setFan(kFujitsuFanAuto); break;
        case 1: fujitsuAC.setFan(kFujitsuFanLow); break;
        case 2: fujitsuAC.setFan(kFujitsuFanMed); break;
        case 3: fujitsuAC.setFan(kFujitsuFanHigh); break;
      }
    } else {
      fujitsuAC.off();
    }
    fujitsuAC.send();
    publishState();
  }
  // 5. Samsung Protocol
  else if (acState.choose_protocol == 5) {
    if (acState.powerStatus) {
      samsungAC.on();
      samsungAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: samsungAC.setMode(kSamsungAuto); break;
        case 1: samsungAC.setMode(kSamsungCool); break;
        case 3: samsungAC.setMode(kSamsungHeat); break;
        case 4: samsungAC.setMode(kSamsungFan); break;
      }
      switch (acState.fan) {
        case 0: samsungAC.setFan(kSamsungFanAuto); break;
        case 1: samsungAC.setFan(kSamsungFanLow); break;
        case 2: samsungAC.setFan(kSamsungFanMed); break;
        case 3: samsungAC.setFan(kSamsungFanHigh); break;
      }
    } else {
      samsungAC.off();
    }
    samsungAC.send();
    publishState();
  }
  // 6. Panasonic Protocol
  else if (acState.choose_protocol == 6) {
    if (acState.powerStatus) {
      panasonicAC.on();
      panasonicAC.setTemp(acState.temperature);
      switch (acState.operation) {
        case 0: panasonicAC.setMode(kPanasonicAuto); break;
        case 1: panasonicAC.setMode(kPanasonicCool); break;
        case 3: panasonicAC.setMode(kPanasonicHeat); break;
        case 4: panasonicAC.setMode(kPanasonicFan); break;
      }
      switch (acState.fan) {
        case 0: panasonicAC.setFan(kPanasonicFanAuto); break;
        case 1: panasonicAC.setFan(kPanasonicFanLow); break;
        case 2: panasonicAC.setFan(kPanasonicFanMed); break;
        case 3: panasonicAC.setFan(kPanasonicFanHigh); break;
      }
    } else {
      panasonicAC.off();
    }
    panasonicAC.send();
    publishState();
  }*/

  // Continue for the remaining protocols following the same structure:

  // 7. Toshiba Protocol
  // 8. Sharp Protocol
  // 9. Hitachi Protocol
  // 10. Haier Protocol
  // 11. Midea Protocol
  // 12. Gree Protocol
  // 13. Whirlpool Protocol
  // 14. Electra Protocol
  // 15. Sanyo Protocol
  // 16. Trane Protocol
  // 17. Airwell Protocol
  // 18. Daewoo Protocol
  // 19. Delonghi Protocol
  // 20. Hisense Protocol
  // 21. Carrier Protocol
  // 22. TCL Protocol
  // 23. Siemens Protocol
  // 24. Voltas Protocol
  // 25. NEC Protocol
  // 26. JVC Protocol
  // 27. Mirage Protocol
  // 28. AUX Protocol
  // 29. Onida Protocol
  // 30. Kenwood Protocol
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
  client.publish(ac_stat_topic, message.c_str());
  leds[0] = CRGB::Blue;// blue blink
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(120);
  leds[0] = CRGB::Black;
  FastLED.show();
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
  leds[0] = CRGB::Red;
  FastLED.show();
  }
  if (!client.connected()) {
    reconnectMQTT();
    leds[0] = CRGB::Red;
    FastLED.show();
  }
  if (WiFi.status() == WL_CONNECTED && client.connected()) {leds[0] = CRGB::Black;FastLED.show();}


  // Read temperature and humidity using the function
  /*
  if (count == 50) {
  if (ReadTempSensor(temperature, humidity)) {
    const char* sensor_1 = String("Temperature: " + String(temperature) + ", Humidity: " + String(humidity)).c_str();

    Serial.print(sensor_1);

    if (client.connected()) {
      client.publish(temp_topic, sensor_1);
        leds[0] = CRGB::Blue;
        FastLED.show();
        delay(120);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(120);
        leds[0] = CRGB::Blue;
        FastLED.show();
        delay(120);
        leds[0] = CRGB::Black;
        FastLED.show();
    }
  } else {
    Serial.println("Failed to read from sensor.");
  }count = 0;
  }*/

/*
  if (irrecv.decode(&results)) {
    // Print the decoded data in human-readable form
    const char* mycode;
    mycode = (resultToHumanReadableBasic(&results)).c_str();
    Serial.println(mycode);
    leds[0] = CRGB::Green;// green blink
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Black;
    FastLED.show();
    client.publish(ac_remot_topic, mycode);
    leds[0] = CRGB::Blue;// green blink
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Blue;
    FastLED.show();
    delay(120);
    leds[0] = CRGB::Black;
    FastLED.show();
    // Resume the IR receiver to capture the next signal
    irrecv.resume();
  }
*/
  client.loop(); // Handle incoming MQTT messages
  delay(50); // Publish data every .1 seconds
  count = count + 1;
}

/**********----  -----   -------   ----- end  main    ------   -------   -------------***** */
/************************************************************************************* */
