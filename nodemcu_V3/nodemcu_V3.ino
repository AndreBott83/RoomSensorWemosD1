#include <ArduinoOTA.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>


#define wifi_ssid "BTWLAN"
#define wifi_password "X3nopusLaevis"
#define SERIAL_BAUD 115200

#define room "Studio"
#define location "OG2/"room
#define mqtt_server "192.168.1.8"

#define mqtt_user "nodemcu_"room
#define mqtt_password "your_password"

#define humidity_topic "Home/"location"/Humidity"
#define temperature_topic "Home/"location"/Temperature"
#define pressure_topic "Home/"location"/Pressure"
#define altitude_topic "Home/"location"/Altitude"
#define dewpoint_topic "Home/"location"/Dewpoint"
#define debug_topic "Home/Debug"


WiFiClient espClient;
PubSubClient client(espClient);

/* ==== Global Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
bool serial_output = false; // false or true;
bool debugOutput = false; // false or true;
bool OTAMode = false;
bool uploadMode = false;
long lastMsg = 0;
int avgOverXMeasurements = 10;

float newTemp(NAN), newHum(NAN), newPres(NAN);
float newAltitude = 0.0;
float newDewPoint = 0.0;

const int sleepSeconds = 5 * 60;

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client, int avgOverXMeasurements);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

void setup() {  
  pinMode(D3, INPUT); //flash taster  
  pinMode(D4, OUTPUT); // Blue LED
  digitalWrite(D4, HIGH);
  if (serial_output) {
    Serial.begin(SERIAL_BAUD);  
    while (!Serial) {} // Wait
  }
  client.setServer(mqtt_server, 1883); 
  setup_wifi();


  ArduinoOTA.onStart([]() {
    writeLineToOutput(String(location) + ": Start OTA Update");
  });
  ArduinoOTA.onEnd([]() {
    writeLineToOutput(String(location) + ": OTA Update finished");
    //flash the led to signal OTA END
    digitalWrite(D4, HIGH);
    delay(2000);
    digitalWrite(D4, LOW);
    delay(600);
    digitalWrite(D4, HIGH);
    delay(200);
    digitalWrite(D4, LOW);
    delay(600);
    digitalWrite(D4, HIGH);
    delay(200);
    digitalWrite(D4, LOW);
    delay(1000);
    digitalWrite(D4, HIGH);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    char buffer[130];
    sprintf(buffer, "Error[%u]: ", error);
    writeLineToOutput(String(location) + String(":\n") + String(buffer));    
    if (error == OTA_AUTH_ERROR) writeLineToOutput("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) writeLineToOutput("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) writeLineToOutput("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) writeLineToOutput("Receive Failed");
    else if (error == OTA_END_ERROR) writeLineToOutput("End Failed");
  });
  ArduinoOTA.setHostname(location);
  ArduinoOTA.begin();
    
  while (!bme.begin()) {
      writeLineToOutput(String(location) + String(": Could not find BME280 sensor!"));
//      if (serial_output) {
//        Serial.println("Could not find BME280 sensor!");
//      }
      delay(1000);
    }
}


void loop() {
  if (!uploadMode) {
    if (!client.connected()) {
      reconnect();
    }  
      
    printBME280Data(&Serial, avgOverXMeasurements, serial_output);
    printBME280CalculatedData(&Serial, serial_output);
    
    client.publish(temperature_topic, String(newTemp).c_str(), true);
    client.publish(humidity_topic, String(newHum).c_str(), true);
    client.publish(pressure_topic, String(newPres).c_str(), true);
    client.publish(altitude_topic, String(newAltitude).c_str(), true);
    client.publish(dewpoint_topic, String(newDewPoint).c_str(), true);
    client.loop();
    //  }
    bool flashButtonPressed = !digitalRead(D3);
    writeLineToOutput(String(location) + ": Value of Taster = " + String(flashButtonPressed));
    if (flashButtonPressed) {
      uploadMode = true;
    } else {
      ESP.deepSleep(sleepSeconds * 1000000);
    }
  } else {
    if (!OTAMode) {
      writeLineToOutput("Entered Upload mode in main loop! ");
      OTAMode = true;
      digitalWrite(D4, LOW);
    }    
    ArduinoOTA.handle();
  }
}


/* ==== Functions ==== */
void writeLineToOutput(String line) {
  if (serial_output){
     Serial.println(line);
  } 
  if (debugOutput || OTAMode) {
    if (!client.connected()) {
      reconnect();
    } 
    client.publish(debug_topic, line.c_str(), true);
    client.loop();
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  if (serial_output) {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);
  }
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (serial_output) {
      Serial.print(".");
    }
  }
  writeLineToOutput(String(location) + ": " + String("WiFi connected, IP adress: ") + WiFi.localIP().toString());
//  if (serial_output) {
//    Serial.println("");
//    Serial.println("WiFi connected");
//    Serial.println("IP address: ");
//    Serial.println(WiFi.localIP());
//  }
}

void reconnect() {
  // Try to reconnect 3 times than go to deep sleep
  int retryCounter = 3;
  while (!client.connected() && retryCounter > 0) {    
    if (serial_output) {
      Serial.print("Attempting MQTT connection...");
      }
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {      
      if (serial_output) {
        Serial.println("connected");
      }
    } else {      
      if (serial_output) {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
      }
      retryCounter = retryCounter - 1;
      delay(5000);
    }
  }
  if (retryCounter <=0 && !client.connected()) {
    if (serial_output) {
      Serial.println("Failed to connect to MQTT... entering deep sleep");
    }
    ESP.deepSleep(sleepSeconds * 1000000);
  }
}

//bool checkBound(float newValue, float prevValue, float maxDiff) {
//  return !isnan(newValue) &&
//         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
//}

void printBME280Data(Stream* client, int avgOverXMeasurements, bool serial_output) {
  uint8_t pressureUnit(0);                                           // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

  newPres = 0;
  newTemp = 0;
  newHum = 0;
  for (int i = 0; i <= avgOverXMeasurements; i++) {
    float pres(NAN), temp(NAN), hum(NAN);
    bme.read(pres, temp, hum, metric, pressureUnit);                   // Parameters: (float& pressure, float& temp, float& humidity, bool celsius = false, uint8_t pressureUnit = 0x0)
    delay(50);
    //client->print(String(i) + ": " + String(temp) + "\n");
    newPres = newPres + pres / 100; // convert to hPa
    newTemp = newTemp + temp;
    newHum = newHum + hum;
  }
  newPres = newPres / avgOverXMeasurements;
  newTemp = newTemp / avgOverXMeasurements;
  newHum = newHum / avgOverXMeasurements;

  /* Alternatives to ReadData():
    float temp(bool celsius = false);
    float pres(uint8_t unit = 0);
    float hum();
    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
  */
  writeLineToOutput(String(location) + ": " + String("Temp: ") + String(newTemp) + String("°") + String(metric ? 'C' : 'F') + String("\t\tHumidity: ") + String(newHum) + String("% RH\t\tPressure: ") + String(newPres) + String(" hPa"));
//  if (serial_output) {
//    client->print("Temp: ");
//    client->print(newTemp);
//    client->print("°" + String(metric ? 'C' : 'F'));
//    client->print("\t\tHumidity: ");
//    client->print(newHum);
//    client->print("% RH");
//    client->print("\t\tPressure: ");
//    client->print(newPres);
//    client->print(" hPa");
//  }
}
void printBME280CalculatedData(Stream* client, bool serial_output) {
  newAltitude = bme.alt(metric);
  newDewPoint = bme.dew(metric);
  writeLineToOutput(String(location) + ": " + String("Altitude: ") + String(newAltitude) + String(metric ? "m" : "ft") + String("\t\tDew point: ") + String(newDewPoint) + String("°") + String(metric ? 'C' : 'F'));
//  if (serial_output) {
//  client->print("\t\tAltitude: ");
//  client->print(newAltitude);
//  client->print((metric ? "m" : "ft"));
//  client->print("\t\tDew point: ");
//  client->print(newDewPoint);
//  client->println("°" + String(metric ? 'C' : 'F'));
//  }

}
/* ==== END Functions ==== */


