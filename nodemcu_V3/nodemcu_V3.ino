
#include <BME280I2C.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>


#define wifi_ssid "BTWLAN"
#define wifi_password "X3nopusLaevis"

#define room "Bad"
#define location "OG1/"room
#define mqtt_server "192.168.1.8"

#define mqtt_user "nodemcu_"room
#define mqtt_password "your_password"

#define humidity_topic "Home/"location"/Humidity"
#define temperature_topic "Home/"location"/Temperature"
#define pressure_topic "Home/"location"/Pressure"
#define altitude_topic "Home/"location"/Altitude"
#define dewpoint_topic "Home/"location"/Dewpoint"


WiFiClient espClient;
PubSubClient client(espClient);

/* ==== Global Variables ==== */
BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
long lastMsg = 0;
int avgOverXMeasurements = 10;

float newTemp(NAN), newHum(NAN), newPres(NAN);
float newAltitude = 0.0;
float newDewPoint = 0.0;;
/* ==== Defines ==== */
#define SERIAL_BAUD 115200
/* ==== END Defines ==== */

const int sleepSeconds = 5 * 60;

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client, int avgOverXMeasurements);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {} // Wait
  while (!bme.begin()) {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //  long now = millis();
  //  if (now - lastMsg > 10000) {
  //    lastMsg = now;

  printBME280Data(&Serial, avgOverXMeasurements);
  printBME280CalculatedData(&Serial);


  client.publish(temperature_topic, String(newTemp).c_str(), true);
  client.publish(humidity_topic, String(newHum).c_str(), true);
  client.publish(pressure_topic, String(newPres).c_str(), true);
  client.publish(altitude_topic, String(newAltitude).c_str(), true);
  client.publish(dewpoint_topic, String(newDewPoint).c_str(), true);

  //  }

  ESP.deepSleep(sleepSeconds * 1000000);
}


/* ==== Functions ==== */

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//bool checkBound(float newValue, float prevValue, float maxDiff) {
//  return !isnan(newValue) &&
//         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
//}

void printBME280Data(Stream* client, int avgOverXMeasurements) {
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

  client->print("Temp: ");
  client->print(newTemp);
  client->print("°" + String(metric ? 'C' : 'F'));
  client->print("\t\tHumidity: ");
  client->print(newHum);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(newPres);
  client->print(" hPa");
}
void printBME280CalculatedData(Stream* client) {
  newAltitude = bme.alt(metric);
  newDewPoint = bme.dew(metric);
  client->print("\t\tAltitude: ");
  client->print(newAltitude);
  client->print((metric ? "m" : "ft"));
  client->print("\t\tDew point: ");
  client->print(newDewPoint);
  client->println("°" + String(metric ? 'C' : 'F'));

}
/* ==== END Functions ==== */


