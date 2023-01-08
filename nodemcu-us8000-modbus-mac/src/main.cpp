#include <Arduino.h>
#include <ESP8266Wifi.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define MAX485_DE D1
#define MAX485_RE_NEG D2

#define RXD2 4
#define TXD2 0

// const char *ssid = "KS 24 BLIMBING OUTDOOR";
// const char *password = "pancongnyamantap";
const char *ssid = "such a person";
const char *password = "zidanedane";
// const char *ssid = "Kombukei";
// const char *password = "cobamisoa";
// const char *ssid = "Mavens LT 2 2G";
// const char *password = "adminmavens";

const char *mqtt_server = "118.98.64.212";
const char *userBroker = "admin";
const char *passBroker = "adminmavens";

const int defaultBaudRate = 9600;
int timerTask1, timerTask2, timerTask3;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;

String namaData[] = {" WL ", " TEM "};
int data[] = {1234, 1234};

unsigned long startTime, currentTime, currentTime2, previousMillis;

const long periodeKirimData = 20000;
const long periodeAkuisisi = 2000;

//=================================================================
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

//====================================================================

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[10];
char status_msg[25];
int value = 0;

//============================================================
// fungsi modul untuk koneksi wifi
void setup_wifi()
{

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//============================================================

//============================================================
// fungsi connect MQTT ke Server Broker
void connectMQTT()
{
  // Loop sampai reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // membuat client ID random
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), userBroker, passBroker))
    {
      Serial.println("connected");
      // Jika connected, publish topic sekali...
      // client.publish("sensor/suara", "Pembacaan Sensor Suara");
      // ... dan resubscribe
      // client.subscribe("sensor/suara");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Delay 5 detik sampai tersambung lagi
      delay(5000);
    }
  }
}

//============================================================<

// fungsi kirim data format json
void sendJsonData(int wl, int tem)
{
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();

  JSONencoder["WL"] = wl;
  JSONencoder["TEM"] = tem;

  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Kirim data analisa to MQTT Broker");
  Serial.println(JSONmessageBuffer);

  if (client.publish("logger/awlr", JSONmessageBuffer) == true)
  {
    Serial.println("Success sending message");
  }
  else
  {
    Serial.println("Error sending message");
  }

  client.loop();
  Serial.println("-------------");
}
//============================================================>

uint8_t setOutputLoadPower(uint8_t state)
{
  Serial.print("Writing coil 0x0006 value to: ");
  Serial.println(state);

  delay(10);
  // Set coil at address 0x0006 (Force the load on/off)
  result = node.writeSingleCoil(0x0006, state);

  if (result == node.ku8MBSuccess)
  {
    node.getResponseBuffer(0x00);
    Serial.println("Success.");
  }

  return result;
}

// callback to on/off button state changes from the Blynk app

uint8_t readOutputLoadState()
{
  delay(10);
  result = node.readHoldingRegisters(0x903D, 1);

  if (result == node.ku8MBSuccess)
  {
    loadPoweredOn = (node.getResponseBuffer(0x00) & 0x02) > 0;

    Serial.print("Set success. Load: ");
    Serial.println(loadPoweredOn);
  }
  else
  {
    // update of status failed
    Serial.println("readHoldingRegisters(0x903D, 1) failed!");
  }
  return result;
}

// reads Load Enable Override coil
uint8_t checkLoadCoilState()
{
  Serial.print("Reading coil 0x0006... ");

  delay(10);
  result = node.readCoils(0x0006, 1);

  Serial.print("Result: ");
  Serial.println(result);

  if (result == node.ku8MBSuccess)
  {
    loadPoweredOn = (node.getResponseBuffer(0x00) > 0);

    Serial.print(" Value: ");
    Serial.println(loadPoweredOn);
  }
  else
  {
    Serial.println("Failed to read coil 0x0006!");
  }

  return result;
}

// -----------------------------------------------------------------
void AddressRegistry_0000()
{
  delay(10);
  result = node.readHoldingRegisters(0x0000, 3);

  if (result == node.ku8MBSuccess)
  {
    data[0] = node.getResponseBuffer(0x00);
    data[1] = node.getResponseBuffer(0x01);
    data[2] = node.getResponseBuffer(0x02);

    // Serial.print("Distance : ");
    // Serial.println(loadPoweredOn);

    for (int j = 0; j < 3; j++)
    {
      Serial.print(namaData[j]);
      Serial.print(" | ");
      // dataHigh[j] = 0;
    }
    Serial.println();

    for (int k = 0; k < 3; k++)
    {
      Serial.print(data[k]);
      Serial.print(" | ");
    }
    Serial.println();
    Serial.println();
  }
  else
  {
    // update of status failed
    Serial.println("readHoldingRegisters(0x0000, 1) failed!");
  }
}

/*
void AddressRegistry_3100()
{
  result = node.readInputRegisters(0x3100, 6);

  if (result == node.ku8MBSuccess)
  {

    pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
    Serial.print("PV Voltage: ");
    Serial.println(pvvoltage);

    pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
    Serial.print("PV Current: ");
    Serial.println(pvcurrent);

    pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    Serial.print("PV Power: ");
    Serial.println(pvpower);

    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    Serial.print("Battery Voltage: ");
    Serial.println(bvoltage);

    battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;
    Serial.print("Battery Charge Current: ");
    Serial.println(battChargeCurrent);
  }
}

void AddressRegistry_3106()
{
  result = node.readInputRegisters(0x3106, 2);

  if (result == node.ku8MBSuccess) {
    battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
    Serial.print("Battery Charge Power: ");
    Serial.println(battChargePower);
  }
}

void AddressRegistry_310D()
{
  result = node.readInputRegisters(0x310D, 3);

  if (result == node.ku8MBSuccess) {
    lcurrent = node.getResponseBuffer(0x00) / 100.0f;
    Serial.print("Load Current: ");
    Serial.println(lcurrent);

    lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
    Serial.print("Load Power: ");
    Serial.println(lpower);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x310D failed!");
  }
}

void AddressRegistry_311A() {
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess) {
    bremaining = node.getResponseBuffer(0x00) / 1.0f;
    Serial.print("Battery Remaining %: ");
    Serial.println(bremaining);

    btemp = node.getResponseBuffer(0x01) / 100.0f;
    Serial.print("Battery Temperature: ");
    Serial.println(btemp);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x311A failed!");
  }
}

void AddressRegistry_331B() {
  result = node.readInputRegisters(0x331B, 2);

  if (result == node.ku8MBSuccess) {
    battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    Serial.print("Battery Discharge Current: ");
    Serial.println(battOverallCurrent);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x331B failed!");
  }
}*/

// A list of the regisities to query in order
typedef void (*RegistryList[])();

RegistryList Registries = {
    AddressRegistry_0000,
    // AddressRegistry_3100,
    //  AddressRegistry_3106,
    //  AddressRegistry_310D,
    //  AddressRegistry_311A,
    //  AddressRegistry_331B,
};

// keep log of where we are
uint8_t currentRegistryNumber = 0;

// function to switch to next registry
void nextRegistryNumber()
{
  // better not use modulo, because after overlow it will start reading in incorrect order
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries))
  {
    currentRegistryNumber = 0;
  }
}

// ****************************************************************************

// --------------------------------------------------------------------------------

// exec a function of registry read (cycles between different addresses)
void executeCurrentRegistryFunction()
{
  Registries[currentRegistryNumber]();
}

SoftwareSerial mySerial;
void setup()
{
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(9600);
  mySerial.begin(defaultBaudRate, SWSERIAL_8N1, RXD2, TXD2, false);

  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Modbus slave ID 1
  node.begin(1, mySerial);

  // callbacks to toggle DE + RE on MAX485
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // client.setCallback(callback);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Setup OK!");
  Serial.println("----------------------------");
  Serial.println();
}

void loop()
{
  // put your main code here, to run repeatedly:

  currentTime = millis();
  if (currentTime - previousMillis >= periodeAkuisisi)
  {

    // Serial.println("debug periode akuisisi");
    executeCurrentRegistryFunction();
    nextRegistryNumber();

    previousMillis = currentTime;
  }

  currentTime = millis();
  if (currentTime - startTime >= periodeKirimData)
  {
    Serial.println("Kirim Data ....");

    /*
    String data1 = "Aman";
    int data2 = 1000;
    int data3 = 2000;
    int data4 = 3000;
    int data5 = 4000;

    sendJsonData(data1, data2, data3, data4, data5);*/

    sendJsonData(data[0], data[1]);
    startTime = currentTime;
  }

  //==============================================<
  // Reconnect MQTT
  if (!client.connected())
  {
    connectMQTT();
  }
  client.loop();
  //==============================================>
}