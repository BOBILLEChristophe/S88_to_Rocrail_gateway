/*

Ce programme est une passerelle entre un bus de rétro signalisation S88 et un logiciel de gestion de réseau
comme Rocrail, iTrain ou JMRI et quelques autres.

Il permet de scanner les décodeurs S88 (M88 etc...), traite les modifications d'états sur les capteurs et
renvoi au logiciel les seules modifications constatées pour éviter la surcharge du logiciel.

Le transfert des données peut s'opérer soit en mode TCP, soit en WiFi ou encore sur un bus CAN

*/



#define PROJECT "S88 gateway for Rocrail"
#define VERSION "0.3"
#define AUTHOR "Christophe BOBILLE - www.locoduino.org"


#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "S88.h"

//----------------------------------------------------------------------------------------
//  Select a communication mode
//----------------------------------------------------------------------------------------
#define TCP
//#define WIFI
//#define CAN
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
//  S88
//----------------------------------------------------------------------------------------

#define S88_NB_SLAVE 2          // Nombre de modules S88
#define S88_NB_SLAVE_INPUTS 16  // Nombre d'entrées par module



//----------------------------------------------------------------------------------------
//  TCP et WIFI
//----------------------------------------------------------------------------------------
#if defined(TCP) || defined(WIFI)
IPAddress ip(192, 168, 1, 125);
const uint port = 15731;
#endif

//----------------------------------------------------------------------------------------
//  TCP
//----------------------------------------------------------------------------------------
#if defined(TCP)
#include <Ethernet.h>
#include <SPI.h>
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
EthernetServer server(port);
EthernetClient client;

//----------------------------------------------------------------------------------------
//  WIFI
//----------------------------------------------------------------------------------------
#elif defined(WIFI)
#include <WiFi.h>
const char *ssid = "**********";
const char *password = "***********";
WiFiServer server(port);
WiFiClient client;

//----------------------------------------------------------------------------------------
//  CAN
//----------------------------------------------------------------------------------------
#elif defined(CAN)
#include <ACAN_ESP32.h>  // https://github.com/pierremolinaro/acan-esp32.git
static const uint32_t DESIRED_BIT_RATE = 250UL * 1000UL;  // Marklin CAN baudrate = 250Kbit/s

#endif


//----------------------------------------------------------------------------------------
//  This gateway hash + Rocrail hash
//----------------------------------------------------------------------------------------
const uint16_t hash = 0x1810;  // this gateway hash
uint16_t rrHash;               // for Rocrail hash


//----------------------------------------------------------------------------------------
//  Queues
//----------------------------------------------------------------------------------------

QueueHandle_t s88Queue;
QueueHandle_t debugQueue;

//----------------------------------------------------------------------------------------
//  Task
//----------------------------------------------------------------------------------------

void S88receiveTask(void *pvParameters);
#if defined(TCP) || defined(WIFI)
void tcpListenTask(void *pvParameters);
void tcpSendTask(void *pvParameters);
#elif defined(CAN)
void canSendTask(void *pvParameters);
#endif
void debugTask(void *pvParameters);

const uint8_t BUFFER_SIZE = 13;

//----------------------------------------------------------------------------------------
//  SETUP
//----------------------------------------------------------------------------------------

void setup() {
  // Initialisation de la communication série pour le debug
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

#ifndef TCP || WIFI || CAN
  Serial.print("Select a communication mode.");
  while (1) {}
#endif

#if defined(TCP)
  Serial.println("Waiting for Ethernet connection : ");
  // Initialisation de l'Ethernet
  Ethernet.init(5);  // MKR ETH Shield (changer selon votre matériel)
  Ethernet.begin(mac, ip);
  server.begin();
  Serial.print("IP address = ");
  Serial.println(Ethernet.localIP());
  Serial.printf("Port = %d\n", port);

#elif defined(WIFI)
  WiFi.begin(ssid, password);
  Serial.print("Waiting for WiFi connection : \n\n");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address : ");
  Serial.println(WiFi.localIP());
  server.begin();

#elif defined(CAN)
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  //settings.mDriverReceiveBufferSize = 50;
  settings.mDriverTransmitBufferSize = 50;
  settings.mRxPin = GPIO_NUM_22;  // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_23;  // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  if (errorCode) {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
    return;
  } else
    Serial.print("Configuration CAN OK\n\n");
#endif


  // Initialisation des queues
  s88Queue = xQueueCreate(50, sizeof(Message));
  debugQueue = xQueueCreate(50, sizeof(char) * 128);

  // Création des tâches

  xTaskCreatePinnedToCore(S88receiveTask, "S88receiveTask", 4 * 1024, (void *)s88Queue, 5, NULL, 1);  // Priorité 1, Core 1
#if defined(TCP) || defined(WIFI)
  xTaskCreatePinnedToCore(tcpListenTask, "tcpListenTask", 4 * 1024, NULL, 1, NULL, 1);  // Priorité 1, Core 1
  xTaskCreatePinnedToCore(tcpSendTask, "tcpSendTask", 4 * 1024, NULL, 5, NULL, 0);      // Priorité 5, Core 0
#elif defined(CAN)
  xTaskCreatePinnedToCore(canSendTask, "canSendTask", 4 * 1024, NULL, 5, NULL, 0);  // Priorité 5, Core 0
#endif
  xTaskCreatePinnedToCore(debugTask, "debugTask", 4 * 1024, NULL, 1, NULL, 1);  // Priorité 1, Core 1
}  // end setup

//----------------------------------------------------------------------------------------
//  LOOP
//----------------------------------------------------------------------------------------

void loop() {
  // nothing to do
}


//----------------------------------------------------------------------------------------
//  tcpListenTask
//----------------------------------------------------------------------------------------
#if defined(TCP) || defined(WIFI)
void tcpListenTask(void *pvParameters) {
  while (true) {
    if (!client || !client.connected()) {
      client = server.available();  // Écouter les connexions entrantes
      if (client) {
        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Client connecté, IP : %s", client.remoteIP().toString().c_str());
        xQueueSend(debugQueue, debugMsg, portMAX_DELAY);
      }
    }
    vTaskDelay(100 * portTICK_PERIOD_MS);  // Vérifier toutes les 100ms
  }
}
#endif

//----------------------------------------------------------------------------------------
//  S88receiveTask
//----------------------------------------------------------------------------------------

void S88receiveTask(void *pvParameters) {

  QueueHandle_t s88Queue = static_cast<QueueHandle_t>(pvParameters);

  S88 s88(s88Queue);
  s88.setup();
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (true) {
    s88.readS88();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));  // toutes les x ms
  }
}

#if defined(TCP) || defined(WIFI)

//----------------------------------------------------------------------------------------
//  tcpSendTask
//----------------------------------------------------------------------------------------

void tcpSendTask(void *pvParameters) {
  Message message;
  byte sBuffer[BUFFER_SIZE];
  const uint16_t hash = 0x1810;
  const uint8_t command = 0x22;
  const bool response = true;
  const uint8_t dlc = 8;

  while (true) {
      if (xQueueReceive(s88Queue, &message, portMAX_DELAY)) {
      if (client && client.connected()) {
        sBuffer[0] = 0x00;
        sBuffer[1] = command | response;
        sBuffer[2] = (hash & 0xFF00) >> 8;  // hash
        sBuffer[3] = hash & 0x00FF;
        sBuffer[4] = dlc;
        sBuffer[5] = (message.module & 0xFF00) >> 8;
        sBuffer[6] = message.module & 0x00FF;
        sBuffer[7] = (message.sensor & 0xFF00) >> 8;
        sBuffer[8] = message.sensor & 0x00FF;
        sBuffer[9] = 0x00;
        sBuffer[10] = message.value;
        sBuffer[11] = 0x00;
        sBuffer[12] = 0x0F;
        client.write(sBuffer, BUFFER_SIZE);
      }
    }
  }
}
#elif defined(CAN)
//----------------------------------------------------------------------------------------
//  canSendTask
//----------------------------------------------------------------------------------------
void canSendTask(void *pvParameters) {
  char message[BUFFER_SIZE];
  while (true) {
    //Serial.println("canSendTask");
    if (xQueueReceive(s88Queue, message, portMAX_DELAY)) {
      if (client && client.connected()) {
        client.write(message, BUFFER_SIZE);
        char debugMsg[128];
        snprintf(debugMsg, sizeof(debugMsg), "Envoyé au client TCP: %s", message);
        xQueueSend(debugQueue, debugMsg, portMAX_DELAY);
      }
    }
  }
}
#endif


//----------------------------------------------------------------------------------------
//  debugTask
//----------------------------------------------------------------------------------------

void debugTask(void *pvParameters) {
  char debugMsg[128];
  while (true) {
    if (xQueueReceive(debugQueue, debugMsg, portMAX_DELAY)) {
      Serial.println(debugMsg);
    }
  }
}
