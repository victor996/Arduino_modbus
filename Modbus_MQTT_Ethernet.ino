ut#include <Ethernet.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "MemoryFree.h"
#include <avr/wdt.h>


SoftwareSerial mySerial(2, 3);
int output[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x02, 0x00, 0x00};
int _buff[8] = {};
String input[8] = {};
String value = "";

// configuração rede
byte mac[] = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0x01 };
IPAddress ip(192, 168, 65, 200);//ip arduino
IPAddress server(192, 168, 65, 102); //ip servidor

// funções MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
EthernetClient ethClient;
PubSubClient client(ethClient);

void reconnect() {
  //Loop until we're reconnected
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println(F("connected"));
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void computeCRC() {
  byte num_of_char = 6;
  int *buff = _buff;
  char k, j;                        //variaveis local
  long checksum = 0XFFFF;           //resultado do crc

  for (k = 0; k < num_of_char; k++) {
    checksum ^= *buff;           // executa uma operação ex or (XOR)
    buff++;                      // pega proximo caracter da sequencia(rotaciona uma unidade a direita no vetor)
    for (j = 0; j < 8; j++) {       // checa os 8 bits do caracter
      if ((checksum & 0x01) == 0) { // se LSB bit = 0
        checksum >>= 1;          //apenas rotaciona o bit sem nenhum tratamento
      } else {
        checksum = (checksum >> 1) ^ 0XA001;//rotaciona o bit e faz um ou logico(OR) com 0XA001
      }
    }
  }

  _buff[6] = checksum;      // CRC (MSB)
  _buff[7] = checksum >> 8; // CRC (MSB)
}
//fim computa crc

void setup() {
  wdt_disable();//desabilita o watchdog
  pinMode(6, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  // put your setup code here, to run once:
  //  output[0] = 0x01;
  //  output[1] = 0x03;
  //  output[2] = 0x00;//registroH;
  //  output[3] = 0x04;//registroL;
  //  output[4] = 0x00;
  //  output[5] = 0x02;//só vai lerum registrador
  //  output[6] = 0x65;
  //  output[7] = 0xCB;
  client.setServer(server, 1883);
  client.setCallback(callback);
  Ethernet.begin (mac, ip);
  delay(1500);
}

void loop() {
  while (!client.connected()){
    reconnect();
  }
  funcao();
  delay(100);
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
//  delay(2000);
  for (int i=0;i<10;i++){
    delay(100);
    client.loop();
    delay(100);
  }
  int memoria = freeMemory();
  if (memoria < 900) {

    wdt_enable(WDTO_15MS); //ativa o watchdog para resetar em 15ms.
    while (true) {

    } //entra em loop até resetar...
  }
}

void funcao() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  mySerial.flush();
  digitalWrite(6, HIGH);
  memcpy(_buff, output, sizeof(_buff));
  computeCRC();
  for (int i = 0; i < 8; i++) {
    //    Serial.write(_buff[i]);
    mySerial.write(_buff[i]);
    delay(1);
  }
  //  Serial.flush();
  //  mySerial.flush();
  memset(_buff, 0, sizeof(_buff));
  digitalWrite(6, LOW);
  delay(10);
  if (mySerial.available() > 0) {
    for (int j = 0; j < 9; j++) {
      value = String(mySerial.read(), HEX);
      Serial.println(value);
      input[j] = value;
      value = "";
    }
    String valor = "";
    for (int k = 0; k < 9; k++) {
      //Serial.println(input[k]);
      valor += input[k];
    }
    memset(input, 0, sizeof(input));
    char teste[22] = {};
    Serial.print(F("Valor= "));
    Serial.print(valor);
    valor.toCharArray(teste, 22);
    Serial.print(F("Teste= "));
    Serial.print(teste);

    boolean pubresult = client.publish("teste/modbus", teste);
    while (!pubresult) {
      Ethernet.begin(mac, ip);
      reconnect();
      client.loop();
      boolean pubresult = client.publish("teste/modbus", teste);
    }
    Serial.println();
    valor = "";
  }
  //  Serial.println();
  Serial.flush();
  mySerial.flush();
  memset(input, 0, sizeof(input));
  value = "";
  client.loop();
  Serial.print("freeMemory()=");
  Serial.println(freeMemory());
}



