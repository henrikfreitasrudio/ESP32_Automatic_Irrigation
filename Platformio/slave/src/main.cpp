#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define BOARD_ID 1
#define DHTPIN 0
#define DHTTYPE DHT11
// #define ldr1 12
// #define ldr2 13    
#define voltagePin 36

#define sensor_nivel 39
int leitura_nivel = 0;

const int sensor = 32;

int rele = 17;
int rele_status;

String motor_status = "";

// int ldr1_read = 0;
// int ldr2_read = 0;

float leitura = 0.0;
float voltage = 0.0;

String LDR_quality = "";

DHT dht(DHTPIN, DHTTYPE);

// Endereço MAC do ESP Master
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estrutura de dados que queremos enviar
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    float sensor; 
    String motor; 
    //String solar; 
    int readingId;
} struct_message;

//Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 2000;        // Interval at which to publish sensor readings (era 10000)

// Controle do número de leituras enviadas
unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "Freitas";

// Verifica a rede e obtém o canal
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

float measPot(){
  float sen = analogRead(sensor);
  int s = map(sen, 0, 4095, 100, 0);
  Serial.print("Leitura de umidade: ");
  Serial.println(s);

  if(s > 20) {
    digitalWrite(rele, LOW);
    rele_status = 0;
  }
  else{
    digitalWrite(rele, HIGH);
    rele_status = 1;
  }

  return s;
}

String motor(){
  if(rele_status == 1){
    motor_status = "Ligado";
  }
  else if(rele_status == 0){
    motor_status = "Desligado";
  }
  Serial.println(motor_status);
  
  return motor_status;
}

// String geracao_solar(){
//   ldr1_read = analogRead(ldr1);
//   ldr2_read = analogRead(ldr2);

//   float ldr_media = (ldr1_read + ldr2_read) / 2;
  
//   if(ldr_media < 1700){
//     fflush(stdin);
//     LDR_quality = "High";
//   }
//   else if((ldr_media > 1700) and (ldr_media < 3000)){
//     LDR_quality = "Medium";
//   }
//   else{
//     fflush(stdin);
//     LDR_quality = "Low";
//   }
//   Serial.println(LDR_quality);

//   ldr1_read = 0;
//   ldr2_read = 0;

//   return LDR_quality;
// }

float level_water(){
  leitura_nivel = analogRead(sensor_nivel);
  
  int level = (leitura_nivel / 1023) * 3.3;
  Serial.print("Leitura do nível: ");
  Serial.println(level);

  delay(1000);

  return(leitura_nivel);
}

float leitura_bateria(){
  leitura = analogRead(voltagePin);
  voltage = (leitura / 1023) * 4.56;
  Serial.print("Tensão da bateria: ");
  Serial.print(voltage);
  Serial.println("V");
  return voltage;
}

// Medições da temperatura
float readDHTTemperature() {
  float t = dht.readTemperature();
  if (isnan(t)) {    
    Serial.println("Falha na leitura do sensor DHT11!");
    return 0;
  }
  else {
    Serial.println(t);
    return t;
  }
}

float readDHTHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Falha na leitura do sensor DHT11!");
    return 0;
  }
  else {
    Serial.println(h);
    return h;
  }
}

// Função de retorno, imprime se a mensagem foi entregue com sucesso ou não
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Enviado com sucesso" : "Falha no envio");
}
 
void setup() {
  Serial.begin(115200);

  pinMode(rele, OUTPUT); 
  pinMode(voltagePin, INPUT); 

  dht.begin();
 
  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Inicia o ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro na inicialização do ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  //Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  
  //Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  level_water();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // Setando os valores para envio
    myData.id = BOARD_ID;
    myData.temp = readDHTTemperature();
    myData.hum = readDHTHumidity();
    myData.sensor = measPot();             
    myData.motor = motor();                
    //myData.solar = geracao_solar();           
    myData.readingId = readingId++;
     
    // Enviando mensagem via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Enviado com sucesso");
    }
    else {
      Serial.println("Erro no envio");
    }
  }
}