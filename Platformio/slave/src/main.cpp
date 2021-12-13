#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define BOARD_ID 1
#define DHTPIN 0
#define DHTTYPE DHT11   
#define voltagePin 36

#define sensor_nivel 39

const int sensor = 32;

int rele = 17;
int rele_status;

int level;

String motor_status = "";
String nivel;

float leitura = 0.0;
double voltage = 0.0;

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
    String level;
    float voltage;
    int readingId;
} struct_message;

//Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 2000;        // Interval at which to publish sensor readings (era 10000)

// Controle do número de leituras enviadas
unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "ESP32-Henrik";

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

String level_water(){
  int leitura_nivel = analogRead(sensor_nivel);
  level = (leitura_nivel / 1023) * 3.3;

  if(level > 2){
    nivel = "Baixo";
  }
  else if(level < 2){
    nivel = "Normal";
  }
  Serial.println(nivel);
  delay(1000);

  return(nivel);
}

float measPot(){
  float sen = analogRead(sensor);
  int s = map(sen, 0, 4095, 100, 0);
  Serial.print("Leitura de umidade: ");
  Serial.println(s);

  if (nivel == "Baixo"){ 
      digitalWrite(rele, LOW);
      rele_status = 0;
  }
  if ((nivel == "Normal") && (s < 5)){
      digitalWrite(rele, HIGH);
      rele_status = 1;
  }
  else{
    digitalWrite(rele, LOW);
    rele_status = 0;
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

float leitura_bateria(){

  leitura = analogRead(voltagePin);
  voltage = (leitura / 1023) * 4.86;
  Serial.print("Tensão da bateria: ");
  Serial.print(voltage);
  Serial.println("V");
  float voltage_env = round(voltage * 10) / 10;

  return voltage_env;
}

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
    myData.level = level_water();     
    myData.voltage = leitura_bateria();
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