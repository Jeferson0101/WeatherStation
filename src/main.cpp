// --------------------------Bibliotecas------------------------------
#include<Arduino.h>

//Biblioteca para enviar pacotes mqtt
#include <PubSubClient.h>

//Biblioteca para trabalhar com json
#include<ArduinoJson.h>

//Este Define tem que estar antes de dar um include na biblioteca GSM
#define TINY_GSM_MODEM_SIM900
//Biblioteca GS M
#include <TinyGsmClient.h>

// bibliotecas sensor umidade e temperatura
#include <DHT_U.h>
// todo sensor_adafruit precisa dessa biblioteca
#include <Adafruit_Sensor.h>

//-----------------------------------GSM------------------------------

//  Acess point name.. Muda de acordo com cada provedor
const char apn[]  = "zap.vivo.com.br";
const char user[] = "vivo";
const char pass[] = "vivo";

TinyGsm modem(Serial1);
TinyGsmClient client(modem);
uint8_t gsmState =0;

//-------------Configurações broker, mqtt e credenciais--------------
PubSubClient mqtt(client);

const char* broker = "pesquisa02.lages.ifsc.edu.br";
int mqttPort = 1883;

//credenciais para acesso ao thingsboard device

#define deviceId  "55f380b0-7a98-11e9-be9f-b32b23f983d8"
#define deviceToken   "feCAh4dXoIthvWQb5uGC"

//--------------------DHT22------------------------------------------

#define DHTPIN            4         // pino do DHT22
#define DHTTYPE           DHT22     // Define qual DHT estará sendo usado

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS; // variável que recebe o valor mínimo de delay
sensor_t sensor;
sensors_event_t event;

// -------------------Criando JsonDoc--------------------------------
  char payload[256];
  StaticJsonDocument<256> doc;
//--------------------------Pluviometro----------------------------------
#define PORTA   6

uint32_t pulso = 0;
double valor = 0;
// -----------------------Biruta------------------------------------
#define pin A2
float valorBiruta = 0;
int winddir = 0;
// ---------------------Anemometro------------------------------
// --- Constantes ---
const float pi = 3.14159265;     //Número de pi
int period = 5000;               //Tempo de medida(miliseconds)
int delaytime = 2000;            //Invervalo entre as amostras (miliseconds)
int radius = 115;                //Raio do anemometro(mm)
uint8_t pinAnem = 5;
unsigned int Sample  = 0;        //Armazena o número de amostras
unsigned int counter = 0;        //Contador para o sensor
unsigned int RPM = 0;            //Rotações por minuto
float speedwind = 0;             //Velocidade do vento (m/s)
float windspeed = 0;             //Velocidade do vento (km/h)
//--------------------------------------------------------------------

//Declarando os métodos para ficarem visíveis ao método principal
void sendDhtData();
void portaHandle();
void sendPluvioData();
void sendAnemometerData();
void sendEndDirection();
void sendData();
boolean mqttConnect();

// - anemometro metodos
void RPMcalc();
void windvelocity();
void WindSpeed();
void SpeedWind();
void addcount();

// variável para controlar de quanto em quanto tempo os dados serão enviados ao servidor
int last_send;
long lastReconnectAttempt = 0;

void setup() {

  last_send=0;

  // -- anemometro
  pinMode(pinAnem, INPUT_PULLUP);        //configura o digital 2 como entrada


  Serial.begin(19200);

  //Comunicação GSM RX TX (0,1)
  Serial1.begin(19200);

  if(gsmState == 0){
    //ligar o GPRS automaticamente sem precisar pressionar o botão.
      pinMode(9,OUTPUT);
      digitalWrite(9, HIGH);
      delay(2000);
      digitalWrite(9, LOW);
      gsmState = 1;
  }

  dht.begin();

  // pluviômetro interrupção
  pinMode(PORTA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PORTA), portaHandle, RISING);

  // Serial.println("Initializing modem...");
  // modem.restart();
  //
  // String modemInfo = modem.getModemInfo();
  // Serial.print("Modem: ");
  // Serial.println(modemInfo);

  // //Unlock your SIM card with a PIN
  // //modem.simUnlock("1234");
  //


  // Serial.print("Waiting for network...");
  // while (!modem.waitForNetwork()) {
  //   Serial.println(" fail");
  //
  // }
  // Serial.println(" OK");
  //
  // Serial.print("Connecting to ");
  // Serial.print(apn);
  // while(!modem.gprsConnect(apn, user, pass)) {
  //   Serial.println(" fail");
  //
  //   Serial.print("Connecting to ");
  //   Serial.print(apn);
  //
  //   // while (true);
  // }
  // Serial.println(" OK");
  //
  // // MQTT Broker setup
  // mqtt.setServer(broker, mqttPort);


}

void loop() {


   if(millis() - last_send > 500){
    if(mqtt.connected()){

    }else{
      Serial.println("Connectin to server..");
      while(!mqttConnect()){
        mqttConnect();
      }
    }

    sendData();
     last_send = millis();
     mqtt.loop();
   }



}

// if (!mqttConnect()) {
//   Serial.println();
//   Serial.println("=== MQTT NOT CONNECTED ===");
//   // Reconnect every 10 seconds
//   unsigned long t = millis();
//   if (t - lastReconnectAttempt > 10000L) {
//     lastReconnectAttempt = t;
//     if (mqttConnect()) {
//       lastReconnectAttempt = 0;
//     }
//   }
//   delay(100);
//   return;
// }

  void sendEndDirection(){
    valorBiruta = analogRead(pin) * (3.3 / 1023.0);

    Serial.print("leitura do sensor :");
    Serial.print(valorBiruta);
    Serial.println(" volt");

    if (valorBiruta <= 0.33) {
    winddir = 270;
    }
    else if (valorBiruta <= 0.60) {
    winddir = 180;
    }
    else if (valorBiruta <= 0.87) {
    winddir = 90;
    }
    else {
    winddir = 000;
    }
    Serial.print("Direcao a :");
    Serial.print(winddir);
    Serial.print(" graus");


    doc["endDirection"]= winddir;
  }

  void sendAnemometerData(){
    Sample++;
    Serial.print(Sample);
    Serial.print(": Start measurement...");
    windvelocity();
    Serial.println("   finished.");
    Serial.print("Counter: ");
    Serial.print(counter);
    Serial.print(";  RPM: ");
    RPMcalc();
    Serial.print(RPM);
    Serial.print(";  Wind speed: ");

  //*****************************************************************
  //print m/s
    WindSpeed();
    Serial.print(windspeed);
    Serial.print(" [m/s] ");

  //*****************************************************************
  //print km/h
    SpeedWind();
    Serial.print(speedwind);
    Serial.print(" [km/h] ");
    Serial.println();

    doc["Anemometer"] = speedwind;
  }

  void sendPluvioData(){
    doc["pluviometer"] = valor;

  }

    void sendDhtData(){

      dht.temperature().getSensor(&sensor);
      dht.humidity().getSensor(&sensor);

      dht.temperature().getEvent(&event);

        if (isnan(event.temperature)) {
          Serial.println(F("Error reading temperature!"));

        }else{
          doc["temperature"] = event.temperature;
          //return true;
        }

        dht.humidity().getEvent(&event);
        if (isnan(event.relative_humidity)) {
            Serial.println(F("Error reading humidity!"));
        }
        else {
          doc["humidity"] = event.relative_humidity;
          //return true;
        }

}

void sendData(){

  sendDhtData();
  sendPluvioData();
  sendAnemometerData();
  sendEndDirection();


  serializeJson(doc, payload);
  Serial.println("Payload:");
  //Serial.println(payload);
  serializeJsonPretty(doc, Serial);
  mqtt.publish( "v1/devices/me/telemetry", payload);



}

  boolean mqttConnect() {
  Serial.println("Connecting to ");
  Serial.println(broker);


  // Connect to MQTT Broker
  boolean status = mqtt.connect(deviceId, deviceToken, NULL);


  if (status == false) {
    Serial.println(" fail");
    return false;
  }
  Serial.println(" OK");

//  mqtt.subscribe(topicLed);
  return mqtt.connected();
}

//Função para medir velocidade do vento
void windvelocity()
{
  speedwind = 0;
  windspeed = 0;

  counter = 0;
  attachInterrupt(pinAnem, addcount, RISING);
  unsigned long millis();
  long startTime = millis();
  while(millis() < startTime + period) {}
}


//Função para calcular o RPM
void RPMcalc()
{
  RPM=((counter)*60)/(period/1000);  // Calculate revolutions per minute (RPM)
}


//Velocidade do vento em m/s
void WindSpeed()
{
  windspeed = ((2 * pi * radius * RPM)/60) / 1000;  //Calcula a velocidade do vento em m/s

} //end WindSpeed


//Velocidade do vento em km/h
void SpeedWind()
{
  speedwind = (((2 * pi * radius * RPM)/60) / 1000)*3.6;  //Calcula velocidade do vento em km/h

} //end SpeedWind


//Incrementa contador
void addcount()
{
  counter++;
}

// pegar valor pluviômetro
void portaHandle() {
  // Read sensor
  Serial.print("\n\tExecutando porta handle...");
  pulso++;
  valor += 0.25;
  Serial.print("\nPulso: ");
  Serial.print(pulso);
  Serial.print("\nValor: ");
  Serial.print(valor);
  Serial.print(" mm");
}
