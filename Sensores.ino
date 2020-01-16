// Modulo Bluetooth
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(0, 1); 

// Sensor de Temperatura y Humedad
#include "DHT.h"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

//Sensor de Sonido
#include <Average.h>

//Radiacion UV
int leituraUV=0; // 
byte indiceUV=0; // 

//ppm2_5
int LED = 4;
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

//////////////////////////////////////////////////////////////////////////////////

void setup() {
  bluetooth.begin(9600);
  Serial.begin(9600);
  pinMode(LED,OUTPUT);
  dht.begin();
}

/////////////////////////////////////////////////////////////////////////////////

void loop() {
  delay(2000);
  //DHT22
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
  Serial.println(F("Failed to read from DHT sensor!"));
  return;
  }

  // MQ-7
  float adc_MQ7 = analogRead(A0); //Lemos la salida analógica  del MQ
  float voltaje_MQ7 = adc_MQ7 * (5/ 1023.0); //Convertimos la lectura en un valor de voltaje
  float Rs_MQ7=10*((5-voltaje_MQ7)/voltaje_MQ7);  //Calculamos Rs con un RL de 10k
  double CO=20.66905256*pow(1000*Rs_MQ7/31.55, -0.656039042); // calculamos la concentración  de CO con la ecuación obtenida.

  //MQ-131
  float adc_MQ131 = analogRead(A1); //Lemos la salida analógica  del MQ
  float voltaje_MQ131 = adc_MQ131 * (5/ 1023.0); //Convertimos la lectura en un valor de voltaje
  float Rs_MQ131=10*((5-voltaje_MQ131)/voltaje_MQ131);  //Calculamos Rs con un RL de 1k
  float O3=17.574*pow(Rs_MQ131/121.72, -0.902); // calculamos la concentración  de O3 con la ecuación obtenida.

  //PPM2_5
  digitalWrite(LED,LOW); // power on the LED
  delayMicroseconds(samplingTime);
  float ppm2_5 = analogRead(A4); // read the dust value
  delayMicroseconds(deltaTime);
  digitalWrite(LED,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);
  float voltajeppm2_5 = (ppm2_5*5)/1024;
  float dustDensity = 32.145*pow(voltajeppm2_5,4) - 58.137*pow(voltajeppm2_5,3) + 27.803*pow(voltajeppm2_5,2) + 1.9091*voltajeppm2_5 + 0.9129;
  
    //SONIDO
   Average<float> ave(100);
   int maxat =0;
   for (int i = 0; i < 100; i++) {
   float sensorValue = analogRead(A2);
   ave.push(sensorValue);
   }
   float db=20*log(ave.maximum(&maxat)-ave.mean());

  //RADIACION-UV
  leituraUV = analogRead(A3); // REALIZA A LEITURA DA PORTA ANALÓGICA 
  indiceUV = map(leituraUV, 0,226,0,11) ; // CONVERTE A FAIXA DE SINAL DO SENSOR DE 0V A 1V PARA O INDICE UV DE 0 A 10.
  
  {
    
  // MQ-7 Monoxido de Carbono 
  //Serial.print("    Monóxido de Carbono:");
  Serial.print(CO);
  Serial.print(String(","));
  //Serial.print("ppm");
  
  //Temperatura y Humedad 
  //Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.print(String(","));
  //Serial.print(" °C ");
  //Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(String(","));
  //Serial.print(" %\t");

  //MQ-131 Ozono
  //Serial.print("    Ozono:");
  Serial.print(O3/1000,6);
  Serial.print(String(","));
  //Serial.println(" ppm");
  
  //PPM2_5
  //Serial.print("    Ozono:");
  Serial.print(dustDensity);
  Serial.print(String(","));
  //Serial.println(" ppm");

  
  //Ruido
  //Serial.print("    Db:");
  Serial.print(db+10);
  Serial.print(String(","));
  //Serial.println(" db");
  
  //Serial.print("Indice UV: ");
  Serial.println(indiceUV);
  
}
}
