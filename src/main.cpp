#include "Wire.h"
#include <WiFi.h>
#include "I2Cdev.h"       // Librerias I2C para controlar el mpu6050
#include "MPU6050.h"      // Librerias I2C para controlar el mpu6050 - Libreria modificada en el archivo MPU6050.h - https://www.youtube.com/watch?v=FMZO4UTVZBk (observar comentarios)
#include <SSD1306Wire.h>  // legacy: #include "SSD1306.h"
#include <NTPClient.h>    // Actualizar la hora por medio del servidor NTPClient - Esta librería ha sido modificada https://github.com/arduino-libraries/NTPClient/issues/36
#include <WiFiUdp.h>      // Udp.cpp: Biblioteca para enviar / recibir paquetes UDP

////////////////////////// Inicializar NTPCliente //////////////////////////


// Network credentials
const char* ssid     = "RED_CANGO";
const char* password = "*/Anita2020_CNN";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String formattedTime;
String dayStamp;


////////////////////////// Inicializar MPU6050 //////////////////////////


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

// Instancia objeto de la clase MPU6050
MPU6050 sensor;

// Valores RAW (sin procesar) del giroscopio en los ejes x,y,z
const int timeData = 3; // Tiempo en recolectar datos en segundos
int16_t accelX = 0, accelY = 0, accelZ = 0;
//int16_t gyroX = 0, gyroY =0, gyroZ = 0;


////////////////////////// Inicializar SSD1306 //////////////////////////


// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, 21, 22);  // ADDRESS, SDA, SCL  -  If not, they can be specified manually.


////////////////////////// CARACTERISTICAS CNN //////////////////////////


// Caracteristicas
double mavXAxis = 0.0,mavYAxis = 0.0,mavZAxis = 0.0;
double rmsXAxis = 0.0,rmsYAxis = 0.0,rmsZAxis = 0.0;
double wlXAxis = 0.0, wlYAxis = 0.0, wlZAxis = 0.0;
double wlaXAxis = 0.0, wlaYAxis = 0.0, wlaZAxis = 0.0;

const int pinStart = 0;
boolean flag = false;
unsigned long debounce = 0; // Tiempo del rebote.


///////////////////////////////// Variables Red Neuronal /////////////////////////////////
double a0[9];
double W1[4][9] = {{0.077,0.97,0.073,-0.684,1.097,-0.151,-0.388,-0.662,-0.369},{-0.548,0.87,-0.566,-0.088,0.095,-0.808,1.231,0.825,0.53},{0.359,-1.012,-0.02,0.926,-1.026,-0.139,-0.27,0.241,0.404},{-0.835,-0.057,0.089,-0.699,-0.163,0.821,-1.176,0.362,0.426}};
double a1[4];
double W2[4][4] = {{-0.775,-0.926,0.629,-0.02},{-1.236,1.386,-0.632,0.057},{0.881,0.573,-0.347,-1.151},{0.273,-1.048,-0.478,1.128}};
double a2[4]; 
double b1[4]= {0.569,0.699,-0.043,0.166};
double b2[4]= {0.641,0.026,-0.657,0.49};
double aux = 0.0;
//////////////////////////////////////////////////////////////////



///////////////////////////////// Preprocesamiento Red Neuronal /////////////////////////////////
double mean[9]={1.917,8.791,2.756,2.568,8.997,3.162,46.166,31.313,27.96};
double dstd[9]={1.167,0.364,0.502,1.438,0.403,0.544,24.855,15.893,7.553};
///////////////////////////////////////////////////////////////////////////////////////////////////////


void ttime() {

  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }

  formattedDate = timeClient.getFormattedDate();
  formattedTime = timeClient.getFormattedTime();

  display.clear();
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(64, 0, "FECHA: ");
  display.drawString(64, 10, String(formattedDate));
  
  display.drawString(64, 30, "HORA: ");
  display.drawString(64, 40, String(formattedTime));
  
  display.display();

}


void featuresExtraction() {
    
  int sizeSample = timeData*10;
  
  mavXAxis = 0.0;
  mavYAxis = 0.0;
  mavZAxis = 0.0;

  rmsXAxis = 0.0;
  rmsYAxis = 0.0;
  rmsZAxis = 0.0;

  wlXAxis = 0.0;
  wlYAxis = 0.0;
  wlZAxis = 0.0;

  for (int k = 0; k<sizeSample;k++)
  { 
    sensor.getAcceleration(&accelX, &accelY, &accelZ);
   
    double ax_m_s2 = accelX * (9.81/16384.0);
    double ay_m_s2 = accelY * (9.81/16384.0);
    double az_m_s2 = accelZ * (9.81/16384.0);

    //Valor absoluto medio (MAV)
    mavXAxis = mavXAxis + abs(ax_m_s2);
    mavYAxis = mavYAxis + abs(ay_m_s2);
    mavZAxis = mavZAxis + abs(az_m_s2);

    ///Valor eficaz (RMS)
    rmsXAxis = rmsXAxis + ax_m_s2*ax_m_s2;
    rmsYAxis = rmsYAxis + ay_m_s2*ay_m_s2;
    rmsZAxis = rmsZAxis + az_m_s2*az_m_s2;

    wlXAxis = wlXAxis + abs(ax_m_s2 - wlaXAxis);
    wlYAxis = wlYAxis + abs(ay_m_s2 - wlaYAxis);
    wlZAxis = wlZAxis + abs(az_m_s2 - wlaZAxis);

    wlaXAxis = ax_m_s2;
    wlaYAxis = ay_m_s2;
    wlaZAxis = az_m_s2;
    
    delay(100);
  }
  
  mavXAxis = mavXAxis/(double)sizeSample;
  mavYAxis = mavYAxis/(double)sizeSample;
  mavZAxis = mavZAxis/(double)sizeSample;
  
  rmsXAxis = sqrt(rmsXAxis/(double)sizeSample);
  rmsYAxis = sqrt(rmsYAxis/(double)sizeSample);
  rmsZAxis = sqrt(rmsZAxis/(double)sizeSample);
}

//Funciones de acticación
double relu(double n) {
  if(n>=0) return n; else if (n<0) return 0;
}


double sigmoid(double n) {
  return 1.0/(1.0 + exp(-n));
}


//Función de normalización
double dataNormalized(double inputData,double mean,double desvStandar) {
  double valueNorm;
  valueNorm = (inputData-mean)/desvStandar;
  return valueNorm;
}


void pulse() {
  if(!digitalRead(pinStart) && (millis()-debounce > 500))
  {
    debounce = millis();
    flag = true;
  }
} 


void setup() {
  
  Serial.begin(115200);    //Iniciando puerto serial

  display.init();
  display.setTextAlignment(TEXT_ALIGN_CENTER);

  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);
  delay(1000);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      //Serial.print(".");
  }

  // Print local IP address and start web server
  //Serial.println("");
  //Serial.println("WiFi connected.");

  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(-18000);   //GMT-5

  pinMode(pinStart,INPUT_PULLUP);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  //Serial.println("Iniciando MPU6050");
  if (sensor.testConnection()) delay(500); //Serial.println("Sensor iniciado correctamente"); 
  else delay(500); //Serial.println("Error al iniciar el sensor");

}

void loop() {

  display.clear();
  ttime();    //Inicio del reloj
  display.clear();
  
  //Condición si se presionó el botón, este retorna un true, en la bandera
  if(!digitalRead(pinStart)) {
      pulse();
  }
  
  if(flag) {

    //Mensaje de inicio de ingreso de datos
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 22, "¡Empiece Ahora!");
    display.display();
    delay(500);
    
    //Extracción de datos
    featuresExtraction();

    //Mensaje de fin de ingreso de datos
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 22, "¡Muy bien!");
    display.display();    

    a0[0] = dataNormalized(mavXAxis,mean[0],dstd[0]);
    a0[1] = dataNormalized(mavYAxis,mean[1],dstd[1]);
    a0[2] = dataNormalized(mavZAxis,mean[2],dstd[2]);

    a0[3] = dataNormalized(rmsXAxis,mean[3],dstd[3]);
    a0[4] = dataNormalized(rmsYAxis,mean[4],dstd[4]);
    a0[5] = dataNormalized(rmsZAxis,mean[5],dstd[5]);

    a0[6] = dataNormalized(wlXAxis,mean[6],dstd[6]);
    a0[7] = dataNormalized(wlYAxis,mean[7],dstd[7]);
    a0[8] = dataNormalized(wlZAxis,mean[8],dstd[8]);

    ///////////////////////////////// Estructura Red Neuronal /////////////////////////////////
    for(int i = 0 ; i<4; i++ ) {aux=0.0;for(int j = 0 ; j <9 ; j++ ) { aux=aux+W1[i][j]*a0[j];} a1[i]=relu(aux+b1[i]);}
    double aux1 = 0;
    for(int i = 0 ; i<4; i++ ) {aux=0.0;for(int j = 0 ; j <4 ; j++ ){ aux=aux+W2[i][j]*a1[j];} a2[i]=(aux+b2[i]);aux1=aux1+exp(a2[i]);}
    double minimo = 0.0;
    int classes = 0;
    for(int i = 0;  i<4; i++){a2[i] = exp(a2[i])/aux1;if(a2[i]>minimo){minimo=a2[i];classes=i;}}
    //////////////////////////////////////////////////////////////////////////////////////////
    
    //Serial.print("Numero : ");
    //Serial.println(classes);
    //Serial.println(round(classes));

    switch (classes) {
      case 0:
        Serial.println("Numero : 0");
        display.clear();
        display.setFont(ArialMT_Plain_24);
        display.drawString(64, 22, "3");
        display.display();
        break;
      case 1:
        Serial.println("Numero : 1");
        display.clear();
        display.setFont(ArialMT_Plain_24);
        display.drawString(64, 22, "¤");
        display.display();
        break;
      case 2:
        Serial.println("Numero : 2");
        display.clear();
        display.setFont(ArialMT_Plain_24);
        display.drawString(64, 22, "L");
        display.display();
        break;
      case 3:
        Serial.println("Numero : 3");
        display.clear();
        display.setFont(ArialMT_Plain_24);
        display.drawString(64, 22, "U");
        display.display();
        break;
    }

    //La bandera regresa a false
    flag = false;
    
    delay(2000);
    display.clear();    //Finaliza el proceso y reinicia el reloj
  } 
}