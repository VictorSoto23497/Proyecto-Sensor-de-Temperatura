//-----------------------------------------------------
//Universidad del Valle de Guatemala
//BE3029- Electrónica Digital 2
//Victor Soto 23497
//sot23497@uvg.edu.gt
//19/08/2025
//Nombre:Borrador Proyecto
//Mcu:ESP32 dev kit
//Entradas:Boton
//Salidas:Servo,LEDS,Display
//-----------------------------------------------------


//-----------------------------------------------------
//Librerias
#include <Arduino.h>
#include <stdint.h>
#include "driver/ledc.h"
#include "config.h"
//-----------------------------------------------------

//-----------------------------------------------------
//Definiciones de pines(LEDS,ServoMotor,Display,Boton,Sensor de temperatura)
const int LEDPins[3]={14,27,26};//Llenar Pines 3 necesarios
//Boton temperatura
#define BTNT 12
//Pin del servo motor
#define pinServo 13
//Pin del sensor
#define sensorT 34
//-----------------------------------------------------
//Display 7 segmentos
#define LE 15
#define LD 2
#define LDP 0
#define LC 4
#define LG 16
#define LB 18
#define LF 17
#define LA 5

#define Display1 21
#define Display2 22
#define Display3 23
//-----------------------------------------------------

//-----------------------------------------------------
//Canales PWM (0-16) de LEDS y Servomotor
const int canalPWML1=7;
const int canalPWML2=1;
const int canalPWML3=2;
const int canalServo=4;
//-----------------------------------------------------

//-----------------------------------------------------
//Parámetros para las señales PWM
const int frecuencia=50; // 50 Hz (20 ms)
//Resolución solo de las LEDS
const int resolucionL=6; // bits (0-64)
//Resolución solo del Servo Motor
const int resolucionM=16;
//-----------------------------------------------------

//-----------------------------------------------------
//Variables del Boton(Interrupción)
volatile bool btntpressed;
volatile uint32_t lastISRbtnt=0;
//-----------------------------------------------------


//-----------------------------------------------------
//Prototipos de funciones
void initBotont(void);
void IRAM_ATTR BTNT_ISR(void);
void senalesPWM(float temp);
void LEDSDISPLAY(int num);
void TemperaturaDisplay(float temp);
volatile int decena;
volatile int unidad;
volatile int decimal;
//-----------------------------------------------------

//-----------------------------------------------------
//Rutinas de interrupción y timers
void IRAM_ATTR BTNT_ISR(void){
  uint32_t tiempoActual = millis();
  if (tiempoActual - lastISRbtnt > 250) {
    btntpressed = true;
    lastISRbtnt = tiempoActual;
  }
}

void initTMR0(void);
hw_timer_t *MiRelojito = NULL; 
volatile int cont;
void IRAM_ATTR TMR0_ISR(void){
  digitalWrite(Display1, LOW);
  digitalWrite(Display2, LOW);
  digitalWrite(Display3, LOW);

  switch(cont){
    case 0: // decena
      LEDSDISPLAY(decena);
      digitalWrite(Display1, HIGH);
      digitalWrite(LDP,LOW);
      break;
    case 1: // unidad
      LEDSDISPLAY(unidad);
      digitalWrite(LDP,HIGH);
      digitalWrite(Display2, HIGH);
      break;
    case 2: // decimal
      LEDSDISPLAY(decimal);
      digitalWrite(Display3, HIGH);
      digitalWrite(LDP,LOW);
      break;
  }
  cont++;
  if(cont > 2) cont = 0;
}
//-----------------------------------------------------


double adcRaw = 0; // Y(0)
double alpha = 0.05; // Factor de suavizado (0-1)
double adcFiltrado = adcRaw; // S(0) = Y(0)

AdafruitIO_Feed *canaltemp = io.feed("Temperatura");
AdafruitIO_Feed *canaled = io.feed("feedled");
AdafruitIO_Feed *canalreloj = io.feed("feedreloj");
AdafruitIO_Feed *canalhumor = io.feed("feedhumor");

void setup() {
  Serial.begin(115200);

  //Adafruit
  while(! Serial);
  Serial.print("Conectando a Adafruit IO");
  io.connect();
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());


  initBotont();
  pinMode(sensorT,INPUT);
  pinMode(LEDPins[0],OUTPUT);
  pinMode(LEDPins[1],OUTPUT);
  pinMode(LEDPins[2],OUTPUT);
  for (int i=0; i<3;i++){
    digitalWrite(LEDPins[i],LOW);
  }
  ledcSetup(canalPWML1,frecuencia,resolucionL);//Canal,frecuencia,resolucion
  ledcAttachPin(LEDPins[0],canalPWML1);//Pin y Canal

  ledcSetup(canalPWML2,frecuencia,resolucionL);//Canal,frecuencia,resolucion
  ledcAttachPin(LEDPins[1],canalPWML2);//Pin y Canal

  ledcSetup(canalPWML3,frecuencia,resolucionL);//Canal,frecuencia,resolucion
  ledcAttachPin(LEDPins[2],canalPWML3);

  ledcSetup(canalServo,frecuencia,16);//Canal,frecuencia,resolucion
  ledcAttachPin(pinServo,canalServo);

  pinMode(LE,OUTPUT);
  pinMode(LD,OUTPUT);
  pinMode(LDP,OUTPUT);
  pinMode(LC,OUTPUT);
  pinMode(LG,OUTPUT);
  pinMode(LB,OUTPUT);
  pinMode(LF,OUTPUT);
  pinMode(LA,OUTPUT);

  pinMode(Display1,OUTPUT);
  pinMode(Display2,OUTPUT);
  pinMode(Display3,OUTPUT);


  digitalWrite(LE,LOW);
  digitalWrite(LD,LOW);
  digitalWrite(LDP,LOW);
  digitalWrite(LC,LOW);
  digitalWrite(LG,LOW);
  digitalWrite(LB,LOW);
  digitalWrite(LF,LOW);
  digitalWrite(LA,LOW);

  digitalWrite(Display1,LOW);
  digitalWrite(Display2,LOW);
  digitalWrite(Display3,LOW);

  initTMR0();
}

void loop() {
  adcRaw = analogRead(sensorT);
  adcFiltrado = (alpha * adcRaw) + ((1.0 - alpha) * adcFiltrado);
  float voltaje = (adcFiltrado / 4095.0) * 5;

  // Convertir voltaje a temperatura (LM35: 10mV/°C = 0.01 V/°C)
  float temperatura = voltaje / 0.01;

  if (btntpressed){
     io.run();
    // Imprimir
    Serial.print("ADC filtrado: ");
    Serial.print(adcFiltrado);


    Serial.print("  Mapeado: ");
    Serial.println(temperatura);
    senalesPWM(temperatura);
    TemperaturaDisplay(temperatura);

    Serial.print("Sending ->");
    Serial.println(temperatura);
    canaltemp->save(temperatura);

    if (temperatura<22){
      canaled->save((String)"#0cf017"); 
      canalreloj->save((String)"snowflake-o");
      canalhumor->save((String)"frown-o");
    }
    else if (temperatura>=22 && temperatura<25){
      canaled->save((String)"#ebff0d");
      canalreloj->save((String)"w:day-sunny"); 
      canalhumor->save((String)"meh-o");
    }
    else if (temperatura>=25){
      canaled->save((String)"#ed0510");
      canalreloj->save((String)"fire"); 
      canalhumor->save((String)"smile-o");
    }
    btntpressed = false; 
  }
}
void initBotont(void){
  pinMode(BTNT,INPUT_PULLUP);
  attachInterrupt(BTNT,&BTNT_ISR,FALLING);
}
void senalesPWM(float temp){
  if(temp< 22.0){
    ledcWrite(canalPWML1,63);
    ledcWrite(canalPWML2,0);
    ledcWrite(canalPWML3,0);
    ledcWrite(canalServo,5972);//Al colocar en la maqueta puede cambiar por el caso 3
  }
  else if (temp>22.0 && temp<25.0)
  {
    ledcWrite(canalPWML1,0);
    ledcWrite(canalPWML2,63);
    ledcWrite(canalPWML3,0);
    ledcWrite(canalServo,4915);
  }
  else{
    ledcWrite(canalPWML1,0);
    ledcWrite(canalPWML2,0);
    ledcWrite(canalPWML3,63);
    ledcWrite(canalServo,3957);//Al colocar en la maqueta puede cambiar por el caso 1
  }
}

void LEDSDISPLAY(int num){
  switch(num){
    case 0: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,HIGH); 
      digitalWrite(LE,HIGH); 
      digitalWrite(LF,HIGH); 
      digitalWrite(LG,LOW); 
    break;
    case 1:
      digitalWrite(LA,LOW);
      digitalWrite(LB,HIGH);
      digitalWrite(LC,HIGH);
      digitalWrite(LD,LOW); 
      digitalWrite(LE,LOW); 
      digitalWrite(LF,LOW); 
      digitalWrite(LG,LOW); 
      break;
    case 2: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,LOW);
      digitalWrite(LG,HIGH); 
      digitalWrite(LE,HIGH); 
      digitalWrite(LD,HIGH); 
      digitalWrite(LF,LOW); 
    break;
    case 3: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,HIGH); 
      digitalWrite(LE,LOW);
      digitalWrite(LF,LOW);
      digitalWrite(LG,HIGH); 
      break;
    case 4: 
      digitalWrite(LA,LOW);
      digitalWrite(LD,LOW);
      digitalWrite(LE,LOW);
      digitalWrite(LF,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LG,HIGH); 
      break;
    case 5: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,LOW);
      digitalWrite(LE,LOW);
      digitalWrite(LF,HIGH); 
      digitalWrite(LG,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,HIGH); 
      break;
    case 6: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,LOW);
      digitalWrite(LF,HIGH); 
      digitalWrite(LG,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,HIGH); 
      digitalWrite(LE,HIGH); 
      break;
    case 7: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,LOW);
      digitalWrite(LE,LOW);
      digitalWrite(LF,LOW);
      digitalWrite(LG,LOW);
      break;
    case 8: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,HIGH); 
      digitalWrite(LE,HIGH); 
      digitalWrite(LF,HIGH); 
      digitalWrite(LG,HIGH); 
      break;
    case 9: 
      digitalWrite(LA,HIGH); 
      digitalWrite(LB,HIGH); 
      digitalWrite(LC,HIGH); 
      digitalWrite(LD,LOW); 
      digitalWrite(LE,LOW);
      digitalWrite(LF,HIGH); 
      digitalWrite(LG,HIGH); 
      break;
  }
}
void TemperaturaDisplay(float temp){
  int num=(int)roundf(temp*10);
  decena=num/100; 
  unidad=(num/10)%10;
  decimal=(num%10);
}
void initTMR0(void){

  MiRelojito = timerBegin(0, 80 , true);

  timerAttachInterrupt(MiRelojito, &TMR0_ISR, true);

  timerAlarmWrite(MiRelojito,1000, true);

  timerAlarmEnable(MiRelojito);
}