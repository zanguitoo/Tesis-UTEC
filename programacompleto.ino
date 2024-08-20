//================SD===================
//==================================
#include <SD.h>  // Liberia SD
char dato;
File myFile;


//==balanza variables y librerías==================
//==================================================
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
double peso,peso2,promedio_peso;
int i;
HX711 scale;
double pesoset=30;

//==elevador con PID variables y librearías=========
//==================================================
#include <PID_v1.h>  //PID
unsigned long previousMillis = 0;
const long interval = 20;//20 milisegundos.
int diffPosition;
int prevPosition;
double factorV;
double factorP;
double velocidad;
double posicion;
#define encoder0PinA 18
#define encoder0PinB 19
//--DRIVER L298N 2A---
#define PWM_salida 10 //ENA
#define Forward 9  //IN2
#define Reverse 8  //IN1
volatile long encoder0Pos = 0;
String readString="",cad1,cad2;
int pos1,pos2;
bool read_serial=false;
//----PID POSICION
double Pk1 = 30;  //0.9
double Ik1 = 0.0;     //0.0 reducir el error en estado estado estacionario.
double Dk1 = 0.0; //0.08 mejorar el transistorio
double Setpoint1, Input1, Output1, Output1a;    // PID variables position
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);
//----PID VELOCIDAD-------
double Pk2 = 6.0;  //speed it gets there
double Ik2 = 81.2;
double Dk2 = 0;
double Setpoint2, Input2, Output2, Output2a;    // PID variables velocity
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);
//-----Servo
#include <Servo.h>
Servo servoelevador;
int posdisp=32;
int posagua=130;
bool servoelepos;
int posicion1=136;
int posicion2=136+23; //159
int posicion3=136+23+55; //214

//==dispensador variables y librearías==============
//==================================================


Servo servocompuerta;
int abierto;
int cerrado;
int calibracioncomp=35;
bool servocompabrir;
bool motordisp;


//======Botones y sensores=================
//===============================


bool b1,b2,b3,b4,b5,s1;
int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin


//==================menu y serial======================

int mando=0;
String comando="";
int value;

String estado="";
String estadoanterior="";


//=====================================================SETUP=============================================================

void setup() {
Serial.begin(57600);

//========setup balanza + lcd==========
//===============================
lcd.init();                      // initialize the lcd 
lcd.backlight();
lcd.setCursor(3,0);
lcd.print("Tesis UTEC");
lcd.setCursor(4,1);
lcd.print("Comedero");
scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
scale.set_scale(997.f);
scale.power_up();
scale.tare();



//=====setup ELEVADOR PID=================
//================================
//servogirador
servoelevador.attach(5);
//servoposagua();
servoelevador.write(posagua);
 PID1.SetMode(AUTOMATIC);              
 PID1.SetOutputLimits(-255, 255);
 PID1.SetSampleTime(20);
 PID2.SetMode(AUTOMATIC);              
 PID2.SetOutputLimits(-255, 255);
 PID2.SetSampleTime(20);
 pinMode(PWM_salida, OUTPUT);   //  motor PWMs
 pinMode(Forward, OUTPUT);
 pinMode(Reverse,OUTPUT);
 origen(); // ir a origen antes de cualquier cosa
 pinMode(encoder0PinA, INPUT_PULLUP); //cuando es zero es un pulso
 pinMode(encoder0PinB, INPUT_PULLUP);
 attachInterrupt(4, doEncoderA, CHANGE); // encoder pin on interrupt 4 (pin 18)
 attachInterrupt(5, doEncoderB, CHANGE); // encoder pin on interrupt 5 (pin 19)
 //CALCULO DE FACTOR DE VELOCIDAD Y POSICION.
 factorP=(1.0)*(4.00)/(90.0*11.0*2.0);//posicion mm
 factorV=(factorP*1000.0*1.00)/(float(interval));//mm/seg





//========SETUP DISPENSADOR========
//================================

  cerrado=calibracioncomp;
  abierto=calibracioncomp+90;
 servocompuerta.attach(6);
 servocompuerta.write(cerrado);
pinMode(12,OUTPUT); //NEGRO  AZUL
pinMode(13,OUTPUT); //BLANCO  MORADO
 
//======= SETUP BOTONES=====
//=============================

  pinMode(30,INPUT); //NEGRO  AZUL
  pinMode(32,INPUT); //BLANCO  MORADO
  pinMode(34,INPUT); //GRIS    GRIS
  pinMode(36,INPUT); //MORADO  BLANCO
  pinMode(38,INPUT); //AZUL  NEGRO
  pinMode(7,INPUT); //AZUL  NEGRO

//=================SD==================
//====================================
if (!SD.begin(53)) {
    Serial.println("No se pudo inicializar");
    return;
  }
  Serial.println("Inicialización exitosa");
//listFiles(SD.open("/"), 0);

}





//=====================================================LOOP=============================================================
void loop() {
  // put your main code here, to run repeatedly:
b1=digitalRead(30);
b2=digitalRead(32);
b3=digitalRead(34);
b4=digitalRead(36);
b5=digitalRead(38);
s1=digitalRead(7);
val=analogRead(0);
//servoelevador.write(120);
//servocompuerta.write(cerrado);
if(b1==1&&b5==1)
{pruebas();}
if(b1==1&&b2==1)
{delay(500);
  pesar();}
if(b1==1&&b3==1)
{delay(500);
  tara();}
if(b1==1&&b4==1)
{delay(500);
elevador();}
serialpy();



  if (comando.indexOf("pruebas")>=0) {
    Serial.println("llendo a pruebas");
    delay(2000);
   pruebas();
  } else if (comando.indexOf("pesar")>=0) {
   pesar();
  } else if (comando.indexOf("elevador")>=0) {
   elevador();
  } else if (comando.indexOf("tara")>=0) {
   tara();
  } else {
    value = 0;
  }

if (comando.indexOf("dispensar")>=0) {
dispensadoprueba();
}


    if (comando.indexOf("start")>=0) {
      int delayestado=1000;
   estado="m1";
if (estado=="m1")// ir a posición 1 para iniciar el recorrido
{
serialpy();
delay(delayestado);
Setpoint1=posicion1;
Setpoint2=12.5;
elevador();
estado="m2";}
if (estado=="m2") // ir a posición previa a dispensado
{
serialpy();
delay(delayestado);
servoposdisp();
estado="m3";}
if (estado=="m3") // ir a posición de dispensado
{
serialpy();
delay(delayestado);
Setpoint1=posicion2;
Setpoint2=12.5;
elevador();
estado="m4";}
if (estado=="m4")//dispensado
{
serialpy();
delay(delayestado);
//tara();
dispensadoprueba();
/*
while(1)
{
  pesar();
  serialpy();
if (pesoset<=peso)
{break;}

}*/
estado="m5";}
if (estado=="m5") // elevar bandeja cargada
{
serialpy();
delay(delayestado);
Setpoint1=posicion1;
Setpoint2=12.5;
elevador();
estado="m6";}
if (estado=="m6") // girar previo al agua
{
serialpy();
delay(delayestado);
servoposagua();
estado="m7";}
if (estado=="m7") // ingresar al agua
{
serialpy();
delay(delayestado);
Setpoint1=posicion3;
Setpoint2=12.5;
elevador();
estado="m8";}
esperaagua(); // esperar en el agua
if (estado=="m8") // sacar del agua
{
serialpy();
delay(delayestado);
Setpoint1=posicion1;
Setpoint2=12.5;
elevador();

estado="m9";}
esperapostagua();
if (estado=="m9") // ir a posición previa a pesado
{
serialpy();
delay(delayestado);
servoposdisp();
estado="m10";}
if (estado=="m10") // pesado secado
{
serialpy();
delay(delayestado);
Setpoint1=posicion2;
Setpoint2=12.5;
elevador();
esperapesado();
estado="m11";}
if (estado=="m11") // levantar para terminar
{
serialpy();
delay(delayestado);
Setpoint1=posicion1;
Setpoint2=12.5;
elevador();
estado="m12";}
if (estado=="m12") // ultimo estado
{
serialpy();
delay(delayestado);
servoposagua();
estado="m13";}
if (estado=="m13")//   
{
serialpy();
delay(delayestado);
estado="TERMINO TODO";
serialpy();
delay(delayestado);
estado="";
comando="";}
  }

/*
Serial.print("A0:");
Serial.print(val);
Serial.print(" s1:");
Serial.print(s1);
Serial.print(" b1:");
Serial.print(b1);
Serial.print(" b2:");
Serial.print(b2);
Serial.print(" b3:");
Serial.print(b3);
Serial.print(" b4:");
Serial.print(b4);
Serial.print(" b5:");
Serial.print(b5);
Serial.print(" comando:");
Serial.println(comando);
delay(100);
  */



}



//====================FUNCIONES ELEVADOR======================
//=========================================================
void origen()
{s1=digitalRead(7);
while(s1==1)
{
 s1=digitalRead(7);
 digitalWrite(9,HIGH);
 digitalWrite(8,LOW);
 analogWrite(10,120);
if (s1==0)
{ digitalWrite(9,LOW);
 digitalWrite(8,LOW);
 analogWrite(10,0);
  }
delay(10);
}

}


void elevador()
{   while (1)
  {
b1=digitalRead(30);
b2=digitalRead(32);
b3=digitalRead(34);
b4=digitalRead(36);
b5=digitalRead(38);
serialpy();
if(b1==1&&b2==2)
{break;}
/*if(b3==1)
{
  encoder0Pos=0;
  Setpoint1=0;
  Setpoint2=0;
}*/
if(b4==1)
{
while(s1==1)
{
 s1=digitalRead(7);
 digitalWrite(9,HIGH);
 digitalWrite(8,LOW);
 analogWrite(10,100);
if (s1==0)
{ digitalWrite(9,LOW);
 digitalWrite(8,HIGH);
 analogWrite(10,100);
 delay(500);
 digitalWrite(9,LOW);
 digitalWrite(8,LOW);
 analogWrite(10,0);
 encoder0Pos=0;
  Setpoint1=0;
  Setpoint2=0;
  prevPosition=0;
  }
delay(10);
}
}

if(b1==1)
{
  Setpoint1=posicion1;
  Setpoint2=20;
}
if(b2==1)
{
  Setpoint1=posicion2;
  Setpoint2=20;
}
if(b3==1)
{
  Setpoint1=posicion3;
  Setpoint2=20;
}

  if (comando.indexOf("menu")>=0) {
    lcd.setCursor(0,0);
lcd.print("                ");
  lcd.setCursor(0,1);
lcd.print("                ");
delay(2000);
lcd.setCursor(0,0);
lcd.print("   TESIS UTEC   ");
comando="";
 break;
  }



unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event
          previousMillis = currentMillis;

          //Serial.print(encoder0Pos);
          //Serial.print(" , ");  

          // calculate velocity - change in encoder counts over time

          diffPosition = abs(encoder0Pos - prevPosition);       // calc change over time and make it always position with abs         
          prevPosition = encoder0Pos;

          posicion=factorP*encoder0Pos; 
          velocidad=factorV*diffPosition;//mm/s
 
//-----------READ SERIAL DATA---------------/*
          /*while(Serial.available()){
            char c=Serial.read();
            readString +=c;
          }
          if(readString.length()>0){ //enviar dad ejem:  1000,100,,  dos comas al final
            read_serial=true;
            pos1=readString.indexOf(',');//indica la posicion del la coma
            cad1=readString.substring(0,pos1);//POSICION
            pos2=readString.indexOf(',,',pos1+1);
            cad2=readString.substring(pos1+1,pos2);//VELOCIDAD
            
            Setpoint1=cad1.toInt();//POSICION mm (0-280mm)
            Setpoint2=cad2.toInt();//VELOCIDAD (0-38mm/s)
            readString="";
          }
            */
          Input1 = posicion;
          PID1.Compute();

          Input2 = velocidad;
          PID2.Compute();

          //--SERIAL SEND/*
          /*Serial.print(Setpoint1);//SP POSICION
          Serial.print(",");
          Serial.print(velocidad);//VELOCIDAD
          Serial.print(",");
          Serial.print(Output1a);//CV antes Output1
          Serial.print(",");
          Serial.print(posicion);//PV
          Serial.println();*/

          // drive motor         
          Output1a = abs(Output1);
          //if (Output1a<35){
          //  Output1a=0;
          //}

          if (Output1 < 0)    //FORWARD                                   // decide which way to turn the motor
          {
            
            Output1a = constrain(Output1a,0,Output2);//limit the speed at set maximun from Output2

            analogWrite(PWM_salida, Output1a);                           
            digitalWrite(Forward,HIGH);
            digitalWrite(Reverse,LOW);
          }
          else if (Output1 > 0)      //REVERSE                           // decide which way to turn the motor
          { 
            Output1a = constrain(Output1a,0,Output2);//limit the speed at set maximun from Output2

            analogWrite(PWM_salida, Output1a); 
            digitalWrite(Forward,LOW);
            digitalWrite(Reverse,HIGH);
          } 
          else //STOP
          {
            analogWrite(PWM_salida, 0);
            digitalWrite(Forward,LOW);
            digitalWrite(Reverse,LOW);
          }                  
             
       if(abs(posicion-Setpoint1)<1)
{            analogWrite(PWM_salida, 0);
            digitalWrite(Forward,LOW);
            digitalWrite(Reverse,LOW);
  break;}


      }      // end of timed event
  }
}

void doEncoderA(){  

  if (digitalRead(encoder0PinA) == HIGH) {   // look for a low-to-high on channel A
  if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way encoder is turning
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}

void doEncoderB(){  
  if (digitalRead(encoder0PinB) == HIGH) {   // look for a low-to-high on channel B
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}


//==================termino FUNCIONES PID=================

//==================FUNCIONES DISPENSADOR==================
//=========================================================


//===================Termino FUNCIONES DISPENSADOR=========

//====================FUNCIONES BALANZA====================
//=========================================================

void pesar()
{/*scale.power_up();
delay(100);*/
peso=scale.get_units(3);
/*Serial.println(peso, 0);
  lcd.setCursor(0,1);
  lcd.print("               ");
   lcd.setCursor(0,1);
  lcd.print(peso,0);
 scale.power_down();			        // put the ADC in sleep mode
delay(100);*/
comando="";
}

void pesar2()
{peso2=scale.get_units(10);
comando="";
}


void tara()
{/*scale.power_up();*/
scale.tare();
/*  lcd.setCursor(0,1);
  lcd.print("               ");
   lcd.setCursor(0,1);
  lcd.print("tarando...");*/
delay(2000);
/*  lcd.setCursor(0,1);
  lcd.print("               ");
scale.power_down();*/
comando="";
}

//=======================Termino FUNCIONES BALANZA============



//==== void pruebas==========================
//============================================


void pruebas()
{
  lcd.setCursor(0,0);
lcd.print("                ");
  lcd.setCursor(0,1);
lcd.print("                ");

delay(2000);
lcd.setCursor(0,0);
lcd.print("void de pruebas");

while (1)

{

b1=digitalRead(30);
b2=digitalRead(32);
b3=digitalRead(34);
b4=digitalRead(36);
b5=digitalRead(38);
s1=digitalRead(7);
val=analogRead(0);
val=map(val,0,1023,0,180);

if(b1==1&&b5==1)
{  lcd.setCursor(0,0);
lcd.print("                ");
  lcd.setCursor(0,1);
lcd.print("                ");
delay(2000);
lcd.setCursor(0,0);
lcd.print("   TESIS UTEC   ");
  break;}

serialpy();
  if (comando.indexOf("menu")>=0) {
    lcd.setCursor(0,0);
lcd.print("                ");
  lcd.setCursor(0,1);
lcd.print("                ");
delay(2000);
lcd.setCursor(0,0);
lcd.print("   TESIS UTEC   ");
comando="";
 break;
  }
  if (comando.indexOf("servo")>=0) {
  lcd.setCursor(0,0);
lcd.print(" probando servo ");
  lcd.setCursor(0,1);
lcd.print("   ");
lcd.setCursor(0,1);
lcd.print(val);
servoelevador.write(val);
delay(200);
  }
    if (comando.indexOf("termino")>=0) {
comando="";
  }
//====prueba servos===
if (b1==1)
{lcd.setCursor(0,1);
lcd.print("B1");
  
  servocompabrir=!servocompabrir;
if (servocompabrir==0)
{servocompuerta.write(cerrado);}
if (servocompabrir==1)
{servocompuerta.write(abierto);}
delay(1000);
}

if (b2==1)
{lcd.setCursor(0,1);
lcd.print("B2");
  servoelepos=!servoelepos;
if (servoelepos==0)
{for (int pos = posdisp; pos <= posagua; pos++) {
      servoelevador.write(pos); // mueve el servo a la posición
      delay(50); // espera el tiempo necesario
    }
}
if (servoelepos==1)
{for (int pos = posagua; pos >= posdisp; pos--) {
      servoelevador.write(pos); // mueve el servo a la posición
      delay(50); // espera el tiempo necesario
    }
}
delay(1000);
}


//=========prueba motores=======
if (s1==0)// interlock con sensor de fin de carrera
{lcd.setCursor(4,1);
lcd.print("S1");
digitalWrite(8,LOW);
digitalWrite(9,LOW);
analogWrite(10,0);
}
if (s1==1)// interlock con sensor de fin de carrera
{lcd.setCursor(4,1);
lcd.print("  ");
}

if (b3==1)
{lcd.setCursor(0,1);
lcd.print("B3");
digitalWrite(9,LOW);
digitalWrite(8,HIGH);
analogWrite(10,200);
}
if (b4==1&&s1==1)
{lcd.setCursor(0,1);
lcd.print("B4");
digitalWrite(8,LOW);
digitalWrite(9,HIGH);
analogWrite(10,200);
}
if (b3==0 && b4==0)
{
digitalWrite(8,LOW);
digitalWrite(9,LOW);
analogWrite(10,0);
}

if (b5==1)
{lcd.setCursor(0,1);
lcd.print("B5");
  motordisp=!motordisp;
if (motordisp==0)
{digitalWrite(12,LOW);
digitalWrite(13,LOW);
}
if (motordisp==1)
{digitalWrite(12,HIGH);
digitalWrite(13,LOW);
}
delay(1000);
}

delay(100);
lcd.setCursor(0,1);
lcd.print("  ");

//==== fin prueba=============

}

}

//==========================servoelevador

void servoposagua()
{
for (int pos = posdisp; pos <= posagua; pos++) {
      servoelevador.write(pos); // mueve el servo a la posición
      delay(50); // espera el tiempo necesario
    }
}

void servoposdisp()
{
for (int pos = posagua; pos >= posdisp; pos--) {
      servoelevador.write(pos); // mueve el servo a la posición
      delay(50); // espera el tiempo necesario
    }
}


//===============================Funciones Serial=========================


void serialpy()
{
  while (Serial.available()) {
    char c = Serial.read();
    readString += c;
    delay(1);
  }
  
  if (readString.length() > 0) {
    //pos1=readString.indexOf(',');//indica la posicion del la coma
    //comando=readString.substring(0,pos1);//POSICION
    comando=readString;
    readString = "";


  }
if (comando.indexOf("stop")>=0)
{ estadoanterior=estado;
  estado="stop";
}
Serial.print(estado);
Serial.print(",");
Serial.print(b1);
Serial.print(",");
Serial.print(b2);
Serial.print(",");
Serial.print(b3);
Serial.print(",");
Serial.print(b4);
Serial.print(",");
Serial.print(b5);
Serial.print(",");
Serial.print(Setpoint1);//SP POSICION
Serial.print(",");
Serial.print(velocidad);//VELOCIDAD
Serial.print(",");
Serial.print(Output1a);//CV antes Output1
Serial.print(",");
Serial.print(posicion);//PV
Serial.print(",");
Serial.print(peso);//peso
Serial.print(",");
Serial.print(pesoset);//peso
Serial.print(",");
Serial.println(peso2);//peso
delay(1);
if (estado=="stop")
{
  estado=estadoanterior;
  while(1)
  {
  while (Serial.available()) {
    char c = Serial.read();
    readString += c;
    delay(1);
  }
  
  if (readString.length() > 0) {
    comando=readString;
    readString = "";
  }
if (comando.indexOf("start")>=0)
{ break;
}
  }

}


}

////===============funciones SD==================


void listFiles(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // No hay más archivos en el directorio
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      listFiles(entry, numTabs + 1);
    } else {
      // Archivos: imprime el tamaño del archivo
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

////=====================PRUEBA DISPENSADO====================


void dispensadoprueba()
{pesoset=40;
comando="";
tara();
delay(2000);
while(1)
{
  pesar();
  serialpy();
  servocompuerta.write(abierto);
  digitalWrite(12,HIGH);
  digitalWrite(13,LOW);
if (pesoset<=peso)
{servocompuerta.write(cerrado);
    digitalWrite(12,LOW);
  digitalWrite(13,LOW);
  break;}

}

}


//=======================FUNCIONES ESPERA=========================

void esperaagua()
{int i=0;
while (i <= 5) {
      i++;
      serialpy();
      delay(1000); // espera el tiempo necesario
    }
}
void esperapostagua()
{int i=0;
while (i <= 5) {
      i++;
      serialpy();
      delay(1000); // espera el tiempo necesario
    }

}
void esperapesado()
{int i;
while (i <= 50) {
      i++;
      pesar2();
      serialpy();
      delay(100); // espera el tiempo necesario
    }

}


