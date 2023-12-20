#include <Arduino.h>
/*
Programa FANCOIL Modbus Wifi
Patxi Tortajada.
tortajadajavier@gmail.com
FISABIO FOM
2023
*/

#include <EEPROM.h> 
#include <DallasTemperature.h>
#include <OneWire.h>
#include <ModbusRTU.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>



#define ONE_WIRE_BUS 2 
const char *cadenaFija = "Fancoil_";
const char *password_AP = "mipasword";
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// CONFIGURACION  puerto serie RS485 /////////////////////////////////////////////////////////////////////
SoftwareSerial S485(5, 15);//(rxPin, txPin) we need one serial port for communicating with RS 485 to TTL adapter

ModbusRTU mb;
AsyncWebServer server(80);
// CONFIGURACION salidas/ entradas digitales /////////////////////////////////////////////////////////////////////
// Dependiendo del dispositivo utilizado tiene una correlación u otra de entradas y salidas
// Se define el número de las Digital Inputs / ouputs correspondientes
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int longitudCadena = strlen(cadenaFija) + 1;
int numeroDigitos = 1; 
int rele_valvula = 13;
int rele_pot1 = 16;
int rele_pot2 = 14;
int rele_pot3 = 12;
int resetId = 0;
int Tx_Rx = 4;
const int analogInPin = A0;
int sensorValue = 0;

// CONFIGURACION variables  /////////////////////////////////////////////////////////////////////
int tempEntero1=0;
int tempEntero2=0;
int bucle=0;
int slaveId=50;
int eepromId;
int modbusIdActual=50;
unsigned long temporizador_final = 0;
unsigned long tImpresion=0;
int presencias_detectadas = 0;
unsigned long tAlarmar=0;
int entrada;



uint16_t vers=106; //vers 10 es igual a 1.0
uint16_t potencia=1;
uint16_t toff = 0;

boolean estado_marcha = false; //coil 0 
boolean estado_modo = false;    //coil 1
boolean estado_ir1 = false;
boolean estadoValvula = false;
boolean detectada_presencia = false;
boolean temporizador = false;
boolean imprimir=true;
boolean alarma=false;
boolean Alarmar=true;
boolean retorno=false;
boolean activarWifi = false;    //coil 2
boolean activarWifiFlag=false;
boolean activar=false;

int consigna = 26;
float temperatura_comprobacion1 = 0.0;
float temperatura_comprobacion2 = 0.0;
float temperaturaEntrada = 0.0;
float temperaturaSalida = 0.0;
float histeresis = 0.5;
float version=vers/100.0;
String cadenaWeb="";

// Prototipos de funciones ///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void comprobar_temperatura(void);
void configuracion(void);
void configuracionGPIO(void);
void gestionWifi(void);
void configuracionModbus(void);
void controlPotencia();
void comprobarSalto();
void imprimirSerial();
void gestionFancoil();
void gestionValoresModbus();
void gestionTemporizador();
void gestionTemporizadorAlarma();
void gestionValvula();
void gestionId();

// INICIO SETUP /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Función donde se realizan las configuraciones iniciales
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//##########################################################################################################################
void setup(void){
  configuracionGPIO();
  configuracion();
  configuracionModbus();
  
} // FIN SETUP
//##########################################################################################################################

// INICIO LOOP ////////////////////////////////////////////////////////////////////////////////////////////////////
// Función donde se ejeucta iterativamente el programa (main)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//##########################################################################################################################
void loop(void){ 
     comprobar_temperatura();
     gestionValoresModbus();        
     gestionWifi();
     imprimirSerial();
     gestionFancoil();
     gestionId();   
     mb.task();    
     yield();
} // FIN LOOP
//##########################################################################################################################

// Funciones ///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
  
void comprobar_temperatura(void){

  sensors.requestTemperatures();  
    temperatura_comprobacion1 = sensors.getTempCByIndex(0);//Se obtiene la temperatura en °C del sensor 0
    temperatura_comprobacion2 = sensors.getTempCByIndex(1);//Se obtiene la temperatura en °C del sensor 0
      switch (estado_modo) {
      case false:
          if ((temperatura_comprobacion1 != -127)&(temperatura_comprobacion2 != -127)){
            if(temperatura_comprobacion1>temperatura_comprobacion2){
              temperaturaEntrada = temperatura_comprobacion1;
              temperaturaSalida = temperatura_comprobacion2;
              }else{
                temperaturaEntrada = temperatura_comprobacion2;
                temperaturaSalida = temperatura_comprobacion1;
                }
    
  }
          break;
      case true:
             if ((temperatura_comprobacion2 != -127)&(temperatura_comprobacion2 != -127)){
                if(temperatura_comprobacion1<temperatura_comprobacion2){
                  temperaturaEntrada = temperatura_comprobacion1;
                  temperaturaSalida = temperatura_comprobacion2;
                  }else{
                    temperaturaEntrada = temperatura_comprobacion2;
                    temperaturaSalida = temperatura_comprobacion1;
                }
  }
          break;
      
      }
}

void gestionValvula(){
   
  switch (estado_modo) {
    case true:// MODO CALOR
        if (temperaturaEntrada < (consigna - histeresis)){
           
            digitalWrite(rele_valvula, HIGH);      
            
        }
        if (temperaturaEntrada > (consigna + histeresis)){ 
       
          
          Alarmar=true;
          digitalWrite(rele_valvula, LOW);      
                      
        }
        break;
    case false:
        if (temperaturaEntrada > (consigna + histeresis))  {
    
          digitalWrite(rele_valvula, HIGH);      
             
       } 
       
       if (temperaturaEntrada < (consigna - histeresis)){
          
          Alarmar=true;
          digitalWrite(rele_valvula, LOW);      
          
       }
       break;
           
   
  }  
}

void configuracion(void){
   
  Serial.begin(9600);  
  sensors.begin();
  EEPROM.begin(4);  
}


void gestionWifi(){
 
        if (activarWifi&!activarWifiFlag){          
          
          WiFi.mode(WIFI_AP);
          String ssid_AP = cadenaFija + String(eepromId);
          WiFi.softAP(ssid_AP, password_AP, 1, false, 1);
          server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send(200, "text/plain", (cadenaWeb));
          });
          AsyncElegantOTA.begin(&server);
          server.begin();
          Serial.println("HTTP server started");
          activarWifiFlag=true;
          }
        if (!activarWifi & activarWifiFlag){     
          
          WiFi.softAPdisconnect(false);
          activarWifiFlag=false;          
          }

  
  }

  

void configuracionModbus(void){
     
  EEPROM.get(0, eepromId);
  Serial.println("eeprom id ="+ String(eepromId));
  if (eepromId == 65535){
    Serial.println("Cambio valores");     
   EEPROM.put(0, slaveId );
   EEPROM.commit();
   EEPROM.get(0, eepromId);
   modbusIdActual=eepromId;
    } 
  if (eepromId != 50 ){
   slaveId=eepromId; 
    }
   
  S485.begin(9600, SWSERIAL_8N1);
  mb.begin(&S485,Tx_Rx); // RE/DE   
  mb.slave(slaveId);
  mb.addHreg(100,26); //Registro tempConsigna
  mb.addHreg(101,potencia); 
  mb.addHreg(102,eepromId); //Registro Cambio de Id 
       
  mb.addIreg(0); //Registro Temperatura Ambiente
  mb.addIreg(1); //Registro vers
  mb.addIreg(2); //Registro toff
  mb.addIreg(3); //Registro Temp. Salida
  
  mb.addCoil(100); //Estado Marcha
  mb.addCoil(101); //Estado Modo
  mb.addCoil(102); // Activacion wifi
    
  mb.addIsts(100);//Alarma 
  
}


void configuracionGPIO(void){
  
  pinMode(rele_valvula, OUTPUT);
  pinMode(rele_pot1, OUTPUT);
  pinMode(rele_pot2, OUTPUT);
  pinMode(rele_pot3, OUTPUT); 
  pinMode(resetId, INPUT);

  digitalWrite(rele_valvula, LOW);  
  digitalWrite(rele_pot1, LOW);
  digitalWrite(rele_pot2, LOW);
  digitalWrite(rele_pot3, LOW);
  
}


void controlPotencia(){
    switch (potencia) {
      case 1:
          digitalWrite(rele_pot1, HIGH);
          digitalWrite(rele_pot2, LOW);
          digitalWrite(rele_pot3, LOW);
          break;
      case 2:
          digitalWrite(rele_pot1, LOW);
          digitalWrite(rele_pot2, HIGH);
          digitalWrite(rele_pot3, LOW);
          break;
      case 3:
          digitalWrite(rele_pot1, LOW);
          digitalWrite(rele_pot2, LOW);
          digitalWrite(rele_pot3, HIGH);
          break;
      }
}

void comprobarSalto(){   
  
   if (estado_modo){
    if ((temperaturaSalida-temperaturaEntrada)<2){
      alarma=true;
      }
    }else{
       if ((temperaturaEntrada-temperaturaSalida)<2){
        alarma=true;
        }
      }
   
}

void imprimirSerial(){
 
    if (tImpresion<=millis()){
        imprimir=true;
      }
      if (imprimir==true){
        tImpresion=millis()+5000;
        imprimir=false;
        Serial.println("Inicio trama");        
        Serial.println("Estado Marcha= " + String(estado_marcha));
        Serial.println("Estado Modo= " + String(estado_modo));        
        Serial.println("Activación de Wifi= " + String(activarWifi));   
        Serial.println("Flag Wifi= " + String(activarWifiFlag));
        Serial.println("Potencia= " + String(potencia));
        Serial.println("Temperatura consigna= " + String(consigna));
        Serial.println("Temperatura Entrada= " + String(temperaturaEntrada));
        Serial.println("Temperatura Salida= " + String(temperaturaSalida));    
        Serial.println("presencias_detectadas= " + String(presencias_detectadas));
        Serial.println("Voltaje= " + String(sensorValue));        
        Serial.println("version= " + String(version));
        Serial.println("Tiempo desconexion= " + String(toff)+" minutos");        
        Serial.println("Alarma= " + String(alarma));
        Serial.println("EEprom Id= " + String(eepromId));
        Serial.println("Modbus Id= " + String(modbusIdActual));
        Serial.println("resetId= " + String(digitalRead(resetId)));                                     
        Serial.println("Fin trama");
        Serial.println("");
        Serial.println("");
        Serial.println("");
        cadenaWeb="Estado Marcha= " + String(estado_marcha)+"\n"
        +"Estado Modo= " + String(estado_modo)+"\n"        
        +"Potencia= " + String(potencia)+"\n"
        +"Temperatura consigna= " + String(consigna)+"\n"
        +"Temperatura Entrada= " + String(temperaturaEntrada)+"\n"
        +"Activación Wifi= " + String(activarWifi)+"\n"
        +"Flag activación Wifi= " + String(activarWifiFlag)+"\n"
        +"Temperatura Salida= " + String(temperaturaSalida)+"\n"    
        +"presencias_detectadas= " + String(presencias_detectadas)+"\n"     
        +"Tiempo desconexion= " + String(toff)+" minutos."+"\n"
        +"Alarma= " + String(alarma)+"\n"
        +"version= " + String(version)+"\n"        
        +"Valvula= " + String(digitalRead(rele_valvula))+"\n"
        +"Ventilador marcha 1= " + String(digitalRead(rele_pot1))+"\n"        
        +"Ventilador marcha 2= " + String(digitalRead(rele_pot2))+"\n" 
        +"Ventilador marcha 3= " + String(digitalRead(rele_pot3))+"\n" 
        +"Voltaje= " + String(sensorValue)+"\n" 
        +"Consigna + histeresis= " + String(consigna + histeresis)+"\n"                         
        +"Fin trama";
        } 

  
  }

void gestionValoresModbus(){
 
     tempEntero1 = temperaturaEntrada*100.0;
     tempEntero2 = temperaturaSalida*100.0;
     estado_marcha=mb.Coil(100);     
     estado_modo=mb.Coil(101);          
     consigna=mb.Hreg(100);          
     potencia=mb.Hreg(101);
     modbusIdActual=mb.Hreg(102);     
     activarWifi=mb.Coil(102);     
     mb.Ireg(0, tempEntero1);
     mb.Ireg(1, vers);
     mb.Ireg(2, toff); 
     mb.Ireg(3, tempEntero2);
     mb.Ists(100, alarma);
     
     
  } 

void gestionFancoil(){
  
  if (estado_marcha == true){
   gestionTemporizador();
   gestionValvula();
   controlPotencia();          
   gestionTemporizadorAlarma();
  }else
  {
    Alarmar=true;
    digitalWrite(rele_valvula, LOW);    
    digitalWrite(rele_pot1, LOW);
    digitalWrite(rele_pot2, LOW);
    digitalWrite(rele_pot3, LOW);
    alarma=false;
    bucle=0;
    presencias_detectadas=0;
    temporizador_final = millis() + (15 * 60000);
    toff = 15;        
  }
  
  }
  ////Temporizador de autoapagado.
void gestionTemporizador(){
    sensorValue = analogRead(analogInPin);    
    if (sensorValue > 60){
    temporizador_final = millis() + (15 * 60000);//
    presencias_detectadas=presencias_detectadas+1;
    
 }
 
  if ((millis()>temporizador_final)&(temporizador_final>0)){
      estado_marcha=false;
      mb.Coil(100,estado_marcha) ;                
      temporizador_final=0;
      presencias_detectadas=0;
      toff =0;
      
    }
    
    if (temporizador_final>0){
      toff = (temporizador_final - millis())/60000;
      }
  
  }

// Alarma por salto termico.
void gestionTemporizadorAlarma(){  
  
  if (estadoValvula){    
      if (Alarmar){
        Alarmar=false;
        tAlarmar=millis()+(2*60000);
      }
      if (millis()>tAlarmar){
        comprobarSalto();
        Alarmar=true; 
        }            
    }
    
  }
  void gestionId(){      
    if(!digitalRead(resetId)){
      modbusIdActual=50;
      }  
    if (modbusIdActual != eepromId){
      Serial.println("Entro en bucle");         
      EEPROM.put(0, modbusIdActual);
      EEPROM.commit();
      EEPROM.get(0,eepromId);     
      delay(2000);
      ESP.reset();      
      }
    
    }
