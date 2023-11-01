

/*
Programa FANCOIL Modbus Wifi
Patxi Tortajada.
tortajadajavier@gmail.com
FISABIO FOM
2023
*/
// Cambiar ssid_AP y SLAVE_ID 
#include <EEPROM.h> 
#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <ModbusRTU.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <AsyncElegantOTA.h>
//#include "ESPAsyncWebServer.h"
//#define SLAVE_ID 50

// CONFIGURACION  sensor temperatura ds18b20 /////////////////////////////////////////////////////////////////////
#define ONE_WIRE_BUS 2 // GPIO 2 del ESP8266
const char *ssid_AP = "Fancoil";
const char *password_AP = "MiPassword";
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// CONFIGURACION  sensor puerto serie RS485 /////////////////////////////////////////////////////////////////////
SoftwareSerial S(5, 15);//(rxPin, txPin) we need one serial port for communicating with RS 485 to TTL adapter

ModbusRTU mb;
AsyncWebServer server(80);
// CONFIGURACION salidas/ entradas digitales /////////////////////////////////////////////////////////////////////
// Dependiendo del dispositivo utilizado tiene una correlación u otra de entradas y salidas
// Se define el número de las Digital Inputs / ouputs correspondientes
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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



uint16_t vers=105; //vers 10 es igual a 1.0
uint16_t potencia=1;
uint16_t toff = 0;

boolean estado_marcha = false; //coil 0 
boolean estado_modo = false;    //coil 1
boolean estado_orden = false;
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
//bool orden(float, float, float, bool);
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

  
void comprobar_temperatura(void){

  // Lee las sondas y las ordena como T. Impulsión y T. Ambiente //////////////////////////////////
  sensors.requestTemperatures();  
    temperatura_comprobacion1 = sensors.getTempCByIndex(0);//Se obtiene la temperatura en °C del sensor 0
    temperatura_comprobacion2 = sensors.getTempCByIndex(1);//Se obtiene la temperatura en °C del sensor 0
      switch (estado_modo) {
      case false:
          if (temperatura_comprobacion1 != -127 && temperatura_comprobacion2 != -127){
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
             if (temperatura_comprobacion2 != -127 && temperatura_comprobacion2 != -127){
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


//Da la orden a la valvula si la temperatura sale del rango de histeresis.
void gestionValvula(){
    
  switch (estado_modo) {
    case true:// MODO CALOR
        if (temperaturaEntrada < (consigna - histeresis)){
            // Serial.println("Se pone en marcha el FANCOIL modo calor.");  
            bucle=1;
            digitalWrite(rele_valvula, HIGH);      
            controlPotencia();
        }
        if (temperaturaEntrada > (consigna + histeresis)){ 
    //Serial.println("Se ha cumplido la temperatura consigna objetivo con el modo calor.");       
          bucle=2;
          Alarmar=true;
          digitalWrite(rele_valvula, LOW);      
          controlPotencia();            
        }
        break;
    case false:
        if (temperaturaEntrada > (consigna + histeresis))  {
     // Serial.println("Se pone en marcha el FANCOIL modo frio.");    
          bucle=3;
          digitalWrite(rele_valvula, HIGH);      
          controlPotencia();   
       } 
       
       if (temperaturaEntrada < (consigna - histeresis)){
          bucle=4;
          Alarmar=true;
          digitalWrite(rele_valvula, LOW);      
          controlPotencia();
       }
       break;//Serial.println("Se ha cumplido la temperatura consigna objetivo con el modo frio.");
           
   
  }  
}

void configuracion(void){
   
  Serial.begin(9600);  
  sensors.begin();
  EEPROM.begin(4);  
}

////////     Activa o desactiva la wifi para actualizaciones      /////////////
void gestionWifi(){
  
    if (activarWifi&!activarWifiFlag){          
          WiFi.mode(WIFI_AP);
          WiFi.softAP(ssid_AP, password_AP, 1, false, 1);
          server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
          request->send(200, "text/plain", (cadenaWeb));
          });
          AsyncElegantOTA.begin(&server);    // Start ElegantOTA
          server.begin();
          Serial.println("HTTP server started");
          activarWifiFlag=true;
          }
        if (!activarWifi & activarWifiFlag){     
          
          WiFi.softAPdisconnect(false);
          activarWifiFlag=false;          
          } 

  
  }

  
/////////         Se añaden los registros Modbus       ////////////////  
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
   
  S.begin(9600, SWSERIAL_8N1);
  mb.begin(&S,Tx_Rx); // RE/DE connected to PIN 4 of ESP8266  
  mb.slave(slaveId);
  mb.addHreg(100,26); //Registro tempConsigna
  mb.addHreg(101,potencia); //Registro potencia
  mb.addHreg(102,eepromId); //Registro Cambio de Id 
       
  mb.addIreg(0); //Registro Temperatura Ambiente
  mb.addIreg(1); //Registro vers
  mb.addIreg(2); //Registro toff
  mb.addIreg(3); //Registro Temp. Salida
  
  mb.addCoil(100); //Estado Marcha
  mb.addCoil(101); //Estado Modo
  mb.addCoil(102, false); // Activacion wifi
  
  mb.addIsts(100);//Alarma
  mb.addIsts(101);//Valvula  
  
}


// Se configuran las salidas de los reles y se inicializan a LOW.
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


   ////////////////     Control de la potencia del ventilador     ////////////////
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

////       Da una alarma si no hay salto térmico.      /////////////
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

///       Actualiza e imprime por serie y web los valores cada 5 segundos.      /////////////
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
        +"Estado orden= " + String(estado_orden)+"\n"
        +"Valvula= " + String(digitalRead(rele_valvula))+"\n"
        +"Ventilador marcha 1= " + String(digitalRead(rele_pot1))+"\n"        
        +"Consigna - histeresis= " + String(consigna - histeresis)+"\n"
        +"Consigna + histeresis= " + String(consigna + histeresis)+"\n"
        +"Bucle= " + String(bucle)+"\n"                          
        +"Fin trama";
        } 

  
  }

////////       Gestiona y actualiza los registros Modbus.      ////////////////////////////////
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
     mb.Ists(101, estadoValvula=digitalRead(rele_valvula));
  } 

////////////////            LOGICA DEL FANCOIL        /////////////////////////////////////
void gestionFancoil(){
  
  if (estado_marcha == true){
  ///// Arrancamos el Fancoil     
   gestionTemporizador();
   gestionValvula();          
   gestionTemporizadorAlarma(); /// gestion temporizador de alarma por salto termico  
  }
  else/// Estado marcha = false Apaga el Fancoil.
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

////Inicializamos el temporizador a 15 minutos al detectar presencia.
void gestionTemporizador(){
    sensorValue = analogRead(analogInPin);    
    
    if (sensorValue > 60){
    temporizador_final = millis() + (15 * 60000);//
    presencias_detectadas=presencias_detectadas+1;
    
 }
 ////Se detiene el Fancoil por no detectar presencia.
  if ((millis()>temporizador_final)&(temporizador_final>0)){
      estado_marcha=false;
      mb.Coil(100,estado_marcha) ;                
      temporizador_final=0;
      presencias_detectadas=0;
      toff =0;
      
    }
    ////Actualizamos el valor del tiempo restante al apagado.
    if (temporizador_final>0){
      toff = (temporizador_final - millis())/60000;
      }
  
  }

// Accedemos a la función de alarma por salto termico a los 2 minutos de arrancar la valvula.
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

// Desde esta función podemos cambiar el id modbus.    ///////////////
void gestionId(){      
    if(!digitalRead(resetId)){
      Serial.println("Entro BUCLE ETAPA 1 CAMBIO ID A 50");
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

   
    
  
  