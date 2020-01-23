
#include <Adafruit_BMP280.h>//Especifica del senor
#include <Adafruit_Sensor.h>//comun para todos los sensores de adafruit

//en verdad no hace falta pues utilizo la comunicacion por el metodo I2c y no el spi
#include <SPI.h>

#include <Wire.h> //This library allows you to communicate with I2C / TWI devices
#include <LiquidCrystal_I2C.h>/*libreria para controlar lcd con adaptador */
#include <dht.h> //controla el Tº y humedad 

#define DHT11_PIN 7


dht DHT;
Adafruit_BMP280 bmp; // I2Cdht DHT;
LiquidCrystal_I2C lcd(0x27,16,2); 

//En que caso del switch estamos
short int posicionCanal = 0;
//Variables del clima
int detectarSonido;
unsigned short int velocidad;
int sol; //nada asignado, para cuando incluya el sensor de luz
float agua;// si,no del sensor 
//motor
int motor = analogRead(A2);
//Pines
const short int pinBoton1 = 12;
const short int pinBoton2 = 11;
const short int pinLED = 13;
const short int pinMicrofono = 8;
const int tiempoAntirrebote=2;
//FUNCION PARA BOTONES
int canal=0;

//incremento
int estadoBotonIncremento;
int estadoBotonIncrementoAnterior;

//decremento
int estadoBotonDecremento;
int estadoBotonDecrementoAnterior;

boolean antirebote(int pin) {
  int contador = 0;
  boolean estado;
  boolean estadoAnterior;

  do{
    estado =  digitalRead(pin);
    if(estado != estadoAnterior){
      contador =0;
      estadoAnterior=estado;
      }
       else ++contador;
  }
 
  while(contador<tiempoAntirrebote);
  return estado;
}
byte smile[8] = {
  0b00000000,
  0b00001010,
  0b00001010,
  0b00001010,
  0b00000000,
  0b00010001,
  0b00001110,
  0b00000000,
};
byte sun1[8] = {
  0b00000100,
  0b00010001,
  0b00000100,
  0b00001110,
  0b00000100,
  0b00010001,
  0b00000100,
  0b00000000,
};

byte sunHalf[8] = {
  0b00000100,
  0b00010001,
  0b00000100,
  0b00001110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  };
byte sun[8] = {
  0b00001110,
  0b00011111,
  0b00011111,
  0b00011111,
  0b00011111,
  0b00011111,
  0b00001110,
  0b00000000,
  };
  byte drop[8] = {
  0b00000100,
  0b00000100,
  0b00001110,
  0b00011111,
  0b00010111,
  0b00011011,
  0b00001110,
  0b00000000,
  };
byte rainy[8] = {
  0b00001110,
  0b00011111,
  0b00000000,
  0b00001001,
  0b00000100,
  0b00010001,
  0b00000100,
  0b00000000,
};
 byte cloud[8] = {
  0b00001110,
  0b00011111,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
};
byte wind[8] = {
  0b00010000,
  0b00010000,
  0b00001101,
  0b00001010,
  0b00001100,
  0b00010100,
  0b00010100,
  0b00000100,
};
byte crack[8] = {
  0b00000100,
  0b00001100,
  0b00001100,
  0b00011111,
  0b00000110,
  0b00000110,
  0b00000100,
  0b00000100,
};

void setup(){
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  lcd.init();
  lcd.backlight();
  
  lcd.begin(16, 2); // Indicar a la libreria que tenemos conectada una pantalla de 16x2

  lcd.createChar (0, smile);
  lcd.createChar (1, sun);
  lcd.createChar (2, rainy);
  lcd.createChar (3, cloud);
  lcd.createChar (4, sunHalf);
  lcd.createChar (5, sun1);
  lcd.createChar (6,drop);
  lcd.createChar (7,crack);

 //Dibujo del inicio
  lcd.home ();// Mover el cursor a la primera posición de la pantalla (0, 0)
  lcd.print("Bienvenido");// Imprimir "Bienvenido" en la primera linea
  
  lcd.setCursor ( 0, 1 );// Mover el cursor a la segunda linea (1) primera columna
  lcd.print("Usuario");// Imprimir otra cadena en esta posicion
  lcd.setCursor (8,1);
  lcd.write((byte)0);

  //gota 
  lcd.setCursor (9,1);
  lcd.write((byte)6);
  
//Dibujo del sol posicion y luego lo escribo
  lcd.setCursor (10,1);
  lcd.write((byte)1);
  
 //LLueve. posicion y luego lo escribo
  lcd.setCursor (12,1);
  lcd.write((byte)2);

  lcd.setCursor (13,1);
  lcd.write((byte)3);

  lcd.setCursor (14,1);
  lcd.write((byte)4);

  lcd.setCursor (15,1);
  lcd.write((byte)5);
  
  // Esperar 4 segundos
  delay(4000);
  lcd.clear();

  
  pinMode (pinLED, OUTPUT);
  pinMode (pinMicrofono, INPUT);
  pinMode (pinBoton1, INPUT);
  pinMode (pinBoton2, INPUT);
  
  Serial.begin(9600);
}

void loop()
{ 

Serial.println(motor);
   
  //Actualizar variables
  motor = analogRead(A2);
  motor = motor * 0.2491;
  int chk = int(DHT.read11(DHT11_PIN));
  agua = analogRead(A1);
  detectarSonido = digitalRead(pinMicrofono);
  sol =analogRead(A0);
  
  //REVISAR EL ESTADO DEL BOTON
estadoBotonIncremento = digitalRead(pinBoton1);
if(estadoBotonIncremento !=estadoBotonIncrementoAnterior){
  if(antirebote(pinBoton1)){
    ++canal;
    if(canal >7)canal=0;
    }
  }
  estadoBotonIncrementoAnterior = estadoBotonIncremento;
 
 //Funcion para disminuir el canal 
estadoBotonDecremento = digitalRead(pinBoton2);
if(estadoBotonDecremento !=estadoBotonDecrementoAnterior){
  if(antirebote(pinBoton2)){
    --canal;
    if(canal <0)canal=7;
    }
  }
  estadoBotonDecrementoAnterior = estadoBotonDecremento;
  




switch (canal) {
  case 0:
  //LIMPIEZA DE DATOS ANTERIORES
  lcd.clear();
     //Imprimir resultados Primera Fila
        lcd.home();
        lcd.print("Wa ");// Imprimir el agua arriba  ala izquierda
        lcd.setCursor(3,0);
       /* if(agua>300){
          lcd.write(((byte)6));
          }
          else{
            lcd.write(47);
            }
            agua=analogRead(A1);*/
        lcd.write((agua>300)?((byte)6):(47));
        
        //Luz
        lcd.setCursor(5,0);
        lcd.print("L ");
        lcd.setCursor(7,0);
        lcd.write((sol>700)?((byte)5):((sol<250)?((byte)3):((byte)4)));//muestra la luz ahora
      
        //Presion
        lcd.setCursor(9,0);
        lcd.print("P ");
        lcd.print(bmp.readPressure());
        lcd.print("mB");
      
     //Segunda Fila
        //Sonido
        lcd.setCursor(0,1);
        lcd.print("S ");
        lcd.setCursor(2,1);
        lcd.write(detectarSonido?((byte)7):(47)); //muestra si el sonido supera el especificado o no
      
        //Humedad
        lcd.setCursor(4,1);
        lcd.print("H ");
        lcd.setCursor(6,1);
        lcd.print(DHT.humidity);
        lcd.setCursor(8,1);
        lcd.write(37);
      
        //TªC
         lcd.setCursor(8,1);
         lcd.print(" T*");
         lcd.print(bmp.readTemperature());
    break;
      case 1:
        //Agua
         lcd.clear();
         lcd.home();
         lcd.print("Valor Agua:");
         lcd.setCursor(1,1);
         lcd.print(agua);
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
     case 2:
        // Temperatura
         lcd.clear();
         lcd.home();
         lcd.print("TEMPERATURA :");
         lcd.setCursor(3,1);
         lcd.print(bmp.readTemperature());
         lcd.print(" C");
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
    case 3:
    //HUMEDAD
         lcd.clear();
         lcd.home();
         lcd.print("HUMEDAD RELATIVA");
         lcd.setCursor(3,1);
        lcd.print(int(DHT.humidity));
        lcd.setCursor(5,1);
        lcd.print("% ");
        lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/6");
        
    break;
    case 4:
        //PRESION
        lcd.clear();
         lcd.home();
         lcd.print(" PRESION:");
         lcd.setCursor(0,1);
        lcd.print(bmp.readPressure());
        lcd.print(" PA");
        lcd.setCursor(12,0);
         lcd.print(canal);
         lcd.setCursor(13,0);
         lcd.print("/7");
    break;
    case 5:
    //LUZ
         lcd.clear();
         lcd.home();
         lcd.print("LUMINOSIDAD:");
         lcd.setCursor(3,1);
         lcd.print(sol);
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
    case 6:
    //Motor
         lcd.clear();
         lcd.home();
         lcd.print(" Viento ");
         lcd.setCursor(1,1);
         lcd.print(motor);
         lcd.setCursor(4,1);
         lcd.print("KM/H");
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
    case 7:
    lcd.clear();
         lcd.home();
      //Parte METEOROLOGICO
      
      if(sol>700){
        lcd.print("Hoy hace sol");
        }
      else if(sol>500){
          if (agua>400){
            lcd.print("Se vera el arco");
            lcd.setCursor(0,1);
            lcd.print("iris");
            }
          else lcd.print("Hay nubes y claros");
      }
      else{
        if (agua>400){
          if(detectarSonido==HIGH)lcd.print("Hay tormenta");
          else lcd.print("LLueve fuerte");
        }
        else lcd.print("Nublado");
      }  
    lcd.setCursor(12,1);
    lcd.print(canal);
    lcd.setCursor(13,1);
    lcd.print("/7");
    break;
  default:
    lcd.clear();
         lcd.home();
         lcd.print("   TE SALISTE   ");
         
    break;
}


  delay(1000);
}
