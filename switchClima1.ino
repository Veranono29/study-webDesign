//CREADO POR GERARDO ESCUDERO LÓPEZ PARA LA ASIGNATURA DE PROYECTOS I
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
float agua;// deetectar la cantidad de agua que detecta el sensor 
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
int estadoBotonDecremento; //utilizo dos para ver el cambio de un estado al otro al comparar sus estados
int estadoBotonDecrementoAnterior;

boolean antirebote(int pin) {
  int contador = 0;
  boolean estado;
  boolean estadoAnterior;

  do{
    estado =  digitalRead(pin);
    if(estado != estadoAnterior){ //Si ha cmabido el estado (HIGH/LOW)
      contador =0;                //La cuenta vuelve a empezar 
      estadoAnterior=estado;      //Toma este como el nuevo estado
      }
       else ++contador;           //asi hasta que deje de haber variaciones en el estado del boton en poco tiempo (Ruido/rebote)
  }
  while(contador<tiempoAntirrebote);//Importante no poner un tiempoAntirrebote alto pues en el tiempo que esté
  return estado; //confirmando el cambio no leera nada más(sin romper el ciclo).

}//Esto es en binario para crear cada array de bits y luego poder dibujar figuras en el lcd
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
  
  /* datos por defecto de la hoja de datos */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  lcd.init();                         //inicio del lcd
  lcd.backlight();                    //encender la luz trasera
  
  lcd.begin(16, 2);                   // Indicar a la libreria que tenemos conectada una pantalla de 16x2

  lcd.createChar (0, smile);          //Utiliza los caracteres vacios de la libreria y su tabla AscII  
  lcd.createChar (1, sun);            //para dejarnos las 7 primeras posiciones para asociarlas al array de bits que defini en el Setup
  lcd.createChar (2, rainy);
  lcd.createChar (3, cloud);
  lcd.createChar (4, sunHalf);
  lcd.createChar (5, sun1);
  lcd.createChar (6,drop);
  lcd.createChar (7,crack);

 //Dibujo del inicio
  lcd.home ();                          // Mover el cursor a la primera posición de la pantalla (0, 0)
  lcd.print("Bienvenido");              // Imprimir "Bienvenido" en la primera linea
  
  lcd.setCursor ( 0, 1 );               // Mover el cursor a la segunda linea (1) primera columna
  lcd.print("Usuario");                 // Imprimir otra cadena en esta posicion
  lcd.setCursor (8,1);
  lcd.write((byte)0);                   /*La diferencia entre lcd.write y lcd.print es que el print interpreta los datos antes de
                                          mandarselos al lcd.write que los escribe e la pantalla*/

  //gota 
  lcd.setCursor (9,1);
  lcd.write((byte)6);
  
//Dibujo del sol posicion y luego lo escribo
  lcd.setCursor (10,1);
  lcd.write((byte)1);
  
 //LLueve. posicion y luego lo escribo
  lcd.setCursor (12,1);
  lcd.write((byte)2);
  
  //demas dibujos de la pantalla de carga
  lcd.setCursor (13,1);
  lcd.write((byte)3);

  lcd.setCursor (14,1);
  lcd.write((byte)4);

  lcd.setCursor (15,1);
  lcd.write((byte)5);
  
  // Esperar 4 segundos
  delay(4000);
  lcd.clear(); //Limpio de escritura el lcd

    /*declaracion de los modos de pin del arduino, se declaran los pines digitales(que pueden PWM)
      los analoficos no tiene sentido pues son siempre de entrada*/
  pinMode (pinLED, OUTPUT);
  pinMode (pinMicrofono, INPUT);
  pinMode (pinBoton1, INPUT);
  pinMode (pinBoton2, INPUT);
  
  Serial.begin(9600);     //iniciar comunicacion serial con el arduino a 9600 gaudios.
}

void loop()
{ 

Serial.println(motor);    //escribo y cambio de linea el valor del ventilador
   
  //Actualizar variables
  motor = analogRead(A2);
  motor = motor * 0.2491; //este *0.2491 lo calculé con el coche y una tabla.
  int chk = int(DHT.read11(DHT11_PIN));
  agua = analogRead(A1);
  detectarSonido = digitalRead(pinMicrofono);      //Solo me interesa si supera cierto valor asi que puedo leerlo 
                                                  //como un digital y configurarlo con el potenciometro incorporado
  sol =analogRead(A0);
  
  //REVISAR EL ESTADO DEL BOTON
estadoBotonIncremento = digitalRead(pinBoton1);
if(estadoBotonIncremento !=estadoBotonIncrementoAnterior){ //si está pulsado el boton de aumentar
  if(antirebote(pinBoton1)){                               //y el antirebote lo confirma
    ++canal;                                               //cambia de canal
    if(canal >7)canal=0;                                   //si llegas al maximo de canales, vuelve al 0;
    }
  }
  estadoBotonIncrementoAnterior = estadoBotonIncremento;    //y pon el estado anterior como el actual(util si lo dejas pulsado y luego lo sueltas)
 
 //Funcion para disminuir el canal 
estadoBotonDecremento = digitalRead(pinBoton2);             //Igual que el anterior pero en el sentido contrario
if(estadoBotonDecremento !=estadoBotonDecrementoAnterior){
  if(antirebote(pinBoton2)){
    --canal;
    if(canal <0)canal=7;
    }
  }
  estadoBotonDecrementoAnterior = estadoBotonDecremento;
  




switch (canal) {                                            //una vez obtiene todos los datos, entra en un switch dependiendo del canal en el que se encuentre
  //canal general con todos los datos
  case 0:
  //LIMPIEZA DE DATOS ANTERIORES
  lcd.clear();
     //Imprimir resultados Primera Fila
        lcd.home();                                         //Cambia su posicion al 0,0 del lcd
        lcd.print("Wa ");                                   // Imprimir el agua arriba  ala izquierda
        lcd.setCursor(3,0);
        lcd.write((agua>300)?((byte)6):(47));               //Escribe el dibujo de una gota o la barra lateral en codigo ASCI
        
        //Luz
        lcd.setCursor(5,0);
        lcd.print("L ");
        lcd.setCursor(7,0);
        lcd.write((sol>700)?((byte)5):((sol<250)?((byte)3):((byte)4)));//muestra la luz ahora con 3 dibujos distintos
      
        //Escribe la presion en mB 
        lcd.setCursor(9,0);
        lcd.print("P ");
        lcd.print(bmp.readPressure());
        lcd.setCursor(14,0);                                //(podria dividir entre 100 para hPascales y conversiondirecta a mB,
        lcd.print("mB");                                    // pero si sobre escribo a partir del numero correcto me lo ahorro

      
    //Segunda Fila
        //Sonido
        lcd.setCursor(0,1);
        lcd.print("S ");
        lcd.setCursor(2,1);
        lcd.write(detectarSonido?((byte)7):(47)); //Se muestra si el sonido supera el especificado
      
        //Humedad
        lcd.setCursor(4,1);
        lcd.print("H ");
        lcd.setCursor(6,1);
        lcd.print(DHT.humidity);                  //leer la humedad
        lcd.setCursor(8,1);
        lcd.print("%");
      
        //TªC
         lcd.setCursor(9,1);
         lcd.print("T*");
         lcd.print(bmp.readTemperature());
    break;                                        //Salimos del caso de la pantlla general 
      case 1:                                     //Entramos en el caso leer solo agua
        //Agua
         lcd.clear();
         lcd.home();
         lcd.print("Valor Agua:");
         lcd.setCursor(1,1);
         lcd.print(agua);
        //Posicion en el canal, es repetido a lo largo del codigo
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
         //Posicion en el canal, es repetido a lo largo del codigo
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
        //Posicion en el canal, es repetido a lo largo del codigo
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
        
    break;
    case 4:
        //PRESION
        lcd.clear();
         lcd.home();
         lcd.print(" PRESION:");
         lcd.setCursor(0,1);
        lcd.print(bmp.readPressure());
        lcd.print(" PA");
       //Posicion en el canal, es repetido a lo largo del codigo
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
    case 5:
    //LUZ
         lcd.clear();
         lcd.home();
         lcd.print("LUMINOSIDAD:");
         lcd.setCursor(3,1);
         lcd.print(sol);
        //Posicion en el canal, es repetido a lo largo del codigo
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
       //Posicion en el canal, es repetido a lo largo del codigo
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
    case 7:   //Parte METEOROLOGICO
    lcd.clear();
         lcd.home();
      if(sol>700){                              //Si no hay una sola nuve, no puede llover
        lcd.print("Hoy hace sol");
        }
      else if(sol>500){                         //pero si hay sol y nubes si
          if (agua>400){
            lcd.print("Se vera el arco");
            lcd.setCursor(0,1);
            lcd.print("iris");
            }
          else lcd.print("Hay nubes y claros"); //Pero no obligatoriamente
      }
      else{                                                   //Por el contrario, si no se ve el sol
        if (agua>400){                                        //Hay agua
          if(detectarSonido==HIGH)lcd.print("Hay tormenta"); //Y el sensor de relampagos dice que asi es, pues hay tormenta electrica
          else lcd.print("LLueve fuerte");                    //Como las llaves son opcionales en elementos de una sola condicion, los quito por estetica
        }
        else lcd.print("Nublado");
      }  
    //Posicion en el canal, es repetido a lo largo del codigo
         lcd.setCursor(12,1);
         lcd.print(canal);
         lcd.setCursor(13,1);
         lcd.print("/7");
    break;
  default: //no llegará nunca a salir por la funcion de control pero por si acaso
    lcd.clear();
         lcd.home();
         lcd.print("   TE SALISTE   ");     //Se pueden usar espacios para "Borrar" contenido
         
    break;
}


  delay(1000);                              //ya que estoy borrando y escribinedo todo el rato, habria sensores que cambiarian demasiado rapido como para ver el numero
}
