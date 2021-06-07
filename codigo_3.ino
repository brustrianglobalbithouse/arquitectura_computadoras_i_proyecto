#include <AFMotor.h>

#define left A0                               //habilita puerto analogo cero y lo nombra
#define right A1                              //habilita puerto analogo uno y lo nombra

int estado = 0;                               //guarda el estado del boton
int estadoAnterior = 0;                       //guarda el estado anterior del boton
unsigned long lastDebounceTime = 0;           //ultimo tiempo en que fue presionado el boton
unsigned long debounceDelay = 150;             // tiempo de reaccion al presionar el boton
int pushbtn = 22;                             //variable para push button

#define S0 30                                 //definiendo puertos para sensor de color
#define S1 31
#define S2 32
#define S3 33
#define sensorSalida 34
int Rojo_Frec = 0;                            //variables para sensor de color
int Verde_Frec = 0;
int Azul_Frec = 0;
bool colorLeido = false;

int buzzer = 48;                              //nombrando puerto para buzzer, creando variable para el sensor de flama
int valorSensorFlama = 0;
bool enLlamas = false;
unsigned long lastFlamaTime = 0;           //ultimo tiempo en que fue presionado el boton

AF_DCMotor motor1(1, MOTOR12_1KHZ);           //nombra y habilita las salidas de los motores
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

void setup() {
  motor1.setSpeed(90);                        //da la velocidad de los motores en rpm
  motor2.setSpeed(90);
  motor3.setSpeed(90);
  motor4.setSpeed(90);
  
  pinMode(left,INPUT);                        //declara los puertos como entradas y salidas
  pinMode(right,INPUT);                        
  pinMode(pushbtn,INPUT);

  pinMode(S0, OUTPUT);                         //declarando entradas y salidas para el sensor de color
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorSalida, INPUT);
                           
  digitalWrite(S0,HIGH);                     // Configura la escala de Frecuencia en 20%
  digitalWrite(S1,LOW);

  pinMode(buzzer,OUTPUT);                    //declarando el puerto buzzer como salida

  Serial.begin(9600);
}

void moverMotores(int m1, int m2, int m3, int m4){
  motor1.run(m1);
  motor2.run(m2);
  motor3.run(m3);
  motor4.run(m4);
}

void setVelocidades(int v1, int v2, int v3, int v4){
  motor1.setSpeed(v1);
  motor2.setSpeed(v2);
  motor3.setSpeed(v3);
  motor4.setSpeed(v4);
}

void moverSobreLinea(){                         //funcion con las condiciones para los sensores infrarrojos
  
  if(!enLlamas){
    int leftRead = analogRead(left);
    int rightRead = analogRead(right);
  
    if(leftRead > 400 && rightRead > 400){               //movimiento hacia el frente (no detecta obstaculo en sensores)
      moverMotores(FORWARD, FORWARD, FORWARD, FORWARD);
      setVelocidades(90, 90, 90, 90);
    } else if(leftRead <= 400 && rightRead > 400){       //movimiento hacia la derecha (detecta obstaculo en sensor izquierdo)
      moverMotores(BACKWARD, FORWARD, FORWARD, BACKWARD);
      setVelocidades(200, 200, 200, 200);
    } else if(leftRead > 400 && rightRead <= 400){       //movimiento hacia la izquierda (detecta obstaculo en sensor derecho)
      moverMotores(FORWARD, BACKWARD, BACKWARD, FORWARD);
      setVelocidades(200, 200, 200, 200);
    } else if(leftRead <= 400 && rightRead <= 400){        //detiene los motores (detecta obstaculo en ambos sensores)
      moverMotores(RELEASE, RELEASE, RELEASE, RELEASE);
  
      colorLeido = false;
      estado = 0;
      estadoAnterior = 0;
    }
  }else{
    moverMotores(RELEASE, RELEASE, RELEASE, RELEASE);
  }
   
}

void readPushButton(){
  //lee el estado del botón
  int lectura = digitalRead(pushbtn);
  if(millis() - lastDebounceTime > debounceDelay){
    if(lectura==HIGH && estadoAnterior==LOW){
      estado=1-estado;
    }
    lastDebounceTime = millis();
    estadoAnterior = lectura;
  }
}

void leerColor(){
  // Configura el filtor ROJO para tomar lectura
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  Rojo_Frec= pulseIn(sensorSalida, LOW);
  delay(50);
  
  // Configura el filtor VERDE para tomar lectura
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  Verde_Frec = pulseIn(sensorSalida, LOW);
  delay(50);
  
  // Configura el filtor AZUL para tomar lectura
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  Azul_Frec = pulseIn(sensorSalida, LOW);
  delay(50);

  if(Rojo_Frec <= 120 && Verde_Frec <= 120 && Azul_Frec <= 120){
    if(Rojo_Frec < Verde_Frec && Rojo_Frec < Azul_Frec){
      //Mover carro hacia izquierda
      moverMotores(FORWARD, BACKWARD, BACKWARD, FORWARD);
      setVelocidades(200, 200, 200, 200);
      delay(350);
    }else if(Verde_Frec < Rojo_Frec && Verde_Frec < Azul_Frec){
      //Mover carro hacia el frente
    }else if(Azul_Frec < Rojo_Frec && Azul_Frec < Verde_Frec){
      //Mover carro hacia derecha
      moverMotores(BACKWARD, FORWARD, FORWARD, BACKWARD);
      setVelocidades(200, 200, 200, 200);
      delay(350);
    }
    colorLeido = true;
  }else{
    estado = 0;
    estadoAnterior = 0;
  }

}

void checkSensorFlama(){
  valorSensorFlama=analogRead(A8);
  Serial.println(valorSensorFlama);

  if(valorSensorFlama < 500){
    if(millis() - lastFlamaTime >= 250){
      digitalWrite(buzzer, HIGH);
      enLlamas = true;
    }
    if(millis() - lastFlamaTime >= 1250){
      digitalWrite(buzzer, LOW);
      lastFlamaTime = millis();
    }
  }else{
    digitalWrite(buzzer, LOW);
    enLlamas = false;
  }
}

void loop() {
  
  readPushButton();
  checkSensorFlama();

  if(estado == 1){
    if(colorLeido == false){
      leerColor();
    }else{
      moverSobreLinea();
    }
  }
}
