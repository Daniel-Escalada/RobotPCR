//Librerías

#include <Servo.h>
#include <UltrasonicSensor.h>

//Defines con los pines de la placa
//Cambair los pines para que sean ~ 

#define pin_sensorCodo A0  //Pin sensor del codo
#define pin_servoCodo 1    //Pin servo del codo
#define pin_sensorBase A1  //Pin sensor de la base
#define pin_motorBase_1 2  //Pin salida 1 del motor de la base
#define pin_motorBase_2 3  //Pin salida 2 del motor de la base
#define pin_servoMuneca 4    //Pin servo de la muñeca
#define pin_servoPinza 5    //Pin servo de la pinza
#define pin_Ultra_Trigger 6   //Pin Trigger del ultrasonido
#define pin_Ultra_Echo  7  //Pin Echo del ultrasonido
#define pin_EnableRobot         //Pin botón de enable 
#define pin_Enable_motorBase //Pin enable del motor 1



//Estados

enum Estado
{
    E_Base,          //Posición de reposo que espera el boton de enable
    E_Inicial,       //Posición inicial, pinza abierta
    E_Aposito,       //Pinza alineada con el apósito (rotación codo)
    E_CogerAposito,  //Cierre de la pinza (mov. sin realimentación
    E_Colocar,       //Posición en el ángulo de la entrada (rotación codo)
    E_Acercar,       //Acercamiento a la entrada (traslación)
    E_Rotar,         //Rotación del apósito (rotación muñeca)
    E_Alejar,        //Alejamiento de la entrada (traslación)
    E_Vuelta,        //Vuelta a la posición del apósito (rotación codo)
    E_SoltarAposito  //Apertura de la pinza (mov. sin realimentar)
  //E_VueltaInicial  //Vuelta al estado inicial, no sé si hay que hacer este estado o por cinemática se puede evitar
};

//Actuadores

Servo servoCodo, servoMuneca, servoPinza;  

//Variables globales

Estado estadoActual;

////PID DC

double kp_DC = 9;
double kd_DC = 0;
double ki_DC = 0;

double Input_DC, Output_DC, Setpoint_DC;
double salida_DC;
unsigned long tiempoActual_DC, tiempoAnterior_DC;
double tiempoTranscurrido_DC;
double error_DC, errorAnterior_DC, integralError_DC, derivadaError_DC;

///PID SERVO

double kp_SERVO = 0.5;
double kd_SERVO = 0;
double ki_SERVO = 0;

double Input_SERVO, Output_SERVO, Setpoint_SERVO;
double salida_SERVO;
unsigned long tiempoActual_SERVO, tiempoAnterior_SERVO;
double tiempoTranscurrido_SERVO;
double error_SERVO, errorAnterior_SERVO, integralError_SERVO, derivadaError_SERVO;

////Sensores

UltrasonicSensor ultrasonico(pin_Ultra_Trigger, pin_Ultra_Echo);

int raw_sensorCodo;  //Lectura sin procesa del sensor del codo
int raw_sensorBase;  //Lectura sin procesar del sensor de la base
int distancia_Ultra;  //Lectura del ultrasonidos en mm



//Llamada a transiciones de estado 


void Estado_Inicial()
{
    if ()                                           //Si se encuenta en la posición inicial (sensor codo) 
        cambiarEstado(Estado::E_Base);
}

void Estado_Base()
{
    if (digitalRead(pin_EnableRobot))                                           //Si se pulsa el botón enable
        cambiarEstado(Estado::E_Aposito);
}

void Estado_Aposito()
{
    if (1)                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_CogerAposito);
}

void Estado_CogerAposito()
{
    if (1)                                           //Si la pinza se ha cerrado, este mov. no está realimentado así que habrá que poner un delay y ya
        cambiarEstado(Estado::E_Colocar);
}

void Estado_Colocar()
{
    if (1)                                           //Si se ha colocado en el ángulo de entrada (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_Acercar);
}

void Estado_Acercar()
{
    if (1)                                           //Si se acercado hasta la entrada (sensor base (sensor codo redundante)) 
        cambiarEstado(Estado::E_Rotar);
}

void Estado_Rotar()
{
    if (1)                                           //Si la muñeca ha rotado, este mov. no esta realimentado así que habrá que poner un delay y ya 
        cambiarEstado(Estado::E_Alejar);
}

void Estado_Alejar()
{
    if (1)                                           //Si se ha alejado de la entrada hasta la posición inicial (sensor base (sensor codo redundante)) 
        cambiarEstado(Estado::E_Vuelta);
}

void Estado_Vuelta()
{
    if (1)                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_SoltarAposito);
}

void Estado_SoltarAposito()
{
    if (1)                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_Inicial);           //Esto depende si se añade nuevo estado    
}

//Salidas de cada estado (acciones)

void Salida_Inicial()
{
    //Mover a la posición inicial (servo codo, motores base y servo muñeca) y abrir pinza (servo pinza)
    //Codo
    Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
    Setpoint_SERVO = 90;                                      //Posición inicial
    salida_SERVO = 0;
    int count = 0;
    while(1) {
        Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
        Output_SERVO = calcPID_SERVO(Input_SERVO);

        if (not isnan(Output_SERVO)) salida_SERVO = salida_SERVO + Output_SERVO;  //La primera iteración output es "nan"

        servoCodo.write(salida_SERVO);             

        Output_SERVO = raw_sensorCodo;

        if (Output_SERVO == 0) count++;
        if (count > 20) break;

        delay(100);
    }
 
}

void Salida_Base()
{
    //No hace nada, es un estado transitorio
}

void Salida_Aposito()
{
    //Mover a la posición del apósito (servo codo)

    Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
    Setpoint_SERVO = 180;                                      //Posición aposito
    salida_SERVO = 0;
    int count = 0;
    while (1) {
        Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
        Output_SERVO = calcPID_SERVO(Input_SERVO);

        if (not isnan(Output_SERVO)) salida_SERVO = salida_SERVO + Output_SERVO;  //La primera iteración output es "nan"

        servoCodo.write(salida_SERVO);

        Output_SERVO = raw_sensorCodo;

        if (Output_SERVO == 0) count++;
        if (count > 20) break;

        delay(100);
    }
}

void Salida_CogerAposito()
{
    //Cerrar pinza (servo pinza) 
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)

    servoPinza.write(0);
    delay(5000);
    
}

void Salida_Colocar()
{
    //Mover al ángulo de entrada (servo codo)

    Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
    Setpoint_SERVO = 0;                                      //Posición entrada
    salida_SERVO = 0;
    int count = 0;
    while (1) {
        Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
        Output_SERVO = calcPID_SERVO(Input_SERVO);

        if (not isnan(Output_SERVO)) salida_SERVO = salida_SERVO + Output_SERVO;  //La primera iteración output es "nan"

        servoCodo.write(salida_SERVO);

        Output_SERVO = raw_sensorCodo;

        if (Output_SERVO == 0) count++;
        if (count > 20) break;

        delay(100);
    }
}

void Salida_Acercar()
{
    //Mover a la posición de entrada (motores base)

    Setpoint_DC = 950;                            //Se elige la salida deseada (0-1023)
    int count = 0;
    
while(1) {

    Input_DC = raw_sensorBase;                     //entrada del sistema (0-1023)
    Output_DC = calcPID_DC(Input_DC);


    if (Input_DC < 100 or Input_DC > 3000) {        //Límites de seguridad 
      
        digitalWrite(pin_Enable_motorBase, HIGH); //Si pasa los límites se apaga el motor 
        digitalWrite(pin_motorBase_1, LOW);
        digitalWrite(pin_motorBase_2, LOW);
    }
    else {
        if (Output_DC < 0) {                                  //Salida del pid negativa (hacia un lado)
            salida_DC = map(abs(Output_DC), 0, kp_DC * 1023, 50, 255);       //Pasar la salida a valores de pwm (de 0 al valor máximo, el 50 es para que no vaya demasiado lento)                        
            analogWrite(pin_Enable_motorBase, abs(salida));                     

            digitalWrite(pin_motorBase_1, HIGH);
            digitalWrite(pin_motorBase_2, LOW);

        }
        if (Output_DC > 0) {                          //Salida del controlador
            salida_DC = map(Output_DC, 0, kp_DC * 1023, 50, 255);   //Función map que adecua los posibles valores de 
                                                     //salida del sensor a un ciclo de trabajo de 25%-100% del motor                  
            analogWrite(pin_Enable_motorBase, abs(salida_DC));             //Señal PWM enviada al motor

            digitalWrite(pin_motorBase_1 LOW);                   //Señales que controlan el sentido de rotación
            digitalWrite(pin_motorBase_2, HIGH);
        }
        if (Output == 0) {
            digitalWrite(pin_Enable_motorBase, HIGH);       //Si el error es cero no se mueve 

            digitalWrite(pin_motorBase_1, LOW);
            digitalWrite(pin_motorBase_2, LOW);
        }
        if (Output_DC == 0) count++;
        if (count > 20) break;
    }
}

void Salida_Rotar()
{
    //Rotar muñeca (servo muñeca)
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)

    for (int i = 0; i < 3; i++) {    //Hace 3 movimientos de rotacion
        servoMuneca.write(90);
        delay(500);
        servoMuneca.write(135);
        delay(500);
        servoMuneca.write(45);
        delay(500);
        servoMuneca.write(90);
    }
    delay(1000);
}

void Salida_Alejar()
{
    //Alejar de la posición de entrada (motores base)

    Setpoint_DC = 200;                            //Se elige la salida deseada (0-1023)
    int count = 0;

    while (1) {

        Input_DC = raw_sensorBase;                     //entrada del sistema (0-1023)
        Output_DC = calcPID_DC(Input_DC);

        if (Input_DC < 100 or Input_DC > 3000) {        //Límites de seguridad 

            digitalWrite(pin_Enable_motorBase, HIGH); //Si pasa los límites se apaga el motor 
            digitalWrite(pin_motorBase_1, LOW);
            digitalWrite(pin_motorBase_2, LOW);
        }
        else {
            if (Output_DC < 0) {                                  //Salida del pid negativa (hacia un lado)
                salida_DC = map(abs(Output_DC), 0, kp_DC * 1023, 50, 255);       //Pasar la salida a valores de pwm (de 0 al valor máximo, el 50 es para que no vaya demasiado lento)                        
                analogWrite(pin_Enable_motorBase, abs(salida));

                digitalWrite(pin_motorBase_1, HIGH);
                digitalWrite(pin_motorBase_2, LOW);

            }
            if (Output_DC > 0) {                          //Salida del controlador
                salida_DC = map(Output_DC, 0, kp_DC * 1023, 50, 255);   //Función map que adecua los posibles valores de 
                                                         //salida del sensor a un ciclo de trabajo de 25%-100% del motor                  
                analogWrite(pin_Enable_motorBase, abs(salida_DC));             //Señal PWM enviada al motor

                digitalWrite(pin_motorBase_1 LOW);                   //Señales que controlan el sentido de rotación
                digitalWrite(pin_motorBase_2, HIGH);
            }
            if (Output == 0) {
                digitalWrite(pin_Enable_motorBase, HIGH);       //Si el error es cero no se mueve 

                digitalWrite(pin_motorBase_1, LOW);
                digitalWrite(pin_motorBase_2, LOW);
            }
        }
        if (Output_DC == 0) count++;
        if (count > 20) break;
    }
}

void Salida_Vuelta()
{
    //Mover a la posición del apósito (servo codo)

    Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
    Setpoint_SERVO = 180;                                      //Posición aposito
    salida_SERVO = 0;
    int count = 0;

    while (1) {
        Input_SERVO = map(raw_sensorCodo, 61, 602, 0, 180);
        Output_SERVO = calcPID_SERVO(Input_SERVO);

        if (not isnan(Output_SERVO)) salida_SERVO = salida_SERVO + Output_SERVO;  //La primera iteración output es "nan"

        servoCodo.write(salida_SERVO);

        Output_SERVO = raw_sensorCodo;

        if (Output_SERVO == 0) count++;
        if (count > 20) break;

        delay(100);
    }
}

void Salida_SoltarAposito()
{
    //Abrir pinza (servo pinza) 
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)    

    servoPinza.write(180);
    delay(5000);
}

void setup()
{
    //Asociar pines a servos
    servoCodo.attach(pin_servoCodo); 
    servoMuneca.attach(pin_servoMuneca);
    servoPinza.attach(pin_servoPinza);

    pinMode(pin_EnableRobot, INPUT);    

    Serial.begin(9600);              //Consola
    estadoActual = E_Inicial;        //Estado inicial
    Salida_Inicial();                //Llevar a estado inicial
}

void loop()
{
    leerEntrada();                   //Función que lee las entradas (sensores) constantemente
    actualizarEstado();              //Función que actualiza los estados (no los cambia)

    Serial.print("Distancia: ");
    Serial.print(distancia_Ultra);
    Serial.println(" mm");
}

//Función que actualiza los estados (no los cambia)
void actualizarEstado()
{
    switch (estadoActual)
    {
    case E_Inicial: Estado_Inicial(); break;
    case E_Base: Estado_Base(); break;
    case E_Aposito: Estado_Aposito(); break;
    case E_CogerAposito: Estado_CogerAposito(); break;
    case E_Colocar: Estado_Colocar(); break;
    case E_Acercar: Estado_Acercar(); break;
    case E_Rotar: Estado_Rotar(); break;
    case E_Alejar: Estado_Alejar(); break;
    case E_Vuelta: Estado_Vuelta(); break;
    case E_SoltarAposito: Estado_SoltarAposito(); break;
 
    }
}

//Función que lee todas las entradas (sensores) y actualiza las variables globales
void leerEntrada()
{

    raw_sensorCodo = analogRead(pin_sensorCodo);  //Guarda la lectura del sensor del codo sin pasar a grados
    raw_sensorBase = analogRead(pin_sensorBase);  //Guarda la lectura del sensor de la base sin pasar a grados
    distancia_Ultra = ultrasonico.distanceInMillimeters();

}

//Funcion que cambia el estado y llama a las salidas

void cambiarEstado (Estado estadoNuevo)
{
    estadoActual = estadoNuevo;

    switch (estadoActual)
    {
    case E_Inicial: Salida_Inicial(); break;
    case E_Base: Salida_Base(); break;
    case E_Aposito: Salida_Aposito(); break;
    case E_CogerAposito: Salida_CogerAposito(); break;
    case E_Colocar: Salida_Colocar(); break;
    case E_Acercar: Salida_Acercar(); break;
    case E_Rotar: Salida_Rotar(); break;
    case E_Alejar: Salida_Alejar(); break;
    case E_Vuelta: Salida_Vuelta(); break;
    case E_SoltarAposito: Salida_SoltarAposito(); break;
    default: break;
    }
}
double calcPID_DC(double inp) {

    tiempoActual_DC = millis();                                          //Obtiene el tiempo actual con la función millis
    tiempoTranscurrido_DC = (double)(tiempoActual_DC - tiempoAnterior_DC);     //Calcula el tiempo transcurrido

    error_DC = Setpoint_DC - inp;                                           //Calcula el error entre la referencia y la realimentación
    integralError_DC += error_DC * tiempoTranscurrido_DC;                      //Calcular la integral del error
    derivadaError_DC = (error_DC - errorAnterior_DC) / tiempoTranscurrido_DC;     //Calcular la derivada del error

    double output = kp_DC * error_DC + ki_DC * integralError_DC + kd_DC * derivadaError_DC;   //Salida del PID

    if (abs(output) < 10) output = 0;     //Para quitar cambios si está cerca de 0
    
    errorAnterior_DC = error_DC;                                            //Guarda error anterior
    tiempoAnterior_DC = tiempoActual_DC;                                    //Guarda el tiempo anterior

    return output;
}

double calcPID_SERVO(double inp) {

    tiempoActual_SERVO = millis();                                          //Obtiene el tiempo actual con la función millis
    tiempoTranscurrido_SERVO = (double)(tiempoActual_SERVO - tiempoAnterior_SERVO);     //Calcula el tiempo transcurrido

    error_SERVO = Setpoint_SERVO - inp;                                           //Calcula el error entre la referencia y la realimentación
    integralError_SERVO += error_SERVO * tiempoTranscurrido_SERVO;                      //Calcular la integral del error
    derivadaError_SERVO = (error_SERVO - errorAnterior_SERVO) / tiempoTranscurrido_SERVO;     //Calcular la derivada del error

    double output = kp_SERVO * error_SERVO + ki_SERVO * integralError_SERVO + kd_SERVO * derivadaError_SERVO;   //Salida del PID

    if (abs(output) < 10) output = 0;     //Para quitar cambios si está cerca de 0

    errorAnterior_SERVO = error_SERVO;                                            //Guarda error anterior
    tiempoAnterior_SERVO = tiempoActual_SERVO;                                    //Guarda el tiempo anterior

    return output;
}

