//Librerías

#include <Servo.h>

//Defines con los pines de la placa

#define pin_sensorCodo A0  //Pin sensor del codo
#define pin_servoCodo 1    //Pin servo del codo
#define pin_sensorBase A1  //Pin sensor de la base
#define pin_motorBase_1 2  //Pin salida 1 del motor de la base
#define pin_motorBase_2 3  //Pin salida 2 del motor de la base
#define pin_servoMuneca 4    //Pin servo de la muñeca
#define pin_servoPinza 5    //Pin servo de la pinza


//Estados

enum Estado
{
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

////Sensores

int raw_sensorCodo;  //Lectura sin procesa del sensor del codo
int raw_sensorBase;  //Lectura sin procesar del sensor de la base



//Llamada a transiciones de estado 

void Estado_Inicial()
{
    if ()                                           //Si se encuenta en la posición inicial (sensor codo y sensor base) 
        cambiarEstado(Estado::E_Aposito);
}

void Estado_Aposito()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_CogerAposito);
}

void Estado_CogerAposito()
{
    if ()                                           //Si la pinza se ha cerrado, este mov. no está realimentado así que habrá que poner un delay y ya
        cambiarEstado(Estado::E_Colocar);
}

void Estado_Colocar()
{
    if ()                                           //Si se ha colocado en el ángulo de entrada (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_Acercar);
}

void Estado_Acercar()
{
    if ()                                           //Si se acercado hasta la entrada (sensor base (sensor codo redundante)) 
        cambiarEstado(Estado::E_Rotar);
}

void Estado_Rotar()
{
    if ()                                           //Si la muñeca ha rotado, este mov. no esta realimentado así que habrá que poner un delay y ya 
        cambiarEstado(Estado::E_Alejar);
}

void Estado_Alejar()
{
    if ()                                           //Si se ha alejado de la entrada hasta la posición inicial (sensor base (sensor codo redundante)) 
        cambiarEstado(Estado::E_Vuelta);
}

void Estado_Vuelta()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_SoltarAposito);
}

void Estado_SoltarAposito()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        cambiarEstado(Estado::E_Inicial);        //Esto depende si se añade nuevo estado    
}

//Salidas de cada estado (acciones)

void Salida_Inicial()
{
    //Mover a la posición inicial (servo codo, motores base y servo muñeca) y abrir pinza (servo pinza)
}

void Salida_Aposito()
{
    //Mover a la posición del apósito (servo codo)

    servoCodo.write();
}

void Salida_CogerAposito()
{
    //Cerrar pinza (servo pinza) 
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)

    servoPinza.write();
    delay(5000);
    
}

void Salida_Colocar()
{
    //Mover al ángulo de entrada (servo codo)

    servoCodo.write();
}

void Salida_Acercar()
{
    //Mover a la posición de entrada (motores base)

    //Una forma: mover el motor hasta que el sensor detecte que está en posición

    do {

    } while ();
}

void Salida_Rotar()
{
    //Rotar muñeca (servo muñeca)
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)

    for (int i = 0; i < 3; i++) {    //Hace 3 movimientos de rotacion
        servoMuneca.write(90);
        servoMuneca.write(0);
        servoMuneca.write(180);
        servoMuneca.write(90);
    }
    delay(10000);
}

void Salida_Alejar()
{
    //Alejar de la posición de entrada (motores base)

    //Una forma: mover el motor hasta que el sensor detecte que está en posición

    do {

    } while ();
}

void Salida_Vuelta()
{
    //Mover a la posición del apósito (servo codo)

    servoCodo.write();
}

void Salida_SoltarAposito()
{
    //Abrir pinza (servo pinza) 
    //Posible delay para asegurarse que le da tiempo a realizar acción (no está realimentado)    

    servoPinza.write();
    delay(5000);
}

void setup()
{
    //Asociar pines a servos
    servoCodo.attach(pin_servoCodo); 
    servoMuneca.attach(pin_servoMuneca);
    servoPinza.attach(pin_servoPinza);

    Serial.begin(9600);              //Consola
    estadoActual = E_Inicial;        //Estado inicial
    Salida_Inicial();                //Llevar a estado inicial
}

void loop()
{
    leerEntrada();                   //Función que lee las entradas (sensores) constantemente
    actualizarEstado();              //Función que actualiza los estados (no los cambia)
}

//Función que actualiza los estados (no los cambia)
void actualizarEstado()
{
    switch (estadoActual)
    {
    case E_Inicial: Estado_Inicial(); break;
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

}

//Funcion que cambia el estado y llama a las salidas

void cambiarEstado (Estado estadoNuevo)
{
    estadoActual = estadoNuevo;

    switch (estadoActual)
    {
    case E_Inicial: Salida_Inicial(); break;
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
