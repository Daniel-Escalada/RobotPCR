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
  //E_VueltaInicial  //Vuelta al estado inicial, no sé si hay que hacer este estado o por cinemática vale con el inicial
};

//Variables globales

Estado estadoActual;

//Llamada a transiciones de estado 

void Estado_Inicial()
{
    if ()                                           //Si se encuenta en la posición inicial (sensor codo y sensor base) 
        actualizarEstado(Estado::E_Aposito);
}

void Estado_Aposito()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_CogerAposito);
}

void Estado_CogerAposito()
{
    if ()                                           //Si la pinza se ha cerrado, este mov. no está realimentado así que habrá que poner un delay y ya
        actualizarEstado(Estado::E_Colocar);
}

void Estado_Colocar()
{
    if ()                                           //Si se ha colocado en el ángulo de entrada (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_Acercar);
}

void Estado_Acercar()
{
    if ()                                           //Si se acercado hasta la entrada (sensor base (sensor codo redundante)) 
        actualizarEstado(Estado::E_Rotar);
}

void Estado_Rotar()
{
    if ()                                           //Si la muñeca ha rotado, este mov. no esta realimentado así que habrá que poner un delay y ya 
        actualizarEstado(Estado::E_Alejar);
}

void Estado_Alejar()
{
    if ()                                           //Si se ha alejado de la entrada hasta la posición inicial (sensor base (sensor codo redundante)) 
        actualizarEstado(Estado::E_Vuelta);
}

void Estado_Vuelta()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_SoltarAposito);
}

void Estado_SoltarAposito()
{
    if ()                                           //Si se encuenta en la posición del apósito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_Inicial);         //Esto depende si se añade nuevo estado    
}
