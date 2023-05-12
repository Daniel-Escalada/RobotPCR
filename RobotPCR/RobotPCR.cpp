enum Estado
{
    E_Inicial,       //Posici�n inicial, pinza abierta
    E_Aposito,       //Pinza alineada con el ap�sito (rotaci�n codo)
    E_CogerAposito,  //Cierre de la pinza (mov. sin realimentaci�n
    E_Colocar,       //Posici�n en el �ngulo de la entrada (rotaci�n codo)
    E_Acercar,       //Acercamiento a la entrada (traslaci�n)
    E_Rotar,         //Rotaci�n del ap�sito (rotaci�n mu�eca)
    E_Alejar,        //Alejamiento de la entrada (traslaci�n)
    E_Vuelta,        //Vuelta a la posici�n del ap�sito (rotaci�n codo)
    E_SoltarAposito  //Apertura de la pinza (mov. sin realimentar)
  //E_VueltaInicial  //Vuelta al estado inicial, no s� si hay que hacer este estado o por cinem�tica vale con el inicial
};

//Variables globales

Estado estadoActual;

//Llamada a transiciones de estado 

void Estado_Inicial()
{
    if ()                                           //Si se encuenta en la posici�n inicial (sensor codo y sensor base) 
        actualizarEstado(Estado::E_Aposito);
}

void Estado_Aposito()
{
    if ()                                           //Si se encuenta en la posici�n del ap�sito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_CogerAposito);
}

void Estado_CogerAposito()
{
    if ()                                           //Si la pinza se ha cerrado, este mov. no est� realimentado as� que habr� que poner un delay y ya
        actualizarEstado(Estado::E_Colocar);
}

void Estado_Colocar()
{
    if ()                                           //Si se ha colocado en el �ngulo de entrada (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_Acercar);
}

void Estado_Acercar()
{
    if ()                                           //Si se acercado hasta la entrada (sensor base (sensor codo redundante)) 
        actualizarEstado(Estado::E_Rotar);
}

void Estado_Rotar()
{
    if ()                                           //Si la mu�eca ha rotado, este mov. no esta realimentado as� que habr� que poner un delay y ya 
        actualizarEstado(Estado::E_Alejar);
}

void Estado_Alejar()
{
    if ()                                           //Si se ha alejado de la entrada hasta la posici�n inicial (sensor base (sensor codo redundante)) 
        actualizarEstado(Estado::E_Vuelta);
}

void Estado_Vuelta()
{
    if ()                                           //Si se encuenta en la posici�n del ap�sito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_SoltarAposito);
}

void Estado_SoltarAposito()
{
    if ()                                           //Si se encuenta en la posici�n del ap�sito (sensor codo (sensor base redundante)) 
        actualizarEstado(Estado::E_Inicial);         //Esto depende si se a�ade nuevo estado    
}
