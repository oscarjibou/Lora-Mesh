#### Tipos de variables básicas en C++

```
- int
- unsigned
- long 
- byte
- float
- double 
- bool
- char (caracteres)
- const
- string
- uint8_t  == unsigned char
- uint16_t == unsigned short
- uint32_t == unsigned long
```

```C++
struct SeenPacket 
{
    uint8_t src;
    uint16_t seq;
};

/* 
Esto no declara una variable, solo dice crea un nuevo tipo de dato que se llama SeenPacket y tiene esos campos. 
*/

extern SeenPacket 
/* 
Aqui ya llamamos a la variable que esta declarada en otro archivo. Pero pero podríamos hacer un #define 
*/
```

Si usamos la variable `extern`, no definimos una variable sino que la declaramos. La definición de esta esta en otro archivo. 

Cuando usamos una variable con `static`, aunque este dentro de un loop(). - se inicializa solo una vez, la primera vez que se ejecuta la función
- mantiene su valor entre llamadas
- funciona como una variable global privada dentro de la función

```C++
def randomSeed():
'''
Inicializa el generador de números aleatorios con una semilla que depende de:
	1.	Ruido analógico del pin A0 (entropia)
	2.	El ID del nodo (MY_ID)
	3.	El tiempo en el que arrancó (millis())

No se almacena en ninguna variable visible, pero se guarda un valor en la librería `random()`. Basicamente, define un punto de inicio de la secuencia de números aleatorios que generará `random()`
'''
```

En el comando: `Serial.printf("[MESH] Nodo ID: %d - Generador aleatorio inicializado\n", MY_ID)`. En el `%d` introduce el argumento de MY_ID


```C++
def meshRadioSetup():
'''

'''
```

```C++
def onRadioIrq():

    if (expectingTX)
        {
            txDone = true;
            expectingTX = false;
            expectingRX = true;
        }
        // Si estábamos esperando RX, es RX-done
        else if (expectingRX)
        {
            paqueteRecibido = true;
        }

`radio.setDio1Action(onRadioIrq)`
    /*
    Sirve para cuando hay una interrupción en el pin DIO1, se llama a la función onRadioIrq.
    Por ejemplo, cuando termina de transmitir o recibir un paquete se produce una interrupción en el pin DIO1.
    La función onRadioIrq se encarga de actualizar los flags txDone y paqueteRecibido.
    */

```

Comparación que devuelve un true o false:
```C++
variable = ( cond1 >= cond2 ); //se comporta como una condición lógica 
```

FALTA TERMINAR
