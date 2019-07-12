/*
 * File:   main.c
 * Author: Ing. Christian Barajas
 * 
 * Programa especifico para la cerradura electromecanica, donde se incluye un nuevo driver de 
 * potencia para el control del motor dc, un sensor de corriente para el control de errores 
 * mecanicos y fuerza excesiva del motor, sensor inductivo para reemplazar el sensor magnetico
 * y parametros preestablecidos desde un menu controlado con un display doble de siete segmentos
 * y dos pulsadores.
 * 
 * BotLAB co
 * Created on 28 de abril de 2019, 11:29 AM
 */

#include <xc.h>
#include "Configuration.h"

#define _XTAL_FREQ 20000000                 // Define la frecuencia de oscilacion de la CPU para el uso de los Delays
#define Display PORTB                       // Display de siete segmentos
#define Display_One PORTCbits.RC7           // Transistor del primer display
#define Display_Two PORTCbits.RC6           // Transistor del segundo display
#define Inductive_Sensor PORTAbits.RA1      // Entrada digital para sensor inductivo
#define Open_Contact PORTAbits.RA4          // Entrada digital para Boton de apertura
#define Engine_Direction_A PORTCbits.RC1    // Salida digital que define la direccion del motor
#define Engine_Direction_B PORTCbits.RC2    // Salida digital que define la direccion del motor
#define End_Stop_Open PORTAbits.RA2         // Entrada digital fin de carrera
#define End_Stop_Close PORTAbits.RA3        // Entrada digital fin de carrera
#define Time 250000                         // Constante de tiempo para detencion de emergencia
#define Time_Auto_Close 1000000             // Constante de tiempo para cierre automatico
#define Set_Point_Current 544               // Set point para el ajuste de la corriente

// Constantes que definen el muestreo de la corriente 
#define Samples 500            // Número de muestras para evitar el pico de corriente de arranque
int Count_Peake_Current = 0;  // Contador de picos de corriente
int Current = 0;              // Variable para almacenar el valor de la corriente del sensor

_Bool Last_Inductive_State = 0;
_Bool Last_Open_Contact_State = 1;
_Bool Inductive_State;
_Bool Open_Contact_State;
_Bool End_Stop_Close_State;
_Bool End_Stop_Open_State;
long Count_Time_Close = 0;
long Count_Auto_Close = 0;
//int Count_Inductive_Active = 0; Descomente si quiere contar los eventos del sensor inductivo

int ADC;
unsigned char Buffer1[16];

void Close_Lock(void);
void Open_Lock(void);
void Closing(void);
int Analog_Read(void);
void Sense_Current(void); // Descomente si quiere ajustar el Set Point de la corriente

void main(void) {   // Funcion principal
    ADCON1 = 0X0E;       // Todos los pines analogos como digitales excepto RA0

    TRISAbits.RA0 = 1;   // Configuracion como entrada analoga

    TRISAbits.RA1 = 1;   // Configuracion como entrada digital "Inductive Sensor"
    TRISAbits.RA4 = 1;   // Configuracion como entrada digital "Open Contact"
    TRISCbits.RC1 = 0;   // Configuracion como salida digital "Engine Direction"
    TRISCbits.RC2 = 0;   // Configuracion como salida digital "Engine Direction"
    TRISCbits.RC6 = 0;   // Configuracion como salida digital "Display One"
    TRISCbits.RC7 = 0;   // Configuracion como salida digital "Display Two"
    TRISAbits.RA2 = 1;   // Configuracion como entrada digital "End Stop Open"
    TRISAbits.RA3 = 1;   // Configuracion como entrada digital "End Stop Close"

    // Configuracion del ADC
    ADCON0bits.CHS = 0;     // Seleccion del canal analogo 0
    //ADCON1bits.VCFG0 = 0;  // Referencia negativa = Vss (Ya se configuro previamente con ADCON1)
    //ADCON1bits.VCFG1 = 0;  // Referencia positiva = Vdd (Ya se configuro previamente con ADCON1)
    ADCON2bits.ADCS = 0b000; // Frecuencia de conversion
    ADCON2bits.ADFM = 1;     // Justificacion a la derecha
    ADCON0bits.ADON = 1;     // ADC encendido o habilitado

    TRISB = 0x00;        // Configuracion del puerto B como salida
    Display = 0xFF;      // Inicia apagado todo el puerto B
    
    Display_One = 0;
    Display_Two = 0;
    Engine_Direction_A = 0;
    Engine_Direction_B = 0;
    
    while(1) {   
        Close_Lock();
        Open_Lock();
    }
    return;
}

void Close_Lock(void) {
    Inductive_State = Inductive_Sensor;
    __delay_ms(50);
    if(Inductive_State == 1 && Last_Inductive_State == 0) {   // Deteccion de flanco ascendente en el sensor magnetico      
        __delay_ms(50);
        End_Stop_Open_State = End_Stop_Open;
        if(Inductive_State == 1 && Last_Inductive_State == 0 && End_Stop_Open_State == 0) {
            __delay_ms(1500);
            Closing();
            /* Descomente si quiere contar cada evento o deteccion del 
               sensor inductivo para el cierre de la puerta.

            Count_Inductive_Active++;   
            if(Count_Inductive_Active == 2) {
                Closing();
                Count_Inductive_Active = 0;
            }*/
        }
    }
    Last_Inductive_State = Inductive_State;
}

void Open_Lock(void) {
    Open_Contact_State = Open_Contact;
    __delay_ms(50);
    if(Open_Contact_State == 0 && Last_Open_Contact_State == 1) {   // Deteccion de flanco ascendente en el sensor magnetico      
        __delay_ms(50);                                // Antirebote
        End_Stop_Open_State = End_Stop_Open;
        if(Open_Contact_State == 0 && Last_Open_Contact_State == 1 && End_Stop_Open_State == 1) {
            do {
                Engine_Direction_B = 1;
                End_Stop_Open_State = End_Stop_Open;
                Count_Time_Close++;
                Sense_Current();
                if(Count_Time_Close == Time) {
                    Count_Time_Close = 0;
                    break;
                } else if (Current > Set_Point_Current) {  // Omite el pico de corriente del arranque del motor
                    Count_Peake_Current++;
                    if (Count_Peake_Current > Samples) {
                        Count_Peake_Current = 0;
                        break;
                    }
                }
            } while(End_Stop_Open_State == 1);
            Engine_Direction_B = 0;
            Count_Time_Close = 0;
            Count_Peake_Current = 0;
            Inductive_State = Inductive_Sensor;
            do {
                Count_Auto_Close++;
                Inductive_State = Inductive_Sensor;
                if(Count_Auto_Close == Time_Auto_Close) {
                    End_Stop_Open_State = End_Stop_Open;
                    if(End_Stop_Open_State == 0) {
                        Closing();
                        Count_Auto_Close = 0;
                    }
                    break;
                }
            } while(Inductive_State == 1);
            Count_Auto_Close = 0;
        }
    }
    Last_Open_Contact_State = Open_Contact_State;
}

void Closing(void) {
    do {
        Engine_Direction_A = 1;
        End_Stop_Close_State = End_Stop_Close;
        Count_Time_Close++;
        Current = Analog_Read();
        Sense_Current();
        if(Count_Time_Close == Time) {
            Count_Time_Close = 0;
            break;
        } else if (Current > Set_Point_Current) {  // Omite el pico de corriente del arranque del motor
            Count_Peake_Current++;
            if (Count_Peake_Current > Samples) {
                Count_Peake_Current = 0;
                break;
            }
        }
    } while(End_Stop_Close_State == 1);
    Engine_Direction_A = 0;
    Count_Time_Close = 0;
    Count_Peake_Current = 0;
}

int Analog_Read(void) {
    ADCON0bits.GO_nDONE = 1;    // Lanza la conversion
        while(ADCON0bits.GO_nDONE);
            ADC = ADRESH;
            ADC = ADC << 8;
            ADC = ADC + ADRESL;
            
    return ADC;
}

/* Funcion especifica para periodo de prueba.
   Imprime por el display doble las letras CL, que corresponden a
   Current Limited, sirve para ajustar el Set Point de la corriente
   sensada. El indicador aparece mientras el Set Point es superado 
   por la corriente sensada.
*/
   void Sense_Current(void) {
    Current = Analog_Read();        
        if (Current > Set_Point_Current) {
            Display = 0x8D;
            Display_One = 1;
            __delay_us(5);
            Display_One = 0;
            Display = 0x8F;
            Display_Two = 1;
            __delay_us(5);
            Display_Two = 0;
        } else {
            Display_One = 0;
            Display_Two = 0;
            Display = 0x11;
        }
}