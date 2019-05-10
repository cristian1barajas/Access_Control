/*
 * File:   main.c
 * Author: BotLAB
 * Tomado del canal de Youtube Mecatronica TEC
 * Created on 28 de abril de 2019, 11:29 AM
 */

#include <xc.h>
#include "Configuration.h"

#define _XTAL_FREQ 20000000                 // Define la frecuencia de oscilación de la CPU para el uso de los Delays
#define Magnet_Sensor PORTAbits.RA1         // Entrada digital para sensor magnetico
#define Open_Contact PORTBbits.RB2                  // Entrada digital para Botón de apertura
#define Engine_Direction_A PORTBbits.RB3    // Salida digital que define la dirección del motor
#define Engine_Direction_B PORTBbits.RB4    // Salida digital que define la dirección del motor
#define End_Stop_Open PORTBbits.RB6            // Entrada digital fin de carrera
#define End_Stop_Close PORTBbits.RB7            // Entrada digital fin de carrera
#define Time 850000
#define Time_Auto_Close 1000000

_Bool Last_Magnet_State = 0;
_Bool Last_Open_Contact_State = 1;
_Bool Magnet_State;
_Bool Open_Contact_State;
_Bool End_Stop_Close_State;
_Bool End_Stop_Open_State;
long Count_Time_Close = 0;
long Count_Auto_Close = 0;


void Close_Lock(void);
void Open_Lock(void);

void main(void) {   // Función principal
    
    ADCON1 = 0X0F;       // Todos los pines analogos como digitales
    TRISAbits.RA1 = 1;   // Configuración como entrada digital
    TRISB = 0xE7;        // Configuración del puerto B
    
    Engine_Direction_A = 0;
    Engine_Direction_B = 0;
    
    while(1) {                          
        Close_Lock();
        Open_Lock();
    }
    return;
}

void Close_Lock(void) {
    Magnet_State = Magnet_Sensor;
    __delay_ms(50);
    if(Magnet_State == 1 && Last_Magnet_State == 0) {   // Detección de flanco ascendente en el sensor magnetico      
        __delay_ms(50);                                // Antirebote
        End_Stop_Open_State = End_Stop_Open;
        if(Magnet_State == 1 && Last_Magnet_State == 0 && End_Stop_Open_State == 0) {
            do {
                Engine_Direction_A = 1;
                End_Stop_Close_State = End_Stop_Close;
                Count_Time_Close++;
                if(Count_Time_Close == Time) {
                    Count_Time_Close = 0;
                    break;
                }
            } while(End_Stop_Close_State == 1);
            Engine_Direction_A = 0;
            Count_Time_Close = 0;
        }
    }
    Last_Magnet_State = Magnet_State;
}

void Open_Lock(void) {
    Open_Contact_State = Open_Contact;
    __delay_ms(50);
    if(Open_Contact_State == 0 && Last_Open_Contact_State == 1) {   // Detección de flanco ascendente en el sensor magnetico      
        __delay_ms(50);                                // Antirebote
        End_Stop_Open_State = End_Stop_Open;
        if(Open_Contact_State == 0 && Last_Open_Contact_State == 1 && End_Stop_Open_State == 1) {
            do {
                Engine_Direction_B = 1;
                End_Stop_Open_State = End_Stop_Open;
                Count_Time_Close++;
                if(Count_Time_Close == Time) {
                    Count_Time_Close = 0;
                    break;
                }
            } while(End_Stop_Open_State == 1);
            Engine_Direction_B = 0;
            Count_Time_Close = 0;
            Magnet_State = Magnet_Sensor;
            do {
                Count_Auto_Close++;
                Magnet_State = Magnet_Sensor;
                if(Count_Auto_Close == Time_Auto_Close) {
                    End_Stop_Open_State = End_Stop_Open;
                    if(End_Stop_Open_State == 0) {
                        do {
                            Engine_Direction_A = 1;
                            End_Stop_Close_State = End_Stop_Close;
                            Count_Time_Close++;
                            if(Count_Time_Close == Time) {
                                Count_Time_Close = 0;
                                break;        
                            }
                        } while(End_Stop_Close_State == 1);
                        Engine_Direction_A = 0;
                        Count_Time_Close = 0;
                        Count_Auto_Close = 0;
                    }
                    break;
                }
            } while(Magnet_State == 1);
            Count_Auto_Close = 0;
        }
    }
    Last_Open_Contact_State = Open_Contact_State;
}
