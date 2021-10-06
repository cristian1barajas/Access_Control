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
#define Inductive_Sensor PORTAbits.RA3      // Entrada digital para sensor inductivo
#define Open_Contact PORTCbits.RC0          // Entrada digital para Boton de apertura
#define Engine_Direction_A PORTCbits.RC1    // Salida digital que define la direccion del motor
#define Engine_Direction_B PORTCbits.RC2    // Salida digital que define la direccion del motor
#define End_Stop_Open PORTAbits.RA4         // Entrada digital fin de carrera
#define End_Stop_Close PORTAbits.RA5        // Entrada digital fin de carrera
#define Time 250000                         // Constante de tiempo para detencion de emergencia
#define Time_Exit_Menu 7000
#define Set_Point_Current 524               // Set point tarjeta JLCPCB
//#define Set_Point_Current 544               // Set point original primera version
//#define Set_Point_Current 644               // Set point caso excepcional
//#define Set_Point_Current 744               // Set point caso extremo
#define Timing_Display 1600

#define Button_Menu_Up PORTAbits.RA1        // Boton para control del menu
#define Button_Menu_Down PORTAbits.RA2      // Boton para control del menu

// Constantes que definen el muestreo de la corriente 
#define Samples 4000            // N煤mero de muestras para evitar el pico de corriente de arranque
int Count_Peake_Current = 0;  // Contador de picos de corriente
int Current = 0;              // Variable para almacenar el valor de la corriente del sensor

_Bool Last_Inductive_State = 0;
_Bool Last_Open_Contact_State = 1;
_Bool Last_Button_Menu_Up_State = 1;
_Bool Last_Button_Menu_Down_State = 1;
_Bool Inductive_State;
_Bool Open_Contact_State;
_Bool End_Stop_Close_State;
_Bool End_Stop_Open_State;
long Count_Time_Close = 0;
long Count_Auto_Close = 0;
_Bool Button_Menu_Up_State;
_Bool Button_Menu_Down_State;
int Count_Push_Button = 0;
long Count_Save = 0;
long Count_Exit_Menu = 0;
_Bool Toggle_Up = 0;
_Bool Toggle_Down = 0;
_Bool Flag_Menu = 0;
_Bool Latch;                // Variable para guardar y evaluar el valor de la memoria eeprom
_Bool Flag_Latch;
_Bool Flag_End_Stop_Open;
int Time_Inductive_State;  // Variable para guardar y evaluar el valor de la memoria eeprom
int Time_Open_State;       // Variable para guardar y evaluar el valor de la memoria eeprom
int Count_Led = 0;
//int Count_Inductive_Active = 0; Descomente si quiere contar los eventos del sensor inductivo

long Time_Auto_Close = 1;      // Constante de tiempo para cierre automatico

int ADC;
unsigned char Buffer1[16];

int data_test;

void Close_Lock(void);
void Open_Lock(void);
void Closing(void);
int Analog_Read(void);
void Sense_Current(void);

void Menu_In(void);
void Menu(void);
void Exit_Time_Menu(void);

void Draw_P1(void);
void Draw_P2(void);
void Draw_P3(void);
void Draw_P4(void);
void Draw_CL(void);

void Draw_0(void);
void Draw_1(void);
void Draw_2(void);
void Draw_3(void);
void Draw_4(void);
void Draw_5(void);
void Draw_6(void);
void Draw_7(void);
void Draw_8(void);
void Draw_9(void);
void Draw_10(void);
void Draw_11(void);
void Draw_12(void);
void Draw_13(void);
void Draw_14(void);
void Draw_15(void);
void Draw_16(void);
void Draw_18(void);
void Draw_20(void);
void Draw_22(void);
void Draw_24(void);
void Draw_26(void);
void Draw_28(void);
void Draw_30(void);

void Menu_P1(void);
void Menu_P2(void);
void Menu_P3(void);
void Menu_P4(void);

void Clear(void);
void Draw_Save_Dot_One(void);
void Draw_Save_Dot_Two(void);
void Recording(void);

void eeprom_writex(int address, char data);
char eeprom_readx(int address);

void Indicator(void);

void main(void) {   // Funcion principal
    ADCON1 = 0X0E;       // Todos los pines analogos como digitales excepto RA0

    TRISAbits.RA0 = 1;   // Configuracion como entrada analoga "Sensor de corriente"
    TRISAbits.RA3 = 1;   // Configuracion como entrada digital "Inductive Sensor"
    TRISCbits.RC0 = 1;   // Configuracion como entrada digital "Open Contact"
    TRISCbits.RC1 = 0;   // Configuracion como salida digital "Engine Direction"
    TRISCbits.RC2 = 0;   // Configuracion como salida digital "Engine Direction"
    TRISCbits.RC6 = 0;   // Configuracion como salida digital "Display One"
    TRISCbits.RC7 = 0;   // Configuracion como salida digital "Display Two"
    TRISAbits.RA4 = 1;   // Configuracion como entrada digital "End Stop Open"
    TRISAbits.RA5 = 1;   // Configuracion como entrada digital "End Stop Close"
    TRISAbits.RA1 = 1;   // Configuracion como entrada digital "Button Menu"
    TRISAbits.RA2 = 1;   // Configuracion como entrada digital "Button Menu"

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

    Latch = eeprom_readx(0x01);
    Time_Inductive_State = eeprom_readx(0x02);
    Time_Open_State = eeprom_readx(0x04);
    
    while(1) {
        Indicator();
        Menu_In();
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
            __delay_ms(50);
            Count_Time_Close = 0;
            Time_Inductive_State = eeprom_readx(0x02);
            Time_Inductive_State =  Time_Inductive_State * 1000;
            switch (Time_Inductive_State)  // Retardo de arranque de cierre de los pasadores
            {
            case 0: __delay_ms(500); break;
            case 1000: __delay_ms(1000); break;
            case 2000: __delay_ms(2000); break;
            case 3000: __delay_ms(3000); break;
            case 4000: __delay_ms(4000); break;
            case 5000: __delay_ms(5000); break;
            case 6000: __delay_ms(6000); break;
            case 7000: __delay_ms(7000); break;
            case 8000: __delay_ms(8000); break;
            case 9000: __delay_ms(9000); break;
            case 10000: __delay_ms(10000); break;
            case 11000: __delay_ms(10000); __delay_ms(1000); break;
            case 12000: __delay_ms(10000); __delay_ms(2000); break;
            case 13000: __delay_ms(10000); __delay_ms(3000); break;
            case 14000: __delay_ms(10000); __delay_ms(4000); break;
            case 15000: __delay_ms(10000); __delay_ms(5000); break;
            default: __delay_ms(500); break;
            }
            Engine_Direction_A = 1;
            __delay_ms(1000);
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
            Time_Open_State = eeprom_readx(0x04);
            Time_Open_State = Time_Open_State * 1000;
            switch (Time_Open_State)  // Retardo de arranque despues de presionar el boton
            {
            case 0: __delay_ms(0); break;
            case 1000: __delay_ms(1000); break;
            case 2000: __delay_ms(2000); break;
            case 3000: __delay_ms(3000); break;
            case 4000: __delay_ms(4000); break;
            case 5000: __delay_ms(5000); break;
            case 6000: __delay_ms(6000); break;
            case 7000: __delay_ms(7000); break;
            case 8000: __delay_ms(8000); break;
            case 9000: __delay_ms(9000); break;
            case 10000: __delay_ms(10000); break;
            case 11000: __delay_ms(10000); __delay_ms(1000); break;
            case 12000: __delay_ms(10000); __delay_ms(2000); break;
            case 13000: __delay_ms(10000); __delay_ms(3000); break;
            case 14000: __delay_ms(10000); __delay_ms(4000); break;
            case 15000: __delay_ms(10000); __delay_ms(5000); break;
            default: __delay_ms(0); break;
            }
            Flag_End_Stop_Open = 1; // Bandera de control para el do-while
            do {
                Engine_Direction_B = 1;
                End_Stop_Open_State = End_Stop_Open; // Primer motivo para detenci髇 de la cerradura
                Count_Time_Close++;                  // Segundo motivo para detenci髇 de la cerradura
                Sense_Current();                     // Tercer motivo para detenci髇 de la cerradura
                if (End_Stop_Open_State == 0) {
                    __delay_ms(250);
                    End_Stop_Open_State = End_Stop_Open; // Antirrebote se vuelve a validar el valor del fin de carrera
                    if (End_Stop_Open_State == 0) {
                        Flag_End_Stop_Open = 0;
                        int Value_Latch = eeprom_readx(0x01);
                        if (Value_Latch == 1) {
                            __delay_ms(3000);   // Retardo de sostenido del picaporte
                            Engine_Direction_B = 0;
                            Engine_Direction_A = 1;
                            __delay_ms(500);
                            Engine_Direction_A = 0;
                        }
                    }
                }
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
            //} while(End_Stop_Open_State == 1);
            } while(Flag_End_Stop_Open == 1);
            Engine_Direction_B = 0;
            Count_Time_Close = 0;
            Count_Peake_Current = 0;
            Inductive_State = Inductive_Sensor;
            Time_Auto_Close = eeprom_readx(0x03);
            Time_Auto_Close = Time_Auto_Close * 500000;
            do {
                Count_Auto_Close++;
                Inductive_State = Inductive_Sensor;
                if(Count_Auto_Close == Time_Auto_Close) {
                    End_Stop_Open_State = End_Stop_Open;
                    if(End_Stop_Open_State == 0) {
                        Engine_Direction_A = 1;
                        __delay_ms(500);
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
    Count_Time_Close = 0;
    __delay_ms(1000);
    do {
        Engine_Direction_A = 1;
        End_Stop_Close_State = End_Stop_Close;
        if(End_Stop_Close_State == 0) {
            __delay_ms(250);
            End_Stop_Close_State = End_Stop_Close;
            if(End_Stop_Close_State == 0) {
                break;
            }
        }
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
    __delay_ms(20);
    Engine_Direction_A = 0;
    Count_Time_Close = 0;
    Count_Peake_Current = 0;
    Toggle_Up = 0;
    Toggle_Down = 0;
}

int Analog_Read(void) {
    ADCON0bits.GO_nDONE = 1;    // Lanza la conversion
        while(ADCON0bits.GO_nDONE) 
        ;
            ADC = ADRESH;
            ADC = ADC << 8;
            ADC = ADC + ADRESL;
            
    return ADC;
}

/*  Funcion especifica para periodo de prueba.
    Imprime por el display doble las letras CL, que corresponden a
    Current Limited, sirve para ajustar el Set Point de la corriente
    sensada. El indicador aparece mientras el Set Point es superado 
    por la corriente sensada.
*/
void Sense_Current(void) {
    Current = Analog_Read();        
        if (Current > Set_Point_Current) {
            Draw_CL();
        } else {
            Clear();
        }
}

/*  Funciones de escritura y lectura de la memoria EEPROM del micro.
    Estas librerias fueron escritas por:
    Instagram  http://instagram.com/robertelectronica
    Facebook   https://www.facebook.com/Robertelectronico/
    Youtube    https://www.youtube.com/channel/UCYze8bs8C1qF6UbV2r4LrAA/videos?view_as=subscriber
*/
void eeprom_writex(int address, char data) {
    EEADR = address;		//  1.  Escribir la direcci贸n de la memoria en el registro EEADR
    EEDATA = data;          //  2.	Escribir el dato a guardar en el registro EEDAT
    EECON1bits.EEPGD = 0;	//  3.	Acceder a la memoria de datos de la EEPROM
    EECON1bits.CFGS = 0;	//  4.	Acceder a la EEPROM
    EECON1bits.WREN = 1;	//  5.	Habilitar escritura
    INTCONbits.GIE = 0;     //  6.	Deshabilitar interrupciones

    EECON2 = 0x55;          //  7.  Enviar secuencia EECON2 = 0x55 y EECON2 = 0xAA y 
    EECON2 = 0xaa;          //      empezar la escritura en la memoria EEPROM
    EECON1bits.WR = 1;      //      Bit que empieza la escritura en la memoria
    
    INTCONbits.GIE = 1;     //  8.	Habilitar interrupciones
    while (EECON1bits.WR);  //  9.	Esperar hasta que la escritura se complete               
}

char eeprom_readx(int address) {
    EEADR = address;        //  1.	Escribir la direcci贸n de la memoria a leer en el registro EEADR  
    EECON1bits.EEPGD = 0;	//  2.	Acceder a la memoria de datos de la EEPROM
    EECON1bits.CFGS = 0;	//  3.	Acceder a la EEPROM
    EECON1bits.RD = 1;      //  4.	Leer datos de la EEPROM
    return(EEDATA);
}

void Menu_In(void) {
    Button_Menu_Up_State = Button_Menu_Up;
    __delay_ms(1);
    if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
        Toggle_Up = 1;
        if (Toggle_Up == 1 && Toggle_Down == 1) {
            Flag_Menu = 1;
            Toggle_Up = 0;
            Toggle_Down = 0;
            Count_Push_Button = 0;
            __delay_ms(300);
            Menu();
        }   
    }
    Last_Button_Menu_Up_State = Button_Menu_Up_State;

    Button_Menu_Down_State = Button_Menu_Down;
    __delay_ms(1);
    if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
        Toggle_Down = 1;
        if (Toggle_Down == 1 && Toggle_Up == 1) {
            Flag_Menu = 1;
            Toggle_Down = 0;
            Toggle_Up = 0;
            Count_Push_Button = 0;
            __delay_ms(300);
            Menu();
        }
    }
    Last_Button_Menu_Down_State = Button_Menu_Down_State;
}

void Menu(void) {
    do {
        switch (Count_Push_Button)
        {
        case 0: Draw_P1(); break;
        case 1: Draw_P2(); break;
        case 2: Draw_P3(); break;
        case 3: Draw_P4(); break;
        default: Count_Push_Button = 0; break;
        }
        // Detecci贸n del pulso del bot贸n Down...!
        Button_Menu_Down_State = Button_Menu_Down;
        if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
            Count_Push_Button++;
            Count_Exit_Menu = 0;
        }
        Last_Button_Menu_Down_State = Button_Menu_Down_State;

        // Detecci贸n del pulso del bot贸n Enter...!
        Button_Menu_Up_State = Button_Menu_Up;
        if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
            Count_Exit_Menu = 0;
            switch (Count_Push_Button)
            {
            case 0:
                Count_Push_Button = 0;
                __delay_ms(300);
                Menu_P1();
                break;
            case 1:
                Count_Push_Button = 0;
                __delay_ms(300);
                Menu_P2();
                break;
            case 2:
                Count_Push_Button = 1;
                __delay_ms(300);
                Menu_P3();
                break;
            case 3:
                Count_Push_Button = 0;
                __delay_ms(300);
                Menu_P4();
                break;
            default:
                Count_Push_Button = 0;
                __delay_ms(300);
                break;
            }
        }
        Last_Button_Menu_Up_State = Button_Menu_Up_State;
        Exit_Time_Menu();
    } while(Flag_Menu == 1);
}

void Exit_Time_Menu(void) {
    Count_Exit_Menu++;
    if (Count_Exit_Menu == Time_Exit_Menu) {
        Flag_Menu = 0;
        Count_Exit_Menu = 0;
    }
}

void Menu_P1(void) {    // Parametro 1. Accionamiento del picaporte
    do {
        switch (Count_Push_Button)
        {
        case 0: Draw_0(); break;
        case 1: Draw_1(); break;
        default: Count_Push_Button = 0; break;
        }
        // Detecci贸n del pulso del bot贸n Down...!
        Button_Menu_Down_State = Button_Menu_Down;
        if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
            Count_Push_Button++;
            Count_Exit_Menu = 0;
        }
        Last_Button_Menu_Down_State = Button_Menu_Down_State;
         
        // Detecci贸n del pulso del bot贸n Enter...!
        Button_Menu_Up_State = Button_Menu_Up;
        Count_Exit_Menu = 0;
        if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
            switch (Count_Push_Button)
            {
            case 0: eeprom_writex(0x01, 0x00); Recording(); break;
            case 1: eeprom_writex(0x01, 0x01); Recording(); break;
            default: Count_Push_Button = 0; break;
            }
        }
        Last_Button_Menu_Up_State = Button_Menu_Up_State;
        Exit_Time_Menu();
    } while (Flag_Menu == 1);
}

void Menu_P2(void) {    // Parametro 2. Tiempo de retardo despues de la se帽al del sensor inductivo
    do {
        switch (Count_Push_Button)
        {
            case 0: Draw_0(); break;
            case 1: Draw_1(); break;
            case 2: Draw_2(); break;
            case 3: Draw_3(); break;
            case 4: Draw_4(); break;
            case 5: Draw_5(); break;
            case 6: Draw_6(); break;
            case 7: Draw_7(); break;
            case 8: Draw_8(); break;
            case 9: Draw_9(); break;
            case 10: Draw_10(); break;
            case 11: Draw_11(); break;
            case 12: Draw_12(); break;
            case 13: Draw_13(); break;
            case 14: Draw_14(); break;
            case 15: Draw_15(); break;
            default: Count_Push_Button = 0; break;
        }
        // Detecci贸n del pulso del bot贸n Down...!
        Button_Menu_Down_State = Button_Menu_Down;
        if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
            Count_Push_Button++;
            Count_Exit_Menu = 0;
        }
        Last_Button_Menu_Down_State = Button_Menu_Down_State;

        // Detecci贸n del pulso del bot贸n Enter...!
        Button_Menu_Up_State = Button_Menu_Up;
        Count_Exit_Menu = 0;
        if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
            switch (Count_Push_Button)
            {
                case 0: eeprom_writex(0x02, 0x00); Recording(); break;
                case 1: eeprom_writex(0x02, 0x01); Recording(); break;
                case 2: eeprom_writex(0x02, 0x02); Recording(); break;
                case 3: eeprom_writex(0x02, 0x03); Recording(); break;
                case 4: eeprom_writex(0x02, 0x04); Recording(); break;
                case 5: eeprom_writex(0x02, 0x05); Recording(); break;
                case 6: eeprom_writex(0x02, 0x06); Recording(); break;
                case 7: eeprom_writex(0x02, 0x07); Recording(); break;
                case 8: eeprom_writex(0x02, 0x08); Recording(); break;
                case 9: eeprom_writex(0x02, 0x09); Recording(); break;
                case 10: eeprom_writex(0x02, 0x0A); Recording(); break;
                case 11: eeprom_writex(0x02, 0x0B); Recording(); break;
                case 12: eeprom_writex(0x02, 0x0C); Recording(); break;
                case 13: eeprom_writex(0x02, 0x0D); Recording(); break;
                case 14: eeprom_writex(0x02, 0x0E); Recording(); break;
                case 15: eeprom_writex(0x02, 0x0F); Recording(); break;
                default: Count_Push_Button = 0; break;
            }
        }
        Last_Button_Menu_Up_State = Button_Menu_Up_State;
        Exit_Time_Menu();
    } while (Flag_Menu == 1);
}

void Menu_P3(void) {    // Parametro 3. Tiempo de espera del cierre automatico
    do {
        switch (Count_Push_Button)
        {
            //case 0: Draw_0(); break;
            case 1: Draw_2(); break;
            case 2: Draw_4(); break;
            case 3: Draw_6(); break;
            case 4: Draw_8(); break;
            case 5: Draw_10(); break;
            case 6: Draw_12(); break;
            case 7: Draw_14(); break;
            case 8: Draw_16(); break;
            case 9: Draw_18(); break;
            case 10: Draw_20(); break;
            case 11: Draw_22(); break;
            case 12: Draw_24(); break;
            case 13: Draw_26(); break;
            case 14: Draw_28(); break;
            case 15: Draw_30(); break;
            default: Count_Push_Button = 1; break;
        }
        // Detecci贸n del pulso del bot贸n Down...!
        Button_Menu_Down_State = Button_Menu_Down;
        if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
            Count_Push_Button++;
            Count_Exit_Menu = 0;
        }
        Last_Button_Menu_Down_State = Button_Menu_Down_State;

        // Detecci贸n del pulso del bot贸n Enter...!
        Button_Menu_Up_State = Button_Menu_Up;
        Count_Exit_Menu = 0;
        if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
            switch (Count_Push_Button)
            {
                //case 0: eeprom_writex(0x03, 0x00); Recording(); break;
                case 1: eeprom_writex(0x03, 0x01); Recording(); break;
                case 2: eeprom_writex(0x03, 0x02); Recording(); break;
                case 3: eeprom_writex(0x03, 0x03); Recording(); break;
                case 4: eeprom_writex(0x03, 0x04); Recording(); break;
                case 5: eeprom_writex(0x03, 0x05); Recording(); break;
                case 6: eeprom_writex(0x03, 0x06); Recording(); break;
                case 7: eeprom_writex(0x03, 0x07); Recording(); break;
                case 8: eeprom_writex(0x03, 0x08); Recording(); break;
                case 9: eeprom_writex(0x03, 0x09); Recording(); break;
                case 10: eeprom_writex(0x03, 0x0A); Recording(); break;
                case 11: eeprom_writex(0x03, 0x0B); Recording(); break;
                case 12: eeprom_writex(0x03, 0x0C); Recording(); break;
                case 13: eeprom_writex(0x03, 0x0D); Recording(); break;
                case 14: eeprom_writex(0x03, 0x0E); Recording(); break;
                case 15: eeprom_writex(0x03, 0x0F); Recording(); break;
                default: Count_Push_Button = 1; break;
            }
        }
        Last_Button_Menu_Up_State = Button_Menu_Up_State;
        Exit_Time_Menu();
    } while (Flag_Menu == 1);
}

void Menu_P4(void) {    // Parametro 4. Retardo despues de la se帽al del boton de apertura
    do {
        switch (Count_Push_Button)
        {
        case 0: Draw_0(); break;
        case 1: Draw_1(); break;
        case 2: Draw_2(); break;
        case 3: Draw_3(); break;
        case 4: Draw_4(); break;
        case 5: Draw_5(); break;
        case 6: Draw_6(); break;
        case 7: Draw_7(); break;
        case 8: Draw_8(); break;
        case 9: Draw_9(); break;
        case 10: Draw_10(); break;
        case 11: Draw_11(); break;
        case 12: Draw_12(); break;
        case 13: Draw_13(); break;
        case 14: Draw_14(); break;
        case 15: Draw_15(); break;
        default: Count_Push_Button = 0; break;
        }
        // Detecci贸n del pulso del bot贸n Down...!
        Button_Menu_Down_State = Button_Menu_Down;
        if (Button_Menu_Down_State == 0 && Last_Button_Menu_Down_State == 1) {
            Count_Push_Button++;
            Count_Exit_Menu = 0;
        }
        Last_Button_Menu_Down_State = Button_Menu_Down_State;

        // Detecci贸n del pulso del bot贸n Enter...!
        Button_Menu_Up_State = Button_Menu_Up;
        Count_Exit_Menu = 0;
        if (Button_Menu_Up_State == 0 && Last_Button_Menu_Up_State == 1) {
            switch (Count_Push_Button)
            {
            case 0: eeprom_writex(0x04, 0x00); Recording(); break;
            case 1: eeprom_writex(0x04, 0x01); Recording(); break;
            case 2: eeprom_writex(0x04, 0x02); Recording(); break;
            case 3: eeprom_writex(0x04, 0x03); Recording(); break;
            case 4: eeprom_writex(0x04, 0x04); Recording(); break;
            case 5: eeprom_writex(0x04, 0x05); Recording(); break;
            case 6: eeprom_writex(0x04, 0x06); Recording(); break;
            case 7: eeprom_writex(0x04, 0x07); Recording(); break;
            case 8: eeprom_writex(0x04, 0x08); Recording(); break;
            case 9: eeprom_writex(0x04, 0x09); Recording(); break;
            case 10: eeprom_writex(0x04, 0x0A); Recording(); break;
            case 11: eeprom_writex(0x04, 0x0B); Recording(); break;
            case 12: eeprom_writex(0x04, 0x0C); Recording(); break;
            case 13: eeprom_writex(0x04, 0x0D); Recording(); break;
            case 14: eeprom_writex(0x04, 0x0E); Recording(); break;
            case 15: eeprom_writex(0x04, 0x0F); Recording(); break;
            default: Count_Push_Button = 0; break;
            }
        }
        Last_Button_Menu_Up_State = Button_Menu_Up_State;
        Exit_Time_Menu();
    } while (Flag_Menu == 1);
} 

void Clear(void) {
    Display_One = 0;
    Display_Two = 0;
    Display = 0x11;
}

void Draw_CL(void) {
    Display = 0xAA;        // "C" En el display de siete segmentos
    Display_One = 1;
    __delay_us(5);
    Display_One = 0;
    Display = 0xAB;       // "L" En el display de siete segmentos
    Display_Two = 1;
    __delay_us(5);
    Display_Two = 0;
}

void Draw_P1(void) {
    Display = 0xB0;          // "P" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x7D;         // "1" En el display de siete segmentos
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_P2(void) {
    Display = 0xB0;        // "P" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xA4;       // "2" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_P3(void) {
    Display = 0xB0;        // "P" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x64;       // "3" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_P4(void) {
    Display = 0xB0;        // "P" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x71;       // "4" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_0(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x28;       // "0" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_1(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x7D;       // "1" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_2(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xA4;       // "2" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_3(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x64;       // "3" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_4(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x71;       // "4" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_5(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x62;       // "5" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_6(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x22;       // "6" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_7(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x7C;       // "7" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_8(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x20;       // "8" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_9(void) {
    Display = 0xFF;        // " " En el display de siete segmentos
    Display_One = 0;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x60;       // "9" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_10(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x28;       // "0" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_11(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x7D;       // "1" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_12(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xA4;       // "2" En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_13(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x64;       // "3" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_14(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x71;       // "4" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_15(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x62;       // "5" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_16(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x22;       // "6" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_18(void) {
    Display = 0x7D;        // "1" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x20;       // "8" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_20(void) {
    Display = 0xA4;        // "2" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x28;       // "0" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_22(void) {
    Display = 0xA4;        // "2" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xA4;       // "2" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_24(void) {
    Display = 0xA4;        // "2" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x71;       // "4" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_26(void) {
    Display = 0xA4;        // "2" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x22;       // "6" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_28(void) {
    Display = 0xA4;        // "2" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x20;       // "8" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_30(void) {
    Display = 0x64;        // "3" En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0x28;       // "0" En el display de siete segmentos  
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
} 

void Draw_Save_Dot_One(void) {
    Display = 0xDF;        // "." En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xFF;       // " " En el display de siete segmentos 
    Display_Two = 0;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Draw_Save_Dot_Two(void) {
    Display = 0xDF;        // "." En el display de siete segmentos
    Display_One = 1;
    __delay_us(Timing_Display);
    Display_One = 0;
    Display = 0xDF;       // "." En el display de siete segmentos 
    Display_Two = 1;
    __delay_us(Timing_Display);
    Display_Two = 0;
}

void Recording(void) {      
    Clear();
    __delay_ms(500);
    do
    {
        Draw_Save_Dot_One();
        Count_Save++;
    } while (Count_Save < 250);
    Count_Save = 0;
    do
    {
        Draw_Save_Dot_Two();
        Count_Save++;
    } while (Count_Save < 250);
    Count_Save = 0;
    Flag_Menu = 0;
}

void Indicator(void) {
    Inductive_State = Inductive_Sensor;
    End_Stop_Close_State = End_Stop_Close;
    if (Inductive_State == 1 && End_Stop_Close_State == 0) {
        Display = 0xFF;
        Display_One = 1;
    } else {
        Count_Led++;
        if (Count_Led < 2) {
            Display = 0xFF;
            Display_One = 1;
        } else if (Count_Led >= 2 && Count_Led < 4) {
            Display = 0xFF;
            Display_One = 0;
        } else {
            Display = 0xFF;
            Display_One = 0;
            Count_Led = 0;
        }
    }
}