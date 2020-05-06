/*Programme initial écrit à l'ENSTA par G Le Maillot et T Le Mezo
 * Compilé à l'ENSTA avec mikroC PRO for PIC v6.4
 * Repris à l'Ifremer par M HAMON avec compilateur micochip XC8 v2.00
 * Adaptation syntaxe au compilateur xc8 (manière différente pour adresser les bits, type différent...)
 * Adaptation au vérin électrique IFREMER
 * Oscillateur interne sur quartz 16MHZ
Hardware:
  18F14K22  DIP20,SOIC
  
  pin 1     VDD Alim +5V
  pin 2     OSC1/RA5
  pin 3     OSC2/RA4
  pin 4     RA3/MCLR ---> pogrammateur
  pin 5     RC5/P1A  ---> sortie PWM P1A, relié à l'entrée DIR du MD13S
  pin 6     RC4 ---> sortie relié à l'entrée PWM du MD13S
  pin 7     RC3
  pin 8     RC6/AN8 --> entrée analogique permettant de mesurer la tension pile 12V au travers d'un pont diviseur 40.2K et 100K 
  pin 9     RC7
  pin 10    RB7 ---> UART TX pic
  pin 11    RB6 ---> SCL I2C clock input
  pin 12    RB5 ---> UART RX pic
  pin 13    RB4 ---> SDA I2C data output
  pin 14    RC2 ---> LED BLANCHE
  pin 15    RC1 ---> Lecture cavalier J6 (Pas utilisé)
  pin 16    RC0 ---> LED ROUGE
  pin 17    RA2/INT2/T0CKI   ---> entrée relié au microrupteur intermédiaire du vérin électrique permettant de connaître l'arrivée sur le piston
  pin 18    RA1/INT1 ---> entrée relié à la sortie S1 du vérin électrique / pogrammateur
  pin 19    RA0/INT0 ---> entrée relié à la sortie S2 du vérin électrique / pogrammateur
  pin 20    VSS Alim
*/

// PIC18F14K22 Configuration Bit Settings

// CONFIG1H
#pragma config FOSC = IRC       // Oscillator Selection bits (Internal RC oscillator)
#pragma config PLLEN = OFF      // 4 X PLL Enable bit (PLL is under software control)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 19        // Brown Out Reset Voltage bits (VBOR set to 1.9 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 256    // Watchdog Timer Postscale Select bits (1:32768)  4ms*256=1.024s

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RA3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "uart.h"

#define _XTAL_FREQ 16000000

#define CODE_VERSION 0x22

// buffer I2C
const char ADDRESS_I2C = 0x38; // linux I2C Adresse
#define SIZE_RX_BUFFER 8
char rxbuffer_tab[SIZE_RX_BUFFER];
char tmp_rx = 0;
char nb_tx_octet = 0;
char nb_rx_octet = 0;

#define S1 PORTAbits.RA0
#define S2 PORTAbits.RA1
#define S_PISTON PORTAbits.RA2
#define LED_ROUGE PORTCbits.RC0
#define LED_BLANCHE PORTCbits.RC2
#define MD13S_ENABLE PORTCbits.RC4

#define debug 0

// Sensors
char hall_sensors_state=0;
//char state1;
//char state2;
int nb_pulse = 0;  // Nombre d'impulsions obtenus avec les capteurs à effet hall du vérin électrique
char butee_out = 0;
char butee_in = 0;
//char lecture_gros_piston;

// Motor
char motor_speed_in = 15; 
char motor_speed_out = 15; 
char motor_speed_out_reset = 50; 
char motor_current_speed = 50; 
#define MOTOR_STOP 50

// Regulation
int position_set_point = 0;
int error = 0;
unsigned int position_reached_max_value = 40000;
unsigned int position_reached_cpt = 0;
char position_reached_enable = 1;
int error_interval = 5;

char zero_shift_error = 0;
char time_zero_shift_error = 5;

// State machine
char motor_on = 1;
enum robot_state {RESET_OUT,REGULATION,EMERGENCY, STOP1};
char state = RESET_OUT;

// Tension pile
char tension_pile;
char flag_mesure_tension_pile;

// Watchdog
char watchdog_restart = 30;
char watchdog_restart_default = 30; // 45 s

//char debug = 1;


void i2c_read_data_from_buffer(){
    char i=0;
    char nb_data = nb_rx_octet-1;

    for(i=0;i<nb_data;i++){
        switch(rxbuffer_tab[i+0]){
            case 0x01:
                state = RESET_OUT;
                #if (debug) 
                    sendString("\r\nRESET_OUT");
                #endif
                watchdog_restart = watchdog_restart_default;
                break;
            case 0x02:
                motor_on = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x03:
                MD13S_ENABLE = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x04:
                LED_ROUGE = (rxbuffer_tab[i+1]!=0x00);
                break;
            case 0x05:
                error_interval = rxbuffer_tab[i+1];
                break;
            case 0x06:
                position_reached_enable = rxbuffer_tab[i+1];
                break;
            case 0x07:
                time_zero_shift_error = rxbuffer_tab[i+1];
                break;
            case 0x10:  // consigne de postion
                if(nb_data >= 2)
                    position_set_point = (rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8));
                break;
            case 0x12:  // consigne de vitesse in
                if(rxbuffer_tab[i+1] <=50)
                    motor_speed_in = rxbuffer_tab[i+1];
                break;
            case 0x13:  // consigne de vitesse out
                if(rxbuffer_tab[i+1] <=50)
                    motor_speed_out = rxbuffer_tab[i+1];
                break;
            case 0x14:  // consigne de vitesse out (reset)
                if(rxbuffer_tab[i+1] <=50)
                    motor_speed_out_reset = rxbuffer_tab[i+1];
                break;
            case 0xA0:  // Wait until release couple
                if(nb_data >= i+2){
                    position_reached_max_value = (rxbuffer_tab[i+1] | (rxbuffer_tab[i+2] << 8));
                    i++;
                }
                break;
            case 0xB0: // emergency mode
                state = EMERGENCY;
                #if (debug) 
                    sendString("\r\nEMERGENCY");
                #endif  
                break;
            case 0xB1:
                state = STOP1;
                #if (debug) 
                    sendString("\r\nSTOP1");
                #endif  
                break;
//            case 0xB2:
//                debug = (rxbuffer_tab[i+1]!=0x00);
//                break;
            default:
                break;
        }
    }
}

/**
 * @brief Fonction qui crée la trame I2C
 * @param num
 */
void i2c_write_data_to_buffer(char nb_tx_octet){
    switch(rxbuffer_tab[0]+nb_tx_octet){
    case 0x00:
        SSPBUF = nb_pulse;
        break;
    case 0x01:
        SSPBUF = (nb_pulse >> 8);
        break;
    case 0x02:
        SSPBUF = (butee_out & 0b1)
                | ((butee_in & 0b1)<<1)
                | ((state & 0b11) <<2)
                | ((motor_on & 0b1) << 4)
                | ((MD13S_ENABLE & 0b1) << 5);
        break;
    case 0x03:
        SSPBUF = position_set_point;
        break;
    case 0x04:
        SSPBUF = position_set_point >> 8;
        break;
    case 0x05:
        SSPBUF = motor_current_speed;
        break;
    case 0x06:
        SSPBUF = motor_speed_in;
        break;
    case 0x07:
        SSPBUF = motor_speed_out;
        break;
    case 0xA0:
        SSPBUF = error;
        break;
    case 0xA1:
        SSPBUF = (error >> 8);
        break;
    case 0xB0:
        SSPBUF = tension_pile;
        break;
    case 0xB1:
        SSPBUF=S_PISTON;
        break;
    case 0xC0:
        SSPBUF = CODE_VERSION;
        break;
    case 0xC1:
        SSPBUF = debug;
        break;
    default:
        SSPBUF = 0x00;
        break;
    }
}

/**
 * @brief Arret du Moteur
 */
void set_motor_cmd_stop(){
    if(motor_current_speed != MOTOR_STOP){
        CCPR1L = MOTOR_STOP >> 2;
        CCP1CONbits.DC1B0 = ((MOTOR_STOP & 0x01)==0x01);
        CCP1CONbits.DC1B1 = ((MOTOR_STOP & 0x02)==0x02);
        
        motor_current_speed = 50;
        MD13S_ENABLE = 1;
        LED_ROUGE=0;
    }

    if(position_reached_enable == 1){
      if(position_reached_cpt>position_reached_max_value){
        MD13S_ENABLE = 0;
        LED_ROUGE=0;
      }
      else
        position_reached_cpt++;
    }
}

/**
 * @brief Commande du moteur
 * @param i
 * out : [50, 100]
 * in : [0, 50]
 */
void set_motor_cmd(char speed){

    if(motor_current_speed != speed){
        position_reached_cpt = 0;
        motor_current_speed = speed;
        
        CCPR1L = speed >> 2;
        CCP1CONbits.DC1B0 = ((speed & 0x01)==0x01);
        CCP1CONbits.DC1B1 = ((speed & 0x02)==0x02);
        
        MD13S_ENABLE = 1;  
    }    
}

/**
 * @brief set_motor_cmd_out
 * @param speed
 */
void set_motor_cmd_out(char speed){
    set_motor_cmd(MOTOR_STOP + speed);
}

/**
 * @brief set_motor_cmd_in
 * @param speed
 */
void set_motor_cmd_in(char speed){
    set_motor_cmd(MOTOR_STOP - speed);
}

/**
 * @brief Lecture des capteurs à effet hall
 * Les 2 capteurs S1 et S2 sont positionnés à 45° l'un de l'autre
 * Cela permet d'avoir la séquence 00->10->11->01->00... dans le sens rentrée du vérin (nb_pulse++)
 * et 00->01->11->10->00.... dans le sens sortie du vérin (nb_pulse--)
 */
void read_hall_sensors(){
    char new_state;
    new_state=(S2<<1)|S1;
    if (new_state!=hall_sensors_state)
    {
        
        switch(hall_sensors_state){
        case 0x00:
            if(new_state == 0x1)
                nb_pulse--;
            else if(new_state == 0x2)
                nb_pulse++;
            break;
        case 0x01:
            if(new_state == 0x3)
                nb_pulse--;
            else if(new_state == 0x0)
                nb_pulse++;
            break;
        case 0x02:
            if(new_state == 0x0)
                nb_pulse--;
            else if(new_state == 0x3)
                nb_pulse++;
            break;
        case 0x03:
            if(new_state == 0x1)
                nb_pulse++;
            else if(new_state == 0x2)
                nb_pulse--;
            break;
        default:
            break;
        }
        LED_ROUGE=!LED_ROUGE;    
    }
    hall_sensors_state = new_state;  // store the current state value to optical_state value this value will be used in next call
}

/**
 * @brief mesure_tension_pile
 * Fonction de lecture de la tension pile
 * réf de tension du CAN -> VDD (5V)
 * Pont diviseur 40.2K - 100K
 * Coeff : 173781 = 5*(100+40.2)/40.2 * 100000 
 * *100000 pour avoir la meilleure résolution avec un U32
 * Tension mesurée = ((valeur CAN+3)/1023) * Coeff /10000
 * +3 permet d'avoir un arrondi lors du troncage pour ramener le résultat dans un octet
 * La tension mesurée est en 1/10V, ce qui permet un retour du résultat sur 1 octet
 */
void mesure_tension_pile(){
    unsigned long calcul_tension_pile;   // 32 bits
    unsigned int result;
    
    if (ADCON0bits.GO_nDONE==0){
        result = ADRESH*256+ADRESL;
        calcul_tension_pile=(result+3)*1743781;
        calcul_tension_pile=calcul_tension_pile/1023;
        calcul_tension_pile=calcul_tension_pile/10000;
        tension_pile = (char)(calcul_tension_pile);
        ADCON0bits.GO_nDONE=1;
    }
    
}

/**
 * @brief init_timer0
 * Fonction d'initialisation du TIMER0
 * Prescaler 1:64; TMR0 Preload = 3036; Actual Interrupt Time : 1 s
 * Fréquence TMR0 = Fosc/4/prescaler. 16000000/4/64=62500Hz
 * Preload=65535-(tempo souhaitée)*(Fréquence TMR0)
 */
void init_timer0(){
  //T0CON = 0x85; // TIMER0 ON (1 s)
  T0CONbits.TMR0ON=1;  //Enables Timer0
  T0CONbits.T08BIT=0;  //Timer0 is configured as a 16-bit timer/counter
  T0CONbits.T0CS=0;  //Internal instruction cycle clock (CLKOUT)
  T0CONbits.T0SE=1;  //Increment on high-to-low transition on T0CKI pin
  T0CONbits.PSA=0;  //Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
  T0CONbits.T0PS2=1; //T0PS<2:0>: Timer0 Prescaler Select bits -->101 = 1:64 prescale value
  T0CONbits.T0PS1=0;
  T0CONbits.T0PS0=1;
  TMR0H = 0x0B;
  TMR0L = 0xDC;
}

/**
 * @brief init_timer1
 * Fonction d'initialisation du TIMER1
 * Prescaler 1:1; TMR1 Preload = 65335; Actual Interrupt Time : 
 * Fréquence TMR1 = Fosc/4/prescaler. 16000000/4/1=4MHz
 * Preload=65535-(tempo souhaitée)*(Fréquence TMR1)
 */
//void init_timer1(){
//  T1CONbits.RD16=0;  //Enables register read/write of Timer1 in two 8-bit operations
//  T1CONbits.T1RUN=0;
//  T1CONbits.T1CKPS1=0;
//  T1CONbits.T1CKPS0=0;
//  T1CONbits.T1OSCEN=0;
//  T1CONbits.TMR1CS=0;
//  T1CONbits.TMR1ON=0;  //Disable Timer1
////  TMR1H = 0xF0;
////  TMR1L = 0x5F;
//}

/**
 * @brief Initialisation des entrées sorties du PIC
 */
void init_io(){
    // entrée RC6 / AN8 utilisée en entrée analogique pour lire la tension pile
    ANSEL = 0x00;
    ANSELH = 0x00;
    TRISCbits.TRISC6 = 1;
    ANSELHbits.ANS8 = 1;   
    ADCON0bits.CHS3 = 1;  // AN8 sélectionné
    ADCON0bits.CHS2 = 0;
    ADCON0bits.CHS1 = 0;
    ADCON0bits.CHS0 = 0;
    ADCON0bits.ADON = 1;  // ADC enabled
    ADCON1bits.NVCFG1 = 0; // 00 = Positive voltage reference supplied internally by VDD.
    ADCON1bits.NVCFG0 = 0; 
    ADCON1bits.NVCFG1 = 0; // 00 = Negative voltage reference supplied internally by VSS.
    ADCON1bits.NVCFG0 = 0;
    ADCON2bits.ADFM = 1;  // 1 = Right justified
    ADCON2bits.ADCS2 = 1; // 101 = FOSC/16, temps de 1µs à 16MHz
    ADCON2bits.ADCS1 = 0;
    ADCON2bits.ADCS0 = 1;
    ADCON0bits.GO_nDONE=1; // Lancement d'une première CAN
    
    // Pas de comparateurs
    CM1CON0 = 0x00; 
    CM2CON0 = 0x00; 

    // Capteurs effet hall S1 et S2
    TRISAbits.TRISA0 = 1; // RA0 en entrée
    TRISAbits.TRISA1 = 1; // RA1 en entrée

    // Microrupteur piston 
    TRISAbits.TRISA2 = 1; // RA2 en entrée

    // **** IO I2C **** //
    TRISBbits.TRISB4 = 1; // RB4 en entrée
    TRISBbits.TRISB6 = 1; // RB6 en entrée

    // UART
    TRISBbits.TRISB5=1;   // RX
    TRISBbits.TRISB7=0;   // TX

    // LEDs
    TRISCbits.TRISC0 = 0; // RC0 en sortie
    TRISCbits.TRISC2 = 0; // RC2 en sortie
    
    // Entrée cavalier J6
    TRISCbits.TRISC1 = 1; // RC1 en entrée
    
    // MD13S
    TRISCbits.TRISC4 = 0; // RC4 en sortie
    TRISCbits.TRISC5 = 0; // RC5 en sortie
}

/**
 * @brief initialisation de l'I2C en mode esclave
 */

void init_i2c(){

  // **** ADDRESS **** //
  SSPADD = (ADDRESS_I2C << 1); // Address Register, Get address (7-1 bit). Lsb is read/write flag
  SSPMSK = 0xFF; // A zero (‘0’) bit in the SSPMSK register has the effect of making
                 // the corresponding bit in the SSPSR register a “don’t care”

  // **** SSPSTAT **** //
  SSPSTATbits.SMP = 1; // Slew Rate Control bit
  // 1 = Slew rate control disabled for standard Speed mode (100 kHz and 1 MHz)
  // 0 = Slew rate control enabled for High-Speed mode (400 kHz)
  SSPSTATbits.CKE = 1; // // SMBusTM Select bit (1 = Enable SMBus specific inputs)

    // **** SSPCON2 **** //
  SSPCON2 = 0x00;
  SSPCON2bits.GCEN = 0; // General Call Enable bit (0 = disabled)
  SSPCON2bits.SEN = 1; // Start Condition Enable/Stretch Enable bit (1 = enabled)

  // **** SSPCON1 **** //
  SSPCON1bits.WCOL = 0; // Write Collision Detect bit
  SSPCON1bits.SSPOV = 0; // Receive Overflow Indicator bit
  SSPCON1bits.CKP = 1; // SCK Release Control bit (1=Release clock)
  SSPCON1bits.SSPM3 = 0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled (1-> with S/P, 0 -> without)
  SSPCON1bits.SSPM2 = 1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1bits.SSPM1 = 1; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1bits.SSPM0 = 0; // I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
  SSPCON1bits.SSPEN = 1; // Synchronous Serial Port Enable bit

  }

void init_interrupt()
{
    // IT à chaque changement d'état de RA0 (S1) et RA1 (S2) et RA2 (S_PISTON)
    INTCON2bits.RABIP = 0; //RA and RB Port Change Interrupt Priority bit, low priority
    IOCAbits.IOCA0 = 1; // Interrupt-on-change enabled sur RA0
    IOCAbits.IOCA1 = 1; // Interrupt-on-change enabled sur RA1
    IOCAbits.IOCA2 = 1; // Interrupt-on-change enabled sur RA2
    IOCB = 0x00;    // Interrupt-on-change disabled sur le PORTB
    INTCONbits.RABIE = 1; // RA and RB Port Change Interrupt Enable bit
    
    // IT I2C
    IPR1bits.SSPIP = 1; //Master Synchronous Serial Port Interrupt Priority bit, high priority
    PIR1bits.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
    PIR2bits.BCLIF = 0;
    PIE1bits.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
    PIE2bits.BCLIE = 1;// Bus collision

    // UART
    IPR1bits.TXIP=0; //EUSART Transmit Interrupt Priority bit, low priority
    
    // Timer 0
    INTCON2bits.TMR0IP=0; //Timer 0, TMR0 Overflow Interrupt Priority bit, low priority  
    INTCONbits.TMR0IE = 1;  //TMR0 Overflow Interrupt Enable bit
    T0CONbits.TMR0ON = 1; // Start TIMER0   
 
    // Timer 1
//    IPR1bits.TMR1IP=0;  //TMR1 Overflow Interrupt Priority bit, low priority
//    PIE1bits.TMR1IE=1;  //TMR1 Overflow Interrupt Enable bit
//    T1CONbits.TMR1ON=0;
    
    RCONbits.IPEN = 1;  //Enable priority levels on interrupts
    INTCONbits.GIEH = 1; //enable all high-priority interrupts
    INTCONbits.GIEL = 1; //enable all low-priority interrupts
    INTCONbits.GIE = 1; // Global Interrupt Enable bit
    INTCONbits.PEIE = 1; // Peripheral Interrupt Enable bit
}

void init_PWM()
{
    // Period = 4 * TOSC * (PR2 + 1) * (TMR2 Prescale Value)
    // Pulse Width = TOSC * (CCPR1L<7:0>:CCP1CON<5:4>) * (TMR2 Prescale Value)
    // Delay = 4 * TOSC * (PWM1CON<6:0>)
    PR2 = 24;
    T2CON = 0b00000111; // ON et 16 bit
    PWM1CON = 1; // Delay

    // Max value = 4*(PR2+1)
    // Mid = 2*(PR2+1)
    // Min value = 0
    // Constraint : 4*(PR2+1) < 1023 (ie PR2<255)
    CCPR1L = MOTOR_STOP >> 2;
    CCP1CONbits.DC1B0 = ((MOTOR_STOP & 0x01)==0x01);
    CCP1CONbits.DC1B1 = ((MOTOR_STOP & 0x02)==0x02);

    // P1M : 00 (Single output: P1A, P1B, P1C and P1D controlled by steering)
    // CCP1 : 1100 : PWM mode; P1A, P1C active-high; P1B, P1D active-high
    CCP1CONbits.P1M0 = 0; 
    CCP1CONbits.P1M1 = 0; 
    CCP1CONbits.CCP1M3 = 1; 
    CCP1CONbits.CCP1M2 = 1;
    CCP1CONbits.CCP1M1 = 0;
    CCP1CONbits.CCP1M0 = 0;
    // STRA: Steering Enable bit A
    // P1A pin has the PWM waveform with polarity control from CCP1M<1:0>
    PSTRCONbits.STRA = 1;
    
    MD13S_ENABLE = 0;
}
/**
 * @brief main
 */
void main(){
    // Oscillateur interne de 16Mhz
    OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block

    WDTCONbits.SWDTEN = 1; //armement du watchdog
    init_io(); // Initialisation des I/O
    init_i2c(); // Initialisation de l'I2C en esclave
    init_timer0(); // Initialisation du TIMER0 toutes les 1 secondes
//    init_timer1();
    init_PWM();
    init_uart();
    init_interrupt(); // Initialisation des ITs

    motor_current_speed = 50;
//    debug = 0;
    flag_mesure_tension_pile=0;
    __delay_ms(100);
    #if (debug) 
        sendString("\r\nInit\n");
    #endif
    #if (debug) 
        sendString("\r\nRESET_OUT\n");
    #endif
    while(1){
        CLRWDT();
        if (flag_mesure_tension_pile){
            mesure_tension_pile();
            flag_mesure_tension_pile=0;
        }
        // State machine
        switch (state){
        case RESET_OUT:
            LED_BLANCHE = 1;
            if (watchdog_restart==0){
                state = REGULATION;
                #if (debug) 
                    sendString("\r\nREGULATION");
                #endif
                nb_pulse = 0; // Reset Nb pulse 
                position_set_point = 0;
                set_motor_cmd_stop();
            }
            else
                set_motor_cmd_out(motor_speed_out_reset); // [50, 100]
            break;

        case REGULATION:
//            read_hall_sensors();

            LED_BLANCHE = 0;
            // Regulation
            error = position_set_point - nb_pulse;

            if(error > error_interval)
                set_motor_cmd_in(motor_speed_in); // [0, 50]
            else if(error < -error_interval)
                set_motor_cmd_out(motor_speed_out); // [50, 100]
            else // position reached
                set_motor_cmd_stop();
            break;

        case EMERGENCY:
            LED_BLANCHE = 1;
            set_motor_cmd_out(motor_speed_out_reset);
            break;

        case STOP1:
            set_motor_cmd_stop();
            break;

        default:
            break;
        }

        // I2C
        if(nb_rx_octet>1 && SSPSTATbits.P  == 1){
            i2c_read_data_from_buffer();
            nb_rx_octet = 0;
        }
    }
}

/**
 * @brief Fonction de gestion des interruptions:
 * interruption sur changement des entrées RA0 et RA1 (capteurs effet hall S1 et S2 du vérin électrique), low priority
 * interruption toutes les 1s sur TIMER0 pour gestion de la tempo permettant de sortir le vérin à la mise sous tension, low priority
 * interruption sur le bus I2C, high priority
 */
void interrupt low_priority LowISR(void){

    short num;
    if (INTCONbits.RABIF){
//        state1=(S2<<1)|S1;
//        TMR1H = 0xF0;
//        TMR1L = 0x5F;
        TMR1H = 0xE0;
        TMR1L = 0xBF;
//        T1CONbits.TMR1ON=1;  //enable Timer1
//        INTCONbits.RABIE = 0; // RA and RB Port Change Interrupt Disable bit


/*        if ((S_PISTON==0) && (motor_current_speed>50)){  // voir ce qu'on fait, on sort le vérin et on arrive au début de sortie du piston
            position_set_point = nb_pulse;  //arrête le vérin
        }*/

        
        
//        state2=(S2<<1)|S1;
//        if (state1==state2){
        read_hall_sensors();
//    }

        INTCONbits.RABIF=0;
    }
//    if (PIR1bits.TMR1IF){
//        T1CONbits.TMR1ON=0;  //disable Timer1
//        state2=(S2<<1)|S1;
//        if (state1==state2){
//            read_hall_sensors();
//            LED_ROUGE=!LED_ROUGE;
//        }
//        INTCONbits.RABIE = 1; // RA and RB Port Change Interrupt Enable bit
//        PIR1bits.TMR1IF=0;
//        
//    }
    if (INTCONbits.TMR0IF){
        if(watchdog_restart>0)
          watchdog_restart--;  

        // Auto reset if wrong zero ref
/*        if(position_set_point==0 && motor_current_speed == MOTOR_STOP && butee_out==0){
            if(zero_shift_error<time_zero_shift_error)
                zero_shift_error++;
            else{
                state = RESET_OUT;
                zero_shift_error = 0;
            }
        }
        else
            zero_shift_error=0;*/
        #if (debug) 
            if (nb_pulse>=0){
                num=(short)nb_pulse;
                sendString("\r\n nb_pulse:");sendNum(num);}
            else{
                num=(short)-nb_pulse;
                sendString("\r\n nb_pulse:-");sendNum(num);}
            if (error>=0){
                num=(short)error;
                sendString("\r\n error:");sendNum(num);}
            else{
                num=(short)-error;
                sendString("\r\n error:-");sendNum(num);}
        #endif
        flag_mesure_tension_pile=1;       
        TMR0H = 0x0B;
        TMR0L = 0xDC;
        INTCONbits.TMR0IF = 0;
    }
    // si flag registre d'émission vide et interruption autorisées
    if ((PIR1bits.TXIF)&&(PIE1bits.TXIE))
    {
        interruptTx();	// alors, émission du caractère
    }
}

/**
 * @brief interrupt_high
 */
//void interrupt IT(){
//    
//    if(PIR1bits.SSPIF)
//    {
//        tmp_rx = SSPBUF;
//        if(!SSPSTATbits.R_nW) //Le bit R_nW est à 0 -> le master nous envoie un setI2C, 
//        {
//              SSPCON1bits.CKP = 1;
//              if (!SSPSTATbits.D_A)  // le bit SSPSTATbits.D_nA est à 0, l'octet recu est l'adresse
//              {
//                nb_rx_octet = 0;
//              }
//              else // le bit SSPSTATbits.D_nA est à 1, l'octet recu est une data
//              {
//                rxbuffer_tab[nb_rx_octet] = tmp_rx;
//                nb_rx_octet++;
//              }
//        }
//        else //Le bit R_nW est à 1 -> le master nous envoie un getI2C, 
//        {
//            if (!SSPSTATbits.D_A)  // le bit SSPSTATbits.D_nA est à 0, l'octet recu est l'adresse
//            {
//              nb_tx_octet = 0;  
//            }
//            i2c_write_data_to_buffer(nb_tx_octet);
//            SSPCON1bits.CKP = 1;
//            nb_tx_octet++;
//        }
//        if (SSPCON1bits.SSPOV|| SSPCON1bits.WCOL) //If overflow or collision
//        {
//            SSPCON1bits.SSPOV = 0; // Clear the overflow flag
//            SSPCON1bits.SSPOV = 0; // Clear the collision bit
//            tmp_rx = SSPBUF; // Read the previous value to clear the buffer
//            SSPCON1bits.CKP = 1;
//        }
//        PIR1bits.SSPIF = 0;
//    }
// 
//    
//} 


void interrupt IT(){
    
    if(PIR1bits.SSPIF)
    {
        tmp_rx = SSPBUF;
        if(SSPSTATbits.R_nW) //Le bit R_nW est à 1 -> le master nous envoie un getI2C,
        {
            if (!SSPSTATbits.D_A)  // le bit SSPSTATbits.D_nA est à 0, l'octet recu est l'adresse
            {
              nb_tx_octet = 0;  
            }
            i2c_write_data_to_buffer(nb_tx_octet);
            SSPCON1bits.CKP = 1;
            nb_tx_octet++;
        }
        else //Le bit R_nW est à 0 -> le master nous envoie un setI2C, 
        {
              SSPCON1bits.CKP = 1;
              if (!SSPSTATbits.D_A)  // le bit SSPSTATbits.D_nA est à 0, l'octet recu est l'adresse
              {
                nb_rx_octet = 0;
              }
              else // le bit SSPSTATbits.D_nA est à 1, l'octet recu est une data
              {
                rxbuffer_tab[nb_rx_octet] = tmp_rx;
                nb_rx_octet++;
              }
        }
        if (SSPCON1bits.SSPOV|| SSPCON1bits.WCOL) //If overflow or collision
        {
            SSPCON1bits.SSPOV = 0; // Clear the overflow flag
            SSPCON1bits.SSPOV = 0; // Clear the collision bit
            tmp_rx = SSPBUF; // Read the previous value to clear the buffer
            SSPCON1bits.CKP = 1;
        }

        PIR1bits.SSPIF = 0;
    }
 
    
} 
