/*
* robot_utils.c
*
* Implementació de la llibreria per controlar motors Dynamixel AX-12
* i sensor AX-S1 utilitzant el microcontrolador MSP432P401R
*
* Creat: maig 2025
* Autor: Albert Villanueva
*/

#include <msp.h>
#include <msp432p401r.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "robot_utils.h"
#include "lib_PAE.h"

// Variables globals per a la gestió de comunicació
static uint8_t mode = MODE_ACTUAL;
uint8_t dada_UART;
volatile bool dada_rebuda = false;
volatile uint16_t valorsDistancia[3] = {0, 0, 0}; // Esquerra, Centre, Dreta
volatile uint8_t RxBuffer[32];       // Buffer de recepci�

// Ports i pins per UART segons mode
#define PORT_UART_EMULAT_SEL0  P1SEL0
#define PORT_UART_EMULAT_SEL1  P1SEL1
#define PORT_UART_REAL_SEL0    P3SEL0
#define PORT_UART_REAL_SEL1    P3SEL1



/************************************************************
 * Configurar UART
 */
/**
 * @brief Configura la UART segons el mode d'operaci� (emulat o real)
 * @param mode Mode d'operaci� (MODE_EMULAT o MODE_REAL)
 */

void init_UART(uint8_t mode) {

    if (mode == MODE_EMULAT) {
        // Mode emulat: UCA0 (P1.2 = RX, P1.3 = TX)
        UCA0CTLW0 |= UCSWRST; // Posar UART en estat de reset

        // Configurar clock source (SMCLK a 24MHz)
        UCA0CTLW0 |= UCSSEL__SMCLK;

        // Configurar baud rate a 500000 bps
        UCA0BRW = 3;       // 24MHz / 500000 = 48
        UCA0MCTLW = UCOS16; // Sobremostreig x16

        UCA0MCTLW |= (0x25 << 8); // Part fractional del baud rate

        // Configurar pins UART (P1.2 i P1.3)
        P1SEL0 |= BIT2 | BIT3;
        P1SEL1 &= ~(BIT2 | BIT3);

        UCA0CTLW0 &= ~UCSWRST;              // Reactivem la linia de comunicacions
        EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG; // Netejem el flag d'interrupció de RX
        EUSCI_A0->IE |= EUSCI_A_IE_RXIE;    // Habilitem les interrupcions de Rx

        NVIC->ICPR[0] |= 1 <<((EUSCIA0_IRQn) & 31); // Mirem que no hi hagi interrupcions residuals
        NVIC->ISER[0] |= 1 <<((EUSCIA0_IRQn) & 31); // Habilitem interrupcions de la USCI

    } else {
        // Mode real: UCA2 (P3.2 = RX, P3.3 = TX)
        UCA2CTLW0 |= UCSWRST;
        UCA2CTLW0 |= UCSSEL__SMCLK;

        // Configurar baud rate a 500000 bps
        UCA2BRW = 3;
        UCA2MCTLW = UCOS16;

        // Configurar pins UART (P3.2 i P3.3)
        P3SEL0 |= BIT2 | BIT3;
        P3SEL1 &= ~(BIT2 | BIT3);

        // Configurar pin de direcció(P3.0)
        P3DIR |= BIT0;
        P3OUT &= ~BIT0; // Mode recepció per defecte


        //Reactivem la linia de comunicacions
        UCA2CTLW0 &= ~UCSWRST;

        EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG; // Netejem el flag d'interrupció de RX
        EUSCI_A2->IE |= EUSCI_A_IE_RXIE;     // Habilitem les interrupcions de Rx

        NVIC->ICPR[0] |= 1 <<((EUSCIA2_IRQn) & 31); // Mirem que no hi hagi interrupcions residuals
        NVIC->ISER[0] |= 1 <<((EUSCIA2_IRQn) & 31); // Habilitem interrupcions de la USCI
    }
}

/***********************************************************
 * Enviar i rebre paquets
 */
// checked
void TxUACx(uint8_t bTxdData)
{
    if (mode == MODE_REAL) {
        // Mode real, fem servir UCA2
        while(!TXD2_READY); // Espera al buffer de transmisió
        UCA2TXBUF = bTxdData;
    } else {
        // Mode emulat, fem servir UCA0
        while(!TXD0_READY); // Espera al buffer de transmisió
        UCA0TXBUF = bTxdData;
    }
}

// checked per emulador i real
byte TxPacket(byte id, byte longitud_params, byte instruccio, byte parametres[16]) {
    byte comptador, checksum = 0;
    byte buffer_tx[TX_BUFFER_SIZE];

    __disable_irq();        // Desactiva interrupcions globals
    configurar_dades_tx();  // Port de direcció en mode transmissió

    // Construcció del paquet
    buffer_tx[0] = 0xFF;    // Header
    buffer_tx[1] = 0xFF;    // Header
    buffer_tx[2] = id;      // ID
    buffer_tx[3] = longitud_params + 2; // Length (params + Instruction + Checksum)
    buffer_tx[4] = instruccio; // Instruction

    // Verificació de seguretat (només en mode real)
    if(mode == MODE_REAL){
        if ((instruccio == INSTR_WRITE_DATA) && (parametres[0] <= 0x05)) {
            __enable_irq();
            return 0;
        }
    }

    // Afegir paràmetres al missatge
    for (comptador = 0; comptador < longitud_params; comptador++) {
        buffer_tx[5 + comptador] = parametres[comptador];
    }

    // Càlcul checksum (suma desde ID fins a l'últim paràmetre)
    for (comptador = 2; comptador < 5 + longitud_params; comptador++) {
        checksum += buffer_tx[comptador];
    }
    checksum = ~checksum; // Complement a 1
    buffer_tx[5 + longitud_params] = checksum; // Posició correcta del checksum

    // Enviament del paquet sencer
    for(comptador = 0; comptador < 6 + longitud_params; comptador++) {
        TxUACx(buffer_tx[comptador]);
    }

    // Espera final de transmissió
    if(mode == MODE_REAL) {
        while(UCA2STATW & UCBUSY); // Espera transmissió UART real
    } else {
        while(UCA0STATW & UCBUSY); // Espera transmissió UART emulada
    }

    configurar_dades_rx(); // Tornem a mode recepció
    __enable_irq();       // Activa interrupcions globals

    return 6 + longitud_params; // Retorna nombre total de bytes transmesos
}

/** *checked
 * @brief Llegeix un paquet de resposta dels dispositius Dynamixel.
 * @return Estructura RxReturn amb el paquet rebut i l'estat del timeout.
 */
struct RxReturn RxPacket(void) {
    struct RxReturn resposta;
    byte count, length, timeout=0;
    byte checksum = 0;

    // Configura la l�nia de dades per recepci� (half-duplex)
    configurar_dades_rx();

    // Control de timeout
    encendre_timeout_A1();

    // Inicialitzaci� del buffer 4 primers valors (cap�alera + ID + longitud)
    for (count = 0; count < 4; count++) {
        reiniciar_timeout_A1();
        dada_rebuda = false;

        while(!dada_rebuda){
            timeout = Timeout(1000);
            if(timeout) break;
        }
        if(timeout) break;

        resposta.StatusPacket[count] = dada_UART;
    }

    // Si no hi ha hagut timeout, llegeix la resta del paquet
    if (!timeout) {

        length = resposta.StatusPacket[3]; // byte de longitud

        // Llegeix els bytes restants
        for (count = 0; count < length; count++) {

            reiniciar_timeout_A1();
            dada_rebuda = false;

            while(!dada_rebuda){
                timeout = Timeout(1000);
                if(timeout) break;
            }
            if(timeout) break;
            resposta.StatusPacket[4 + count] = dada_UART;
        }

        // Si la dada s'ha rebut correctament es valida el checksum
        if (!timeout){
            checksum = 0;

            // Es llegeix des de l'id fins l'útlim paràmetre
            for(count = 2; count < 3 + length; count++){
                checksum += resposta.StatusPacket[count];
            }
            checksum = ~checksum & 0xff;

            byte checksum_packet = resposta.StatusPacket[3 + length];
            // Si els checksums no coincideixen, hi ha un error en el paquet
            if(checksum != checksum_packet){
                resposta.error_checksum = 1;
            } else{
                resposta.error_checksum = 0;
            }
        }
    }
    // Ja no cal el timer A1 per controlar el timeout i el temps de timeout s'afageix a la resposta
    apagar_timeout_A1();
    resposta.timeOut = timeout;
    return resposta;
}



//checked
struct RxReturn tramitar_paquet(byte id, byte longitud_params, byte instuccio, byte parametres[16]){
    struct RxReturn resposta;

    // Inicialitzem amb valors d'error per assegurar l'entrada al bucle
    resposta.timeOut = 1;
    resposta.error_checksum = 1;
    resposta.StatusPacket[4] = 1;

    // Bucle fins que la resposta és correcta
    while (resposta.timeOut || resposta.error_checksum || resposta.StatusPacket[4]) {
        TxPacket(id, longitud_params, instuccio, parametres);
        resposta = RxPacket();
    }

    return resposta;
}


// checked
void configurar_dades_rx(void) {
    P3OUT &= ~BIT0; // Half duplex dels motors en mode recepció
}

//checked
void configurar_dades_tx(void) {
    P3OUT |= BIT0; // Half duplex dels motors en mode transmissió
}

/*******************************************************
 * Timer A1 timeout
 */
//checked
void init_timer_A1(void)
{
    const uint16_t ACTL_CLK = 0x100;
    uint16_t limit = 0x4234;
    uint8_t A1_IE = 0x02;
    uint8_t mode_up = 0x10;

    TA1CTL |= ACTL_CLK;     //ACTL CLK
    TA1CCR0 = limit;        // Valor final
    TA1CTL |= A1_IE;        // INTERRUPT ENABLE
    TA1CTL |= mode_up;      //UP MODE
    TA1CCTL0 |= CCIE;

    // TIMER1, TA1CCTL0.CCIFG és (0x0100) o BIT8
    NVIC->ICPR[0] |= BIT8;
    NVIC->ISER[0] |= BIT8;
}

//checked
void encendre_timeout_A1(void){
    TA1CTL = 0;                 // Inicia configuraci� timer A0
    TA1CTL |= TASSEL_2 | MC_1;  // SMCLK com a rellotge mode UP
    TA1CCR0 = 48000;            // Timeout 24MHz/500 = 48000
    TA1CCTL0 |= CCIE;           // Habilita interrupcions
    TA1CTL |= TACLR;            // Neteja el comptador
}
//checked
void apagar_timeout_A1(void){
    TA1CCTL0 &= ~CCIE;  // Deshabilita interrupcions
    TA1CTL = MC_0;      // Atura el timer
    TA1R = 0;           // Reinicialitza el comptador
}

//checked
void TA1_0_IRQHandler(void){
    TA1CCTL0 &= ~CCIFG; // Esborra la flag de A0
}
//checked

uint8_t Timeout(uint32_t temps){
    static uint32_t comptador = 0;

    // Si es rep una interrupci� al flag provinent del timer A0
    if(TA1CCTL0 & CCIFG){
        TA1CCTL0 &= ~CCIFG;
        comptador++;

        if (comptador >= temps) { // Si ha arribat al tempts reseteja el comptador
            comptador = 0;
            return 1;
        }
    }
        return 0; // No s'efectua timeout
}

void reiniciar_timeout_A1(void){
    TA1R = 0; // Reinicia el comptador del timer A1
    TA1CCTL0 &= ~CCIFG; // Reinicia la flag del timeout
}

/****************************************************************
 * MOVIMENT ROBOT
 */

//checked
void moure_robot(byte id, byte longitud_parametres, byte instruccio, byte* parametres) {
    if (mode == MODE_EMULAT) {
        TxPacket(id, longitud_parametres, instruccio, parametres);
    } else {
        tramitar_paquet(id, longitud_parametres, instruccio, parametres);
    }
}

void config_angle_limit(byte id) {
    byte cw_param[] = {REG_CW_ANGLE_LIMIT, 0x00, 0x00};
    byte ccw_param[] = {REG_CCW_ANGLE_LIMIT, 0, 0};

    moure_robot(id, 3, WRITE_DATA, cw_param);
    moure_robot(id, 3, WRITE_DATA, ccw_param);
}

//checked
void moure_endavant(byte id_dreta, byte id_esquerre, byte velocitat) {
    // printf("  Moure endavant: velocitat = %d\n", velocitat);
    byte byte_dreta[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_HORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_ANTIHORARI};

    moure_robot(id_dreta, 3, WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, 3, WRITE_DATA, byte_esquerre);
}

//checked
void moure_endarrere(byte id_dreta, byte id_esquerre, byte velocitat) {
    byte byte_dreta[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_HORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_ANTIHORARI};

    moure_robot(id_dreta, 3, WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, 3, WRITE_DATA, byte_esquerre);
}

void moure_esquerra(byte id_dreta, byte id_esquerre, byte vel_major, byte vel_menor) {
    byte byte_dreta[] = {REG_MOVING_SPEED_L, vel_major, SENTIT_HORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, vel_menor, SENTIT_HORARI};

    moure_robot(id_dreta, 3, WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, 3, WRITE_DATA, byte_esquerre);
}

void moure_dreta(byte id_dreta, byte id_esquerre, byte velocitat_dreta, byte velocitat_esquerra) {
    byte byte_dreta[] = {REG_MOVING_SPEED_L, velocitat_dreta, SENTIT_ANTIHORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, velocitat_esquerra, SENTIT_ANTIHORARI};

    moure_robot(id_dreta, 3, WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, 3, WRITE_DATA, byte_esquerre);
}

void gir_continu_eix(byte id_dreta, byte id_esquerre, byte velocitat) {
    byte byte_dreta[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_HORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, velocitat, SENTIT_HORARI};

    moure_robot(id_dreta, 3, WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, 3, WRITE_DATA, byte_esquerre);
}


void pivotar_eix_graus(byte motor_dret, byte motor_esquerre, float angle) {
    // Comprova si l'angle és múltiple de 15 graus
    if ((int)angle % 15 != 0) {
        // Missatge d'error
        char missatge1[16], missatge2[16];
        //sprintf(missatge1, "Error: angle no és");
        //sprintf(missatge2, "multiple de 15");

        //halLcdClearScreen(0);
        //halLcdPrintLine(missatge1, 0, 0);
        //halLcdPrintLine(missatge2, 1, 0);

        delay_ms(200);
        return;
    }

    // Calcula el temps en mil·lisegons per a l'angle indicat
    float trams_15 = angle / 15.0;
    int temps = (int)(trams_15 * 375);  // 375 ms per cada 15º

    // Mostra informació a la pantalla
    char info_gir[16];
    //sprintf(info_gir, "Gir %.0f graus", angle);

    //halLcdClearScreen(0);
    //halLcdPrintLine(info_gir, 1, 0);

    delay_ms(200); // Breu pausa abans de girar

    // Inicia el gir
    gir_continu_eix(motor_dret, motor_esquerre, 0xFF);

    // Espera en mil·lisegons
    delay_ms(temps);

    // Atura el robot
    aturar_robot(motor_dret, motor_esquerre);
}

void aturar_robot(byte id_dreta, byte id_esquerre) {
    // Poner velocidad 0
    byte byte_dreta[] = {REG_MOVING_SPEED_L, 0, SENTIT_HORARI};
    byte byte_esquerre[] = {REG_MOVING_SPEED_L, 0, SENTIT_HORARI};

    moure_robot(id_dreta, sizeof(byte_dreta), WRITE_DATA, byte_dreta);
    moure_robot(id_esquerre, sizeof(byte_esquerre), WRITE_DATA, byte_esquerre);
}



/*****************************************************
 * Sensors
 */

int llegir_sensor(byte id_sensor, byte adreca_sensor) {
    uint8_t param_sensor[] = {adreca_sensor, 1};


    return tramitar_paquet(id_sensor, sizeof(param_sensor),INSTR_READ_DATA, param_sensor).StatusPacket[5];
}

int sensor_centre(byte id_sensor) {
    return llegir_sensor(id_sensor, REG_SENSOR_CENTRE);
}

int sensor_esquerra(byte id_sensor) {
    return llegir_sensor(id_sensor, REG_SENSOR_ESQUERRE);
}

int sensor_dreta(byte id_sensor) {
    return llegir_sensor(id_sensor, REG_SENSOR_DRETA);
}


/***********************************************
 * Timer A0 per espera
 */
void init_timer_A0_delay(void){
    TA0CTL = 0;                 // Atura i neteja la configuració
    TA0CTL |= TASSEL_2 | MC_1;  // SMCLK, Mode UP
    TA0CCR0 = 3000;             // Amb SMCLK=3MHz → 3000 = 1 ms
    TA0CCTL0 |= CCIE;           // Interrupció activada
    TA0CTL |= TACLR;            // Reinicia comptador
}


void TA0_0_IRQHandler(void){
    TA0R = 0;           // Reset comptador
    TA0CCTL0 &= ~CCIFG; // Esborra la flag
}

void delay_ms(uint32_t temps_ms){
    uint32_t comptador = 0;
    init_timer_A0_delay(); // Neteja comptador i flag

    while(comptador < temps_ms){
        if (TA0CCTL0 & CCIFG){
            TA0CCTL0 &= ~CCIFG;  // Esborra la flag
            comptador++;
        }
    }
}

/********************************
 * Inicialitzar la pantalla
 */

void init_LCD(void) {
    halLcdInit();
    halLcdClearScreen(0);
}


 /**
  * Handlers d'interrupció per a la EUSCI en mode real i emulat
  */

#if MODE_ACTUAL == MODE_EMULAT
void EUSCIA0_IRQHandler(void) {
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG; // Esborra la flag de RX
    UCA0IE &= ~UCRXIE;                   // Desactiva interrupcions RX
    dada_UART = UCA0RXBUF;         // Llegeix el byte
    dada_rebuda = true;               // Marca que s’ha rebut
    UCA0IE |= UCRXIE;                   // Reactiva interrupcions RX
}
#else
void EUSCIA2_IRQHandler(void) {
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG; // Esborra la flag de RX
    UCA2IE &= ~UCRXIE;                   // Desactiva interrupcions RX
    dada_UART = UCA2RXBUF;         // Llegeix el byte
    dada_rebuda = true;               // Marca que s’ha rebut
    UCA2IE |= UCRXIE;                   // Reactiva interrupcions RX
}
#endif
