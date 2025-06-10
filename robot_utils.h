/*
 * robot_utils.h
 *
 * Llibreria per controlar els motors Dynamixel AX-12 i sensor AX-S1
 * utilitzant el microcontrolador MSP432P401R
 *
 * Creat: abril 2025
 * Autor: Albert Villanueva, Yasmina Dermouh i Leonardo Menéndez
 */

#ifndef ROBOT_UTILS_H_
#define ROBOT_UTILS_H_

#include <stdint.h>

// Definici� de tipus
typedef uint8_t byte;

// Constants per seleccionar mode d'operaci�
#define MODE_EMULAT 0
#define MODE_REAL 1
#define MODE_ACTUAL MODE_REAL

#if MODE_ACTUAL == MODE_EMULAT
// IDs dels dispositius Dynamixel
#define MOTOR_DRET_ID      1
#define MOTOR_ESQUERRE_ID  2
#define SENSOR_ID          3   // 100 per real, 3 per emulat

#else
// IDs dels dispositius Dynamixel robot 2
#define MOTOR_DRET_ID      2
#define MOTOR_ESQUERRE_ID  4
#define SENSOR_ID          100   // 100 per real, 3 per emulat
#define UMBRAL_ESQUINA 120  // Más bajo que DISTANCIA_SEGURETAT


#endif

// Instruccions Dynamixel
#define INSTR_PING         0x01
#define INSTR_READ_DATA    0x02
#define INSTR_WRITE_DATA   0x03
#define INSTR_REG_WRITE    0x04
#define INSTR_ACTION       0x05
#define INSTR_RESET        0x06
#define INSTR_SYNC_WRITE   0x83

// Registres del Dynamixel AX-12
#define WRITE_DATA          0x03
#define REG_BAUD_RATE       0x04
#define REG_RETURN_DELAY    0x05

#define REG_CW_ANGLE_LIMIT  0x06
#define REG_CCW_ANGLE_LIMIT 0x08


#define REG_GOAL_POSITION_L 0x1E
#define REG_GOAL_POSITION_H 0x1F

#define REG_MOVING_SPEED_L  0x20

#define REG_PRESENT_POSITION_L 0x24
#define REG_PRESENT_POSITION_H 0x25

#define REG_PRESENT_SPEED_L 0x26
#define REG_PRESENT_SPEED_H 0x27

#define REG_PRESENT_LOAD_L  0x28
#define REG_PRESENT_LOAD_H  0x29

// Registres del sensor AX-S1
#define REG_SENSOR_ESQUERRE         0x1A
#define REG_SENSOR_CENTRE           0x1B
#define REG_SENSOR_DRETA            0x1C

// Constants per control de direcci�
#define DIRECCIO_ENDAVANT   0
#define DIRECCIO_ENDARRERE  1
#define SENTIT_HORARI       0x04      // Sentit horari per motors
#define SENTIT_ANTIHORARI   0x00      // Sentit anti-horari per motors

// Mida de buffers i altres constants
#define RX_BUFFER_SIZE      64
#define TX_BUFFER_SIZE      32
#define MAX_PARAMS          16

// Constants d'error del paquet d'estat
#define ERROR_INSTRUCTION   0x40
#define ERROR_OVERLOAD      0x20
#define ERROR_CHECKSUM      0x10
#define ERROR_RANGE         0x08
#define ERROR_OVERHEATING   0x04
#define ERROR_ANGLE_LIMIT   0x02
#define ERROR_INPUT_VOLTAGE 0x01


#define REG_LED             0x19

// Constants de velocitat
#define VELOCITAT_MAX_RPM      114      // Velocitat m�xima en RPM
#define VELOCITAT_MIN_RPM      0        // Velocitat m�nima
#define VELOCITAT_MAX_UNITATS  1023     // Valor m�xim per als registres de velocitat

#define DISTANCIA_SEGURETAT 90   // Dist�ncia m�nima a obstacles (0-1023)
#define DISTANCIA_MINIMA 100
#define TEMPS_CALIBRACIO 3000     // Temps per a calibraci� inicial (ms)
// Añadir al inicio del archivo
#define VELOCITAT_NORMAL  150
#define VELOCITAT_GIRO    200  // Mayor que la normal para giros
#define TEMPS_GIRO_90     800  // ms para girar 90 grados
#define TEMPS_RETROCESSO  600  // ms para retroceder

// Constants per a conversions
#define RPM_A_UNITATS(rpm)     ((uint16_t)((rpm) * (VELOCITAT_MAX_UNITATS / VELOCITAT_MAX_RPM)))
#define UNITATS_A_RPM(unitats) ((uint16_t)((unitats) * (VELOCITAT_MAX_RPM / VELOCITAT_MAX_UNITATS)))


#define TIMEOUT_RX_PACKET 50000  // Timeout en cicles de CPU
#define TXD0_READY (UCA0IFG & UCTXIFG)
#define TXD2_READY (UCA2IFG & UCTXIFG)


// Variables globals d'estat
extern volatile byte ultimPaquetID;
extern volatile byte ultimPaquetError;
extern volatile uint16_t valorsDistancia[3]; // Esquerra, Centre, Dreta

extern volatile uint8_t RxBufferSize;
extern volatile bool RxComplete;

struct RxReturn {
    byte StatusPacket[20];  // Buffer pel paquet de resposta
    byte timeOut;              // Flag que indica si s'ha exhaurit el temps d'espera
    byte error_checksum;
};

void init_LCD(void);

void init_UART(uint8_t mode);
void EUSCIA2_IRQHandler(void);
void EUSCIA0_IRQHandler(void);

void TxUACx(uint8_t txData);
byte TxPacket(byte id, byte longitud_params, byte instruccio, byte parametres[16]);
struct RxReturn RxPacket(void);
struct RxReturn tramitar_paquet(byte id, byte longitud_params, byte instuccio, byte parametres[16]);
void configurar_dades_rx(void);
void configurar_dades_tx(void);

void init_timer_A1(void);
void encendre_timer_A1(void);
void apagar_timer_A1(void);
void reiniciar_timeout_A1(void);
void TA1_0_IRQHandler(void);

void init_timer_A0_delay(void);
void TA0_0_IRQHandler(void);
void delay_ms(uint32_t temps_ms);

void moure_robot(byte id, byte longitud_parametres, byte instruccio, byte* parametres);
void config_angle_limit(byte id);
void moure_endavant(byte id_dreta, byte id_esquerre, byte velocitat);
void moure_endarrere(byte id_dreta, byte id_esquerre, byte velocitat);
void moure_esquerra(byte id_dreta, byte id_esquerre, byte vel_major, byte vel_menor);
void moure_dreta(byte id_dreta, byte id_esquerre, byte velocitat_dreta, byte velocitat_esquerra);
void gir_continu_eix(byte id_dreta, byte id_esquerre, byte velocitat);
void pivotar_eix_graus(byte motor_dret, byte motor_esquerre, float angle);
void aturar_robot(byte id_dreta, byte id_esquerre);

int llegir_sensor(byte id_sensor, byte adreca_sensor);
int sensor_centre(byte id_sensor);
int sensor_esquerra(byte id_sensor);
int sensor_dreta(byte id_sensor);

typedef struct RobotState{
    uint8_t estat;          // 0=buscar, 1=seguir, 2=evitar
    uint8_t sentit;         // 0=dreta, 1=esquerra
    uint16_t distancia_obj; // Dist�ncia objectiu a paret
} RobotState;

#endif
