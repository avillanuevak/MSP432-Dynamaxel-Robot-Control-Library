/*
* main.c - Pr�ctica 4: UART i llibreria del robot
*
* Programaci� d'Arquitectures Encastades
*
* Aquest programa demostra l'�s de la llibreria de funcions per controlar el robot.
* Implementa la comunicació UART i fa proves de moviment del robot.
*
* Creat: maig 2025
* Estudiant: Albert Villanueva, Yasmina Dermouh i Leonardo Menéndez
*/

#include <msp.h>
#include <msp432p401r.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <robot_utils.h>
#include <lib_PAE.h>

// Constants

// Variables globals
volatile uint8_t mode = MODE_ACTUAL;

volatile uint16_t valors_sensors[3];      // E, C, D
volatile bool paret_trobada = false;
volatile bool seguint_paret = false;
volatile uint8_t sentit_seguiment = 0;    // 0=dreta, 1=esquerra

/**
 * @brief Mostra els noms dels membres del grup a la pantalla LCD
 */
void mostrar_noms_grup(void) {
    halLcdClearScreen(0);
    halLcdPrintLine("Projecte PAE", 0, 0);
    halLcdPrintLine("Albert V.", 1, 0);
    halLcdPrintLine("Yasmina D.", 2, 0);
    halLcdPrintLine("Leonardo M.", 3, 0);
    //delay_ms(2000);
}

/**
 * @brief Configura el Watchdog Timer per aturar-lo.
 */
void init_WDT(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Atura el Watchdog Timer
}

/******************************************************
 * Tests de robot i emulador
 */

void test_robot_functions(void) {
    uint16_t valors_sensors[3];

    printf("Mode: %s\n", (mode == MODE_EMULAT) ? "EMULADOR" : "REAL");

    // 1. Test moviment endavant
    moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 1);
    delay_ms(8000);

    // 2. Test aturada
    aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);
    delay_ms(8000);

    // 3. Test moviment endarrere
    moure_endarrere(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100);
    delay_ms(8000);

    // 4. Test gir dreta
    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100, 100);
    delay_ms(7500);

    // 5. Test gir esquerra
    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100, 100);
    delay_ms(7500);

    // 6. Test gir continu sobre el seu eix
    gir_continu_eix(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100);
    delay_ms(7500);

    // 7. Pivotar sobre el seu eix
    pivotar_eix_graus(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100);
    delay_ms(7500);

    // 8. Test combinat
    moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100);
    delay_ms(7000);
    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100, 100);
    delay_ms(7000);
    moure_endarrere(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100);
    delay_ms(7000);
    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 100, 100);
    delay_ms(7000);

    // 7. Test sensors
    valors_sensors[0] = sensor_esquerra(SENSOR_ID);
    valors_sensors[1] = sensor_centre(SENSOR_ID);
    valors_sensors[2] = sensor_dreta(SENSOR_ID);

    if (valors_sensors[0] != -1 && valors_sensors[1] != -1 && valors_sensors[2] != -1) {
        printf("Valors sensors: E[%d] C[%d] D[%d]\n",
               valors_sensors[0], valors_sensors[1], valors_sensors[2]);
    } else {
        printf("Error en llegir sensors\n");
    }

    // 8. Finalització
    aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);

}


/**
 * @brief Funci� per enviar un paquet de prova a l'emulador
 */
void test_comunicacio_emulador(void) {
    byte parametres[3] = {0x1A, 0x03, 0x00}; // Llegir 3 bytes a partir de 0x1A

    if (TxPacket(SENSOR_ID, 3, INSTR_READ_DATA, parametres)) {
        printf("Paquet enviat correctament\n");
    } else {
        printf("Error en enviar paquet\n");
    }

    // Esperar resposta
    delay_ms(1000);
}

/*****************************************************************
 * Lectura i test de sensors
 */

/**
 * @brief Llegeix tots els sensors i actualitza els valors globals
 */
void llegir_sensors(void) {
    valors_sensors[0] = sensor_esquerra(SENSOR_ID);
    valors_sensors[1] = sensor_centre(SENSOR_ID);
    valors_sensors[2] = sensor_dreta(SENSOR_ID);

    // Mostrar valors a la pantalla (opcional)
    char msg[16];
    //sprintf(msg, "E:%d C:%d D:%d", valors_sensors[0], valors_sensors[1], valors_sensors[2]);
    halLcdClearScreen(1);
    halLcdPrintLine(msg, 0, 0);
}

/**
 * @brief Calibració inicial dels sensors
 */
void calibrar_sensors(void) {
    halLcdClearScreen(0);
    halLcdPrintLine("Calibrant...", 0, 0);

    // Llegir sensors múltiples vegades per estabilitzar
    int i;
    for (i = 0; i < 10; i++) {
        llegir_sensors();
        delay_ms(100);
    }

    halLcdPrintLine("Calibrat!", 1, 0);
    delay_ms(1000);
}

/*******************************************************
 * Buscar i seguir paret
 */

/**
 * @brief Busca una paret propera
 */
void buscar_paret(void) {
    halLcdClearScreen(0);
    halLcdPrintLine("Buscant paret...", 0, 0);

    while (!paret_trobada) {
        moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
        llegir_sensors();

        // Si algun sensor detecta un obstacle proper
        if (valors_sensors[0] > DISTANCIA_SEGURETAT ||
            valors_sensors[1] > DISTANCIA_SEGURETAT ||
            valors_sensors[2] > DISTANCIA_SEGURETAT) {
            paret_trobada = true;
            aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);

            // Determinar sentit de seguiment (per defecte dreta)
            if (valors_sensors[0] > valors_sensors[2]) {
                sentit_seguiment = 1; // Esquerra
            } else {
                sentit_seguiment = 0; // Dreta
            }
        }

        delay_ms(100);
    }

    halLcdPrintLine("Paret trobada!", 1, 0);
    delay_ms(1000);
}

/**
 * @brief Segueix la paret en el sentit determinat
 */
void seguir_paret(void) {
    char msg[16];
    sprintf(msg, "Seguint %s", sentit_seguiment ? "ESQUERRA" : "DRETA");
    halLcdClearScreen(0);
    halLcdPrintLine(msg, 0, 0);

    while (1) {
        llegir_sensors();

        // Cas 1: Obstacle al centre - girar per evitar
        if (valors_sensors[1] > DISTANCIA_SEGURETAT) {
            aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);
            halLcdPrintLine("Obstacle!", 1, 0);

            if (sentit_seguiment) {
                moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
            } else {
                moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
            }
            delay_ms(500);
        }
        // Cas 2: Seguiment normal de paret
        else {
            if (sentit_seguiment) {
                // Seguiment per esquerra
                if (valors_sensors[0] < DISTANCIA_SEGURETAT/2) {
                    // Massa lluny de la paret - girar cap a ella
                    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL+20, VELOCITAT_NORMAL);

                } else if (valors_sensors[0] > DISTANCIA_SEGURETAT*1.5) {
                    // Massa aprop de la paret - girar allunyant-se
                    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL, VELOCITAT_NORMAL+20);
                } else {
                    // Distància correcta - seguir recte
                    moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                }
            } else {
                // Seguiment per dreta
                if (valors_sensors[2] < DISTANCIA_SEGURETAT/2) {
                    // Massa lluny de la paret - girar cap a ella
                    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL, VELOCITAT_NORMAL+20);

                } else if (valors_sensors[2] > DISTANCIA_SEGURETAT*1.5) {
                    // Massa aprop de la paret - girar allunyant-se
                    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL+20, VELOCITAT_NORMAL);
                } else {
                    // Distància correcta - seguir recte
                    moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                }
            }
        }

        delay_ms(100);
    }
}

void seguir_paret_mejorado(void) {
    // Factors de correcció per ajustar la resposta segons distància
    float factor_cerca = 1.5f;
    float factor_lluny = 0.6f;

    // Llegir sensors abans d’actuar
    llegir_sensors();

    if (sentit_seguiment) {  // Seguir paret a l'ESQUERRA
        if (valors_sensors[0] < DISTANCIA_SEGURETAT * factor_lluny) {
            // Massa lluny de la paret - acostar-se
            moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID,
                          VELOCITAT_NORMAL + 50, VELOCITAT_NORMAL - 20);
        }
        else if (valors_sensors[0] > DISTANCIA_SEGURETAT * factor_cerca) {
            // Massa aprop - allunyar-se
            moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID,
                       VELOCITAT_NORMAL - 20, VELOCITAT_NORMAL + 50);
        } else {
            // Distància òptima - seguir recte
            moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
        }
    }
    else {  // Seguir paret a la DRETA
        if (valors_sensors[2] < DISTANCIA_SEGURETAT * factor_lluny) {
            // Massa lluny - acostar-se
            moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID,
                       VELOCITAT_NORMAL - 20, VELOCITAT_NORMAL + 50);
        }
        else if (valors_sensors[2] > DISTANCIA_SEGURETAT * factor_cerca) {
            // Massa aprop - allunyar-se
            moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID,
                          VELOCITAT_NORMAL + 50, VELOCITAT_NORMAL - 20);
        } else {
            // Distància òptima - seguir recte
            moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
        }
    }

    delay_ms(100);  // Petita pausa per estabilitat
}



typedef enum {
    BUSCANT_PARET,
    SEGUINT_PARET,
    EVITANT_OBSTACLE,
    ESCAPANT_CANTONADA,
    BLOQUEJAT_RECUPERANT
} EstatRobot;

bool sensor_frontal_detecta() {
    return valors_sensors[1] > DISTANCIA_SEGURETAT;
}

bool sensor_lateral_detecta() {
    return valors_sensors[0] > DISTANCIA_SEGURETAT || valors_sensors[2] > DISTANCIA_SEGURETAT;
}

bool cantonada_detectada() {
    #define UMBRAL_ESQUINA 120  // Más bajo que DISTANCIA_SEGURETAT

    bool frontal = valors_sensors[1] > DISTANCIA_SEGURETAT;
    bool lateral_esq = valors_sensors[0] > UMBRAL_ESQUINA;
    bool lateral_dret = valors_sensors[2] > UMBRAL_ESQUINA;

    return frontal && ((sentit_seguiment && lateral_esq) || (!sentit_seguiment && lateral_dret));
}


int determinar_sentit() {
    return valors_sensors[0] > valors_sensors[2] ? 1 : 0;
}

void girar_per_evitar(int sentit) {
    if (sentit) {
        moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
    } else {
        moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
    }
    delay_ms(400);
}


void control_robot_mejorado(void) {
    EstatRobot estat = BUSCANT_PARET;
    uint8_t bloquejos = 0;
    uint8_t intentos_esquina = 0;

    while (1) {
        llegir_sensors();

        switch (estat) {
            case BUSCANT_PARET:
                moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                if (sensor_frontal_detecta() || sensor_lateral_detecta()) {
                    aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);
                    sentit_seguiment = determinar_sentit();
                    estat = SEGUINT_PARET;
                    halLcdPrintLine("Paret trobada!", 1, 0);
                }
                break;

            case SEGUINT_PARET:
                if (cantonada_detectada()) {
                    estat = ESCAPANT_CANTONADA;
                    intentos_esquina = 0;
                } else if (sensor_frontal_detecta()) {
                    estat = EVITANT_OBSTACLE;
                } else {
                    // Seguimiento mejorado de pared
                    seguir_paret_mejorado();
                    bloquejos = 0;  // Reset si no está bloqueado
                }
                break;

            case EVITANT_OBSTACLE:
                aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);
                delay_ms(100);

                // Inicia un gir continuat
                while (sensor_frontal_detecta()) {
                    if (sentit_seguiment) {
                        // Girar dreta contínuament
                        moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, 50);
                    } else {
                        // Girar esquerra contínuament
                        moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 50, VELOCITAT_GIRO);
                    }
                    delay_ms(100);  // Pausa petita per estabilitat
                    llegir_sensors();  // Torna a llegir sensors per saber si l’obstacle ha desaparegut
                }

                // Quan l’obstacle frontal desapareix, seguir paret
                estat = SEGUINT_PARET;
                break;


            case ESCAPANT_CANTONADA:
                // 1. Detener completamente
                aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);
                delay_ms(200);

                // 2. Retroceder un poco
                moure_endarrere(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                delay_ms(600);

                // 3. Girar 90 grados pronunciado
                if (sentit_seguiment) {
                    // Girar derecha más pronunciado
                    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO/2);
                } else {
                    // Girar izquierda más pronunciado
                    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO/2, VELOCITAT_GIRO);
                }
                delay_ms(1000);  // Tiempo suficiente para 90 grados

                // 4. Avanzar un poco para alejarse de la esquina
                moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                delay_ms(300);

                bloquejos++;
                estat = (bloquejos > 2) ? BLOQUEJAT_RECUPERANT : SEGUINT_PARET;
                break;

            case BLOQUEJAT_RECUPERANT:
                // 1. Retroceder más
                moure_endarrere(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_NORMAL);
                delay_ms(1000);

                // 2. Girar 180 grados + pequeño ángulo aleatorio (15-30°)
                uint8_t direccion = rand() % 2;  // 0=derecha, 1=izquierda
                if (direccion) {
                    moure_dreta(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
                } else {
                    moure_esquerra(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, VELOCITAT_GIRO, VELOCITAT_GIRO);
                }
                delay_ms(1600 + (rand() % 300));  // 180° + aleatorio

                // 3. Cambiar sentido de seguimiento
                sentit_seguiment = !sentit_seguiment;
                bloquejos = 0;
                estat = BUSCANT_PARET;
                break;
        }

        delay_ms(50);  // Menor retardo para mayor capacidad de respuesta
    }
}


/**
 * main
 */

int main(void) {
    // Inicialitzacions bàsiques
    init_WDT();
    init_ucs_24MHz();
    init_timer_A1();
    init_LCD();
    init_UART(mode);

    config_angle_limit(MOTOR_DRET_ID);
    config_angle_limit(MOTOR_ESQUERRE_ID);

   // test_comunicacio_emulador();
   //test_robot_functions();

    // Mostrar noms del grup
    mostrar_noms_grup();

    // Calibrar sensors
    calibrar_sensors();

    // Habilitar interrupcions globals
    __enable_irq();

    //buscar_paret();
    //seguir_paret();

    // Cerca inicial de paret
    //buscar_paret2();

    // Iniciar seguiment de paret
    //seguir_paret2();

    control_robot_mejorado();


    // Bucle principal
    while (1) {
        //aturar_robot(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID);

        //moure_endavant(MOTOR_DRET_ID, MOTOR_ESQUERRE_ID, 1000);
        //__WFI();
    }
}
