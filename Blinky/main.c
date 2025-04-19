/******************************************************************************
 * @file        main.c
 * @brief       Главный файл проекта
 * @version     v1.0
 * @date        19.04.25
 *
 * @note TDM LAB
 *
 * Пример работы цифровых входов и выходов
 * Настройка выхода тактового сигнала на вывод SERVEN
 * Настройка фильтра по цифровому входу
 ******************************************************************************/

#include "main.h"

#define PIN_PORT GPIOA
#define LED_PIN 15
#define BUTTON_PIN 14

#define BLINK_FAST 100000
#define BLINK_SLOW 500000


//Глобальная переменная периода счета таймера (может быть изменена при остановке расчета в окне просмотра переменных)
int TimerPeriod = BLINK_FAST;

// Глобальная переменная для хранения состояния светодиода
int ledState = 0;
int ButtonState = 0;
int ButtonClickFlag = 0;

//Функция мигания светодиодом
void LED_blink (void)
{
    static int timer;   //переменная таймера

    if (timer > TimerPeriod) {
        timer = 0;
        ledState ^= 0x1;

        if (ledState == 0x01)
            PIN_PORT->DATA &= ~(1 << LED_PIN);
        else
            PIN_PORT->DATA |= (1 << LED_PIN);
    }
    timer++;
}

int16 main (void){
    //Инициализация микроконтроллера: настройка таймеров, инициализаци периферийных устройств
    SystemInit();
/*
 *
    CLKOUTCTL Регистр настройки выдачи тактового сигнала, ножка SERVEN
    CLKOUTCFG Регистр настройки выходного тактового сигнала
    0x0101 выход PLL без деления PLLCLK
    0x0201 выход частоты кварца OSECLK
*/
    SIU->CLKOUTCTL = (1 << 0);
    RCU->CLKOUTCFG = 0x0101;

    //Инициализация периферии для управления диодами
    GPIOA->DENSET = (1 << LED_PIN | 1 << BUTTON_PIN);   // Регистр разрешения цифровой функции порта
    GPIOA->OUTENSET = (1 << LED_PIN);                   // Ножка настраивается на выход
    GPIOA->ALTFUNCCLR = (1 << LED_PIN);
    GPIOA->PULLMODE |= (1 << 28);                       // Подтягиваем PA14 Pull-up

    GPIOA->QUALSET |= (1 << BUTTON_PIN);                // Регистр включения фильтров портов
    GPIOA->QUALMODESET |= (1 << BUTTON_PIN);            // Режим измерения уровня входа по 6 отсчетам
    GPIOA->QUALSAMPLE = 255;                            // Регистр настройки фильтра порта, значение в тактах FCLK

    while(1)
    {
        if((GPIOA->DATA & (1 << BUTTON_PIN)) == 0 && ButtonClickFlag == 0) {
            ButtonClickFlag = 1;
            ButtonState++;
            if (ButtonState > 1){
                ButtonState = 0;
            }

            if(ButtonState == 1){
                TimerPeriod = BLINK_FAST;
                RCU->CLKOUTCFG = 0x0201;
            }
            else {
                TimerPeriod = BLINK_SLOW;
                RCU->CLKOUTCFG = 0x0101;
            }
        }
        else if (GPIOA->DATA & (1 << BUTTON_PIN)) {
            ButtonClickFlag = 0;
        }
        LED_blink();    // Мигание светодиодом
    }
}

/*@}*/

