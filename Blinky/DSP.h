/******************************************************************************
 * @file		DSP.h
 * @brief		Файл подключения заголовочных файлов периферии и объявления системных констант
 * @version		v1.0
 * @date		11 декабря 2015
 *
 * @note
 * ООО "НПФ Вектор", все права защищены. Наш сайт: http://motorcontrol.ru
 *
 * @par
 * ООО "НПФ Вектор" распространяет это программное обеспечение в демонстрационных
 * целях, и оно может распространяться свободно.
 *
 * @par
 * Данное программное обеспечение распространяется "как есть", и Вы,
 * его пользователь, принимаете на себя все риски, связанные с его использованием.
 * ООО "НПФ Вектор" не несет никакой ответственности за возможные убытки,
 * связанные с его использованием.
 *
 ******************************************************************************/

#ifndef VINCLUDE_DSP_H_
#define VINCLUDE_DSP_H_

#define __CM4_REV               0x0001	//в K1921VK01T.h есть объявление __CM4F_REV, но в core_cm4.h проверяется именно __CM4_REV. Если не объявлено там объявляется нулем и больше нигде, вроде, не используется эта константа
#define __CHECK_DEVICE_DEFINES


#include <stdint.h>

#include "K1921VK035.h"
#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
#include <string.h>	 				//для memcpy

typedef char int8;
typedef unsigned char Uint8;
typedef unsigned short int Uint16;
typedef unsigned int Uint32;
typedef short int int16;
typedef int int32;
typedef unsigned long long Uint64;
typedef long long int64;

//Указатели на секцию памяти fastcode, где лежат функции для копирования из флеша в оперативку.
//Определены в файле компоновки памяти.
extern int __fastcode_ram_start;
extern int __fastcode_ram_end;
extern int __fastcode_flash_start;

extern int __isr_vector_flash_start;
extern int __isr_vector_ram_start;
extern int __isr_vector_ram_end;

//! Запрет прерываний
#define DINT __disable_irq()
//! Разрешение прерываний
#define EINT __enable_irq()

#define IRQ_PRIORITY_TZ 1
#define IRQ_PRIORITY_ADC 2
#define IRQ_PRIORITY_10K 3
#define IRQ_PRIORITY_CAP 4
#define IRQ_PRIORITY_CAN 5
#define IRQ_PRIORITY_1K 6
#define IRQ_PRIORITY_EQEP 6

#endif /* VINCLUDE_DSP_H_ */
