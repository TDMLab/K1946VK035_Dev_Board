/******************************************************************************
 * @file        main.c
 * @brief       Главный файл проекта
 * @version     v1.0
 * @date        25.05.25
 *
 * @note TDM LAB
 *
 * Пример передачи стандартных CAN-сообщений
 ******************************************************************************/

//-- Includes ------------------------------------------------------------------
#include "K1921VK035.h"
#include <stdio.h>
//-- Defines -------------------------------------------------------------------
//-- Types ---------------------------------------------------------------------

#define PIN_PORT GPIOA
#define LED_PIN 15
#define BUTTON_PIN 14

#define CAN_DATA_LENGTH 8

typedef enum {
    CAN0,
    CAN1,
} CAN_Port_TypeDef;
typedef enum {
    CAN_OPERATION_TX,
    CAN_OPERATION_RX,
    CAN_OPERATION_TXRX
} CAN_Operation_TypeDef;
typedef enum {
    CAN_MESSAGE_REMOTE,
    CAN_MESSAGE_COMMON
} CAN_Message_TypeDef;

//-- Variables -----------------------------------------------------------------
uint32_t counter = 0;
uint8_t ledState = 0;
uint32_t Data_L;
uint32_t Data_H;

static uint32_t OK_MODATAL = 0;
static uint32_t ERR_MODATAL = 0;
static uint32_t OK_MODATAH = 0;
static uint32_t ERR_MODATAH = 0;
volatile static uint32_t IRQ_COUNT = 0;

void LED_blink (void) {
    if (ledState == 0) {
        PIN_PORT->DATA &= ~(1 << LED_PIN);
        ledState = 1;
    }
    else {
        PIN_PORT->DATA |= (1 << LED_PIN);
        ledState = 0;
    }
}

//-- CAN service functions -----------------------------------------------------
void CAN_Object_Location(uint32_t obj_first_num, uint32_t obj_last_num, uint32_t list_num) {
    unsigned int x;
    // LOCATION OBJECTS TO THE LISTS
    for (x = obj_first_num; x <= obj_last_num; x++) {
    // PANCMD_field=0x02-static location objects to one of the CAN-lists
        CAN->PANCTR = (0x2 << CAN_PANCTR_PANCMD_Pos) |
                      (x << CAN_PANCTR_PANAR1_Pos) |
                      (list_num << CAN_PANCTR_PANAR2_Pos);
        while ((CAN->PANCTR_bit.BUSY) | (CAN->PANCTR_bit.RBUSY)) {
        };
    }
}

void CAN_Object_Config(uint32_t obj_num, CAN_Operation_TypeDef op_type, CAN_Message_TypeDef msg_type) {
    if (op_type == CAN_OPERATION_TX) {
        if (msg_type == CAN_MESSAGE_COMMON)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN0_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN1_Msk;
        else if (msg_type == CAN_MESSAGE_REMOTE)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_RESDIR_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN0_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN1_Msk;
    } else if (op_type == CAN_OPERATION_RX) {
        if (msg_type == CAN_MESSAGE_COMMON)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_RESDIR_Msk | CANMSG_Msg_MOCTR_SETRXEN_Msk;
        else if (msg_type == CAN_MESSAGE_REMOTE)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk | CANMSG_Msg_MOCTR_SETRXEN_Msk;
    } else if (op_type == CAN_OPERATION_TXRX) {
        if (msg_type == CAN_MESSAGE_COMMON)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk | CANMSG_Msg_MOCTR_SETTXEN0_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN1_Msk | CANMSG_Msg_MOCTR_SETRXEN_Msk;
        else if (msg_type == CAN_MESSAGE_REMOTE)
            CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_RESDIR_Msk | CANMSG_Msg_MOCTR_SETTXEN0_Msk |
                                         CANMSG_Msg_MOCTR_SETTXEN1_Msk | CANMSG_Msg_MOCTR_SETRXEN_Msk;
    }
}

void CAN_Object_Transmit(uint32_t obj_num) {
    CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_SETTXRQ_Msk | CANMSG_Msg_MOCTR_SETMSGVAL_Msk;
}
void CAN_Object_Receive(uint32_t obj_num) {
    CANMSG->Msg[obj_num].MOCTR = CANMSG_Msg_MOCTR_SETMSGVAL_Msk;
}

//-- Peripheral init functions -------------------------------------------------
void can_init() {
    // IO
    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;
    GPIOB->ALTFUNCSET = (1 << 14 | 1 << 15);
    GPIOB->DENSET = (1 << 14 | 1 << 15);
    // Clock and reset
    RCU->HCLKCFG_bit.CANEN = 1;
    RCU->HRSTCFG_bit.CANEN = 1;
    CAN->CLC_bit.DISR = 0;
    while ((CAN->CLC_bit.DISS) & (CAN->PANCTR_bit.PANCMD)) {
    };
    CAN->FDR = (0x01 << CAN_FDR_DM_Pos) | (0x3FF << CAN_FDR_STEP_Pos); // normal divider mode
    // Enable the change configuration of the CAN node's
    // CAN0 are disconnected from the bus
    CAN->Node[CAN0].NCR = CAN_Node_NCR_CCE_Msk | CAN_Node_NCR_INIT_Msk;
    CAN->Node[CAN0].NPCR = 0;
    CAN->Node[CAN0].NBTR = (0 << CAN_Node_NBTR_DIV8_Pos) | (1 << CAN_Node_NBTR_TSEG2_Pos) | (6 << CAN_Node_NBTR_TSEG1_Pos) |
                        (1 << CAN_Node_NBTR_SJW_Pos) | (9 << CAN_Node_NBTR_BRP_Pos);
    CAN->Node[CAN0].NCR &= !(CAN_Node_NCR_CCE_Msk | CAN_Node_NCR_INIT_Msk);

//    CAN->Node[0].NCR = CAN_Node_NCR_TRIE_Msk;
// choosing number lines for node's interrupts

// NVIC interrupts
//    NVIC_EnableIRQ(CAN0_IRQn);
}

//-- Main ----------------------------------------------------------------------
int main()
{
    SystemInit();

    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;

    GPIOA->DENSET = (1 << LED_PIN | 1 << BUTTON_PIN);   // Регистр разрешения цифровой функции порта
    GPIOA->OUTENSET = (1 << LED_PIN);                   // Ножка настраивается на выход
    GPIOA->ALTFUNCCLR = (1 << LED_PIN);

    can_init();

    while (CAN->PANCTR_bit.BUSY) {
    };
    // Location 0-3 objects to the 1 list (0 node)
    CAN_Object_Location(0, 1, 1);
    // Location 58-60 objects to the 2 list (1 node)
//    CAN_Object_Location(58, 60, 2);

    //Build CAN0 FIFO with 0 - FIFO basic object, 1-3 - FIFO slave objects
//    CANMSG->Msg[0].MOFCR_bit.MMC = CANMSG_Msg_MOFCR_MMC_TXObj;
//    CANMSG->Msg[0].MOFGPR_bit.CUR = 0;
//    CANMSG->Msg[0].MOFGPR_bit.BOT = 0;
//    CANMSG->Msg[0].MOFGPR_bit.SEL = 0;
//    CANMSG->Msg[0].MOFGPR_bit.TOP = 1;
//    CANMSG->Msg[0].MOCTR = CANMSG_Msg_MOCTR_SETMSGVAL_Msk;
//    CANMSG->Msg[0].MOFCR_bit.OVIE = 1; // interrupt on FIFO overflow (MOFGPR.SEL==MOFGPR.CUR)
//    CANMSG->Msg[0].MOIPR_bit.RXINP = 2; // line 2
//    temp_MOAR = (0x2 << CANMSG_Msg_MOAR_PRI_Pos) | // filtration by identifier
//                CANMSG_Msg_MOAR_IDE_Msk;           // extended identifier

//    Data_L = 0x78563412;
//    Data_H = 0xF0DEBC9A;
//    CANMSG->Msg[0].MODATAL = Data_L;
//    CANMSG->Msg[0].MODATAH = Data_H;

//    Data_L = 0xA0A0A0A0;
//    Data_H = 0xF0F0F0F0;
//    CANMSG->Msg[1].MODATAL = Data_L;
//    CANMSG->Msg[1].MODATAH = Data_H;

        CAN_Object_Config(0, CAN_OPERATION_TX, CAN_MESSAGE_COMMON);
        CANMSG->Msg[0].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk | CANMSG_Msg_MOCTR_SETTXEN0_Msk | CANMSG_Msg_MOCTR_SETTXEN1_Msk;
        CANMSG->Msg[0].MOFCR = (CAN_DATA_LENGTH << CANMSG_Msg_MOFCR_DLC_Pos) | CANMSG_Msg_MOFCR_TXIE_Msk | CANMSG_Msg_MOFCR_RXIE_Msk;
        CANMSG->Msg[0].MOFCR = CANMSG->Msg[0].MOFCR | CANMSG_Msg_MOFCR_STT_Msk; // Бит задания однократной пересылки данных
        CANMSG->Msg[0].MOAR = (2 << CANMSG_Msg_MOAR_PRI_Pos) | (0 << CANMSG_Msg_MOAR_IDE_Pos) | (0x77700000 >> 2);


//        CAN_Object_Config(1, CAN_OPERATION_TX, CAN_MESSAGE_COMMON);
//        CANMSG->Msg[1].MOCTR = CANMSG_Msg_MOCTR_SETDIR_Msk | CANMSG_Msg_MOCTR_SETTXEN0_Msk | CANMSG_Msg_MOCTR_SETTXEN1_Msk;
//        CANMSG->Msg[1].MOFCR = (CAN_DATA_LENGTH << CANMSG_Msg_MOFCR_DLC_Pos) | CANMSG_Msg_MOFCR_TXIE_Msk | CANMSG_Msg_MOFCR_RXIE_Msk;
//        CANMSG->Msg[1].MOFCR = CANMSG->Msg[1].MOFCR | CANMSG_Msg_MOFCR_STT_Msk; // Бит задания однократной пересылки данных
//        CANMSG->Msg[1].MOAR = (2 << CANMSG_Msg_MOAR_PRI_Pos) | (0 << CANMSG_Msg_MOAR_IDE_Pos) | (0x66600000 >> 2);

    while (1) {
        LED_blink();

//        Data_L = 0xA0A0A0A0;
//        Data_H = 0xF0F0F0F0;
//        CANMSG->Msg[0].MODATAL = Data_L;
//        CANMSG->Msg[0].MODATAH = Data_H;
//        CAN_Object_Transmit(0);

        while(counter < 0xFFF) {
            counter++;
        }
        counter = 0;

        Data_L = 0x78563412;
        Data_H = 0xF0DEBC9A;
        CANMSG->Msg[0].MODATAL = Data_L;
        CANMSG->Msg[0].MODATAH = Data_H;
        CAN_Object_Transmit(0);
    }
}
//-- IRQ handlers --------------------------------------------------------------
// objects
void CAN0_IRQHandler(void)
{
    IRQ_COUNT++;
}
