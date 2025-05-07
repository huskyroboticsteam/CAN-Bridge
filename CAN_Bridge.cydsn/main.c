/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "main.h"
#include "cyapicallbacks.h"
#include "CAN_Stuff.h"
#include "FSM_Stuff.h"
#include "HindsightCAN/CANLibrary.h"
#define UNKNOWN_PACKET 0xFFFF

// testing out the headers
#include "cytypes.h"
#include "cyfitter.h"
#include "cydevice_trm.h"

// LED stuff
volatile uint8_t CAN_time_LED = 0;
volatile uint8_t ERROR_time_LED = 0;
uint8 uart_rx_len = 0;

// UART stuff
char txData[TX_DATA_SIZE];

// CAN stuff
//CANPacket can_recieve;
// CANPacket can_send;
// uint8 address = 0;

// from CAN packet
char8 uart_tx[64];
char8 uart_rx[64];
CANPacket can_tx;
CANPacket can_rx;



CY_ISR(Period_Reset_Handler) {
    CAN_time_LED++;
    ERROR_time_LED++;
    
    /*if (ERROR_time_LED >= 3) {
        LED_ERR_Write(OFF);
    }
    if (CAN_time_LED >= 3) {
        LED_CAN_Write(OFF);
    }*/
}

/*
CY_ISR(Button_1_Handler) {
    LED_DBG_Write(!LED_DBG_Read());
}
*/

uint8 s2x(char8 c) {
    switch(c) {
        case '1': return 1; case '2': return 2;
        case '3': return 3; case '4': return 4;
        case '5': return 5; case '6': return 6;
        case '7': return 7; case '8': return 8;
        case '9': return 9; case 'A': return 10;
        case 'B': return 11;case 'C': return 12;
        case 'D': return 13;case 'E': return 14;
        case 'F': return 15;case 'a': return 10;
        case 'b': return 11;case 'c': return 12;
        case 'd': return 13;case 'e': return 14;
        case 'f': return 15;default:  return 0;
    }
}

void parseLine(CANPacket* p, char8 line[], int length) {
    uint16 pr = (uint16) s2x(line[0]) << 10;
    uint16 dg = (uint16) s2x(line[2]) << 6;
    uint16 sn = (uint16) s2x(line[4]) << 4 | s2x(line[5]);
    p->id = (pr | dg | sn);
    
    int i = 0;
    int j = 8;
    while (j < length && i < 8) {
        p->data[i] = (s2x(line[j-1])<<4) | s2x(line[j]);
        j += 3;
        i++;
    }
    p->dlc = i;
}

/*uint32_t decodeFromBytes(int msb_index, int lsb_index, uint8_t data[]) {
    uint32_t result = 0;
    if (msb_index < lsb_index)
        for (int i = msb_index; i <= lsb_index; i++)
            result = result << 8 | data[i];
    else
        for (int i = msb_index; i >= lsb_index; i--)
            result = (result | data[i]) << 8;
    return result;
}*/

void sprintCANPacket(CANPacket* packet, char* buffer) {
    uint8 pri = (packet->id & 0x400) >> 10;
    uint8 dg = (packet->id & 0x3C0) >> 6;
    uint8 sn = (packet->id & 0x03F) >> 0;
    
    int len = sprintf(buffer, "%01X %02X %02X ", pri, dg, sn);
    for(int i = 0; i < packet->dlc; i++)
        len += sprintf(buffer+len," %02X", packet->data[i]);

    sprintf(buffer+len,"\r\n");
}

void DebugPrint(char input) {
    switch(input) {
        case 'f':
            sprintf(txData, "Mode: %x State:%x \r\n", GetMode(), GetState());
            break;
        case 'x':
            sprintf(txData, "bruh\r\n");
            break;
        default:
            sprintf(txData, "what\r\n");
            break;
    }
    Print(txData);
}

void DisplayErrorCode(uint8_t code) {    
    ERROR_time_LED = 0;
    LED_ERR_Write(ON);
    
    Print("check if code is entering this block");
    
    sprintf(txData, "Error %X\r\n", code);
    Print(txData);
    
    switch(code)
    {
        case ERROR_INVALID_TTC:
            Print("Cannot Send That Data Type!\n\r");
            break;
        default:
            //some error
            break;
    }
}

// remove a occurrences of serial address, since not mentioned on the board
void Initialize(void) {
    CyGlobalIntEnable; /* Enable global interrupts. LED arrays need this first */
    
    // address = getSerialAddress();
    InitCAN(0x04, 0xF5);
    CAN_Start();
    CAN_1_Start();
    
    DBG_UART_Start();
    DBG_UART_1_Start();
    
    Print(txData);
    
    // LED_DBG_Write(0);
    
    // InitCAN(0x4, (int)address);
    // InitCAN();
    
    Timer_Period_Reset_Start();
    isr_Period_Reset_StartEx(Period_Reset_Handler);
}

/*int main(void)
{ 
    // intialize the two CAN blocks here
   
    Initialize();
    // toggle for debugging
    
    // int err;
    // we can read CAN packets and pass it into uart first
    Print("format:\r\n");
    Print("X XX XX  XX XX XX XX XX XX XX XX\r\n");
    Print("p |  |   |  |  |  |  |  |  |  |\r\n");
    Print("  dg |   |  |  |  |  |  |  |  |\r\n");
    Print("     sn  |  |  |  |  |  |  |  |\r\n");
    Print("         0  1  2  3  4  5  6  7 (data)\r\n");
    
    for(;;)
    {   
        if (CAN_time_LED > 0) {
            LED_CAN_Write(0);
            CAN_time_LED--;
        } else {
            LED_CAN_Write(0);
        }
        
        uint32 c = DBG_UART_UartGetChar();
        if (c) {
            if (c == '\r') {
                parseLine(&can_tx, uart_rx+1, uart_rx_len-1);
                if (SendCANPacket(&can_tx) == 0) {
                    sprintCANPacket(&can_tx, uart_tx);
                    Print("sent ");
                    Print(uart_tx);
                } else {
                    Print("Epic FAIL\r\n");
                }
            } else {
                uart_rx[uart_rx_len] = c;
                uart_rx_len++;
            }
        }
        
        int rx_result = PollAndReceiveCANPacket(&can_rx);
        if (rx_result == ERROR_NONE) {
            
            LED_CAN_Write(1); // Turn LED OFF (active low)
            sprintCANPacket(&can_rx, uart_tx);
            Print(uart_tx);
            
        } /*else {
            // Periodically show error code (not on every loop to avoid flooding UART)
            static uint32_t error_print_counter = 0;
            if (error_print_counter++ % 100 == 0) { // Print every 100th error
                sprintf(uart_tx, "CAN RX Error: 0x%02X\r\n", rx_result);
                Print(uart_tx);
            }
        }
        
        CyDelay(100);
    }
}
*/
char odrivePacket[64];
int main() {
    CyGlobalIntEnable;

    // initialize the CAN block
    CAN_Start();           // PSoC-Designer name for the CAN Top-Level
    CAN_1_Start();         // if you have a second instance
    InitCAN();             // our helper: sets up mailbox 0, mask=0, IRQ, etc.

    DBG_UART_Start();      // for debug prints
    Print("CAN initialized\r\n");
    
    // now send a test command
    CANPacket cmd;
    cmd.id  = (0x04 << 4) | 0x0D;   // node_id=0x01 (axis 0), cmd_id=0x0D (Set Input Velocity)
    cmd.dlc = 6;                    // ODrive velocity packets use 6 bytes
    // fill 6‐byte little endian velocity, e.g. 10000 → 0x002710
    int32_t velocity = 10000;
    for (int i = 0; i < 4; ++i) {
        cmd.data[i] = (velocity >> (8*i)) & 0xFF;
    }
    // two extra zero bytes
    cmd.data[4] = 0; 
    cmd.data[5] = 0;
    
    
    
    if (SendCANPacket(&cmd) == ERROR_NONE) {
        int len = sprintf(odrivePacket, "ID=0x%03X DLC=%u DATA=", cmd.id, cmd.dlc);
        // now append each data byte in hex
        for (int i = 0; i < cmd.dlc; ++i) {
            len += sprintf(odrivePacket + len, "%02X ", cmd.data[i]);
        }
        // print it out
        Print(odrivePacket);
        Print("\r\n");
        Print("ODrive velocity command sent\r\n");
    } else {
        Print("Send error\r\n");
    }

    for(;;) {
        // optionally poll for responses
        if (PollAndReceiveCANPacket(&cmd) == ERROR_NONE) {
            LED_CAN_1_Write(0);
            sprintCANPacket(&cmd, uart_tx);
            Print(uart_tx);
            LED_CAN_1_Write(1);
        }
    }
}


/* [] END OF FILE */
