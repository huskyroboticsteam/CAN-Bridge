#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cyapicallbacks.h"
#include "CAN_Stuff.h"
#include "FSM_Stuff.h"
#include "HindsightCAN/CANLibrary.h"

// Define node_id as per Odrive configuration
#define NODE_ID 0x3F // 63

// Buffer for UART output
char8 uart_tx[64];

void sprintCANPacket(CANPacket* packet, char* buffer) {
    // Extract priority (bit 10), device group (bits 9:6), and serial number (bits 5:0)
    uint8 pri = (packet->id & 0x400) >> 10; // Bit 10
    uint8 dg = (packet->id & 0x3C0) >> 6;   // Bits 9:6
    uint8 sn = (packet->id & 0x03F);        // Bits 5:0
    
    // Format ID components into buffer
    int len = sprintf(buffer, "%01X %02X %02X ", pri, dg, sn);
    
    // Append data bytes in hex format
    for (int i = 0; i < packet->dlc; i++) {
        len += sprintf(buffer + len, " %02X", packet->data[i]);
    }

    // Add newline
    sprintf(buffer + len, "\r\n");
}

void Initialize(void) {
    CyGlobalIntEnable; // Enable global interrupts
    InitCAN(); // Set up CAN interface
    CAN_Start();
    CAN_1_Start();
    DBG_UART_Start();
    DBG_UART_1_Start();
}

int main() {
    Initialize();
    Print("CAN initialized\r\n");

    // Send Set_Input_Vel command
    CANPacket cmd;
    uint8_t cmd_id = 0x0D; // Set_Input_Vel
    cmd.id = (NODE_ID << 5) | cmd_id; // CAN ID: (node_id << 5) | cmd_id
    cmd.dlc = 8; // 8-byte payload
    
    // Set velocity to 10.0 rev/s
    float velocity = 10.0f;
    memcpy(cmd.data, &velocity, 4); // First 4 bytes: velocity (float32)

    // Set Input_Torque_FF to 0.0
    float torque_ff = 0.0f;
    memcpy(cmd.data + 4, &torque_ff, 4); // Next 4 bytes: torque feedforward (float32)

    // Send the packet
    if (SendCANPacket(&cmd) == ERROR_NONE) {
        char odrivePacket[64];
        int len = sprintf(odrivePacket, "ID=0x%03X DLC=%u DATA=", cmd.id, cmd.dlc);
        for (int i = 0; i < cmd.dlc; ++i) {
            len += sprintf(odrivePacket + len, "%02X ", cmd.data[i]);
        }
        Print(odrivePacket);
        Print("\r\n");
        Print("ODrive velocity command sent\r\n");
    } else {
        Print("Send error\r\n");
    }
    
    // Loop to receive responses
    for (;;) {
        CANPacket rx_cmd;
        if (PollAndReceiveCANPacket(&rx_cmd) == ERROR_NONE) {
            sprintCANPacket(&rx_cmd, uart_tx);
            Print("Received: ");
            Print(uart_tx);
        }
    }

    return 0;
}
