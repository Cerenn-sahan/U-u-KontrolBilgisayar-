/*
 * LoRa_E22.h
 *
 *  Created on: Feb 16, 2026
 *      Author: Berkay
 */

#ifndef INC_LORA_E22_H_
#define INC_LORA_E22_H_

#include "stm32f4xx_it.h"
#include "main.h"
#include "RingBuffer.h"
#include <string.h>
#include "uart.h"

#define LORA_DEVICE_COUNT 2       // How many devices connected to the microcontroller
#define LORA_MAX_BUFFER_SIZE 240  // Max payload size for lora
#define LORA_COMMAND_TIMEOUT_MS 1000  // Timeout for command responses

/* --- DO NOT MODIFY BELOW THIS --- */

// REGISTERS
#define LORA_REG_ADDH         0x00 // ADDRESS HIGH
#define LORA_REG_ADDL         0x01 // ADDRESS LOW
#define LORA_REG_NETID        0x02
#define LORA_REG_REG0         0x03
#define LORA_REG_REG1         0x04
#define LORA_REG_REG2         0x05
#define LORA_REG_REG3         0x06
#define LORA_REG_CRYPT_H      0x07 // KEY HIGH
#define LORA_REG_CRYPT_L      0x08 // KEY LOW
#define LORA_REG_PRODUCT_INFO 0x80 // Product Information 0x80 - 0x86
// REGISTERS ONLY ACCESSIBLE IN NORMAL OR WOR MODE
#define LORA_REG_AMBIENT_RSSI 0x00
#define LORA_REG_RSSI         0x01

// COMMANDS
#define LORA_CMD_SET_REG        0xC0
#define LORA_CMD_READ_REG       0xC1
#define LORA_CMD_SET_TEMP_REG   0xC2
#define LORA_CMD_WIRELESS_CONFIG 0xC2

// BAUD RATE SETTINGS
#define LORA_UART_BAUD_RATE_1200   0X0
#define LORA_UART_BAUD_RATE_2400   0X1
#define LORA_UART_BAUD_RATE_4800   0X2
#define LORA_UART_BAUD_RATE_9600   0X3
#define LORA_UART_BAUD_RATE_19200  0X4
#define LORA_UART_BAUD_RATE_38400  0X5
#define LORA_UART_BAUD_RATE_57600  0X6
#define LORA_UART_BAUD_RATE_115200 0X7

// AIR DATA RATE SETTINGS
#define LORA_AIR_DATA_RATE_0_3  0x0
#define LORA_AIR_DATA_RATE_1_2  0x1
#define LORA_AIR_DATA_RATE_2_4  0x2
#define LORA_AIR_DATA_RATE_4_8  0x3
#define LORA_AIR_DATA_RATE_9_6  0x4
#define LORA_AIR_DATA_RATE_19_2 0x5
#define LORA_AIR_DATA_RATE_38_4 0x6
#define LORA_AIR_DATA_RATE_62_5 0x7

// PACKET SIZE SETTINGS
#define LORA_SUB_PACKET_SIZE_240 0x0
#define LORA_SUB_PACKET_SIZE_128 0x1
#define LORA_SUB_PACKET_SIZE_64  0x2
#define LORA_SUB_PACKET_SIZE_32  0x3

// TRANSMITTING POWER SETTINGS
#define LORA_TRANSMITTING_POWER_30 0x0
#define LORA_TRANSMITTING_POWER_27 0x1
#define LORA_TRANSMITTING_POWER_24 0x2
#define LORA_TRANSMITTING_POWER_21 0x3

#define LORA_INVALID_FORMAT_RESPONSE ((const char[]){0xFF, 0xFF, 0xFF})

#define DEFAULT_PREFIX 0xAE
#define DEFAULT_SUFFIX 0xDD


typedef enum {
    LORA_MODE_NORMAL = 0,
    LORA_MODE_WOR,
    LORA_MODE_CONFIGURATION,
    LORA_MODE_DEEP_SLEEP
} Lora_Mode_t;

typedef enum {
    LORA_COMMAND_READ_REG,
    LORA_COMMAND_SET_REG,
    LORA_COMMAND_SET_TEMP_REG,
    LORA_COMMAND_WIRELESS_CONFIG
} Lora_Command_t;

typedef enum {
    LORA_STATUS_SUCCESS,
    LORA_STATUS_FAIL,
    LORA_STATUS_TIMEOUT,
    LORA_STATUS_WRONG_FORMAT,
    LORA_STATUS_INVALID_MODE,
    LORA_STATUS_INVALID_CONFIG
} Lora_Status_t;

typedef enum {
    LORA_PARSE_MODE_NORMAL_DATA,      // Normal veri paketi (prefix/suffix ile)
    LORA_PARSE_MODE_COMMAND_RESPONSE  // Command yanıtı (raw bytes)
} Lora_ParseMode_t;

typedef enum {
    // Normal data parsing states
    LORA_PARSE_STATE_WAITING_PREFIX,
    LORA_PARSE_STATE_WAITING_LENGTH,
    LORA_PARSE_STATE_RECEIVING_DATA,
    LORA_PARSE_STATE_WAITING_SUFFIX,
    LORA_PARSE_STATE_WAITING_RSSI,

    // Command response parsing states
    LORA_PARSE_STATE_CMD_RECEIVING,
    LORA_PARSE_STATE_CMD_COMPLETE
} Lora_ParseState_t;

typedef struct {
    uint8_t ADDH;
    uint8_t ADDL;
    uint8_t NETID;
    uint8_t AirDataRate: 3;
    uint8_t ParityBit: 2;
    uint8_t BaudRate: 3;
    uint8_t TransmittingPower: 2;
    uint8_t Reserved: 3;
    uint8_t AmbientRSSI: 1;
    uint8_t SubPacketSize: 2;
    uint8_t Channel;
    uint8_t WORCycle: 3;
    uint8_t WORTransceiverControl: 1;
    uint8_t LBTEnabled: 1;
    uint8_t RepeaterEnabled: 1;
    uint8_t FixedPointTransmission: 1;
    uint8_t RSSIEnabled: 1;
    uint16_t KEY;
} Lora_Config_t;

typedef struct {
    UART_HandleTypeDef *uart;
    uint32_t baudRate;
    GPIO_TypeDef *m0Port;
    GPIO_TypeDef *m1Port;
    GPIO_TypeDef *auxPort;
    uint16_t m0Pin;
    uint16_t m1Pin;
    uint16_t auxPin;
    uint16_t bufferSize;
    uint8_t prefix;
    uint8_t suffix;
} Lora_Init_t;

typedef struct {
    // Hardware configuration
    Lora_Init_t *init;
    UartHandler_t *uartHandler;

    // Module state
    Lora_Mode_t mode;
    Lora_Config_t config;

    // Parser state machine
    Lora_ParseMode_t parseMode;
    Lora_ParseState_t parseState;
    uint8_t dataIndex;
    uint8_t expectedLength;

    // Data buffers
    uint8_t rxBuffer[LORA_MAX_BUFFER_SIZE];
    uint8_t tempBuffer[LORA_MAX_BUFFER_SIZE];
    uint8_t messageLength;

    // Command buffers
    uint8_t cmdBuffer[LORA_MAX_BUFFER_SIZE];
    uint8_t cmdExpectedBytes;
    uint8_t cmdReceivedBytes;
    uint32_t cmdStartTick;

    // Status flags
    uint8_t dataReady;
    uint8_t cmdResponseReady;
    uint8_t moduleReady;

    // RSSI
    uint8_t RSSI;
} Lora_t;

Lora_Status_t Lora_Init(Lora_t *lora, Lora_Init_t *initConfig, UartHandler_t *uartHandler);

Lora_Status_t Lora_SetMode(Lora_t *lora, Lora_Mode_t newMode); // TODO: baud rate change

Lora_Status_t Lora_SetConfig(Lora_t *lora, Lora_Config_t *config);
Lora_Status_t Lora_GetConfig(Lora_t *lora);

Lora_Status_t Lora_Write(Lora_t *lora, uint16_t targetAddress, uint8_t targetChannel, uint8_t *data, uint8_t length);

uint8_t Lora_Read(Lora_t *lora, uint8_t *data, uint8_t length);
uint8_t Lora_ReadCommandResponse(Lora_t *lora, uint8_t *response, uint8_t maxLen);

//Lora_Status_t Lora_GetAmbientRSSI(Lora_t *lora, uint8_t *ambientRssi);

Lora_Status_t Lora_WriteRegister(Lora_t *lora, uint8_t registerAddress, uint8_t length, uint8_t *parameter);
Lora_Status_t Lora_ReadRegister(Lora_t *lora, uint8_t registerAddress, uint8_t length, uint8_t *outBuffer, uint8_t outLen);

Lora_Status_t Lora_UpdateModuleStatus(Lora_t *lora);
uint8_t Lora_IsModuleReady(Lora_t *lora);
uint8_t Lora_IsDataReady(Lora_t *lora);
uint8_t Lora_IsCommandResponseReady(Lora_t *lora);

void Lora_Process(Lora_t *lora);
void Lora_AUX_IRQHandler(uint16_t GPIO_Pin);

#endif /* INC_LORA_E22_H_ */
