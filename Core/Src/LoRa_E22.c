/*
 * lora.c
 *
 *  Created on: Feb 16, 2026
 *      Author: Berkay
 */

#include "LoRa_E22.h"

Lora_t *loraDevices[LORA_DEVICE_COUNT];
static uint8_t loraDeviceCount = 0;

// Static fonksiyon prototipleri
static void Lora_ProcessNormalData(Lora_t *lora, uint8_t byte);
static Lora_Status_t Lora_WaitCommandResponse(Lora_t *lora);

Lora_Status_t Lora_Init(Lora_t *lora, Lora_Init_t *initConfig, UartHandler_t *uartHandler)
{
    if(lora == NULL || initConfig == NULL || uartHandler == NULL)
        return LORA_STATUS_FAIL;

    // Init config'i kaydet
    lora->init = initConfig;
    lora->uartHandler = uartHandler;

    // State'leri başlat
    lora->parseMode = LORA_PARSE_MODE_NORMAL_DATA;
    lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
    lora->dataIndex = 0;
    lora->expectedLength = 0;
    lora->messageLength = 0;
    lora->dataReady = 0;
    lora->cmdResponseReady = 0;
    lora->moduleReady = 0;
    lora->RSSI = 0;

    // Device listesine ekle
    if(loraDeviceCount < LORA_DEVICE_COUNT)
    {
        loraDevices[loraDeviceCount++] = lora;
    }

    Lora_SetMode(lora, LORA_MODE_CONFIGURATION);
    HAL_Delay(100);
    Lora_GetConfig(lora);
    HAL_Delay(100);
    Lora_SetMode(lora, LORA_MODE_NORMAL);

    return LORA_STATUS_SUCCESS;
}

Lora_Status_t Lora_SetMode(Lora_t *lora, Lora_Mode_t newMode)
{
    if(lora == NULL)
        return LORA_STATUS_FAIL;

    // M0 ve M1 pinlerini ayarla
    switch(newMode)
    {
        case LORA_MODE_NORMAL:
            HAL_GPIO_WritePin(lora->init->m0Port, lora->init->m0Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(lora->init->m1Port, lora->init->m1Pin, GPIO_PIN_RESET);
            break;

        case LORA_MODE_WOR:
            HAL_GPIO_WritePin(lora->init->m0Port, lora->init->m0Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(lora->init->m1Port, lora->init->m1Pin, GPIO_PIN_RESET);
            break;

        case LORA_MODE_CONFIGURATION:
            HAL_GPIO_WritePin(lora->init->m0Port, lora->init->m0Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(lora->init->m1Port, lora->init->m1Pin, GPIO_PIN_SET);
            break;

        case LORA_MODE_DEEP_SLEEP:
            HAL_GPIO_WritePin(lora->init->m0Port, lora->init->m0Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(lora->init->m1Port, lora->init->m1Pin, GPIO_PIN_SET);
            break;

        default:
            return LORA_STATUS_INVALID_MODE;
    }

    //if(newMode == LORA_MODE_CONFIGURATION) {
    //    lora->uartHandler->uart->Init.BaudRate = 9600;
    //}
    //else {
    //    lora->uartHandler->uart->Init.BaudRate = lora->init->baudRate;
    //}
    lora->mode = newMode;
	HAL_Delay(10);
    return LORA_STATUS_SUCCESS;
}

void Lora_SetParseMode(Lora_t *lora, Lora_ParseMode_t newParseMode) {

	if(newParseMode == LORA_PARSE_MODE_COMMAND_RESPONSE) {
	    lora->parseMode = LORA_PARSE_MODE_COMMAND_RESPONSE;
	    lora->parseState = LORA_PARSE_STATE_CMD_RECEIVING;
	    lora->cmdReceivedBytes = 0;
	    lora->cmdResponseReady = 0;
	    lora->expectedLength = 0;
	    memset(lora->cmdBuffer, 0, LORA_MAX_BUFFER_SIZE);
	}
	else {
        lora->parseMode = LORA_PARSE_MODE_NORMAL_DATA;
        lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
	}
	return;

}

void Lora_Process(Lora_t *lora)
{
    if(lora == NULL)
        return;

    // Command bekleme sırasında Lora_Process çağrılırsa yoksay.
    // Byte'lar Lora_WaitCommandResponse tarafından tüketilir.
    if(lora->parseMode == LORA_PARSE_MODE_COMMAND_RESPONSE)
        return;

    uint8_t byte;
    while(Uart_ReadByte(lora->uartHandler, &byte))
    {
        Lora_ProcessNormalData(lora, byte);
    }
}

static void Lora_ProcessNormalData(Lora_t *lora, uint8_t byte)
{
    switch(lora->parseState)
    {
        case LORA_PARSE_STATE_WAITING_PREFIX:
            if(byte == lora->init->prefix)
            {
                lora->parseState = LORA_PARSE_STATE_WAITING_LENGTH;
                lora->dataIndex = 0;
            }
            break;

        case LORA_PARSE_STATE_WAITING_LENGTH:
            lora->expectedLength = byte;
            if(lora->expectedLength > 0 && lora->expectedLength <= LORA_MAX_BUFFER_SIZE)
            {
                lora->parseState = LORA_PARSE_STATE_RECEIVING_DATA;
            }
            else
            {
                lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
            }
            break;

        case LORA_PARSE_STATE_RECEIVING_DATA:
            lora->tempBuffer[lora->dataIndex++] = byte;

            if(lora->dataIndex >= lora->expectedLength)
            {
                lora->parseState = LORA_PARSE_STATE_WAITING_SUFFIX;
            }
            break;

        case LORA_PARSE_STATE_WAITING_SUFFIX:
            if(byte == lora->init->suffix)
            {
                if(lora->config.RSSIEnabled) {
                    lora->parseState = LORA_PARSE_STATE_WAITING_RSSI;
                }
                else {
                    memcpy(lora->rxBuffer, lora->tempBuffer, lora->expectedLength);
                    lora->messageLength = lora->expectedLength;
                    lora->dataReady = 1;
                    lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
                }
            }
            else
            {
                lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
            }
            break;

        case LORA_PARSE_STATE_WAITING_RSSI:
            lora->RSSI = byte;

            memcpy(lora->rxBuffer, lora->tempBuffer, lora->expectedLength);
            lora->messageLength = lora->expectedLength;
            lora->dataReady = 1;
            lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
            break;

        default:
            lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;
            break;
    }
}

static void Lora_ProcessCommandByte(Lora_t *lora, uint8_t byte)
{
    if(lora->parseState != LORA_PARSE_STATE_CMD_RECEIVING)
        return;

    lora->cmdBuffer[lora->cmdReceivedBytes++] = byte;

    if(lora->cmdReceivedBytes >= lora->cmdExpectedBytes)
    {
        lora->cmdResponseReady = 1;
        lora->parseState       = LORA_PARSE_STATE_CMD_COMPLETE;
    }
}

Lora_Status_t Lora_Write(Lora_t *lora, uint16_t targetAddress, uint8_t targetChannel, uint8_t *data, uint8_t length)
{
    if(lora == NULL || data == NULL)
        return LORA_STATUS_FAIL;

    if(lora->mode != LORA_MODE_NORMAL && lora->mode != LORA_MODE_WOR)
        return LORA_STATUS_INVALID_MODE;

    if(!Lora_IsModuleReady(lora))
        return LORA_STATUS_FAIL;

    if(length == 0 || length > LORA_MAX_BUFFER_SIZE)
        return LORA_STATUS_FAIL;

    // Paket formatı: [ADDH] [ADDL] [CHANNEL] [PREFIX] [LENGTH] [DATA] [SUFFIX]
    static uint8_t payload[LORA_MAX_BUFFER_SIZE + 7];
    uint8_t payloadIndex = 0;

    // Target Address (2 bytes)
    payload[payloadIndex++] = (uint8_t)((targetAddress >> 8) & 0xFF);  // ADDH
    payload[payloadIndex++] = (uint8_t)(targetAddress & 0xFF);         // ADDL

    // Target Channel
    payload[payloadIndex++] = targetChannel;

    // Prefix
    payload[payloadIndex++] = lora->init->prefix;

    // Length
    payload[payloadIndex++] = length;

    // Data
    memcpy(&payload[payloadIndex], data, length);
    payloadIndex += length;

    // Suffix
    payload[payloadIndex++] = lora->init->suffix;

    if(!Uart_Write(lora->uartHandler, payload, payloadIndex))
    {
        return LORA_STATUS_FAIL;
    }

    return LORA_STATUS_SUCCESS;
}

uint8_t Lora_Read(Lora_t *lora, uint8_t *data, uint8_t length)
{
    if(lora == NULL || data == NULL)
        return 0;

    if(!lora->dataReady || lora->messageLength == 0)
        return 0;

    if(length < lora->messageLength)
        return 0; // Buffer yetersiz

    memcpy(data, lora->tempBuffer, lora->messageLength);
    uint8_t readLength = lora->messageLength;

    // Buffer'ı temizle ve flag'leri sıfırla
    lora->dataReady = 0;
    lora->messageLength = 0;
    lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;

    return readLength;
}

uint8_t Lora_ReadCommandResponse(Lora_t *lora, uint8_t *response, uint8_t maxLen)
{
    if(lora == NULL || response == NULL || !lora->cmdResponseReady)
        return 0;

    // Header'ı çıkar (ilk 3 byte: CMD, REG, LEN)
    if(lora->cmdReceivedBytes < 3)
        return 0;  // Geçersiz response

    uint8_t dataLen = lora->cmdReceivedBytes - 3;  // Header'sız data uzunluğu
    uint8_t copyLen = (dataLen < maxLen) ? dataLen : maxLen;

    memcpy(response, lora->cmdBuffer + 3, copyLen);

    // Response okunduktan sonra temizle
    lora->cmdResponseReady = 0;
    lora->cmdReceivedBytes = 0;
    lora->cmdExpectedBytes = 0;
    memset(lora->cmdBuffer, 0, LORA_MAX_BUFFER_SIZE);
    lora->parseState = LORA_PARSE_STATE_WAITING_PREFIX;

    return copyLen;
}

Lora_Status_t Lora_SetConfig(Lora_t *lora, Lora_Config_t *config)
{
    if(lora == NULL || config == NULL)
        return LORA_STATUS_FAIL;

    // Config'i byte array'e dönüştür
    uint8_t configBytes[7];
    configBytes[0] = config->ADDH;
    configBytes[1] = config->ADDL;
    configBytes[2] = config->NETID;

    configBytes[3] = (config->BaudRate & 0x07) |
                     ((config->ParityBit & 0x03) << 3) |
                     ((config->AirDataRate & 0x07) << 5);

    configBytes[4] = (config->SubPacketSize & 0x03) |
                     ((config->AmbientRSSI & 0x01) << 2) |
                     ((config->Reserved & 0x07) << 3) |
                     ((config->TransmittingPower & 0x03) << 6);

    configBytes[5] = config->Channel;

    configBytes[6] = (config->RSSIEnabled & 0x01) |
                     ((config->FixedPointTransmission & 0x01) << 1) |
                     ((config->RepeaterEnabled & 0x01) << 2) |
                     ((config->LBTEnabled & 0x01) << 3) |
                     ((config->WORTransceiverControl & 0x01) << 4) |
                     ((config->WORCycle & 0x07) << 5);


    return Lora_WriteRegister(lora, LORA_REG_ADDH, 7, configBytes);
}

Lora_Status_t Lora_GetConfig(Lora_t *lora)
{
    if(lora == NULL)
        return LORA_STATUS_FAIL;

    uint8_t raw[7];
    Lora_Status_t status = Lora_ReadRegister(lora, LORA_REG_ADDH, 7, raw, sizeof(raw));
    if(status != LORA_STATUS_SUCCESS)
        return status;

    lora->config.ADDH    = raw[0];
    lora->config.ADDL    = raw[1];
    lora->config.NETID   = raw[2];

    lora->config.BaudRate    = (raw[3] >> 0) & 0x07;
    lora->config.ParityBit   = (raw[3] >> 3) & 0x03;
    lora->config.AirDataRate = (raw[3] >> 5) & 0x07;

    lora->config.SubPacketSize    = (raw[4] >> 0) & 0x03;
    lora->config.AmbientRSSI      = (raw[4] >> 2) & 0x01;
    lora->config.Reserved         = (raw[4] >> 3) & 0x07;
    lora->config.TransmittingPower = (raw[4] >> 6) & 0x03;

    lora->config.Channel = raw[5];

    lora->config.RSSIEnabled            = (raw[6] >> 0) & 0x01;
    lora->config.FixedPointTransmission = (raw[6] >> 1) & 0x01;
    lora->config.RepeaterEnabled        = (raw[6] >> 2) & 0x01;
    lora->config.LBTEnabled             = (raw[6] >> 3) & 0x01;
    lora->config.WORTransceiverControl  = (raw[6] >> 4) & 0x01;
    lora->config.WORCycle               = (raw[6] >> 5) & 0x07;

    return LORA_STATUS_SUCCESS;
}

uint8_t Lora_IsDataReady(Lora_t *lora)
{
    if(lora == NULL)
        return 0;

    return lora->dataReady;
}

uint8_t Lora_IsCommandResponseReady(Lora_t *lora)
{
    if(lora == NULL)
        return 0;

    return lora->cmdResponseReady;
}

uint8_t Lora_IsModuleReady(Lora_t *lora)
{
    if(lora == NULL)
        return 0;

    // AUX pin kontrolü yapılabilir
    return HAL_GPIO_ReadPin(lora->init->auxPort, lora->init->auxPin) == GPIO_PIN_SET;
}

static Lora_Status_t Lora_RegisterIO(Lora_t *lora, Lora_Command_t command, uint8_t registerAddress, uint8_t length, uint8_t *data)
{
    if(lora == NULL)
        return LORA_STATUS_FAIL;

    // Zaten bir command bekleniyorsa hata ver
    if(lora->parseMode == LORA_PARSE_MODE_COMMAND_RESPONSE)
        return LORA_STATUS_FAIL;

    static uint8_t cmd[3 + LORA_MAX_BUFFER_SIZE];
    uint8_t cmdLength           = 3;
    uint8_t expectedResponseBytes = 3 + length; // [CMD_ECHO][REG][LEN][DATA...]

    switch(command)
    {
        case LORA_COMMAND_READ_REG:
            cmd[0] = LORA_CMD_READ_REG;
            cmd[1] = registerAddress;
            cmd[2] = length;
            break;

        case LORA_COMMAND_SET_REG:
            cmd[0] = LORA_CMD_SET_REG;
            cmd[1] = registerAddress;
            cmd[2] = length;
            if(data != NULL) { memcpy(&cmd[3], data, length); cmdLength += length; }
            break;

        case LORA_COMMAND_SET_TEMP_REG:
            cmd[0] = LORA_CMD_SET_TEMP_REG;
            cmd[1] = registerAddress;
            cmd[2] = length;
            if(data != NULL) { memcpy(&cmd[3], data, length); cmdLength += length; }
            break;

        case LORA_COMMAND_WIRELESS_CONFIG:
            cmd[0] = LORA_CMD_WIRELESS_CONFIG;
            cmd[1] = registerAddress;
            cmd[2] = length;
            if(data != NULL) { memcpy(&cmd[3], data, length); cmdLength += length; }
            break;

        default:
            return LORA_STATUS_FAIL;
    }

    // Command mode'a geç
    Lora_SetParseMode(lora, LORA_PARSE_MODE_COMMAND_RESPONSE);
    lora->cmdExpectedBytes = expectedResponseBytes;

    // Komutu gönder
    if(!Uart_Write(lora->uartHandler, cmd, cmdLength))
    {
        Lora_SetParseMode(lora, LORA_PARSE_MODE_NORMAL_DATA);
        return LORA_STATUS_FAIL;
    }

    // Yanıtı blocking olarak bekle
    return Lora_WaitCommandResponse(lora);
}

Lora_Status_t Lora_WriteRegister(Lora_t *lora, uint8_t registerAddress,
                                  uint8_t length, uint8_t *parameter)
{
    if(parameter == NULL)
        return LORA_STATUS_FAIL;

    return Lora_RegisterIO(lora, LORA_COMMAND_SET_REG, registerAddress, length, parameter);
}

Lora_Status_t Lora_ReadRegister(Lora_t *lora, uint8_t registerAddress, uint8_t length, uint8_t *outBuffer, uint8_t outLen)
{
    if(outBuffer == NULL)
        return LORA_STATUS_FAIL;

    Lora_Status_t status = Lora_RegisterIO(lora, LORA_COMMAND_READ_REG,
                                           registerAddress, length, NULL);
    if(status != LORA_STATUS_SUCCESS)
        return status;

    // Response format: [CMD_ECHO][REG][LEN][DATA...]
    // İlk 3 byte header, geri kalanı data
    if(lora->cmdReceivedBytes < 3)
        return LORA_STATUS_WRONG_FORMAT;

    uint8_t dataLen = lora->cmdReceivedBytes - 3;
    uint8_t copyLen = (dataLen < outLen) ? dataLen : outLen;
    memcpy(outBuffer, lora->cmdBuffer + 3, copyLen);

    lora->cmdResponseReady = 0;

    return LORA_STATUS_SUCCESS;
}

static Lora_Status_t Lora_WaitCommandResponse(Lora_t *lora)
{
    uint32_t startTick = HAL_GetTick();
    uint8_t  byte;

    while(!lora->cmdResponseReady)
    {
        if(HAL_GetTick() - startTick > LORA_COMMAND_TIMEOUT_MS)
        {
            Lora_SetParseMode(lora, LORA_PARSE_MODE_NORMAL_DATA);
            return LORA_STATUS_TIMEOUT;
        }

        if(Uart_ReadByte(lora->uartHandler, &byte))
        {
            Lora_ProcessCommandByte(lora, byte);
        }
    }

    Lora_SetParseMode(lora, LORA_PARSE_MODE_NORMAL_DATA);
    return LORA_STATUS_SUCCESS;
}

void Lora_AUX_IRQHandler(uint16_t GPIO_Pin)
{
    // Hangi LoRa modülüne ait olduğunu bul ve update et
    for(uint8_t i = 0; i < loraDeviceCount; i++)
    {
        if(loraDevices[i]->init->auxPin == GPIO_Pin)
        {
            Lora_UpdateModuleStatus(loraDevices[i]);
            break;
        }
    }
}
