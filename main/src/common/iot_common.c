#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "iot_common.h"

const uint16_t polynom = 0xA001;
RingBuffer uart2Buffer;
RingBuffer dataRegisterBuffer;

static int CharToInt(char ch)  
{  
        // 如果是数字，则用数字的ASCII码减去48, 如果ch = '2' ,则 '2' - 48 = 2  
        if(isdigit(ch)) {
			return ch - 48;  
		}
  
        // 如果是字母，但不是A~F,a~f则返回  
        if( ch < 'A' || (ch > 'F' && ch < 'a') || ch > 'z' ) {
			return -1; 
		}
  
        // 如果是大写字母，则用数字的ASCII码减去55, 如果ch = 'A' ,则 'A' - 55 = 10  
        // 如果是小写字母，则用数字的ASCII码减去87, 如果ch = 'a' ,则 'a' - 87 = 10  
        if(isalpha(ch))  {
			return isupper(ch) ? ch - 55 : ch - 87;  
		}
  
        return -1;  
} 

uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;
 
	if (len == 0) {
		len = 1;
	}
	while (len--) {
		crc ^= *ptr;
		for (i = 0; i<8; i++)
		{
			if (crc & 1) {
				crc >>= 1;
				crc ^= polynom;
			}
			else {
				crc >>= 1;
			}
		}
		ptr++;
	}
	return(crc);
}

int CheckCRC16(uint8_t *ptr, uint16_t len, uint16_t rcrc)
{
    uint16_t crc;

    crc = crc16bitbybit(ptr, len);
    if (rcrc != crc) {
        return -1; 
    }

    return 0;
}

uint16_t CalSumCheckData(uint8_t *data, uint16_t len)
{
    int i, num, sum = 0;
    char str[8] = {0};
    uint16_t checksum = 0;

    for (i = 0; i < len; i++) {
        sum += data[i]; 
    }

    num = sprintf(str, "%x", sum);
    for (i = 0; i < num; i++) {
        if (str[i] > 'F') {
            str[i] -= 32;
        }
    }

    if (num >= 2) {
        memcpy(&checksum, &str[num - 2], sizeof(checksum));
    }

    return checksum;
}

int CheckSumData(uint8_t *data, uint16_t len, uint16_t checksum)
{
    uint16_t check;

    check = CalSumCheckData(data, len);
    if (check != checksum) {
        return -1;
    }

    return 0;
}

uint16_t CalReadDataRegister(uint8_t *data)
{
	uint16_t rdata;

	rdata = CharToInt(data[2]) * 4096 + CharToInt(data[3]) * 256 + CharToInt(data[0]) * 16 + CharToInt(data[1]);
	return rdata;
}

uint16_t CalSerialReadDataRegister(uint8_t *data)
{
	uint16_t rdata;

	rdata = CharToInt(data[0]) * 4096 + CharToInt(data[1]) * 256 + CharToInt(data[2]) * 16 + CharToInt(data[3]);
	return rdata;
}

int UART_InitBuffer(void)
{
    if (RING_InitBuffer(&uart2Buffer, UART_BUFF_SIZE) != 0) {
        return -1;
    }

    return 0;
}

int UART_WriteBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_WriteBufferBytes(&uart2Buffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

int UART_ReadBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_ReadBufferBytes(&uart2Buffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

int FXPLC_InitBuffer(void)
{
    if (RING_InitBuffer(&dataRegisterBuffer, FXPLC_BUFF_SIZE) != 0) {
        return -1;
    }

    return 0;
}

int FXPLC_WriteBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_WriteBufferBytes(&dataRegisterBuffer, data, size) != 0) {
        return -1;
    }
	return 0;
}

int FXPLC_ReadBufferBytes(uint8_t *data, uint32_t size)
{
    if (RING_ReadBufferBytes(&dataRegisterBuffer, data, size) != 0) {
        return -1;
    }
	return 0;
}