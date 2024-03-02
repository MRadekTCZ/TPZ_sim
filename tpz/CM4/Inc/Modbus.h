/*
 * Modbus.h
 *
 *  Created on: 2 mar 2024
 *      Author: Maciek
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#define ASCII
#ifndef ASCII

#define ASCII_offset 0x0
#define REG_OFFSET 1000
#define WRITE_OFFSET 40
#endif
#ifdef ASCII
#define ASCII_offset 0x30
#define REG_OFFSET 10
#endif
#define READ_INPUTS 3
#define READ_REGISTERS 4
#define WRITE_INPUTS 5
#define WRITE_REGISTERS 6
#define READ_DEVICE_INFO_1 14
#define READ_DEVICE_INFO_2 44
#define WRITE_OFFSET 50

//ERRORS
#define ILLEGAL_FUNCTION 0xFFF1
#define ILLEGAL_DATA_ADDRESS 0xFFF2
#define ILLEGAL_DATA_VALUE 0xFFF3
#define CRC_ERROR 0xFFF8
typedef unsigned short int usint;
union Data
{
	unsigned char data_t[2];
	usint data_u;
};

typedef struct Modbus
{
	union Data address;
	union Data function;
	union Data offset_regCount;
	union Data regCount_data[100];
	union Data crc;
}Modbus_handler;

typedef struct BYTE_8
{
	unsigned char b0 : 1;
	unsigned char b1 : 1;
	unsigned char b2 : 1;
	unsigned char b3 : 1;
	unsigned char b4 : 1;
	unsigned char b5 : 1;
	unsigned char b6 : 1;
	unsigned char b7 : 1;
}Byte_8;

typedef union BYTE_u
{
	Byte_8 byte_8;
	unsigned char byte;
}byte_u;


void ModbusInit(unsigned int crcTable[256]);
usint CRC_vector(const char* data, int length, unsigned int crcTable[256]);

Modbus_handler Receive_request(unsigned char mdb_data_buffer[10], unsigned int slave_address, usint* MDB_Data_registers);
void MakeRequest(Modbus_handler Request, char* mdb_buffer);
void MakeResponse(Modbus_handler Response, char* mdb_buffer);




#endif /* INC_MODBUS_H_ */
