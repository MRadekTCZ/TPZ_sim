/*
 * Modbus.c
 *
 *  Created on: 2 mar 2024
 *      Author: Maciek
 */

#include "Modbus.h"

usint CRCTable[256] = {
0x0000,0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
0xc601,0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
0xcc01,0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
0x0a00,0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
0xd801,0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
0x1e00,0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
0x1400,0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
0xd201,0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
0xf001,0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
0x3600,0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
0x3c00,0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
0xfa01,0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
0x2800,0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
0xee01,0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
0xe401,0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
0x2200,0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
0xa001,0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
0x6600,0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
0x6c00,0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
0xaa01,0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
0x7800,0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
0xbe01,0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
0xb401,0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
0x7200,0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
0x5000,0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
0x9601,0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
0x9c01,0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
0x5a00,0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
0x8801,0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
0x4e00,0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
0x4400,0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
0x8201,0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};


usint CRC_vector(const char* data, int length, unsigned int crcTable[256])
{
    unsigned int crc = 0xFFFF;
    for (int i = 0; i < length; i++) {
        crc = (crc << 8) ^ crcTable[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

Modbus_handler Receive_request(unsigned char mdb_data_buffer[10], unsigned int slave_address, usint *MDB_Data_registers)
{
    Modbus_handler MDB_Received_request;
    Modbus_handler MDB_Made_response;
    unsigned short int CRC_computed;
    #ifndef ASCII
    MDB_Received_request.address.data_t[1] = mdb_data_buffer[0];
    MDB_Received_request.address.data_t[0] = mdb_data_buffer[1];
    MDB_Received_request.function.data_t[1] = mdb_data_buffer[2];
    MDB_Received_request.function.data_t[0] = mdb_data_buffer[3];
    MDB_Received_request.offset_regCount.data_t[1] = mdb_data_buffer[4];
    MDB_Received_request.offset_regCount.data_t[0] = mdb_data_buffer[5];
    MDB_Received_request.regCount_data[0].data_t[1] = mdb_data_buffer[6];
    MDB_Received_request.regCount_data[0].data_t[0] = mdb_data_buffer[7];
    CRC_computed = CRC_vector(mdb_data_buffer, 8, CRCTable);
    MDB_Received_request.crc.data_t[1] = mdb_data_buffer[8];
    MDB_Received_request.crc.data_t[0] = mdb_data_buffer[9];
    #endif
    #ifdef ASCII
    //For ASCII request there is conversion to HEX. Even after ASCII request, response is in hex (has to be, data would not fit into ASCII).
    MDB_Received_request.address.data_t[1] = 0;
    MDB_Received_request.address.data_t[0] = (mdb_data_buffer[1] - ASCII_offset) + (mdb_data_buffer[0]- ASCII_offset)*10;
    MDB_Received_request.function.data_t[1] = 0;
    MDB_Received_request.function.data_t[0] = (mdb_data_buffer[3] - ASCII_offset) + (mdb_data_buffer[2] - ASCII_offset) * 10;
    MDB_Received_request.offset_regCount.data_t[1] = 0;
    MDB_Received_request.offset_regCount.data_t[0] = (mdb_data_buffer[5] - ASCII_offset) + (mdb_data_buffer[4] - ASCII_offset) * 10;
    MDB_Received_request.regCount_data[0].data_t[1] = 0;
    MDB_Received_request.regCount_data[0].data_t[0] = (mdb_data_buffer[7] - ASCII_offset) + (mdb_data_buffer[6] - ASCII_offset) * 10;
    CRC_computed = CRC_vector(mdb_data_buffer, 8, CRCTable);
    MDB_Received_request.crc.data_t[1] = mdb_data_buffer[8];
    MDB_Received_request.crc.data_t[0] = mdb_data_buffer[9];
    #endif
    //if (CRC_computed == MDB_Received_request.crc.data_u)
    if(1)
    {
        MDB_Made_response.address.data_u = slave_address;
        if (slave_address == MDB_Received_request.address.data_t[0])
        {

            switch (MDB_Received_request.function.data_u)
            {
            case READ_INPUTS:
                MDB_Made_response.function.data_u = READ_INPUTS;
                    for (usint i = 0; i < MDB_Received_request.regCount_data[0].data_u; i++)
                    {
                        if ((MDB_Received_request.offset_regCount.data_u >= (REG_OFFSET-i)) && MDB_Received_request.regCount_data > 0)
                        {
                            MDB_Made_response.regCount_data[i].data_u = MDB_Data_registers[i + (MDB_Received_request.offset_regCount.data_u - REG_OFFSET)];
                        }
                        else MDB_Made_response.regCount_data[i].data_u = ILLEGAL_DATA_ADDRESS;
                    }
                MDB_Made_response.offset_regCount.data_u = MDB_Received_request.regCount_data[0].data_u;
                break;
            case READ_REGISTERS:
                MDB_Made_response.function.data_u = READ_INPUTS;
                for (usint i = 0; i < MDB_Received_request.regCount_data[0].data_u; i++)
                {
                    if ((MDB_Received_request.offset_regCount.data_u >= (REG_OFFSET-i)) && MDB_Received_request.regCount_data > 0)
                    {
                        MDB_Made_response.regCount_data[i].data_u = MDB_Data_registers[i + (MDB_Received_request.offset_regCount.data_u - REG_OFFSET)];
                    }
                    else MDB_Made_response.regCount_data[i].data_u = ILLEGAL_DATA_ADDRESS;
                }
                MDB_Made_response.offset_regCount.data_u = MDB_Received_request.regCount_data[0].data_u;
                break;
           // #ifndef ASCII
            case WRITE_INPUTS:
                    MDB_Made_response = MDB_Received_request;
                    MDB_Made_response.crc.data_u = 0x00;
                    if (MDB_Received_request.offset_regCount.data_u >= (REG_OFFSET + WRITE_OFFSET))
                    {
                        MDB_Data_registers[MDB_Received_request.offset_regCount.data_u - REG_OFFSET] = MDB_Received_request.regCount_data[0].data_u;
                    }
                break;

            case WRITE_REGISTERS:
                    MDB_Made_response = MDB_Received_request;
                    MDB_Made_response.crc.data_u = 0x00;
                    if (MDB_Received_request.offset_regCount.data_u >= (REG_OFFSET + WRITE_OFFSET))
                    {
                        MDB_Data_registers[MDB_Received_request.offset_regCount.data_u - REG_OFFSET] = MDB_Received_request.regCount_data[0].data_u;
                    }
                break;
            //#endif
            case READ_DEVICE_INFO_1:
                MDB_Made_response.address.data_u = 0x5450; //TP
                MDB_Made_response.function.data_u = 0x5A2D; //Z-
                MDB_Made_response.offset_regCount.data_u = 0x204D; // M
                MDB_Made_response.regCount_data[0].data_u = 0x5242;//RB
                break;
            case READ_DEVICE_INFO_2:
                MDB_Made_response.address.data_u = 0x5450;
                MDB_Made_response.function.data_u = 0x5A2D;
                MDB_Made_response.offset_regCount.data_u = 0x204D;
                MDB_Made_response.regCount_data[0].data_u = 0x5242;
                break;
            default:
                MDB_Made_response = MDB_Received_request;
                MDB_Made_response.regCount_data[0].data_u = ILLEGAL_FUNCTION;
                break;

            }


        }
    }
    else
    {
        MDB_Made_response = MDB_Received_request;
        MDB_Made_response.regCount_data[0].data_u = CRC_ERROR;
    }


    return MDB_Made_response;
}
void MakeRequest(Modbus_handler Request, char* mdb_buffer)
{
    mdb_buffer[0] = Request.address.data_t[1];
    mdb_buffer[1] = Request.address.data_t[0];
    mdb_buffer[2] = Request.function.data_t[1];
    mdb_buffer[3] = Request.function.data_t[0];
    mdb_buffer[4] = Request.offset_regCount.data_t[1];
    mdb_buffer[5] = Request.offset_regCount.data_t[0];
    mdb_buffer[6] = Request.regCount_data[0].data_t[1];
    mdb_buffer[7] = Request.regCount_data[0].data_t[0];
    Request.crc.data_u = CRC_vector(mdb_buffer, 8, CRCTable);
    mdb_buffer[8] = Request.crc.data_t[1];
    mdb_buffer[9] = Request.crc.data_t[0];

}

void MakeResponse(Modbus_handler Response, char* mdb_buffer)
{
    //For multiple register read buffer is/ may be longer
    if (Response.function.data_u == 3 || Response.function.data_u == 4)
    {
        mdb_buffer[0] = Response.address.data_t[1];
        mdb_buffer[1] = Response.address.data_t[0];
        mdb_buffer[2] = Response.function.data_t[1];
        mdb_buffer[3] = Response.function.data_t[0];
        mdb_buffer[4] = Response.offset_regCount.data_t[1];
        mdb_buffer[5] = Response.offset_regCount.data_t[0];
        for (usint i = 0; i < Response.offset_regCount.data_u; i++)
        {
            mdb_buffer[6 + 2 * i] = Response.regCount_data[i].data_t[1];
            mdb_buffer[7 + 2 * i] = Response.regCount_data[i].data_t[0];
        }
        Response.crc.data_u = CRC_vector(mdb_buffer, (6 + Response.offset_regCount.data_u * 2), CRCTable);
        mdb_buffer[6 + Response.offset_regCount.data_u * 2] = Response.crc.data_t[1];
        mdb_buffer[7 + Response.offset_regCount.data_u * 2] = Response.crc.data_t[0];
    }
    else
    {
        mdb_buffer[0] = Response.address.data_t[1];
        mdb_buffer[1] = Response.address.data_t[0];
        mdb_buffer[2] = Response.function.data_t[1];
        mdb_buffer[3] = Response.function.data_t[0];
        mdb_buffer[4] = Response.offset_regCount.data_t[1];
        mdb_buffer[5] = Response.offset_regCount.data_t[0];
        mdb_buffer[6] = Response.regCount_data[0].data_t[1];
        mdb_buffer[7] = Response.regCount_data[0].data_t[0];
        Response.crc.data_u = CRC_vector(mdb_buffer, 8, CRCTable);
        mdb_buffer[8] = Response.crc.data_t[1];
        mdb_buffer[9] = Response.crc.data_t[0];
    }

}


void ModbusInit(unsigned int Table[256]) {
    const unsigned int polynomial = 0x1001; // CRC-16 polinom A001

    for (int i = 0; i < 256; i++) {
        unsigned int crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            }
            else {
                crc >>= 1;
            }
        }
        Table[i] = crc;
    }
}
