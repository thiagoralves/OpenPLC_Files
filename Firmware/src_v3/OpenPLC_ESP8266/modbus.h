//-----------------------------------------------------------------------------
// Copyright 2018 Thiago Alves
// This file is part of the OpenPLC Software Stack.
//
// OpenPLC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenPLC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with OpenPLC.  If not, see <http://www.gnu.org/licenses/>.
//------
//
// This file has all the MODBUS/TCP functions supported by the ESP.
//
// Thiago Alves, Aug 2018
//-----------------------------------------------------------------------------

#define MAX_DISCRETE_INPUT 		8
#define MAX_COILS 				8
#define MAX_HOLD_REGS 			1
#define MAX_INP_REGS			1

#define MB_FC_NONE							0
#define MB_FC_READ_COILS					1
#define MB_FC_READ_INPUTS					2
#define MB_FC_READ_HOLDING_REGISTERS		3
#define MB_FC_READ_INPUT_REGISTERS			4
#define MB_FC_WRITE_COIL					5
#define MB_FC_WRITE_REGISTER				6
#define MB_FC_WRITE_MULTIPLE_COILS			15
#define MB_FC_WRITE_MULTIPLE_REGISTERS		16
#define MB_FC_ERROR							255

#define ERR_NONE							0
#define ERR_ILLEGAL_FUNCTION				1
#define ERR_ILLEGAL_DATA_ADDRESS			2
#define ERR_ILLEGAL_DATA_VALUE				3
#define ERR_SLAVE_DEVICE_FAILURE			4
#define ERR_SLAVE_DEVICE_BUSY				6


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((unsigned char) ((w) & 0xff))
#define highByte(w) ((unsigned char) ((w) >> 8))

bool mb_discrete_input[MAX_DISCRETE_INPUT];
bool mb_coils[MAX_COILS];
uint16_t mb_input_regs[MAX_INP_REGS];
uint16_t mb_holding_regs[MAX_HOLD_REGS];

int MessageLength;

//-----------------------------------------------------------------------------
// Concatenate two bytes into an int
//-----------------------------------------------------------------------------
int create_word(unsigned char byte1, unsigned char byte2)
{
	int returnValue;
	returnValue = (int)(byte1 << 8) | (int)byte2;

	return returnValue;
}

//-----------------------------------------------------------------------------
// Response to a Modbus Error
//-----------------------------------------------------------------------------
void ModbusError(unsigned char *buffer, int mb_error)
{
	buffer[4] = 0;
	buffer[5] = 3;
	buffer[7] = buffer[7] | 0x80; //set the highest bit
	buffer[8] = mb_error;
	MessageLength = 9;
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Read Coils
//-----------------------------------------------------------------------------
void ReadCoils(unsigned char *buffer, int bufferSize)
{
	int Start, ByteDataLength, CoilDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8], buffer[9]);
	CoilDataLength = create_word(buffer[10], buffer[11]);
	ByteDataLength = CoilDataLength / 8; //calculating the size of the message in bytes
	if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;

	//asked for too many coils
	if (ByteDataLength > 255)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_ADDRESS);
		return;
	}

	//preparing response
	buffer[4] = highByte(ByteDataLength + 3);
	buffer[5] = lowByte(ByteDataLength + 3); //Number of bytes after this one
	buffer[8] = ByteDataLength;     //Number of bytes of data

	for(int i = 0; i < ByteDataLength ; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			int position = Start + i * 8 + j;
			if (position < MAX_COILS)
			{
                bitWrite(buffer[9 + i], j, mb_coils[position]);
			}
			else //invalid address
			{
				mb_error = ERR_ILLEGAL_DATA_ADDRESS;
			}
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = ByteDataLength + 9;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Read Discrete Inputs
//-----------------------------------------------------------------------------
void ReadDiscreteInputs(unsigned char *buffer, int bufferSize)
{
	int Start, ByteDataLength, InputDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);
	InputDataLength = create_word(buffer[10],buffer[11]);
	ByteDataLength = InputDataLength / 8;
	if(ByteDataLength * 8 < InputDataLength) ByteDataLength++;

	//asked for too many inputs
	if (ByteDataLength > 255)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_ADDRESS);
		return;
	}

	//Preparing response
	buffer[4] = highByte(ByteDataLength + 3);
	buffer[5] = lowByte(ByteDataLength + 3); //Number of bytes after this one
	buffer[8] = ByteDataLength;     //Number of bytes of data

	for(int i = 0; i < ByteDataLength ; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			int position = Start + i * 8 + j;
			if (position < MAX_DISCRETE_INPUT)
			{
                bitWrite(buffer[9 + i], j, mb_discrete_input[position]);
			}
			else //invalid address
			{
				mb_error = ERR_ILLEGAL_DATA_ADDRESS;
			}
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = ByteDataLength + 9;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Read Holding Registers
//-----------------------------------------------------------------------------
void ReadHoldingRegisters(unsigned char *buffer, int bufferSize)
{
	int Start, WordDataLength, ByteDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);
	WordDataLength = create_word(buffer[10],buffer[11]);
	ByteDataLength = WordDataLength * 2;

	//asked for too many registers
	if (ByteDataLength > 255)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_ADDRESS);
		return;
	}

	//preparing response
	buffer[4] = highByte(ByteDataLength + 3);
	buffer[5] = lowByte(ByteDataLength + 3); //Number of bytes after this one
	buffer[8] = ByteDataLength;     //Number of bytes of data

	for(int i = 0; i < WordDataLength; i++)
	{
		int position = Start + i;
		if (position < MAX_HOLD_REGS)
		{
            buffer[ 9 + i * 2] = highByte(mb_holding_regs[position]);
            buffer[10 + i * 2] = lowByte(mb_holding_regs[position]);
		}
		//invalid address
		else
		{
			mb_error = ERR_ILLEGAL_DATA_ADDRESS;
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = ByteDataLength + 9;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Read Input Registers
//-----------------------------------------------------------------------------
void ReadInputRegisters(unsigned char *buffer, int bufferSize)
{
	int Start, WordDataLength, ByteDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);
	WordDataLength = create_word(buffer[10],buffer[11]);
	ByteDataLength = WordDataLength * 2;

	//asked for too many registers
	if (ByteDataLength > 255)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_ADDRESS);
		return;
	}

	//preparing response
	buffer[4] = highByte(ByteDataLength + 3);
	buffer[5] = lowByte(ByteDataLength + 3); //Number of bytes after this one
	buffer[8] = ByteDataLength;     //Number of bytes of data

	for(int i = 0; i < WordDataLength; i++)
	{
		int position = Start + i;
		if (position < MAX_INP_REGS)
		{
			buffer[ 9 + i * 2] = highByte(mb_input_regs[position]);
			buffer[10 + i * 2] = lowByte(mb_input_regs[position]);
		}
		else //invalid address
		{
			mb_error = ERR_ILLEGAL_DATA_ADDRESS;
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = ByteDataLength + 9;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Write Coil
//-----------------------------------------------------------------------------
void WriteCoil(unsigned char *buffer, int bufferSize)
{
	int Start;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8], buffer[9]);

	if (Start < MAX_COILS)
	{
		unsigned char value;
		if (create_word(buffer[10], buffer[11]) > 0)
		{
			value = 1;
		}
		else
		{
			value = 0;
		}
        
        mb_coils[Start] = value;
	}

	else //invalid address
	{
		mb_error = ERR_ILLEGAL_DATA_ADDRESS;
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		buffer[4] = 0;
		buffer[5] = 6; //Number of bytes after this one.
		MessageLength = 12;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Write Holding Register
//-----------------------------------------------------------------------------
void WriteRegister(unsigned char *buffer, int bufferSize)
{
	int Start;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);

	if (Start < MAX_HOLD_REGS)
	{
        mb_holding_regs[Start] = create_word(buffer[10],buffer[11]);
	}
	else //invalid address
	{
		mb_error = ERR_ILLEGAL_DATA_ADDRESS;
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		buffer[4] = 0;
		buffer[5] = 6; //Number of bytes after this one.
		MessageLength = 12;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Write Multiple Coils
//-----------------------------------------------------------------------------
void WriteMultipleCoils(unsigned char *buffer, int bufferSize)
{
	int Start, ByteDataLength, CoilDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);
	CoilDataLength = create_word(buffer[10],buffer[11]);
	ByteDataLength = CoilDataLength / 8;
	if(ByteDataLength * 8 < CoilDataLength) ByteDataLength++;

	//this request must have all the bytes it wants to write. If it doesn't, it's a corrupted message
	if ( (bufferSize < (13 + ByteDataLength)) || (buffer[12] != ByteDataLength) )
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	//preparing response
	buffer[4] = 0;
	buffer[5] = 6; //Number of bytes after this one.

	for(int i = 0; i < ByteDataLength ; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			int position = Start + i * 8 + j;
			if (position < MAX_COILS)
			{
                mb_coils[position] = bitRead(buffer[13 + i], j);
			}
			else //invalid address
			{
				mb_error = ERR_ILLEGAL_DATA_ADDRESS;
			}
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = 12;
	}
}

//-----------------------------------------------------------------------------
// Implementation of Modbus/TCP Write Multiple Registers
//-----------------------------------------------------------------------------
void WriteMultipleRegisters(unsigned char *buffer, int bufferSize)
{
	int Start, WordDataLength, ByteDataLength;
	int mb_error = ERR_NONE;

	//this request must have at least 12 bytes. If it doesn't, it's a corrupted message
	if (bufferSize < 12)
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	Start = create_word(buffer[8],buffer[9]);
	WordDataLength = create_word(buffer[10],buffer[11]);
	ByteDataLength = WordDataLength * 2;

	//this request must have all the bytes it wants to write. If it doesn't, it's a corrupted message
	if ( (bufferSize < (13 + ByteDataLength)) || (buffer[12] != ByteDataLength) )
	{
		ModbusError(buffer, ERR_ILLEGAL_DATA_VALUE);
		return;
	}

	//preparing response
	buffer[4] = 0;
	buffer[5] = 6; //Number of bytes after this one.

	for(int i = 0; i < WordDataLength; i++)
	{
		int position = Start + i;
		if (position < MAX_HOLD_REGS)
		{
			mb_holding_regs[position] =  create_word(buffer[13 + i * 2], buffer[14 + i * 2]);
		}
		else //invalid address
		{
			mb_error = ERR_ILLEGAL_DATA_ADDRESS;
		}
	}

	if (mb_error != ERR_NONE)
	{
		ModbusError(buffer, mb_error);
	}
	else
	{
		MessageLength = 12;
	}
}

//-----------------------------------------------------------------------------
// This function must parse and process the client request and write back the
// response for it. The return value is the size of the response message in
// bytes.
//-----------------------------------------------------------------------------
int processModbusMessage(unsigned char *buffer, int bufferSize)
{
	MessageLength = 0;

	//check if the message is long enough
	if (bufferSize < 8)
	{
		ModbusError(buffer, ERR_ILLEGAL_FUNCTION);
	}

	//****************** Read Coils **********************
	else if(buffer[7] == MB_FC_READ_COILS)
	{
		ReadCoils(buffer, bufferSize);
	}

	//*************** Read Discrete Inputs ***************
	else if(buffer[7] == MB_FC_READ_INPUTS)
	{
		ReadDiscreteInputs(buffer, bufferSize);
	}

	//****************** Read Holding Registers ******************
	else if(buffer[7] == MB_FC_READ_HOLDING_REGISTERS)
	{
		ReadHoldingRegisters(buffer, bufferSize);
	}

	//****************** Read Input Registers ******************
	else if(buffer[7] == MB_FC_READ_INPUT_REGISTERS)
	{
		ReadInputRegisters(buffer, bufferSize);
	}

	//****************** Write Coil **********************
	else if(buffer[7] == MB_FC_WRITE_COIL)
	{
		WriteCoil(buffer, bufferSize);
	}

	//****************** Write Register ******************
	else if(buffer[7] == MB_FC_WRITE_REGISTER)
	{
		WriteRegister(buffer, bufferSize);
	}

	//****************** Write Multiple Coils **********************
	else if(buffer[7] == MB_FC_WRITE_MULTIPLE_COILS)
	{
		WriteMultipleCoils(buffer, bufferSize);
	}

	//****************** Write Multiple Registers ******************
	else if(buffer[7] == MB_FC_WRITE_MULTIPLE_REGISTERS)
	{
		WriteMultipleRegisters(buffer, bufferSize);
	}

	//****************** Function Code Error ******************
	else
	{
		ModbusError(buffer, ERR_ILLEGAL_FUNCTION);
	}

	return MessageLength;
}