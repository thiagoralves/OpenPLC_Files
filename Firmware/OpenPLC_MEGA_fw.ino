/************************PINOUT CONFIGURATION*************************

Digital In: 22, 24, 26, 28, 30, 32, 34, 36,       (%IX0.0 - %IX0.7)
            38, 40, 42, 44, 46, 48, 50, 52,       (%IX1.0 - %IX1.7)
            14, 15, 16, 17, 18, 19, 20, 21        (%IX2.0 - %IX2.7)
			
Digital Out: 23, 25, 27, 29, 31, 33, 35, 37       (%QX0.0 - %QX0.7)
             39, 41, 43, 45, 47, 49, 51, 53       (%QX1.0 - %QX1.7)

Analog In: A0, A1, A2, A3, A4, A5, A6, A7         (%IW0.0 - %IW0.7)
           A8, A9, A10, A11, A12, A13, A14, A15   (%IW0.8 - %IW0.15)
		   
Analog Out: 2, 3, 4, 5, 6, 7, 8, 9,               (%QW0.0 - %QW0.7)
            10, 11, 12, 13                        (%QW0.8 - %QW0.11)
			
*********************************************************************/


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

struct OPLC_input
{
	uint8_t digital[4];
	uint16_t analog[16];
};

struct OPLC_output
{
	uint8_t digital[2];
	uint16_t analog[12];
};

uint8_t pinMask_DIN[] = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 14, 15, 16, 17, 18, 19, 20, 21};
uint8_t pinMask_DOUT[] = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49, 51, 53};
uint8_t pinMask_AIN[] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};
uint8_t pinMask_AOUT[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

struct OPLC_input input_data;
struct OPLC_output output_data;

uint8_t incommingBuffer[100];
uint8_t outgoingBuffer[100];
int bufferIndex = 0;
bool beginReceiving = false;
bool escapeReceived = false;
bool packetReceived = false;

void parseMessage(uint8_t byteReceived)
{
	if (!beginReceiving && byteReceived == 'S')
	{
		beginReceiving = true;
	}
	
	else if (beginReceiving)
	{
		if (!escapeReceived)
		{
			if (byteReceived == '\\')
			{
				escapeReceived = true;
			}
			else if (byteReceived == 'E')
			{
				//End packet
				escapeReceived = false;
				beginReceiving = false;
				packetReceived = true;
				bufferIndex = 0;
				
			}
			else if (byteReceived == 'S')
			{
				//Missed end of last packet. Drop packet and start a new one
				escapeReceived = false;
				beginReceiving = true;
				packetReceived = false;
				bufferIndex = 0;
			}
			else
			{
				incommingBuffer[bufferIndex] = byteReceived;
				bufferIndex++;
			}
		}
		
		else if (escapeReceived)
		{
			if (byteReceived == '\\' || byteReceived == 'E' || byteReceived == 'S')
			{
				incommingBuffer[bufferIndex] = byteReceived;
				bufferIndex++;
				escapeReceived = false;
			}
			else
			{
				//Invalid sequence! Drop packet
				escapeReceived = false;
				beginReceiving = false;
				packetReceived = false;
				bufferIndex = 0;
			}
		}
	}
}

void updateOutputs()
{
	for (int i = 0; i < sizeof(pinMask_DOUT); i++)
	{
		digitalWrite(pinMask_DOUT[i], bitRead(output_data.digital[i/8], i%8));
	}
	
	for (int i = 0; i < sizeof(pinMask_AOUT); i++)
	{
		analogWrite(pinMask_AOUT[i], output_data.analog[i] / 256);
	}
}

void updateInputs()
{
	for (int i = 0; i < sizeof(pinMask_DIN); i++)
	{
		bitWrite(input_data.digital[i/8], i%8, digitalRead(pinMask_DIN[i]));
	}
	
	for (int i = 0; i < sizeof(pinMask_AIN); i++)
	{
		input_data.analog[i] = analogRead(pinMask_AIN[i]) * 64;
	}
}

void sendPacket()
{
	uint8_t temp[100];
	struct OPLC_input *dataPointer;
	dataPointer = &input_data;
	memcpy(temp, dataPointer, sizeof(struct OPLC_input));
	
	outgoingBuffer[0] = 'S';
	int j = 1;
	
	for (int i = 0; i < sizeof(struct OPLC_input); i++)
	{
		if (temp[i] != 'S' && temp[i] != 'E' && temp[i] != '\\')
		{
			outgoingBuffer[j] = temp[i];
			j++;
		}
		else
		{
			outgoingBuffer[j] = '\\';
			j++;
			outgoingBuffer[j] = temp[i];
			j++;
		}
	}
	outgoingBuffer[j] = 'E';
	j++;
	
	for (int i = 0; i < j; i++)
	{
		Serial.write(outgoingBuffer[i]);
	}
}

void setup() 
{
	Serial.begin(115200);
	for (int i = 0; i < sizeof(pinMask_DIN); i++)
	{
		pinMode(pinMask_DIN[i], INPUT);
	}
	for (int i = 0; i < sizeof(pinMask_DOUT); i++)
	{
		pinMode(pinMask_DOUT[i], OUTPUT);
	}
	for (int i = 0; i < sizeof(pinMask_AIN); i++)
	{
		pinMode(pinMask_AIN[i], INPUT);
	}
	for (int i = 0; i < sizeof(pinMask_AOUT); i++)
	{
		pinMode(pinMask_AOUT[i], OUTPUT);
	}
	
	memset(&input_data,0,sizeof(input_data));
	memset(&output_data,0,sizeof(output_data));
}

void loop() 
{
	updateInputs();
	
	while (Serial.available() > 0)
	{
		parseMessage(Serial.read());
	}
	
	if (packetReceived)
	{
		packetReceived = false;
		
		struct OPLC_output *dataPointer;
		dataPointer = &output_data;
		memcpy(dataPointer, incommingBuffer, sizeof(struct OPLC_output));
		
		updateOutputs();
		sendPacket();
	}
}
