#define sampleFreq	200.0f		// sample frequency in Hz
#define betaDef		12.06f		// 2 * proportional gain -> Derived from equation (50) in Madgwick's Internal Report found:
					            // http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
				            	// omega-dot = 20 degrees/sec


// Call this function at 200Hz or faster (I am calling it a 1kHz)
uint8_t uart2Data[100];

uint8_t bytes[4];
BEPackReal32(bytes, quaterion[0]);
uart2Data[0] = hex2char(bytes[3] >> 4);
uart2Data[1] = hex2char(bytes[3] & 0xF);
uart2Data[2] = hex2char(bytes[2] >> 4);
uart2Data[3] = hex2char(bytes[2] & 0xF);
uart2Data[4] = hex2char(bytes[1] >> 4);
uart2Data[5] = hex2char(bytes[1] & 0xF);
uart2Data[6] = hex2char(bytes[0] >> 4);
uart2Data[7] = hex2char(bytes[0] & 0xF);
uart2Data[8] = ',';
BEPackReal32(bytes, quaterion[1]);
uart2Data[9] = hex2char(bytes[3] >> 4);
uart2Data[10] = hex2char(bytes[3] & 0xF);
uart2Data[11] = hex2char(bytes[2] >> 4);
uart2Data[12] = hex2char(bytes[2] & 0xF);
uart2Data[13] = hex2char(bytes[1] >> 4);
uart2Data[14] = hex2char(bytes[1] & 0xF);
uart2Data[15] = hex2char(bytes[0] >> 4);
uart2Data[16] = hex2char(bytes[0] & 0xF);
uart2Data[17] = ',';
BEPackReal32(bytes, quaterion[2]);
uart2Data[18] = hex2char(bytes[3] >> 4);
uart2Data[19] = hex2char(bytes[3] & 0xF);
uart2Data[20] = hex2char(bytes[2] >> 4);
uart2Data[21] = hex2char(bytes[2] & 0xF);
uart2Data[22] = hex2char(bytes[1] >> 4);
uart2Data[23] = hex2char(bytes[1] & 0xF);
uart2Data[24] = hex2char(bytes[0] >> 4);
uart2Data[25] = hex2char(bytes[0] & 0xF);
uart2Data[26] = ',';
BEPackReal32(bytes, quaterion[3]);
uart2Data[27] = hex2char(bytes[3] >> 4);
uart2Data[28] = hex2char(bytes[3] & 0xF);
uart2Data[29] = hex2char(bytes[2] >> 4);
uart2Data[30] = hex2char(bytes[2] & 0xF);
uart2Data[31] = hex2char(bytes[1] >> 4);
uart2Data[32] = hex2char(bytes[1] & 0xF);
uart2Data[33] = hex2char(bytes[0] >> 4);
uart2Data[34] = hex2char(bytes[0] & 0xF);
uart2Data[35] = ',';
uart2Data[36] = '\n';
