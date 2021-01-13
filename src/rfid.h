#ifndef RFID_H
#define RFID_H

#define F_CPU 16000000UL

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//-----------------------------------------Status----------------------------------------------------
#define STATUS_OK             1	// Success
#define STATUS_ERROR          2	// Error in communication
#define STATUS_COLLISION      3	// Collission detected
#define STATUS_TIMEOUT        4	// Timeout in communication.
#define STATUS_NO_ROOM        5	// A buffer is not big enough.
#define STATUS_INTERNAL_ERROR 6 // Internal error in the code. Should not happen ;-)
#define STATUS_INVALID        7	// Invalid argument.
#define STATUS_CRC_WRONG      8	// The CRC_A does not match
#define STATUS_MIFARE_NACK    9	// A MIFARE PICC responded with NAK.

//----------------------------------------Register---------------------------------------------------
#define CommandReg    0x01 << 1	// starts and stops command execution
#define ComIrqReg     0x04 << 1	// interrupt request bits
#define DivIrqReg     0x05 << 1	// interrupt request bits
#define ErrorReg      0x06 << 1	// error bits showing the error status of the last command executed
#define Status2Reg    0x08 << 1	// receiver and transmitter status bits
#define FIFODataReg   0x09 << 1	// input and output of 64 byte FIFO buffer
#define FIFOLevelReg  0x0A << 1	// number of bytes stored in the FIFO buffer
#define ControlReg    0x0C << 1	// miscellaneous control registers
#define BitFramingReg 0x0D << 1 // adjustments for bit-oriented frames
#define CollReg       0x0E << 1	// bit position of the first bit-collision detected on the RF interface
#define ModeReg       0x11 << 1	// defines general modes for transmitting and receiving
#define TxModeReg     0x12 << 1	// defines transmission data rate and framing
#define RxModeReg     0x13 << 1	// defines reception data rate and framing
#define TxControlReg  0x14 << 1	// controls the logical behavior of the antenna driver pins TX1 and TX2
#define TxASKReg      0x15 << 1	// controls the setting of the transmission modulation
#define CRCResultRegH 0x21 << 1 // shows the MSB and LSB values of the CRC calculation
#define CRCResultRegL 0x22 << 1
#define ModWidthReg   0x24 << 1	// controls the ModWidth setting?
#define TModeReg      0x2A << 1	// defines settings for the internal timer
#define TPrescalerReg 0x2B << 1 // defines settings for the internal timer
#define TReloadRegH   0x2C << 1	// defines the 16-bit timer reload value
#define TReloadRegL   0x2D << 1	// defines the 16-bit timer reload value

//------------------------------Commands sent to the PICC.------------------------------------------
#define PICC_CMD_REQA    0x26 // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_WUPA    0x52 // Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define PICC_CMD_CT      0x88 // Cascade Tag. Not really a command, but used during anti collision.
#define PICC_CMD_SEL_CL1 0x93 // Anti collision/Select, Cascade Level 1
#define PICC_CMD_SEL_CL2 0x95 // Anti collision/Select, Cascade Level 1
#define PICC_CMD_SEL_CL3 0x97 // Anti collision/Select, Cascade Level 1
#define PICC_CMD_HLTA    0x50 // HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
// The commands used for MIFARE Classic (from http://www.nxp.com/documents/data_sheet/MF1S503x.pdf, Section 9)
// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
// The read/write commands can also be used for MIFARE Ultralight.
#define PICC_CMD_MF_AUTH_KEY_A 0x60 // Perform authentication with Key A
#define PICC_CMD_MF_AUTH_KEY_B 0x61 // Perform authentication with Key B
#define PICC_CMD_MF_READ       0x30	// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define PICC_CMD_MF_WRITE      0xA0	// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define PICC_CMD_MF_DECREMENT  0xC0	// Decrements the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_INCREMENT  0xC1	// Increments the contents of a block and stores the result in the internal data register.
#define PICC_CMD_MF_RESTORE    0xC2	// Reads the contents of a block into the internal data register.
#define PICC_CMD_MF_TRANSFER   0xB0	// Writes the contents of the internal data register to a block.
// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define PICC_CMD_UL_WRITE 0xA2 // Writes one 4 byte page to the PICC.

//------------------MFRC522 comands. Described in chapter 10 of the datasheet.-------------------------
#define PCD_Idle             0x00 // no action, cancels current command execution
#define PCD_Mem              0x01 // stores 25 bytes into the internal buffer
#define PCD_GenerateRandomID 0x02 // generates a 10-byte random ID number
#define PCD_CalcCRC          0x03 // activates the CRC coprocessor or performs a self test
#define PCD_Transmit         0x04 // transmits data from the FIFO buffer
#define PCD_NoCmdChange      0x07 // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_Receive          0x08 // activates the receiver circuits
#define PCD_Transceive       0x0C // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define PCD_MFAuthent        0x0E // performs the MIFARE standard authentication as a reader
#define PCD_SoftReset        0x0F // resets the MFRC522

typedef struct
{
	uint8_t size; // Number of bytes in the UID. 4, 7 or 10.
	uint8_t uidByte[10];
	uint8_t sak; // The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

Uid data;

void RFID_Set(void);
void SPI_Set(void);
void SPI_SendByte(uint8_t Reg, uint8_t data);
void SPI_SendByte2(uint8_t Reg, uint8_t byte, uint8_t *data);
uint8_t SPI_ReceiveByte(uint8_t Reg);
void SPI_ReceiveByte2(uint8_t reg,	   // The register to read from. One of the PCD_Register enums.
					  uint8_t count,   // The number of bytes to read
					  uint8_t *values, // Byte array to store the values in.
					  uint8_t rxAlign  // Only bit positions rxAlign..7 in values[0] are updated.
);

uint8_t PICC_Select(Uid *uid,		  // Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
					uint8_t validBits // The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
);

uint8_t PICC_RequestA(uint8_t *bufferATQA, // The buffer to store the ATQA (Answer to request) in
					  uint8_t *bufferSize  // Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
);

uint8_t PICC_REQA_or_WUPA(uint8_t command,	   // The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
						  uint8_t *bufferATQA, // The buffer to store the ATQA (Answer to request) in
						  uint8_t *bufferSize  // Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
);

uint8_t PCD_TransceiveData(uint8_t *sendData,  // Pointer to the data to transfer to the FIFO.
						   uint8_t sendLen,	   // Number of bytes to transfer to the FIFO.
						   uint8_t *backData,  // NULL or pointer to buffer if data should be read back after executing the command.
						   uint8_t *backLen,   // In: Max number of bytes to write to *backData. Out: The number of bytes returned.
						   uint8_t *validBits, // In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
						   uint8_t rxAlign,	   // In: Defines the bit position in backData[0] for the first bit received. Default 0.
						   bool checkCRC	   // In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

uint8_t PCD_CommunicateWithPICC(uint8_t command,	// The command to execute. One of the PCD_Command enums.
								uint8_t waitIRq,	// The bits in the ComIrqReg register that signals successful completion of the command.
								uint8_t *sendData,	// Pointer to the data to transfer to the FIFO.
								uint8_t sendLen,	// Number of bytes to transfer to the FIFO.
								uint8_t *backData,	// NULL or pointer to buffer if data should be read back after executing the command.
								uint8_t *backLen,	// In: Max number of bytes to write to *backData. Out: The number of bytes returned.
								uint8_t *validBits, // In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
								uint8_t rxAlign,	// In: Defines the bit position in backData[0] for the first bit received. Default 0.
								bool checkCRC		// In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

uint8_t PCD_CalculateCRC(uint8_t *data,	 // In: Pointer to the data to transfer to the FIFO for CRC calculation.
						 uint8_t length, // In: The number of bytes to transfer.
						 uint8_t *result // Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
);

bool ReadCardSerial(void);
bool IsNewCardPresent(void);
bool PICC_HaltA(void);
void PCD_StopCrypto1(void);

#endif