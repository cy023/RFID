#include "rfid.h"

void RFID_Set(void)
{
    if (!(PORTB & (1 << PORTB1)))
    {                         // The MFRC522 chip is in power down mode.
        PORTB |= 1 << PORTB1; // Exit power down mode. This triggers a hard reset.
        _delay_ms(50);        // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74µs. Let us be generous: 50ms.
    }
    SPI_SendByte(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    SPI_SendByte(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.
    SPI_SendByte(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    SPI_SendByte(TReloadRegL, 0xE8);
    SPI_SendByte(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    SPI_SendByte(ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

    uint8_t value = SPI_ReceiveByte(TxControlReg);
    if ((value & 0x03) != 0x03)
    {
        SPI_SendByte(TxControlReg, value | 0x03);
    }
}

void SPI_Set(void)
{
    DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3) | (0 << DDB4) | (1 << DDB5); //Set RST & SS & MOSI & SCK output, MISO input
    PORTB |= (1 << PORTB2);
    SPCR |= (1 << SPE) | (0 << DORD) | (1 << MSTR); //Setting enable SPI && MSB transmission && master mode
    SPCR |= (0 << CPOL) | (0 << CPHA);              //Setting Sample (rising) & Setup (falling)
    SPCR |= (1 << SPR0) | (1 << SPR1);              //clk/128
}

void SPI_SendByte(uint8_t Reg, uint8_t data)
{
    PORTB &= ~(1 << PORTB2); //CS pin low

    SPDR = (Reg & 0x7E);
    while (!(SPSR & (1 << SPIF)))
        ; //wait data transmission
    SPDR = data;
    while (!(SPSR & (1 << SPIF)))
        ; //wait data transmission

    PORTB |= 1 << PORTB2; //CS pin High
    _delay_ms(1);
}

void SPI_SendByte2(uint8_t Reg, uint8_t byte, uint8_t *data)
{
    PORTB &= ~(1 << PORTB2); //CS pin low

    SPDR = (Reg & 0x7E);
    while (!(SPSR & (1 << SPIF)))
        ; //wait data transmission

    for (uint8_t i = 0; i < byte; i++)
    {
        SPDR = data[i];
        while (!(SPSR & (1 << SPIF)))
            ; //wait data transmission
    }

    PORTB |= 1 << PORTB2; //CS pin High
}

uint8_t SPI_ReceiveByte(uint8_t Reg)
{
    uint8_t data = 0;
    PORTB &= ~(1 << PORTB2); //CS pin low

    SPDR = (0x80 | (Reg & 0x7E));
    while (!(SPSR & (1 << SPIF)))
        ; //wait data transmission
    SPDR = 0;
    while (!(SPSR & (1 << SPIF)))
        ;
    data = SPDR;

    PORTB |= 1 << PORTB2; //CS pin High
    return data;
}

void SPI_ReceiveByte2(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign)
{
    if (count == 0)
    {
        return;
    }
    uint8_t address = 0x80 | reg; // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;            // Index in values array.
    PORTB &= ~(1 << PORTB2);      //CS pin low
    count--;                      // One read is performed outside of the loop
    SPDR = address;               // Tell MFRC522 which address we want to read
    while (!(SPSR & (1 << SPIF)))
        ; //wait data transmission

    if (rxAlign)
    { // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        SPDR = 0;
        while (!(SPSR & (1 << SPIF)))
            ;
        uint8_t value = SPDR;

        SPDR = address;
        while (!(SPSR & (1 << SPIF)))
            ;

        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }
    while (index < count)
    {
        // Read value and tell that we want to read the same address again.
        SPDR = 0;
        while (!(SPSR & (1 << SPIF)))
            ;
        values[index] = SPDR;
        SPDR = address;
        while (!(SPSR & (1 << SPIF)))
            ;
        index++;
    }
    // Read the final byte. Send 0 to stop reading.
    SPDR = 0;
    while (!(SPSR & (1 << SPIF)))
        ;
    values[index] = SPDR;
    SPDR = 0;
    while (!(SPSR & (1 << SPIF)))
        ;
    PORTB |= (1 << PORTB2); //CS pin high
}

uint8_t PICC_Select(Uid *uid, uint8_t validBits)
{
    bool uidComplete;
    bool selectDone;
    bool useCascadeTag;
    uint8_t cascadeLevel = 1;
    uint8_t result;
    uint8_t count;
    uint8_t index;
    uint8_t uidIndex;              // The first index in uid->uidByte[] that is used in the current Cascade Level.
    uint8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];             // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
    uint8_t bufferUsed;            // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
    uint8_t rxAlign;               // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;            // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    // 		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    // 		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
    // 		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
    // 		Byte 3: UID-data
    // 		Byte 4: UID-data
    // 		Byte 5: UID-data
    // 		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
    //		Byte 7: CRC_A
    //		Byte 8: CRC_A
    // The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
    //		========	=============	=====	=====	=====	=====
    //		 4 bytes		1			uid0	uid1	uid2	uid3
    //		 7 bytes		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 bytes		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Sanity checks
    if (validBits > 80)
    {
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    uint8_t value = SPI_ReceiveByte(CollReg); // ValuesAfterColl=1 => Bits received after collision are cleared.
    SPI_SendByte(CollReg, (value & (~0x80)));

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete)
    {
        // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
        switch (cascadeLevel)
        {
        case 1:
            buffer[0] = PICC_CMD_SEL_CL1;
            uidIndex = 0;
            useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 bytes
            break;

        case 2:
            buffer[0] = PICC_CMD_SEL_CL2;
            uidIndex = 3;
            useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 bytes
            break;

        case 3:
            buffer[0] = PICC_CMD_SEL_CL3;
            uidIndex = 6;
            useCascadeTag = false; // Never used in CL3.
            break;

        default:
            return STATUS_INTERNAL_ERROR;
            break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0)
        {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uidByte[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag)
        {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
        if (bytesToCopy)
        {
            uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (bytesToCopy > maxBytes)
            {
                bytesToCopy = maxBytes;
            }
            for (count = 0; count < bytesToCopy; count++)
            {
                buffer[index++] = uid->uidByte[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag)
        {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone)
        {
            // Find out how many bits and bytes to send and receive.
            if (currentLevelKnownBits >= 32)
            { // All UID bits in this Cascade Level are known. This is a SELECT.
                //Serial.print("SELECT: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
                // Calulate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            }
            else
            { // This is an ANTICOLLISION.
                //Serial.print("ANTICOLLISION: currentLevelKnownBits="); Serial.println(currentLevelKnownBits, DEC);
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8;     // Number of whole bytes in the UID part.
                index = 2 + count;                     // Number of whole bytes: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                     // Having a seperate variable is overkill. But it makes the next line easier to read.
            SPI_SendByte(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
            if (result == STATUS_COLLISION)
            {                                      // More than one PICC in the field => collision.
                result = SPI_ReceiveByte(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (result & 0x20)
                {                            // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0)
                {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits)
                { // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = (currentLevelKnownBits - 1) % 8;                   // The bit to modify
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
                buffer[index] |= (1 << count);
            }
            else if (result != STATUS_OK)
            {
                return result;
            }
            else
            { // STATUS_OK
                if (currentLevelKnownBits >= 32)
                {                   // This was a SELECT.
                    selectDone = 1; // No more anticollision
                                    // We continue below outside the while.
                }
                else
                { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while ( ! selectDone)
        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID bytes from buffer[] to uid->uidByte[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < bytesToCopy; count++)
        {
            uid->uidByte[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0)
        { // SAK must be exactly 24 bits (1 byte + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK)
        {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
        {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04)
        { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else
        {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while ( ! uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;
    return STATUS_OK;
}

uint8_t PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

uint8_t PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
    uint8_t validBits;
    uint8_t status;

    if (bufferATQA == NULL || *bufferSize < 2)
    { // The ATQA response is 2 bytes long.
        return STATUS_NO_ROOM;
    }
    uint8_t value = SPI_ReceiveByte(CollReg); // ValuesAfterColl=1 => Bits received after collision are cleared.
    SPI_SendByte(CollReg, value & (~0x80));

    validBits = 7; // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
    if (status != STATUS_OK)
    {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0)
    { // ATQA must be exactly 16 bits.
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

uint8_t PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC)
{
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

uint8_t PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC)
{
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    SPI_SendByte(CommandReg, PCD_Idle); // Stop any active command.
    SPI_SendByte(ComIrqReg, 0x7F);      // Clear all seven interrupt request bits

    //uint8_t value = SPI_ReceiveByte(FIFOLevelReg);		// FlushBuffer = 1, FIFO initialization
    SPI_SendByte(FIFOLevelReg, 0x80);

    SPI_SendByte2(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
    SPI_SendByte(BitFramingReg, bitFraming);       // Bit adjustments
    SPI_SendByte(CommandReg, command);             // Execute the command
    if (command == PCD_Transceive)
    {
        uint8_t value = SPI_ReceiveByte(BitFramingReg);
        SPI_SendByte(BitFramingReg, (value | 0x80)); // StartSend=1, transmission of data starts
    }

    // Wait for the command to complete.
    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
    // Each iteration of the do-while-loop takes 17.86µs.
    uint16_t i;
    for (i = 2000; i > 0; i--)
    {
        uint8_t n = SPI_ReceiveByte(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq   HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq)
        { // One of the interrupts that signal success has been set.
            break;
        }
        if (n & 0x01)
        { // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
    }
    if (i == 0)
    { // The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = SPI_ReceiveByte(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl   CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13)
    { // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }
    uint8_t _validBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen)
    {
        uint8_t n = SPI_ReceiveByte(FIFOLevelReg); // Number of bytes in the FIFO
        if (n > *backLen)
        {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                        // Number of bytes returned
        SPI_ReceiveByte2(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
        _validBits = SPI_ReceiveByte(ControlReg) & 0x07;     // RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
        if (validBits)
        {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08)
    { // CollErr
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC)
    {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4)
        {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last byte must be received.
        if (*backLen < 2 || _validBits != 0)
        {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        uint8_t n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (n != STATUS_OK)
        {
            return n;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
        {
            return STATUS_CRC_WRONG;
        }
    }
    return STATUS_OK;
}

uint8_t PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result)
{
    SPI_SendByte(CommandReg, PCD_Idle); // Stop any active command.
    SPI_SendByte(DivIrqReg, 0x04);      // Clear the CRCIRq interrupt request bit
    SPI_SendByte(FIFOLevelReg, 0x80);
    SPI_SendByte2(FIFODataReg, length, data); // Write data to the FIFO
    SPI_SendByte(CommandReg, PCD_CalcCRC);    // Start the calculation

    // Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73µs.
    for (uint16_t i = 5000; i > 0; i--)
    {
        uint8_t n = SPI_ReceiveByte(DivIrqReg); // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq   reserved CRCIRq reserved reserved
        if (n & 0x04)
        {                                       // CRCIRq bit set - calculation done
            SPI_SendByte(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = SPI_ReceiveByte(CRCResultRegL);
            result[1] = SPI_ReceiveByte(CRCResultRegH);
            return STATUS_OK;
        }
    }
    return STATUS_TIMEOUT;
}

bool ReadCardSerial(void)
{
    uint8_t result = PICC_Select(&data, 0);
    return (result == STATUS_OK);
}

bool IsNewCardPresent(void)
{
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    SPI_SendByte(TxModeReg, 0x00);
    SPI_SendByte(RxModeReg, 0x00);
    // Reset ModWidthReg
    SPI_SendByte(ModWidthReg, 0x26);

    uint8_t result = PICC_RequestA(bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
}

bool PICC_HaltA(void)
{
    uint8_t result;
    uint8_t buffer[4], zero[4]; //zero useless

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK)
    {
        return result;
    }

    // Send the command.
    // The standard says:
    //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //		HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), zero, 0, NULL, 0, false);
    if (result == STATUS_TIMEOUT)
    {
        return STATUS_OK;
    }
    if (result == STATUS_OK)
    { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
}

void PCD_StopCrypto1(void)
{
    // Clear MFCrypto1On bit
    uint8_t value = SPI_ReceiveByte(Status2Reg);
    SPI_SendByte(Status2Reg, value & (~0x08)); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
}
