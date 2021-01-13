#define F_CPU 16000000UL
#include "./rfid.h"
#include "./uart.h"

int main(void)
{
    NANO_stdio_set();
    SPI_Set();
    RFID_Set();

    printf("--------------------START------------------------\n");
    while (1)
    {
        while (!IsNewCardPresent())
            ;
        while (!ReadCardSerial())
            ;
        printf("A new card detected\n");
        for (uint8_t i = 0; i < data.size; i++)
            printf("data[%d] = %x\n", i, data.uidByte[i]);
        PICC_HaltA();
        PCD_StopCrypto1();
    }
}
