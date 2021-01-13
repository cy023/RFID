FILENAME   = src/main
UART       = src/uart
RFID	   = src/rfid
PORT       = /dev/ttyUSB0
DEVICE     = atmega328p
PROGRAMMER = arduino
BAUD       = 57600
LINKER       = -Wl,--start-group -Wl,--end-group -Wl,--gc-sections -mrelax -Wl,-lm
LINKER      += -Wl,-lprintf_flt -Wl,-lscanf_flt
LINKER      += -Wl,-u,vfprintf -Wl,-u,vfscanf
COMPILE    = avr-gcc -Wall -Os -mmcu=$(DEVICE) $(LINKER)

.PHONY: all compile upload terminal clean

all: compile upload terminal

compile:
	$(COMPILE) -c $(UART).c -o $(UART).o
	$(COMPILE) -c $(RFID).c -o $(RFID).o
	$(COMPILE) -c $(FILENAME).c -o $(FILENAME).o
	$(COMPILE) -o $(FILENAME).elf $(FILENAME).o $(UART).o $(RFID).o
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

upload:
	avrdude -v -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -b $(BAUD) -U flash:w:$(FILENAME).hex:i

terminal:
	putty -serial $(PORT) -sercfg 9600,8,1,n,N

clean:
	rm src/*.o src/*.elf src/*.hex