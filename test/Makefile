default:
	avr-gcc -Wall -Os -DF_CPU=1000000 -mmcu=attiny25 -o blink.o blink.c
	avr-objcopy -j .text -j .data -O ihex blink.o blink.hex

clean: /dev/null
	rm -f blink.o blink.hex