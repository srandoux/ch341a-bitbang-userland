CFLAGS=-std=gnu99 -Wall
ch341: main.c ch341a.c ch341a.h
	gcc $(CFLAGS) ch341a.c main.c -o ch341 -lusb-1.0 -I /opt/homebrew/Cellar/libusb/1.0.26/include/ -L /opt/homebrew/Cellar/libusb/1.0.26/lib/
clean:
	rm *.o ch341 -f
install-udev-rule:
	cp 99-ch341a.rules /etc/udev/rules.d/
	udevadm control --reload-rules
.PHONY: clean install-udev-rule
