ESP_SDK_DIR = ../../ESP8266_NONOS_SDK
CC = xtensa-lx106-elf-gcc
TOOLCHAIN_INC = /usr/lib/gcc/xtensa-lx106-elf/include
GCC_INC = /usr/lib/gcc/xtensa-lx106-elf/$(shell $(CC) -dumpversion)/include
NEWLIB_INC = /usr/include/newlib

CFLAGS = -Iinclude/ -I. -I../ -mlongcalls \
         -I$(TOOLCHAIN_INC) \
	 -I$(GCC_INC) \
         -I$(ESP_SDK_DIR)/driver_lib/include \
	 -I$(ESP_SDK_DIR)/include \
	 -I$(ESP_SDK_DIR)/ld \
	 -I$(ESP_SDK_DIR)/lib \
	 -I$(NEWLIB_INC) \
         -std=c99 -DICACHE_FLASH 
# CFLAGS = -Iinclude/ -I. -I../ -mlongcalls -I$(ESP_OPEN_SDK_DIR)/ESP8266_NONOS_SDK_V1.5.4_16_05_20/driver_lib/include -I$(ARDUINO_SDK_DIR) -std=c99 -DICACHE_FLASH -nostdinc
# LDLIBS = -L$(ESP_SDK_DIR)/lib -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -ldriver -Wl,--end-group -lgcc -Wl,--gc-sections
# LDLIBS = -L$(ESP_SDK_DIR)/lib -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -Wl,--end-group -lgcc -ldriver -Wl,--gc-sections
# LDLIBS = -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -llwip -lpp -lphy -ldriver -Wl,--end-group -lgcc -Wl,--gc-sections
# LDFLAGS = -Teagle.app.v6.ld -L$(ESP_SDK_DIR)/lib
# # Ensure all SDK blobs are inside the group, and add -lc for standard functions
# LDLIBS = -L$(ESP_SDK_DIR)/lib -nostdlib \
#          -Wl,--start-group \
#          -lmain -lnet80211 -lwpa -llwip -lpp -lphy -ldriver -lc \
#          -Wl,--end-group \
#          -lgcc -Wl,--gc-sections
# Add -lcrypto inside the group to resolve the aes_wrap dependency
LDLIBS = -L$(ESP_SDK_DIR)/lib -nostdlib -Wl,--start-group -lmain -lnet80211 -lwpa -lcrypto -llwip -lpp -lphy -ldriver -lc -Wl,--end-group -lgcc -Wl,--gc-sections
# OBJCP = ../../esp-open-sdk/xtensa-lx106-elf/bin/xtensa-lx106-elf-objcopy
OBJCP = /usr/lib/xtensa-lx106-elf/bin/objcopy
PORT0 = /dev/ttyUSB0
PORT1 = /dev/ttyUSB1

ifeq ($(RELEASE),1)
  CERT = ../../pandaextra/certs/releaseesp
else
  CERT = ../certs/debugesp
  CFLAGS += "-DALLOW_DEBUG"
endif

flashall: user1.bin user2.bin
	../python/panda_esptool.py write_flash 0 $(ESP_SDK_DIR)/bin/boot_v1.7.bin 0x01000 user1.bin 0x81000 user2.bin 0x3FE000 $(ESP_SDK_DIR)/bin/blank.bin

proxy-0x00000.bin: proxy
	../python/panda_esptool.py elf2image $^

proxy: proxy.o elm327.o webserver.o sha.o

obj/proxy.o: proxy.c
	$(CC) $(CFLAGS) -c $^ -o $@

obj/elm327.o: elm327.c
	$(CC) $(CFLAGS) -c $^ -o $@

obj/webserver.o: webserver.c obj/cert.h obj/gitversion.h
	$(CC) $(CFLAGS) -c $< -o $@

obj/cert.h: ../crypto/getcertheader.py
	../crypto/getcertheader.py ../certs/debugesp.pub ../certs/releaseesp.pub > obj/cert.h

include ../common/version.mk

obj/sha.o: ../crypto/sha.c
	$(CC) $(CFLAGS) -c $^ -o $@

obj/rsa.o: ../crypto/rsa.c
	$(CC) $(CFLAGS) -c $^ -o $@

oldflash: proxy-0x00000.bin
	../python/panda_esptool.py write_flash 0 proxy-0x00000.bin 0x40000 proxy-0x40000.bin

user1.bin: obj/proxy.o obj/elm327.o obj/webserver.o obj/sha.o obj/rsa.o
	$(CC) $(CFLAGS) $^ -o a.out -L$(ESP_SDK_DIR)/ld -T$(ESP_SDK_DIR)/ld/eagle.app.v6.new.1024.app1.ld $(LDLIBS)
	$(OBJCP) --only-section .text -O binary a.out eagle.app.v6.text.bin
	$(OBJCP) --only-section .data -O binary a.out eagle.app.v6.data.bin
	$(OBJCP) --only-section .rodata -O binary a.out eagle.app.v6.rodata.bin
	$(OBJCP) --only-section .irom0.text -O binary a.out eagle.app.v6.irom0text.bin
	COMPILE=gcc python2 $(ESP_SDK_DIR)/tools/gen_appbin.py a.out 2 0 32 4 0
	# esptool elf2image --flash_mode dio --flash_freq 40m --flash_size 2MB -o $@ a.out
	# rm -f eagle.app.v6.*.bin
	# mv eagle.app.flash.bin $@
	mv a.out $@
	../crypto/sign.py $@ $@ $(CERT)

user2.bin: obj/proxy.o obj/elm327.o obj/webserver.o obj/sha.o obj/rsa.o
	$(CC) $(CFLAGS) $^ -o a.out -L$(ESP_SDK_DIR)/ld -T$(ESP_SDK_DIR)/ld/eagle.app.v6.new.1024.app2.ld $(LDLIBS)
	$(OBJCP) --only-section .text -O binary a.out eagle.app.v6.text.bin
	$(OBJCP) --only-section .data -O binary a.out eagle.app.v6.data.bin
	$(OBJCP) --only-section .rodata -O binary a.out eagle.app.v6.rodata.bin
	$(OBJCP) --only-section .irom0.text -O binary a.out eagle.app.v6.irom0text.bin
	COMPILE=gcc python2 $(ESP_SDK_DIR)/tools/gen_appbin.py a.out 2 0 32 4 0
	# esptool elf2image --flash_mode dio --flash_freq 40m --flash_size 2MB -o $@ a.out
	# rm -f eagle.app.v6.*.bin
	# mv eagle.app.flash.bin $@
	# mv a.out-0x00000.bin $@
	mv a.out $@
	../crypto/sign.py $@ $@ $(CERT)

ota: user1.bin user2.bin
	curl http://192.168.0.10/espupdate1 --upload-file user1.bin
	curl http://192.168.0.10/espupdate2 --upload-file user2.bin

clean:
	rm -f proxy proxy.o proxy-0x00000.bin proxy-0x40000.bin eagle.app.* user1.bin user2.bin a.out obj/*
