import os

# toolchains options
ARCH='Xtensa'
CPU='ESP8266EX'
CROSS_TOOL='gcc'

if os.getenv('RTT_CC'):
	CROSS_TOOL = os.getenv('RTT_CC')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR

if  CROSS_TOOL == 'gcc':
	PLATFORM 	= 'gcc'
	EXEC_PATH 	= '/opt/xtensa-lx106-elf/bin'
else:
    print 'Please make sure your toolchains is GNU GCC!'
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
	EXEC_PATH = os.getenv('RTT_EXEC_PATH')

#choose bin generate(0=eagle.flash.bin+eagle.irom0text.bin, 1=user1.bin, 2=user2.bin)
APP = '1'
#choose spi speed(0=20MHz, 1=26.7MHz, 2=40MHz, 3=80MHz)
SPI_SPEED = '2'
#choose spi mode(0=QIO, 1=QOUT, 2=DIO, 3=DOUT)
SPI_MODE = '0'
'''
choose spi size and map
0= 512KB( 256KB+ 256KB)
2=1024KB( 512KB+ 512KB)
3=2048KB( 512KB+ 512KB)
4=4096KB( 512KB+ 512KB)
5=2048KB(1024KB+1024KB)
6=4096KB(1024KB+1024KB)
7=4096KB(2048KB+2048KB) not support ,just for compatible with nodeMCU board
8=8192KB(1024KB+1024KB)
9=16384KB(1024KB+1024KB)
'''
SPI_SIZE_MAP = 2

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'xtensa-lx106-elf-'
    CC = PREFIX + 'gcc'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'out'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'

    DEVICE = ' -g -fno-aggressive-loop-optimizations -Wpointer-arith -Wundef -Werror -Wl,-EL -fno-inline-functions -nostdlib -mlongcalls -mtext-section-literals -ffunction-sections -fdata-sections -fno-builtin-printf -fno-jump-tables'
    CFLAGS = DEVICE
    AFLAGS = DEVICE
    LFLAGS = ' -Wl,--gc-sections -nostdlib'
    LFLAGS += ' -T ./Libraries/ld/eagle.app.v6.new.1024.app1.ld'
    LFLAGS += ' -T./Libraries/ld/eagle.app.v6.common.ld' + ' -T./Libraries/ld/eagle.rom.addr.v6.ld'
    LFLAGS += ' -Wl,--no-check-sections -Wl,-static -u,call_user_start'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -Os'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

POST_ACTION = OBJDUMP + ' -x -s $TARGET > rtthread-esp8266.dump\n' + \
                OBJDUMP + ' -S $TARGET > rtthread-esp8266.S\n' + \
                OBJCPY + ' --only-section .text -O binary $TARGET eagle.app.v6.text.bin \n' + \
                OBJCPY + ' --only-section .data -O binary $TARGET eagle.app.v6.data.bin\n' + \
                OBJCPY + ' --only-section .rodata -O binary $TARGET eagle.app.v6.rodata.bin\n' + \
                OBJCPY + ' --only-section .irom0.text -O binary $TARGET eagle.app.v6.irom0text.bin\n' + SIZE + ' $TARGET \n'