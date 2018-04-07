/*
 * pi-blaster.c Multiple PWM for the Raspberry Pi
 * Copyright (c) 2013 Thomas Sarlandie <thomas@sarlandie.net>
 *
 * Based on the most excellent servod.c by Richard Hirst
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * This program provides very similar functionality to servoblaster, except
 * that rather than implementing it as a kernel module, servod implements
 * the functionality as a usr space daemon.
 *
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE   700
#define _BSD_SOURCE

#ifdef HAVE_CONFIG_H
#include "config.h"
#else
static char VERSION[] = "SNAPSHOT";
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "mailbox.h"

// MAX_CHANNELS is both the highest gpio we can address
// and the maximum number of channels
#define MAX_CHANNELS	46

// Create default known_pins with raspberry pi list of pins
// to compare against the param received.
static uint8_t known_pins[MAX_CHANNELS] = {
		4,
		5,
		6,
		12,
		13,
		16,
		17,     // P1-11
		18,     // P1-12
		19,
		20,
		21,     // P1-40
		22,     // P1-15
		23,     // P1-16
		24,     // P1-18
		25,     // P1-22
		26,
		27,
//		34,
//		35,
//		36,
		};

// Create a list of reserved GPIO pins
// http://elinux.org/RPi_Low-level_peripherals
static uint8_t banned_pins[MAX_CHANNELS] = { 6, // On Model B, it is in use for the Ethernet function
		28,	// board ID and are connected to resistors R3 to R10 (only on Rev1.0 boards).
		29,	// board ID and are connected to resistors R3 to R10 (only on Rev1.0 boards).
		30,	// board ID and are connected to resistors R3 to R10 (only on Rev1.0 boards).
		31,	// board ID and are connected to resistors R3 to R10 (only on Rev1.0 boards).
//		40,		// used by analogue audio
//		45,		// used by analogue audio
		46,		// HDMI hotplug detect
		47,		// 47 to 53 are used by the SD card interface.
		48,		// 47 to 53 are used by the SD card interface.
		49,		// 47 to 53 are used by the SD card interface.
		50,		// 47 to 53 are used by the SD card interface.
		51,		// 47 to 53 are used by the SD card interface.
		52,		// 47 to 53 are used by the SD card interface.
		53,		// 47 to 53 are used by the SD card interface.
		};

// Set num of possible PWM channels based on the known pins size.

uint8_t num_channels = (sizeof(known_pins) / sizeof(known_pins[0]));

// pin2gpio array is not setup as empty to avoid locking all GPIO
// inputs as PWM, they are set on the fly by the pin param passed.
static uint8_t pin2gpio[MAX_CHANNELS];

#define DEVFILE			"/dev/pi-blaster"
#define DEVFILE_MBOX    "/dev/pi-blaster-mbox"
#define DEVFILE_VCIO	"/dev/vcio"

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

// PERIOD_TIME_US is the period of the PWM signal in us.
// Typically it should be 20ms, or 20000us.

// SAMPLE_US is the pulse width increment granularity, again in microseconds.
// Setting SAMPLE_US too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

//#define CYCLE_TIME_US	20000
//#define SAMPLE_US		20
#define CYCLE_TIME_US	2000
#define SAMPLE_US		2
#define NUM_SAMPLES		(CYCLE_TIME_US/SAMPLE_US)
#define NUM_CBS			(NUM_SAMPLES*2)

#define NUM_PAGES		((NUM_CBS * sizeof(dma_cb_t) + NUM_SAMPLES * 8 + \
					PAGE_SIZE - 1) >> PAGE_SHIFT)

#define DMA_BASE		(periph_virt_base + 0x00007000)
#define DMA_CHAN_SIZE	0x100 /* size of register space for a single DMA channel */
#define DMA_CHAN_MAX	14  /* number of DMA Channels we have... actually, there are 15... but channel fifteen is mapped at a different DMA_BASE, so we leave that one alone */
#define DMA_CHAN_NUM	14  /* the DMA Channel we are using, NOTE: DMA Ch 0 seems to be used by X... better not use it ;) */
#define PWM_BASE_OFFSET 0x0020C000
#define PWM_BASE		(periph_virt_base + PWM_BASE_OFFSET)
#define PWM_PHYS_BASE	(periph_phys_base + PWM_BASE_OFFSET)
#define PWM_LEN			0x28
#define CLK_BASE_OFFSET 0x00101000
#define CLK_BASE		(periph_virt_base + CLK_BASE_OFFSET)
#define CLK_LEN			0xA8
#define GPIO_BASE_OFFSET 0x00200000
#define GPIO_BASE		(periph_virt_base + GPIO_BASE_OFFSET)
#define GPIO_PHYS_BASE	(periph_phys_base + GPIO_BASE_OFFSET)
#define GPIO_LEN		0x100
#define PCM_BASE_OFFSET 0x00203000
#define PCM_BASE		(periph_virt_base + PCM_BASE_OFFSET)
#define PCM_PHYS_BASE	(periph_phys_base + PCM_BASE_OFFSET)
#define PCM_LEN			0x24

#define DMA_SRC_INC        (1<<8)
#define DMA_DST_INC       (1<<4)
#define DMA_NO_WIDE_BURSTS	(1<<26)
#define DMA_WAIT_RESP		(1<<3)
#define DMA_D_DREQ		(1<<6)
#define DMA_PER_MAP(x)		((x)<<16)
#define DMA_END			(1<<1)
#define DMA_RESET		(1<<31)
#define DMA_INT			(1<<2)

#define DMA_CS			(0x00/4)
#define DMA_CONBLK_AD		(0x04/4)
#define DMA_DEBUG		(0x20/4)

#define GPIO_FSEL0		(0x00/4)
#define GPIO_SET0		(0x1c/4)
#define GPIO_CLR0		(0x28/4)
#define GPIO_LEV0		(0x34/4)
#define GPIO_PULLEN		(0x94/4)
#define GPIO_PULLCLK		(0x98/4)

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1

#define PWM_CTL			(0x00/4)
#define PWM_DMAC		(0x08/4)
#define PWM_RNG1		(0x10/4)
#define PWM_FIFO		(0x18/4)

#define PWMCLK_CNTL		40
#define PWMCLK_DIV		41

#define PWMCTL_MODE1		(1<<1)
#define PWMCTL_PWEN1		(1<<0)
#define PWMCTL_CLRF		(1<<6)
#define PWMCTL_USEF1		(1<<5)

#define PWMDMAC_ENAB		(1<<31)
#define PWMDMAC_THRSHLD		((15<<8)|(15<<0))

#define PCM_CS_A		(0x00/4)
#define PCM_FIFO_A		(0x04/4)
#define PCM_MODE_A		(0x08/4)
#define PCM_RXC_A		(0x0c/4)
#define PCM_TXC_A		(0x10/4)
#define PCM_DREQ_A		(0x14/4)
#define PCM_INTEN_A		(0x18/4)
#define PCM_INT_STC_A		(0x1c/4)
#define PCM_GRAY		(0x20/4)

#define PCMCLK_CNTL		38
#define PCMCLK_DIV		39

#define DELAY_VIA_PWM		0
#define DELAY_VIA_PCM		1

/* New Board Revision format: 
 SRRR MMMM PPPP TTTT TTTT VVVV

 S scheme (0=old, 1=new)
 R RAM (0=256, 1=512, 2=1024)
 M manufacturer (0='SONY',1='EGOMAN',2='EMBEST',3='UNKNOWN',4='EMBEST')
 P processor (0=2835, 1=2836)
 T type (0='A', 1='B', 2='A+', 3='B+', 4='Pi 2 B', 5='Alpha', 6='Compute Module')
 V revision (0-15)
 */
#define BOARD_REVISION_SCHEME_MASK (0x1 << 23)
#define BOARD_REVISION_SCHEME_OLD (0x0 << 23)
#define BOARD_REVISION_SCHEME_NEW (0x1 << 23)
#define BOARD_REVISION_RAM_MASK (0x7 << 20)
#define BOARD_REVISION_MANUFACTURER_MASK (0xF << 16)
#define BOARD_REVISION_MANUFACTURER_SONY (0 << 16)
#define BOARD_REVISION_MANUFACTURER_EGOMAN (1 << 16)
#define BOARD_REVISION_MANUFACTURER_EMBEST (2 << 16)
#define BOARD_REVISION_MANUFACTURER_UNKNOWN (3 << 16)
#define BOARD_REVISION_MANUFACTURER_EMBEST2 (4 << 16)
#define BOARD_REVISION_PROCESSOR_MASK (0xF << 12)
#define BOARD_REVISION_PROCESSOR_2835 (0 << 12)
#define BOARD_REVISION_PROCESSOR_2836 (1 << 12)
#define BOARD_REVISION_TYPE_MASK (0xFF << 4)
#define BOARD_REVISION_TYPE_PI1_A (0 << 4)
#define BOARD_REVISION_TYPE_PI1_B (1 << 4)
#define BOARD_REVISION_TYPE_PI1_A_PLUS (2 << 4)
#define BOARD_REVISION_TYPE_PI1_B_PLUS (3 << 4)
#define BOARD_REVISION_TYPE_PI2_B (4 << 4)
#define BOARD_REVISION_TYPE_ALPHA (5 << 4)
#define BOARD_REVISION_TYPE_PI3_B (8 << 4)
#define BOARD_REVISION_TYPE_CM (6 << 4)
#define BOARD_REVISION_TYPE_CM3 (10 << 4)
#define BOARD_REVISION_REV_MASK (0xF)

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...)
#endif

static struct {
	int handle; /* From mbox_open() */
	unsigned mem_ref; /* From mem_alloc() */
	unsigned bus_addr; /* From mem_lock() */
	uint8_t *virt_addr; /* From mapmem() */
} mbox;

typedef struct {
	uint32_t info, src, dst, length, stride, next, pad[2];
} dma_cb_t;

struct ctl {
	uint64_t sample[NUM_SAMPLES];
	dma_cb_t cb[NUM_CBS];
};

static uint32_t periph_virt_base;
static uint32_t periph_phys_base;
static uint32_t mem_flag;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_virt_base; /* base address of all DMA Channels */
static volatile uint32_t *dma_reg; /* pointer to the DMA Channel registers we are using */
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;
static int invert_mode = 0;
static int daemonize = 1;

static float channel_pwm[MAX_CHANNELS];

static void set_pwm(int channel, float value);
static void update_pwm();
static void update_pwm_one(int pin, float width, float last_width);
static void fatal(char *fmt, ...);

int mbox_open() {
	int file_desc;

	// open a char device file used for communicating with kernel mbox driver

	// try to use /dev/vcio first (kernel 4.1+)
	file_desc = open(DEVFILE_VCIO, 0);
	if (file_desc < 0) {
		/* initialize mbox */
		unlink(DEVFILE_MBOX);
		if (mknod(DEVFILE_MBOX, S_IFCHR | 0600, makedev(MAJOR_NUM, 0)) < 0)
			fatal("Failed to create mailbox device\n");
		file_desc = open(DEVFILE_MBOX, 0);
		if (file_desc < 0) {
			printf("Can't open device file: %s\n", DEVFILE_MBOX);
			perror(NULL);
			exit(-1);
		}
	}
	return file_desc;
}

void mbox_close(int file_desc) {
	close(file_desc);
}

static void gpio_set_mode(uint32_t pin, uint32_t mode) {
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin / 10];

	fsel &= ~(7 << ((pin % 10) * 3));
	fsel |= mode << ((pin % 10) * 3);
	gpio_reg[GPIO_FSEL0 + pin / 10] = fsel;
}

static void gpio_set(int pin, int level) {
	int reg_offset = pin / 32;
	int _pin = pin % 32;
	if (level)
		gpio_reg[GPIO_SET0 + reg_offset] = 1 << _pin;
	else
		gpio_reg[GPIO_CLR0 + reg_offset] = 1 << _pin;
}

static void udelay(int us) {
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void terminate(int dummy) {
	int i;

	dprintf("Resetting DMA...\n");
	if (dma_reg && mbox.virt_addr) {
		for (i = 0; i < num_channels; i++)
			channel_pwm[i] = 0;
		update_pwm();
		udelay(CYCLE_TIME_US);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}

	dprintf("Freeing mbox memory...\n");
	if (mbox.virt_addr != NULL) {
		unmapmem(mbox.virt_addr, NUM_PAGES * PAGE_SIZE);
		if (mbox.handle <= 2) {
			/* we need to reopen mbox file */
			mbox.handle = mbox_open();
		}
		mem_unlock(mbox.handle, mbox.mem_ref);
		mem_free(mbox.handle, mbox.mem_ref);
		mbox_close(mbox.handle);
	}dprintf("Unlink %s...\n", DEVFILE);
	unlink(DEVFILE);
	dprintf("Unlink %s...\n", DEVFILE_MBOX);
	unlink(DEVFILE_MBOX);
	printf("pi-blaster stopped.\n");

	exit(1);
}

static void fatal(char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

/* 
 * determine which pi model we are running on 
 */
static void get_model(unsigned mbox_board_rev) {
	int board_model = 0;

	if ((mbox_board_rev & BOARD_REVISION_SCHEME_MASK)
			== BOARD_REVISION_SCHEME_NEW) {
		if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK)
				== BOARD_REVISION_TYPE_PI2_B) {
			board_model = 2;
		} else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK)
				== BOARD_REVISION_TYPE_PI3_B) {
			board_model = 3;
		} else if ((mbox_board_rev & BOARD_REVISION_TYPE_MASK)
				== BOARD_REVISION_TYPE_CM3) {
			board_model = 3;
		} else {
			// no Pi 2, we assume a Pi 1
			board_model = 1;
		}
	} else {
		// if revision scheme is old, we assume a Pi 1
		board_model = 1;
	}

	switch (board_model) {
	case 1:
		periph_virt_base = 0x20000000;
		periph_phys_base = 0x7e000000;
		mem_flag = MEM_FLAG_L1_NONALLOCATING | MEM_FLAG_ZERO;
		break;
	case 2:
	case 3:
		periph_virt_base = 0x3f000000;
		periph_phys_base = 0x7e000000;
		mem_flag = MEM_FLAG_L1_NONALLOCATING | MEM_FLAG_ZERO;
		break;
	default:
		fatal("Unable to detect Board Model from board revision: %#x",
				mbox_board_rev);
		break;
	}
}

static uint32_t mem_virt_to_phys(void *virt) {
	uint32_t offset = (uint8_t *) virt - mbox.virt_addr;

	return mbox.bus_addr + offset;
}

static void *
map_peripheral(uint32_t base, uint32_t len) {
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	void * vaddr;

	if (fd < 0)
		fatal("pi-blaster: Failed to open /dev/mem: %m\n");
	vaddr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED)
		fatal("pi-blaster: Failed to map peripheral at 0x%08x: %m\n", base);
	close(fd);

	return vaddr;
}

// Check if the pin provided is found in the list of known pins.
static int is_known_pin(int pin) {
	int found = 0;
	int i;
	for (i = 0; i < num_channels; i++) {
		if (known_pins[i] == pin) {
			found = 1;
			break;
		}
	}
	return (found);
}

// Check if the pin provided is found in the list of BANNED pins.
static int is_banned_pin(int pin) {
	int found = 0;
	int i;
	for (i = 0; i < num_channels; i++) {
		if (banned_pins[i] == pin) {
			found = 1;
			break;
		}
	}
	return (found);
}

// Set the pin to a pin2gpio element so pi-blaster can write to it,
// and set the width of the PWM pulse to the element with the same index
// in channel_pwm array.
static int set_pin2gpio(int pin, float width) {
	int established = 0;

	int i;
	for (i = 0; i < num_channels; i++) {
		if (pin2gpio[i] == pin || pin2gpio[i] == 0) {
			if (pin2gpio[i] == 0) {
				gpio_set(pin, invert_mode);
				gpio_set_mode(pin, GPIO_MODE_OUT);
			}
			pin2gpio[i] = pin;
			channel_pwm[i] = width;
			established = 1;
			break;
		}
	}

	return (established);
}

// To avoid storing the same pin 2 times after one pin has been released
// we compact the pin2gpio array so all ON PWM pins are at the begining.
static void compact_pin2gpio() {
	int i, j = 0;
	uint8_t tmp_pin2gpio[MAX_CHANNELS];
	float tmp_channel_pwm[MAX_CHANNELS];

	for (i = 0; i < num_channels; i++) {
		if (pin2gpio[i] != 0) {
			tmp_pin2gpio[j] = pin2gpio[i];
			tmp_channel_pwm[j] = channel_pwm[i];
			j++;
		}
	}
	for (i = 0; i < num_channels; i++) {
		pin2gpio[i] = tmp_pin2gpio[i];
		channel_pwm[i] = tmp_channel_pwm[i];
	}
}

// Pins can be relesead after being setup as PWM pins by writing the release <pin>
// command to the /dev/pi-blaster file. We make sure to compact the pin2gpio array
// that contains currently working pwm pins.
static int release_pin2gpio(int pin) {
	int released = 0;

	int i;
	for (i = 0; i < num_channels; i++) {
		if (pin2gpio[i] == pin) {
			channel_pwm[i] = 0;
			pin2gpio[i] = 0;
			released = 1;
			break;
		}
	}

	compact_pin2gpio();
	return (released);

}

// Set each provided pin to one in pin2gpio
static void set_pin(int pin, float width) {
	if (is_known_pin(pin)) {
		set_pin2gpio(pin, width);
	} else {
		fprintf(stderr, "GPIO %d is not enabled for pi-blaster\n", pin);
	}
}

// Function make sure the pin we want to release is a valid pin, if it is
// then calls release_pin2gpio to delete it from currently ON pins.
static void release_pin(int pin) {
	if (is_known_pin(pin)) {
		release_pin2gpio(pin);
	} else {
		fprintf(stderr, "GPIO %d is not enabled for pi-blaster\n", pin);
	}
}

// Releases the PWM pin requested (if found and valid) and updates the calls
// update_pwm to apply the changes to the actual hardware pins.
static void release_pwm(int pin) {
	release_pin(pin);
	update_pwm();
}

// Set pin2gpio pins, channel width and update the pwm send to pins being used.
static void set_pwm(int channel, float width) {
	int increment = 0;
	float last_width = 0;
	for (int i = 0; i < num_channels; i++) {
		if (pin2gpio[i] == channel) {
			last_width = channel_pwm[i];
			channel_pwm[i] = width;
			increment = 1;
			break;
		}
	}
	if (increment) {
		update_pwm_one(channel, width, last_width);
	} else {
		set_pin(channel, width);
		update_pwm();
	}
}

// Run set_pwm on every known pin, setting all of them to the provided width.
static void set_all_pwm(float width) {
	int i;
	for (i = 0; i < num_channels; i++) {
		if (is_known_pin(i)) {
			set_pwm(i, width);
		}
	}
}

/*
 * What we need to do here is:
 *   First DMA command turns on the pins that are >0
 *   All the other packets turn off the pins that are not used
 *
 * For the cpb packets (The DMA control packet)
 *  -> cbp[0]->dst = gpset0: set   the pwms that are active
 *  -> cbp[*]->dst = gpclr0: clear when the sample has a value
 *
 * For the samples     (The value that is written by the DMA command to cbp[n]->dst)
 *  -> dp[0] = mask of the pwms that are active
 *  -> dp[n] = mask of the pwm to stop at time n
 *
 * We dont really need to reset the cb->dst each time but I believe it helps a lot
 * in code readability in case someone wants to generate more complex signals.
 */
static void update_pwm() {

	const uint32_t phys_gpclr0 = GPIO_PHYS_BASE + 0x28;
	const uint32_t phys_gpset0 = GPIO_PHYS_BASE + 0x1c;
	struct ctl *ctl = (struct ctl *) mbox.virt_addr;
	uint64_t mask;
	uint64_t mask_all;

	int i, j;
	/* First we turn on the channels that need to be on */
	/*   Take the first DMA Packet and set it's target to start pulse */
	if (invert_mode)
		ctl->cb[0].dst = phys_gpclr0;
	else
		ctl->cb[0].dst = phys_gpset0;

	/*   Now create a mask of all the pins that should be on */
	mask_all = 0;
	for (i = 0; i < num_channels; i++) {
		// Check the pin2gpio pin has been set to avoid locking all of them as PWM.
		if (channel_pwm[i] > 0 && pin2gpio[i]) {
			mask_all |= (uint64_t) 1 << pin2gpio[i];
		}
	}
	/*   And give that to the DMA controller to write */
	ctl->sample[0] = mask_all;

	/* Now we go through all the samples and turn the pins off when needed */
	for (j = 1; j < NUM_SAMPLES; j++) {
		if (invert_mode) {
			if (ctl->cb[j * 2].dst == phys_gpset0) {
				break;
			}
			ctl->cb[j * 2].dst = phys_gpset0;
		} else {
			if (ctl->cb[j * 2].dst == phys_gpclr0) {
				break;
			}
			ctl->cb[j * 2].dst = phys_gpclr0;
		}
	}
	for (j = 1; j < NUM_SAMPLES; j++) {
		mask = 0;
		for (i = 0; i < num_channels; i++) {
			// Check the pin2gpio pin has been set to avoid locking all of them as PWM.
			if ((float) j / NUM_SAMPLES > channel_pwm[i] && pin2gpio[i])
				mask |= (uint64_t) 1 << pin2gpio[i];
		}
		if (mask == mask_all && ctl->sample[j] == mask_all) {
			break;
		}
		ctl->sample[j] = mask;
	}
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
static void update_pwm_one(int pin, float width, float last_width) {
	struct ctl *ctl = (struct ctl *) mbox.virt_addr;

	int last_idx = last_width * NUM_SAMPLES;
	int idx = width * NUM_SAMPLES;
	last_idx = MAX(1, MIN(last_idx, NUM_SAMPLES-1));
	idx = MAX(1, MIN(idx, NUM_SAMPLES-1));

	if (last_idx == idx) {
		return;
	} else if (last_idx < idx) {
		for (int j = last_idx; j < NUM_SAMPLES && j <= idx; j++) {
			ctl->sample[j] &= ~((uint64_t) 1 << pin);
		}
		ctl->sample[0] |= (uint64_t) 1 << pin;
	} else {
		for (int j = last_idx; j >= 1 && j >= idx; j--) {
			ctl->sample[j] |= (uint64_t) 1 << pin;
		}
		ctl->sample[0] |= (uint64_t) 1 << pin;
	}
}

static void setup_sighandlers(void) {
	int i;

	// Catch all signals possible - it is vital we kill the DMA engine
	// on process exit!
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
	signal(SIGWINCH, SIG_IGN);

}

static void init_ctrl_data(void) {
	dprintf("Initializing DMA ...\n");
	struct ctl *ctl = (struct ctl *) mbox.virt_addr;
	dma_cb_t *cbp = ctl->cb;
	uint32_t phys_fifo_addr;
	uint32_t phys_gpclr0 = GPIO_PHYS_BASE + 0x28;
	uint32_t phys_gpset0 = GPIO_PHYS_BASE + 0x1c;
	uint64_t mask;
	int i;

	if (delay_hw == DELAY_VIA_PWM)
		phys_fifo_addr = PWM_PHYS_BASE + 0x18;
	else
		phys_fifo_addr = PCM_PHYS_BASE + 0x04;
	memset(ctl->sample, 0, sizeof(ctl->sample));

	// Calculate a mask to turn off all the servos
	mask = 0;
	for (i = 0; i < num_channels; i++) {
		mask |= (uint64_t) 1 << known_pins[i];
	}
	for (i = 0; i < NUM_SAMPLES; i++)
		ctl->sample[i] = mask;

	/* Initialize all the DMA commands. They come in pairs.
	 *  - 1st command copies a value from the sample memory to a destination
	 *    address which can be either the gpclr0 register or the gpset0 register
	 *  - 2nd command waits for a trigger from an external source (PWM or PCM)
	 */
	for (i = 0; i < NUM_SAMPLES; i++) {
		/* First DMA command */
		cbp->info = DMA_SRC_INC | DMA_DST_INC | DMA_NO_WIDE_BURSTS
				| DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		if (invert_mode)
			cbp->dst = phys_gpset0;
		else
			cbp->dst = phys_gpclr0;
		cbp->length = 8;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		/* Second DMA command */
		if (delay_hw == DELAY_VIA_PWM)
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ
					| DMA_PER_MAP(5);
		else
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ
					| DMA_PER_MAP(2);
		cbp->src = mem_virt_to_phys(ctl);	// Any data will do
		cbp->dst = phys_fifo_addr;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
	}
	cbp--;
	cbp->next = mem_virt_to_phys(ctl->cb);
}

static void init_hardware(void) {
	dprintf("Initializing PWM/PCM HW...\n");
	struct ctl *ctl = (struct ctl *) mbox.virt_addr;
	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (500 << 12);	// set pwm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = SAMPLE_US;
		udelay(10);
		pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_CLRF;
		udelay(10);
		pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1;
		udelay(10);
	} else {
		// Initialise PCM
		pcm_reg[PCM_CS_A] = 1;				// Disable Rx+Tx, Enable PCM block
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (500 << 12);	// Set pcm div to 500, giving 1MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0 << 31 | 1 << 30 | 0 << 20 | 0 << 16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (SAMPLE_US - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1 << 4 | 1 << 3;		// Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64 << 24 | 64 << 8;// DMA Req when one slot is free?
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1 << 9;			// Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = DMA_INT | DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001; // go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1 << 2;			// Enable Tx
	}
}

static void debug_dump_hw(void) {
#ifdef DEBUG
	printf("pwm_reg: %p\n", (void *) pwm_reg);
	int i;
	struct ctl *ctl = (struct ctl *)mbox.virt_addr;
	dma_cb_t *cbp = ctl->cb;
	i = 0;
	for (i = 0; i < NUM_SAMPLES; i++) {
		printf("DMA Control Block: #%d @0x%08x, \n", i, cbp);
		printf("info:   0x%08x\n", cbp->info);
		printf("src:    0x%08x\n", cbp->src);
		printf("dst:    0x%08x\n", cbp->dst);
		printf("length: 0x%08x\n", cbp->length);
		printf("stride: 0x%08x\n", cbp->stride);
		printf("next:   0x%08x\n", cbp->next);
		cbp++; // next control block
	}
	printf("PWM_BASE: %p\n", (void *) PWM_BASE);
	printf("pwm_reg: %p\n", (void *) pwm_reg);
	for (i=0; i<PWM_LEN/4; i++) {
		printf("%04x: 0x%08x 0x%08x\n", i, &pwm_reg[i], pwm_reg[i]);
	}
	printf("CLK_BASE: %p\n", (void *) CLK_BASE);
	printf("PWMCLK_CNTL: %x\n", PWMCLK_CNTL);
	printf("clk_reg[PWMCLK_CNTL]: %p\n", &clk_reg[PWMCLK_CNTL]);
	printf("PWMCLK_DIV: %x\n", PWMCLK_DIV);
	printf("clk_reg: %p\n", (void *) clk_reg);
	printf("virt_to_phys(clk_reg): %x\n", mem_virt_to_phys(clk_reg));
	for (i=0; i<CLK_LEN/4; i++) {
		printf("%04x: 0x%08x 0x%08x\n", i, &clk_reg[i], clk_reg[i]);
	}
	printf("DMA_BASE: %p\n", (void *) DMA_BASE);
	printf("dma_virt_base: %p\n", (void *) dma_virt_base);
	printf("dma_reg: %p\n", (void *) dma_reg);
	printf("virt_to_phys(dma_reg): %x\n", mem_virt_to_phys(dma_reg));
	for (i=0; i<DMA_CHAN_SIZE/4; i++) {
		printf("%04x: 0x%08x 0x%08x\n", i, &dma_reg[i], dma_reg[i]);
	}
#endif
}

static void debug_dump_samples() {
#ifdef DEBUG
	struct ctl *ctl = (struct ctl *)mbox.virt_addr;
	int i;
	for (i = 0; i < NUM_SAMPLES; i++) {
		printf("#%d @0x%08x, \n", i, ctl->sample[i]);
	}
#endif
}

static void init_pin2gpio(void) {
	int i;
	for (i = 0; i < num_channels; i++)
		pin2gpio[i] = 0;
}

static void init_pwm(void) {
	update_pwm();
}

static void init_channel_pwm(void) {
	dprintf("Initializing Channels...\n");
	int i;
	for (i = 0; i < num_channels; i++)
		channel_pwm[i] = 0;
}

static void go_go_go(void) {
	FILE *fp;

	if ((fp = fopen(DEVFILE, "r+")) == NULL)
		fatal("pi-blaster: Failed to open %s: %m\n", DEVFILE);

	char *lineptr = NULL, nl;
	size_t linelen;

	for (;;) {
		int n, servo;
		float value;

		if ((n = getline(&lineptr, &linelen, fp)) < 0)
			continue;dprintf("[%d]%s", n, lineptr);
		if (!strcmp(lineptr, "debug_regs\n")) {
			debug_dump_hw();
		} else if (!strcmp(lineptr, "debug_samples\n")) {
			debug_dump_samples();
		} else if (sscanf(lineptr, "release %d%c", &servo, &nl) == 2
				&& nl == '\n') {
			release_pwm(servo);
		} else if (sscanf(lineptr, "*=%f%c", &value, &nl) == 2 && nl == '\n') {
			if (value < 0 || value > 1) {
				fprintf(stderr, "Invalid value %f\n", value);
			} else {
				set_all_pwm(value);
			}
		} else if (sscanf(lineptr, "%d=%fus%c", &servo, &value, &nl) == 3
				&& nl == '\n') {
			value = value / (float) CYCLE_TIME_US;
			if (servo < 0) {
				fprintf(stderr, "Invalid channel number %d\n", servo);
			} else if (value < 0 || value > 1) {
				fprintf(stderr, "Invalid value %f\n", value);
			} else {
				set_pwm(servo, value);
			}
		} else if (sscanf(lineptr, "%d=%f%c", &servo, &value, &nl) == 3
				&& nl == '\n') {
			if (servo < 0) {
				fprintf(stderr, "Invalid channel number %d\n", servo);
			} else if (value < 0 || value > 1) {
				fprintf(stderr, "Invalid value %f\n", value);
			} else {
				set_pwm(servo, value);
			}
		} else {
			fprintf(stderr, "Bad input: %s", lineptr);
		}
	}
}

void parseargs(int argc, char **argv) {
	int index;
	int c;
	static uint8_t temp_known_pins[MAX_CHANNELS];
	int i = 0;

	static struct option longopts[] = { { "help", no_argument, 0, 'h' }, {
			"gpio", required_argument, 0, 'g' },
			{ "invert", no_argument, 0, 'i' }, { "pcm", no_argument, 0, 'p' }, {
					"version", no_argument, 0, 'v' }, { 0, 0, 0, 0 } };

	while (1) {

		index = 0;
		c = getopt_long(argc, argv, "Dg:hipv", longopts, &index);

		if (c == -1)
			break;

		switch (c) {
		case 0:
			/* handle flag options (array's 3rd field non-0) */
			break;

		case 'h':
			fprintf(stderr, "%s version %s\n", argv[0], VERSION);
			fprintf(stderr,
					"Usage: %s [-hDgipv]\n"
							"-h (--help)    - this information\n"
							"-D (--daemon)  - Don't daemonize\n"
							"-g (--gpio)    - comma separated list of GPIOs to use\n"
							"                 If omitted, default is \"4,17,18,27,21,22,23,24,25\"\n"
							"-i (--invert)  - invert pin output (pulse LOW)\n"
							"-p (--pcm)     - use pcm for dmascheduling\n"
							"-v (--version) - version information\n", argv[0]);
			exit(-1);

		case 'D':
			daemonize = 0;
			break;

		case 'g':
			if (optarg) {
				int temp = 0;

				const char s[2] = ",";
				char *token;

				/* get the first token */
				token = strtok(optarg, s);

				/* walk through other tokens */
				i = 0;
				while (token != NULL) {
					if ((temp = atoi(token)) && (temp >= 0)
							&& (temp < MAX_CHANNELS)) {
						if (i >= MAX_CHANNELS) {
							printf(
									"ERROR maximum number of channels (%d) exceeded.\n",
									MAX_CHANNELS);
							exit(-1);
						}
						if (!is_banned_pin(temp)) {
							temp_known_pins[i++] = temp;
						} else {
							printf("ERROR '%s' is a banned gpio\n", token);
							exit(-1);
						}
					} else {
						printf("ERROR '%s' is an invalid gpio\n", token);
						exit(-1);
					}
					token = strtok(NULL, s);
				}
			}
			num_channels = i;
			for (i = 0; i < MAX_CHANNELS; i++) {
				if (i < num_channels) {
					known_pins[i] = temp_known_pins[i];
				} else {
					known_pins[i] = 0;
				}
			}
			break;

		case 'i':
			invert_mode = 1;
			break;

		case 'p':
			delay_hw = DELAY_VIA_PCM;
			break;

		case 'v':
			fprintf(stderr, "%s version %s\n", argv[0], VERSION);
			exit(-1);

		case '?':
			/* getopt_long already reported error? */
			exit(-1);

		default:
			exit(-1);
		}
	}
}

int main(int argc, char **argv) {
	parseargs(argc, argv);
	mbox.handle = mbox_open();
	if (mbox.handle < 0)
		fatal("Failed to open mailbox\n");
	unsigned mbox_board_rev = get_board_revision(mbox.handle);
	printf("MBox Board Revision: %#x\n", mbox_board_rev);
	get_model(mbox_board_rev);
	unsigned mbox_dma_channels = get_dma_channels(mbox.handle);
	printf("DMA Channels Info: %#x, using DMA Channel: %d\n", mbox_dma_channels,
	DMA_CHAN_NUM);

	printf("Using hardware:                 %5s\n",
			delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Number of channels:             %5d\n", (int) num_channels);
	printf("PWM frequency:               %5d Hz\n", 1000000 / CYCLE_TIME_US);
	printf("PWM steps:                      %5d\n", NUM_SAMPLES);
	printf("Maximum period (100  %%):      %5dus\n", CYCLE_TIME_US);
	printf("Minimum period (%1.3f%%):      %5dus\n",
			100.0 * SAMPLE_US / CYCLE_TIME_US, SAMPLE_US);
	printf("DMA Base:                  %#010x\n", DMA_BASE);

	setup_sighandlers();

	/* map the registers for all DMA Channels */
	dma_virt_base = map_peripheral(DMA_BASE,
			(DMA_CHAN_SIZE * (DMA_CHAN_MAX + 1)));
	/* set dma_reg to point to the DMA Channel we are using */
	dma_reg = dma_virt_base + DMA_CHAN_NUM * (DMA_CHAN_SIZE / sizeof(dma_reg));
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	/* Use the mailbox interface to the VC to ask for physical memory */
	mbox.mem_ref = mem_alloc(mbox.handle, NUM_PAGES * PAGE_SIZE, PAGE_SIZE,
			mem_flag);
	/* TODO: How do we know that succeeded? */
	dprintf("mem_ref %u\n", mbox.mem_ref);
	mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref);
	dprintf("bus_addr = %#x\n", mbox.bus_addr);
	mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), NUM_PAGES * PAGE_SIZE);
	dprintf("virt_addr %p\n", mbox.virt_addr);

	if ((unsigned long) mbox.virt_addr & (PAGE_SIZE - 1))
		fatal("pi-blaster: Virtual address is not page aligned\n");

	/* we are done with the mbox */
	mbox_close(mbox.handle);
	mbox.handle = -1;

	//fatal("TempFatal\n");

	init_ctrl_data();
	init_hardware();
	init_channel_pwm();
	// Init pin2gpio array with 0/false values to avoid locking all of them as PWM.
	init_pin2gpio();
	// Only calls update_pwm after ctrl_data calculates the pin mask to unlock all pins on start.
	init_pwm();
	unlink(DEVFILE);
	if (mkfifo(DEVFILE, 0666) < 0)
		fatal("pi-blaster: Failed to create %s: %m\n", DEVFILE);
	if (chmod(DEVFILE, 0666) < 0)
		fatal("pi-blaster: Failed to set permissions on %s: %m\n", DEVFILE);

	printf("Initialised, ");
	if (daemonize) {
		if (daemon(0, 1) < 0)
			fatal("pi-blaster: Failed to daemonize process: %m\n");
		else
			printf("Daemonized, ");
	}
	printf("Reading %s.\n", DEVFILE);

	go_go_go();

	return 0;
}

