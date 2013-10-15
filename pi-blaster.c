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

static char VERSION[] = "0.1.0";

// Created new known_pins with raspberry pi list of pins
// to compare against the param received.
static uint8_t known_pins[] = {
        4,      // P1-7
        17,     // P1-11
        18,     // P1-12
        21,     // P1-13
        22,     // P1-15
        23,     // P1-16
        24,     // P1-18
        25,     // P1-22
};

// pin2gpio array is not setup as empty to avoid locking all GPIO
// inputs as PWM, they are set on the fly by the pin param passed.
static uint8_t pin2gpio[8];

// Set num of possible PWM channels based on the known pins size.
#define NUM_CHANNELS    (sizeof(known_pins)/sizeof(known_pins[0]))

#define DEVFILE			"/dev/pi-blaster"

#define PAGE_SIZE		4096
#define PAGE_SHIFT		12

// PERIOD_TIME_US is the period of the PWM signal in us.
// Typically it should be 20ms, or 20000us.

// SAMPLE_US is the pulse width increment granularity, again in microseconds.
// Setting SAMPLE_US too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.

#define CYCLE_TIME_US		10000
#define SAMPLE_US		10
#define NUM_SAMPLES		(CYCLE_TIME_US/SAMPLE_US)
#define NUM_CBS			(NUM_SAMPLES*2)

#define NUM_PAGES		((NUM_CBS * 32 + NUM_SAMPLES * 4 + \
					PAGE_SIZE - 1) >> PAGE_SHIFT)

#define DMA_BASE		0x20007000
#define DMA_LEN			0x24
#define PWM_BASE		0x2020C000
#define PWM_LEN			0x28
#define CLK_BASE	        0x20101000
#define CLK_LEN			0xA8
#define GPIO_BASE		0x20200000
#define GPIO_LEN		0x100
#define PCM_BASE		0x20203000
#define PCM_LEN			0x24

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

typedef struct {
	uint32_t info, src, dst, length,
		 stride, next, pad[2];
} dma_cb_t;

struct ctl {
	uint32_t sample[NUM_SAMPLES];
	dma_cb_t cb[NUM_CBS];
};

typedef struct {
	uint8_t *virtaddr;
	uint32_t physaddr;
} page_map_t;

page_map_t *page_map;

static uint8_t *virtbase;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *pcm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

static int delay_hw = DELAY_VIA_PWM;
static int invert_mode = 0;

static float channel_pwm[NUM_CHANNELS];

static void set_pwm(int channel, float value);
static void update_pwm();

static void
gpio_set_mode(uint32_t pin, uint32_t mode)
{
	uint32_t fsel = gpio_reg[GPIO_FSEL0 + pin/10];

	fsel &= ~(7 << ((pin % 10) * 3));
	fsel |= mode << ((pin % 10) * 3);
	gpio_reg[GPIO_FSEL0 + pin/10] = fsel;
}

static void
gpio_set(int pin, int level)
{
	if (level)
		gpio_reg[GPIO_SET0] = 1 << pin;
	else
		gpio_reg[GPIO_CLR0] = 1 << pin;
}

static void
udelay(int us)
{
	struct timespec ts = { 0, us * 1000 };

	nanosleep(&ts, NULL);
}

static void
terminate(int dummy)
{
	int i;

	if (dma_reg && virtbase) {
		for (i = 0; i < NUM_CHANNELS; i++)
			channel_pwm[i] = 0;
		update_pwm();
		udelay(CYCLE_TIME_US);
		dma_reg[DMA_CS] = DMA_RESET;
		udelay(10);
	}
	unlink(DEVFILE);
	exit(1);
}

static void
fatal(char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	terminate(0);
}

static uint32_t
mem_virt_to_phys(void *virt)
{
	uint32_t offset = (uint8_t *)virt - virtbase;

	return page_map[offset >> PAGE_SHIFT].physaddr + (offset % PAGE_SIZE);
}

static void *
map_peripheral(uint32_t base, uint32_t len)
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	void * vaddr;

	if (fd < 0)
		fatal("pi-blaster: Failed to open /dev/mem: %m\n");
	vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
	if (vaddr == MAP_FAILED)
		fatal("pi-blaster: Failed to map peripheral at 0x%08x: %m\n", base);
	close(fd);

	return vaddr;
}

// Check if the pin provided is found in the list of known pins.
static int
is_known_pin(int pin){
  int found = 0;

  int i;
  for (i = 0; i < NUM_CHANNELS; i++) {
    if (known_pins[i] == pin) {
      found = 1;
      break;
    }
  }
  return(found);
}

// Set the pin to a pin2gpio element so pi-blaster can write to it,
// and set the width of the PWM pulse to the element with the same index
// in channel_pwm array.
static int
set_pin2gpio(int pin, float width){
  int established = 0;

  int i;
  for (i = 0; i < NUM_CHANNELS; i++) {
    if (pin2gpio[i] == pin || pin2gpio[i] == 0) {
      pin2gpio[i] = pin;
      channel_pwm[i] = width;
      established = 1;
      break;
    }
  }

  return(established);

}

// To avoid storing the same pin 2 times after one pin has been released
// we compact the pin2gpio array so all ON PWM pins are at the begining.
static void
compact_pin2gpio(){
  int i, j = 0;
  uint8_t tmp_pin2gpio[] = { 0,0,0,0,0,0,0,0 };
  float tmp_channel_pwm[] = { 0,0,0,0,0,0,0,0 };

  for (i = 0; i < NUM_CHANNELS; i++) {
    if (pin2gpio[i] != 0){
      tmp_pin2gpio[j] = pin2gpio[i];
      tmp_channel_pwm[j] = channel_pwm[i];
      j++;
    }
  }
  for (i= 0 ;i < NUM_CHANNELS; i++){
    pin2gpio[i] = tmp_pin2gpio[i];
    channel_pwm[i] = tmp_channel_pwm[i];
  }
}

// Pins can be relesead after being setup as PWM pins by writing the release <pin>
// command to the /dev/pi-blaster file. We make sure to compact the pin2gpio array
// that contains currently working pwm pins.
static int
release_pin2gpio(int pin)
{
  int released = 0;

  int i;
  for (i = 0; i < NUM_CHANNELS; i++) {
    if (pin2gpio[i] == pin) {
      channel_pwm[i] = 0;
      pin2gpio[i] = 0;
      released = 1;
      break;
    }
  }

  compact_pin2gpio();
  return(released);

}

// Set each provided pin to one in pin2gpio
static void
set_pin(int pin, float width)
{
  if (is_known_pin(pin)){
    set_pin2gpio(pin, width);
  }else{
    printf("Not a known pin for pi-blaster");
  }
}

// Function make sure the pin we want to release is a valid pin, if it is
// then calls release_pin2gpio to delete it from currently ON pins.
static void
release_pin(int pin)
{
  if (is_known_pin(pin)){
    release_pin2gpio(pin);
  }else{
    printf("Not a known pin for pi-blaster");
  }
}

// Releases the PWM pin requested (if found and valid) and updates the calls
// update_pwm to apply the changes to the actual hardware pins.
static void
release_pwm(int pin){
  release_pin(pin);
  update_pwm();
}


// Set pin2gpio pins, channel width and update the pwm send to pins being used.
static void
set_pwm(int channel, float width)
{
  set_pin(channel, width);
  update_pwm();
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
static void
update_pwm()
{
	uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
	uint32_t phys_gpset0 = 0x7e200000 + 0x1c;
	struct ctl *ctl = (struct ctl *)virtbase;
	int i, j;
	uint32_t mask;
	
	/* First we turn on the channels that need to be on */
	/*   Take the first DMA Packet and set it's target to start pulse */
	if (invert_mode)
		ctl->cb[0].dst = phys_gpclr0;
	else
		ctl->cb[0].dst = phys_gpset0;

	/*   Now create a mask of all the pins that should be on */
	mask = 0;
	for (i = 0; i < NUM_CHANNELS; i++) {
    // Check the pin2gpio pin has been set to avoid locking all of them as PWM.
		if (channel_pwm[i] > 0 && pin2gpio[i]) {
			mask |= 1 << pin2gpio[i];
		}
	}
	/*   And give that to the DMA controller to write */
	ctl->sample[0] = mask;
	
	/* Now we go through all the samples and turn the pins off when needed */
	for (j = 1; j < NUM_SAMPLES; j++) {
		if (invert_mode)
			ctl->cb[j*2].dst = phys_gpset0;
		else
			ctl->cb[j*2].dst = phys_gpclr0;
		mask = 0;
		for (i = 0; i < NUM_CHANNELS; i++) {
      // Check the pin2gpio pin has been set to avoid locking all of them as PWM.
			if ((float)j/NUM_SAMPLES > channel_pwm[i] && pin2gpio[i])
				mask |= 1 << pin2gpio[i];
		}
		ctl->sample[j] = mask;
	}
}

static void
make_pagemap(void)
{
	int i, fd, memfd, pid;
	char pagemap_fn[64];

	page_map = malloc(NUM_PAGES * sizeof(*page_map));
	if (page_map == 0)
		fatal("pi-blaster: Failed to malloc page_map: %m\n");
	memfd = open("/dev/mem", O_RDWR);
	if (memfd < 0)
		fatal("pi-blaster: Failed to open /dev/mem: %m\n");
	pid = getpid();
	sprintf(pagemap_fn, "/proc/%d/pagemap", pid);
	fd = open(pagemap_fn, O_RDONLY);
	if (fd < 0)
		fatal("pi-blaster: Failed to open %s: %m\n", pagemap_fn);
	if (lseek(fd, (uint32_t)virtbase >> 9, SEEK_SET) !=
						(uint32_t)virtbase >> 9) {
		fatal("pi-blaster: Failed to seek on %s: %m\n", pagemap_fn);
	}
	for (i = 0; i < NUM_PAGES; i++) {
		uint64_t pfn;
		page_map[i].virtaddr = virtbase + i * PAGE_SIZE;
		// Following line forces page to be allocated
		page_map[i].virtaddr[0] = 0;
		if (read(fd, &pfn, sizeof(pfn)) != sizeof(pfn))
			fatal("pi-blaster: Failed to read %s: %m\n", pagemap_fn);
		if (((pfn >> 55) & 0x1bf) != 0x10c)
			fatal("pi-blaster: Page %d not present (pfn 0x%016llx)\n", i, pfn);
		page_map[i].physaddr = (uint32_t)pfn << PAGE_SHIFT | 0x40000000;
	}
	close(fd);
	close(memfd);
}

static void
setup_sighandlers(void)
{
	int i;

	// Catch all signals possible - it is vital we kill the DMA engine
	// on process exit!
	for (i = 0; i < 64; i++) {
		struct sigaction sa;

		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = terminate;
		sigaction(i, &sa, NULL);
	}
}

static void
init_ctrl_data(void)
{
	struct ctl *ctl = (struct ctl *)virtbase;
	dma_cb_t *cbp = ctl->cb;
	uint32_t phys_fifo_addr;
	uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
	uint32_t phys_gpset0 = 0x7e200000 + 0x1c;
	uint32_t mask;
	int i;

	if (delay_hw == DELAY_VIA_PWM)
		phys_fifo_addr = (PWM_BASE | 0x7e000000) + 0x18;
	else
		phys_fifo_addr = (PCM_BASE | 0x7e000000) + 0x04;

	memset(ctl->sample, 0, sizeof(ctl->sample));
	
	// Calculate a mask to turn off all the servos
	mask = 0;
	for (i = 0; i < NUM_CHANNELS; i++){
    // Check the pin2gpio pin has been set to avoid locking all of them as PWM.
    if (pin2gpio[i])
      mask |= 1 << pin2gpio[i];
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
		cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
		cbp->src = mem_virt_to_phys(ctl->sample + i);
		if (invert_mode)
			cbp->dst = phys_gpset0;
		else
			cbp->dst = phys_gpclr0;
		cbp->length = 4;
		cbp->stride = 0;
		cbp->next = mem_virt_to_phys(cbp + 1);
		cbp++;
		/* Second DMA command */
		if (delay_hw == DELAY_VIA_PWM)
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(5);
		else
			cbp->info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_D_DREQ | DMA_PER_MAP(2);
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

static void
init_hardware(void)
{
	struct ctl *ctl = (struct ctl *)virtbase;

	if (delay_hw == DELAY_VIA_PWM) {
		// Initialise PWM
		pwm_reg[PWM_CTL] = 0;
		udelay(10);
		clk_reg[PWMCLK_CNTL] = 0x5A000006;		// Source=PLLD (500MHz)
		udelay(100);
		clk_reg[PWMCLK_DIV] = 0x5A000000 | (50<<12);	// set pwm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PWMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pwm_reg[PWM_RNG1] = SAMPLE_US * 10;
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
		clk_reg[PCMCLK_DIV] = 0x5A000000 | (50<<12);	// Set pcm div to 50, giving 10MHz
		udelay(100);
		clk_reg[PCMCLK_CNTL] = 0x5A000016;		// Source=PLLD and enable
		udelay(100);
		pcm_reg[PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16; // 1 channel, 8 bits
		udelay(100);
		pcm_reg[PCM_MODE_A] = (SAMPLE_US * 10 - 1) << 10;
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<4 | 1<<3;		// Clear FIFOs
		udelay(100);
		pcm_reg[PCM_DREQ_A] = 64<<24 | 64<<8;		// DMA Req when one slot is free?
		udelay(100);
		pcm_reg[PCM_CS_A] |= 1<<9;			// Enable DMA
		udelay(100);
	}

	// Initialise the DMA
	dma_reg[DMA_CS] = DMA_RESET;
	udelay(10);
	dma_reg[DMA_CS] = DMA_INT | DMA_END;
	dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(ctl->cb);
	dma_reg[DMA_DEBUG] = 7; // clear debug error flags
	dma_reg[DMA_CS] = 0x10880001;	// go, mid priority, wait for outstanding writes

	if (delay_hw == DELAY_VIA_PCM) {
		pcm_reg[PCM_CS_A] |= 1<<2;			// Enable Tx
	}
}

static void
init_pin2gpio(void)
{
  int i;
  for (i = 0; i < NUM_CHANNELS; i++)
    pin2gpio[i] = 0;
}


static void
init_channel_pwm(void)
{
	int i;
	for (i = 0; i < NUM_CHANNELS; i++)
		channel_pwm[i] = 0;
}

static void
go_go_go(void)
{
	FILE *fp;

	if ((fp = fopen(DEVFILE, "r+")) == NULL)
		fatal("pi-blaster: Failed to open %s: %m\n", DEVFILE);

	char *lineptr = NULL, nl;
	size_t linelen;

	for (;;) {
		int n, servo;
		float value;
		
		if ((n = getline(&lineptr, &linelen, fp)) < 0)
			continue;
		//fprintf(stderr, "[%d]%s", n, lineptr);
		n = sscanf(lineptr, "%d=%f%c", &servo, &value, &nl);
		if (n !=3 || nl != '\n') {
			//fprintf(stderr, "Bad input: %s", lineptr);
      n = sscanf(lineptr, "release %d", &servo);
      if (n != 1 || nl != '\n') {
        fprintf(stderr, "Bad input: %s", lineptr);
      } else {
        // Release Pin from pin2gpio array if the release command is received.
        release_pwm(servo);
      }
		} else if (servo < 0){ // removed servo validation against CHANNEL_NUM no longer needed since now we used real GPIO names
			fprintf(stderr, "Invalid channel number %d\n", servo);
		} else if (value < 0 || value > 1) {
			fprintf(stderr, "Invalid value %f\n", value);
		} else {
			set_pwm(servo, value);
		}
	}
}

void
parseargs(int argc, char **argv)
{
	int index;
	int c;

	static struct option longopts[] =
	{
		{"help", no_argument, 0, 'h'},
		{"invert", no_argument, 0, 'i'},
		{"pcm", no_argument, 0, 'p'},
		{"version", no_argument, 0, 'v'},
		{0, 0, 0, 0}
	};

	while (1)
	{

		index = 0;
		c = getopt_long(argc, argv, "hipv", longopts, &index);

		if (c == -1)
			break;

		switch (c)
		{
		case 0:
			/* handle flag options (array's 3rd field non-0) */
			break;

		case 'h':
			fprintf(stderr, "%s version %s\n", argv[0], VERSION);
			fprintf(stderr, "Usage: %s [-hipv]\n"
				"-h (--help)    - this information\n"
				"-i (--invert)  - invert pin output (pulse LOW)\n"
				"-p (--pcm)     - use pcm for dmascheduling\n"
				"-v (--version) - version information\n"
				, argv[0]);
			exit(-1);

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

int
main(int argc, char **argv)
{
	int i;

	parseargs(argc, argv);

	printf("Using hardware:                 %5s\n", delay_hw == DELAY_VIA_PWM ? "PWM" : "PCM");
	printf("Number of channels:             %5d\n", NUM_CHANNELS);
	printf("PWM frequency:               %5d Hz\n", 1000000/CYCLE_TIME_US);
	printf("PWM steps:                      %5d\n", NUM_SAMPLES);
	printf("Maximum period (100  %%):      %5dus\n", CYCLE_TIME_US);
	printf("Minimum period (%1.3f%%):      %5dus\n", 100.0*SAMPLE_US / CYCLE_TIME_US, SAMPLE_US);

	setup_sighandlers();

	dma_reg = map_peripheral(DMA_BASE, DMA_LEN);
	pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);
	pcm_reg = map_peripheral(PCM_BASE, PCM_LEN);
	clk_reg = map_peripheral(CLK_BASE, CLK_LEN);
	gpio_reg = map_peripheral(GPIO_BASE, GPIO_LEN);

	virtbase = mmap(NULL, NUM_PAGES * PAGE_SIZE, PROT_READ|PROT_WRITE,
			MAP_SHARED|MAP_ANONYMOUS|MAP_NORESERVE|MAP_LOCKED,
			-1, 0);
	if (virtbase == MAP_FAILED)
		fatal("pi-blaster: Failed to mmap physical pages: %m\n");
	if ((unsigned long)virtbase & (PAGE_SIZE-1))
		fatal("pi-blaster: Virtual address is not page aligned\n");

	make_pagemap();

	for (i = 0; i < NUM_CHANNELS; i++) {
    // Check the pin2gpio pin has been set to avoid locking all of them as PWM.
    if (pin2gpio[i]){
      gpio_set(pin2gpio[i], invert_mode);
      gpio_set_mode(pin2gpio[i], GPIO_MODE_OUT);
    }
	}

	init_ctrl_data();
	init_hardware();
  // Init pin2gpio array with 0/false values to avoid locking all of them as PWM.
	init_pin2gpio();
	init_channel_pwm();

	unlink(DEVFILE);
	if (mkfifo(DEVFILE, 0666) < 0)
		fatal("pi-blaster: Failed to create %s: %m\n", DEVFILE);
	if (chmod(DEVFILE, 0666) < 0)
		fatal("pi-blaster: Failed to set permissions on %s: %m\n", DEVFILE);

	if (daemon(0,1) < 0)
		fatal("pi-blaster: Failed to daemonize process: %m\n");

	go_go_go();

	return 0;
}

