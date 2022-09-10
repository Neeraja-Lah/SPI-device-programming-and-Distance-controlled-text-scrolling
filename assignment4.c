/*
*   CSE438 Embedded Systems Programming Spring 2022
*   Assignment 4: SPI Device Programming and Distance-controlled Text Scrolling
*/

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/spi/spidev.h>

#include "hcsr_ioctl.h"

#define HCSR04_DEVICE                       "/dev/hcsr04_dev"

// HCSR04 Pin Definitions
#define TRIG_PIN                            22
#define ECHO_PIN                            23

// Default SPI Parameter Definitions
#define SPI_DEVICE                          "/dev/spidev0.0"
#define SPI_MODE                            SPI_MODE_0
#define SPI_SPEED                           10000000
#define SPI_BITS                            8

// MAX7219 Command Register Definitions
#define DECODE_MODE_REG                     0x09
#define INTENSITY_REG                       0x0A
#define SCAN_LIMIT_REG                      0x0B
#define SHUTDOWN_REG                        0x0C
#define DISPLAY_TEST_REG                    0x0F

// Time Conversion Definitions
#define US_TO_NS                            1000
#define MS_TO_NS                            1000000
#define S_TO_NS                             1000000000

#define BLINK_TIME                          500                 // Blink time in milliseconds
#define BLINK_TIME_NS                       (BLINK_TIME * MS_TO_NS)

static uint32_t mode = SPI_MODE;
static uint32_t speed = SPI_SPEED;
static uint32_t bits = SPI_BITS;
static uint16_t delay = 0;

unsigned long distance;

// Buffer to store 16-bit SPI Data
uint8_t spi_tx_buffer[2];
uint8_t spi_rx_buffer[2];

// Flag to monitor Ctlr+C event
uint8_t terminate_flag = 0;

// Flag for blink and display status
uint8_t display_on_flag = 0;
uint8_t blink_flag = 0;

// Thread ID for Keyboard Thread
pthread_t keyboard_thread_id;

// Character message to display "ASU#1" on 8x8 Matrix LED
uint8_t message[] = {0x00, 0x78, 0x16, 0x11, 0x11, 0x16, 0x78, 0x00,                // A
                     0x00, 0x26, 0x49, 0x49, 0x49, 0x49, 0x32, 0x00,                // S
                     0x00, 0x3F, 0x40, 0x40, 0x40, 0x40, 0x3F, 0x00,                // U
                     0x00, 0x14, 0x7F, 0x14, 0x14, 0x7F, 0x14, 0x00,                // #
                     0x00, 0x44, 0x42, 0x7F, 0x40, 0x40, 0x00, 0x00};               // 1

uint8_t count = 0, start_point = 0;

// Function to tramsfer "len" amount of data to SPI Bus
static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
    
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
        printf("Cannot write message\n");
    }
}

// Function to write command and data to SPI Device
void display_write(int fd, uint8_t command, uint8_t data)
{
    spi_tx_buffer[0] = command;
    spi_tx_buffer[1] = data;
    transfer(fd, spi_tx_buffer, spi_rx_buffer, sizeof(spi_tx_buffer));
}

// Function to clear the entire display
void display_clear(int fd)
{
    for(int i=0; i<8; i++) {
        display_write(fd, (uint8_t)(i + 1), 0x00);
    }
}

// Signal handler for Ctlr+C event
void term_signal_handler(int sig_no)
{
    terminate_flag = 1;
}

// Timer expiry function for blink timer
void blink_timer_expiry_func(union sigval timer_data)
{
    int fd = *(int *)timer_data.sival_ptr;

    // Check for display flag and turn on/off based on it
    display_on_flag = !display_on_flag;

    if(display_on_flag) {
        display_write(fd, SHUTDOWN_REG, 0x00);
    }
    else {
        display_write(fd, SHUTDOWN_REG, 0x01);
    }
}

// Handler function for keyboard thread
static void *keyboard_thread_func(void *arg)
{
    int fd = *(int *)arg;

    char value = 0;

    // Create a timer to use for blink event
    timer_t blink_timer_id;
    struct sigevent blink_event;

    struct itimerspec time_expire;

    memset(&blink_event, 0, sizeof(blink_event));

    blink_event.sigev_notify = SIGEV_THREAD;
    blink_event.sigev_value.sival_ptr = (void *)&fd;
    blink_event.sigev_notify_function = blink_timer_expiry_func;

    timer_create(CLOCK_MONOTONIC, &blink_event, &blink_timer_id);

    while(!terminate_flag)
    {
        // Check for key inputs to change intensity or toggle the blinking mode
        value = getc(stdin);
        if(value == '0') {
            display_write(fd, INTENSITY_REG, 0x00);
        }
        else if(value > '0' && value <= '8') {
            display_write(fd, INTENSITY_REG, ((value * 2) - 1));
        }
        else if((value == 'b') || (value == 'B')) {
            if(!blink_flag) {
                display_write(fd, SHUTDOWN_REG, 0x00);
                display_on_flag = 1;
                blink_flag = 1;
                time_expire.it_value.tv_sec = 0;
                time_expire.it_value.tv_nsec = BLINK_TIME_NS;
                time_expire.it_interval.tv_sec = 0;
                time_expire.it_interval.tv_nsec = BLINK_TIME_NS;
                timer_settime(blink_timer_id, 0, &time_expire, NULL);
            }
            else {
                display_write(fd, SHUTDOWN_REG, 0x01);
                blink_flag = 0;
                time_expire.it_value.tv_sec = 0;
                time_expire.it_value.tv_nsec = 0;
                time_expire.it_interval.tv_sec = 0;
                time_expire.it_interval.tv_nsec = 0;
                timer_settime(blink_timer_id, 0, &time_expire, NULL);
            }
        }
    }

    timer_delete(blink_timer_id);

    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    int fd_spi, fd_hcsr04, ret;

    // Set the terminal to accept key inputs without pressing enter key
    static struct termios oldt, newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= (~ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    signal(SIGINT, term_signal_handler);

    // Open the HCSR04 Device
    fd_hcsr04 = open(HCSR04_DEVICE, O_RDWR);
    if(!fd_hcsr04) {
        printf("Cannot open %s device\n", HCSR04_DEVICE);
        return -1;
    }

    // Set the Trigger Pin
    ret = ioctl(fd_hcsr04, SET_TRIGGER, TRIG_PIN);
    if(ret < 0) {
        printf("Cannot Set Trigger\n");
    }
    // Set the Echo Pin
    ret = ioctl(fd_hcsr04, SET_ECHO, ECHO_PIN);
    if(ret < 0) {
        printf("Cannot Set Echo\n");
    }

    // Open the SPI Device
    fd_spi = open(SPI_DEVICE, O_RDWR);
    if(fd_spi < 0) {
        printf("Cannot open SPI Device\n");
        return -1;
    }
    
    // Set the SPI Mode
    ret = ioctl(fd_spi, SPI_IOC_WR_MODE32, &mode);
    if(ret == -1) {
        printf("Cannot write SPI mode\n");
        return -1;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_MODE32, &mode);
    if(ret == -1) {
        printf("Cannot read SPI mode\n");
        return -1;
    }

    // Set the SPI Speed
    ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if(ret == -1) {
        printf("Cannot write SPI speed\n");
        return -1;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    if(ret == -1) {
        printf("Cannot read SPI speed\n");
        return -1;
    }

    // Set the SPI bits per word
    ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if(ret == -1) {
        printf("Cannot write SPI bits per word\n");
        return -1;
    }

    ret = ioctl(fd_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if(ret == -1) {
        printf("Cannot read SPI bits per word\n");
        return -1;
    }

    printf("\n**********SPI Specifications**********\n");
	printf("SPI Mode: 0x%d\n", mode);
	printf("SPI Bits Per Word: %d\n", bits);
	printf("SPI Max Speed: %d Hz (%d MHz)\n", speed, speed/1000000);

    printf("\n**********Input Parameters**********\n");
    printf("Press keys between '0-8' to set intensity (0 is min and 8 is max intensity)\n");
    printf("Press 'b' to toggle the blinking mode\n");
    printf("Press Ctlr+C to terminate the program\n\n");

    // Configure the display with SPI commands
    display_write(fd_spi, DECODE_MODE_REG, 0x00);

    display_write(fd_spi, INTENSITY_REG, 0x07);

    display_write(fd_spi, SCAN_LIMIT_REG, 0x07);

    display_write(fd_spi, SHUTDOWN_REG, 0x01);

    display_write(fd_spi, DISPLAY_TEST_REG, 0x00);

    // Clear the display
    display_clear(fd_spi);
    
    // Create a thread to handle keyboard events
    pthread_create(&keyboard_thread_id, NULL, keyboard_thread_func, &fd_spi);

    while (!terminate_flag)
    {
        // Get the value from HCSR04 and scroll the display accori=dingly
        usleep(distance);
        ret = write(fd_hcsr04, NULL, 0);
        if(ret) {
            printf("Cannot write\n");
        }

        usleep(10);

        ret = read(fd_hcsr04, &distance, sizeof(distance));
        if(ret < 0) {
            printf("Cannot Read\n");
        }

        // printf("Distance = %lu\n", distance);

        count = start_point;
    
        for(int i=0; i<8; i++) {
            display_write(fd_spi, (uint8_t)(i+1), message[(count % sizeof(message))]);
            ++count;
        }

        if(++start_point == sizeof(message))
            start_point = 0;
        // printf("Start: %d\n", start_point);
    }

    // Clear the display
    display_clear(fd_spi);

    // Close the SPI Device
    ret = close(fd_spi);
    if(ret == -1) {
        printf("Cannot close SPI device\n");
    }

    // Close the HCSR04 Device
    ret = close(fd_hcsr04);
    if(ret) {
        printf("Cannot close %s device\n", HCSR04_DEVICE);
    }

    printf("\n\nAll Exit! Program terminated gracefully\n");

    // Revert back to original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    // // Wait for keyboard thread to terminate
    // pthread_join(keyboard_thread_id, NULL);

    return 0;
}
