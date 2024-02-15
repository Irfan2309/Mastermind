/*
 * MasterMind implementation: template; see comments below on which parts need to be completed
 * CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
 * This repo: https://gitlab-student.macs.hw.ac.uk/f28hs-2021-22/f28hs-2021-22-staff/f28hs-2021-22-cwk2-sys

 * Compile: 
 gcc -c -o lcdBinary.o lcdBinary.c
 gcc -c -o master-mind.o master-mind.c
 gcc -o master-mind master-mind.o lcdBinary.o
 * Run:     
 sudo ./master-mind

 OR use the Makefile to build
 > make all
 and run
 > make run
 and test
 > make test

 *************************
 * The Low-level interface to LED, button, and LCD is based on:
 * wiringPi libraries by
 * Copyright (c) 2012-2013 Gordon Henderson.
 *************************
 * See:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 *************************
*/

/* ======================================================= */
/* SECTION: includes                                       */
/* ------------------------------------------------------- */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#include <unistd.h>
#include <string.h>
#include <time.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>

/* --------------------------------------------------------------------------- */
/* Config settings */
/* you can use CPP flags to e.g. print extra debugging messages */
/* or switch between different versions of the code e.g. digitalWrite() in Assembler */
#define DEBUG
#undef ASM_CODE

// =======================================================
// Tunables
// PINs (based on BCM numbering)
// For wiring see CW spec: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf
// GPIO pin for green LED
#define LED 13
// GPIO pin for red LED
#define LED2 5
// GPIO pin for button
#define BUTTON 19
// =======================================================
// delay for loop iterations (mainly), in ms
// in mili-seconds: 0.2s
#define DELAY   200
// in micro-seconds: 3s
#define TIMEOUT 3000000
// =======================================================
// APP constants   ---------------------------------
// number of colours and length of the sequence
#define COLS 3
#define SEQL 3
// =======================================================

// generic constants

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


// =======================================================
// Wiring (see inlined initialisation routine)

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

/* ======================================================= */
/* SECTION: constants and prototypes                       */
/* ------------------------------------------------------- */

// =======================================================
// char data for the CGRAM, i.e. defining new characters for the display

static unsigned char newChar [8] = 
{
  0b11111,
  0b10001,
  0b10001,
  0b10101,
  0b11111,
  0b10001,
  0b10001,
  0b11111,
} ;

/* Constants */

static const int colors = COLS;
static const int seqlen = SEQL;

static char* color_names[] = { "red", "green", "blue" };

static int* theSeq = NULL;

static int *seq1, *seq2, *cpy1, *cpy2;

/* --------------------------------------------------------------------------- */

// data structure holding data on the representation of the LCD
struct lcdDataStruct
{
  int bits, rows, cols ;
  int rsPin, strbPin ;
  int dataPins [8] ;
  int cx, cy ;
} ;

static int lcdControl ;

/* *************************** */
/* INLINED fcts from wiringPi/devLib/lcd.c: */
// HD44780U Commands (see Fig 11, p28 of the Hitachi HD44780U datasheet)

#define	LCD_CLEAR	0x01
#define	LCD_HOME	0x02
#define	LCD_ENTRY	0x04
#define	LCD_CTRL	0x08
#define	LCD_CDSHIFT	0x10
#define	LCD_FUNC	0x20
#define	LCD_CGRAM	0x40
#define	LCD_DGRAM	0x80

// Bits in the entry register

#define	LCD_ENTRY_SH		0x01
#define	LCD_ENTRY_ID		0x02

// Bits in the control register

#define	LCD_BLINK_CTRL		0x01
#define	LCD_CURSOR_CTRL		0x02
#define	LCD_DISPLAY_CTRL	0x04

// Bits in the function register

#define	LCD_FUNC_F	0x04
#define	LCD_FUNC_N	0x08
#define	LCD_FUNC_DL	0x10

#define	LCD_CDSHIFT_RL	0x04

// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices

#define	PI_GPIO_MASK	(0xFFFFFFC0)

static unsigned int gpiobase ;
static uint32_t *gpio ;

static int timed_out = 0;

/* ------------------------------------------------------- */
// misc prototypes

int failure (int fatal, const char *message, ...);
void waitForEnter (void);
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: hardware interface (LED, button, LCD display)  */
/* ------------------------------------------------------- */
/* low-level interface to the hardware */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Either put them in a separate file, lcdBinary.c, and use   */
/* inline Assembler there, or use a standalone Assembler file */
/* You can also directly implement them here (inline Asm).    */
/* ******************** */

/* These are just prototypes; you need to complete the code for each function */

/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
void digitalWrite (uint32_t *gpio, int pin, int value)
{
  if(value == 1)
  {
    asm
    (
      "\tMOV R1, %[gpio] \n"
      "\tMOV R2, #0b1 \n"
      "\tLSL R2, %[pin] \n"
      "\tSTR R2, [R1, #28] \n"
      :
      : [gpio] "r" (gpio)
      , [pin] "r" (pin)
      : "r1", "r2", "cc" 
    );
  }
  else
  {
    asm
    (
      "\tMOV R1, %[gpio]\n"
      "\tMOV R2, #0b1\n"
      "\tLSL R2, %[pin]\n"
      "\tSTR R2, [R1, #40]\n"
      :
      : [gpio] "r" (gpio)
      , [pin] "r" (pin)
      : "r1", "r2", "cc" //registers
    );
  }
}


/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode)
{
  int res;
  int fSel = pin/10; 
  int shift = (pin%10)*3;
  asm
  (/* Set LED to ouput" */
    "\tLDR R1, %[gpio]\n"
    "\tADD R0, R1, %[fSel]\n"  
    "\tLDR R1, [R0, #0]\n"    
    "\tMOV R2, #0b111\n"
    "\tLSL R2, %[shift]\n"
    "\tBIC R1, R1, R2\n"
    "\tMOV R2, %[mode]\n"
    "\tLSL R2, %[shift]\n"
    "\tORR R1, R2\n"
    "\tSTR R1, [R0, #0]\n"
    "\tMOV %[result], R1\n"
    : [result] "=r" (res)
    : [act] "r" (pin)
    , [gpio] "m" (gpio)
    , [fSel] "r" (fSel*4)
    , [shift] "r" (shift)
    , [mode] "r" (mode)
    : "r0", "r1", "r2", "cc"
  );
}
/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value)
{
  digitalWrite(gpio, led, value);
}

/* read a @value@ (LOW or HIGH) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int button)
{
  int res;
  asm
  (/*Set Button to input */
    "\tLDR R1, [%[gpio], #52]\n"
    "\tMOV R2, #0b1\n"
    "\tLSL R2, %[button]\n"
    "\tAND R2, R1\n"
    "\tMOV %[r], R2\n"
    : [r] "=r" (res) 
    : [gpio] "r" (gpio)
    , [button] "r" (button)
    : "r1", "r2", "cc" 
  );
  return res;  
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
void waitForButton (uint32_t *gpio, int button);

/* ======================================================= */
/* SECTION: game logic                                     */
/* ------------------------------------------------------- */
/* AUX fcts of the game logic */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* initialise the secret sequence; by default it should be a random sequence */
void initSeq() {
  int length = 3;
  srand(time(NULL));
  for (int i = 0; i < length; i++)
  {
    theSeq[i] = (rand() % length) + 1;
  }
}

/* display the sequence on the terminal window, using the format from the sample run in the spec */
void showSeq(int *seq) {
  int length = 3;
  printf("Secret: ");
  for (int i = 0; i < length; i++)
  {
    printf("%d  ", seq[i]);
  }
  printf("\n");
}

#define NAN1 8
#define NAN2 9

/* counts how many entries in seq2 match entries in seq1 */
/* returns exact and approximate matches, either both encoded in one value, */
/* or as a pointer to a pair of values */
int* countMatches(int *seq1, int *seq2) {
  int length = 3;
  static int results[2];
  int cpy1[3];
  int cpy2[3];
  int exactmatch = 0;
  int approxmatch = 0;

  //copying the sequence to another array for modification
  for (int i = 0; i < length; i++)
  {
    cpy1[i] = seq1[i];
  }

   //copying the sequence to another array for modification
  for (int i = 0; i < length; i++)
  {
    cpy2[i] = seq2[i];
  }

  //finding exact matches
  for (int i = 0; i < length; i++)
  {
    if (cpy1[i] == cpy2[i])
    {
      exactmatch ++;
      cpy1[i] = 0;
      cpy2[i] = 7;
    } 
  } 

  //finding approx matches
  for (int i = 0; i < length; i++)
  {
    for (int j = 0; j < length; j++)
    {
      if (cpy1[i] == cpy2[j])
      {
        approxmatch ++;
        cpy1[i] = 5;
        cpy2[j] = 6;
      }
    }
  }
  
  results[0] = exactmatch;
  results[1] = approxmatch;
  return results;
}

/* show the results from calling countMatches on seq1 and seq1 */
void showMatches(int* code, /* only for debugging */ int *seq1, int *seq2, /* optional, to control layout */ int lcd_format) {
  printf("Exact: %d\n", code[0]);
  printf("Approximate: %d\n", code[1]);
}

/* parse an integer value as a list of digits, and put them into @seq@ */
/* needed for processing command-line with options -s or -u            */
void readSeq(int *seq, int val) {
  for (int i = 2; i >= 0; i--)
  {
    seq[i] = val % 10;
    val /= 10;
  }
}

/* read a guess sequence fron stdin and store the values in arr */
/* only needed for testing the game logic, without button input */
int readNum(int max) {
  /* *  COMPLETE the code here  *  */
}

/* ======================================================= */
/* SECTION: TIMER code                                     */
/* ------------------------------------------------------- */
/* TIMER code */

/* timestamps needed to implement a time-out mechanism */
static uint64_t startT, stopT;

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* you may need this function in timer_handler() below  */
/* use the libc fct gettimeofday() to implement it      */
uint64_t timeInMicroseconds(){
  struct timeval tv, tNow, tLong, tEnd;
  uint64_t now;
  gettimeofday(&tv, NULL);
  now = (uint64_t)tv.tv_sec * (uint64_t)100000 + (uint64_t)tv.tv_usec;
  return (uint64_t)now;
}

/* this should be the callback, triggered via an interval timer, */
/* that is set-up through a call to sigaction() in the main fct. */
void timer_handler (int signum) {
  static int counter = 0;
  stopT = timeInMicroseconds();
  counter ++;
  fprintf(stderr, "timer expired %d times; (measured interval %f sec)\n", counter, (stopT - startT)/1000000.0);
  startT = timeInMicroseconds();
}


/* initialise time-stamps, setup an interval timer, and install the timer_handler callback */
void initITimer(uint64_t timeout){
  struct sigaction sa;
  struct itimerval timer;

  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = &timer_handler;
  sigaction(SIGVTALRM, &sa, NULL);

  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = timeout;
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = timeout;

  setitimer(ITIMER_VIRTUAL, &timer, NULL);
}

/* ======================================================= */
/* SECTION: Aux function                                   */
/* ------------------------------------------------------- */
/* misc aux functions */

int failure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal) //  && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
  vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}

/*
 * waitForEnter:
 ***************************
 */

void waitForEnter (void)
{
  printf ("Press ENTER to continue: ") ;
  (void)fgetc (stdin) ;
}

/*
 * delay:
 *	Wait for some number of milliseconds
 ***************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}

/* From wiringPi code; comment by Gordon Henderson
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 ***************************
 */

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
#if 0
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
#endif
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}

/* ======================================================= */
/* SECTION: aux functions for game logic                   */
/* ------------------------------------------------------- */

/* ******************** */
/* COMPLETE the code for all of the functions in this SECTION */
/* Implement these as C functions in this file                */
/* ******************** */

/* --------------------------------------------------------------------------- */
/* interface on top of the low-level pin I/O code */

/* blink the led on pin @led@, @c@ times */
void blinkN(uint32_t *gpio, int led, int c) { 
  int a = c;
  for (int i = 0; i < a*2; i++)
  {
    int value1 = ((i % 2) == 0) ? LOW : HIGH;
    int off = (value1 == HIGH) ? 10 : 7;
    *(gpio + off) = 1 << (led & 31);
    delay(500);
  }
  
}

/* ======================================================= */
/* SECTION: main fct                                       */
/* ------------------------------------------------------- */

int main (int argc, char *argv[])
{ // this is just a suggestion of some variable that you may want to use

  int found = 0, attempts = 0, i, j, code;
  int c, d, buttonPressed, rel, foo;
  int *attSeq;

  int pinLED = LED, pin2LED2 = LED2, pinButton = BUTTON;
  int fSel, shift, pin,  clrOff, setOff, off, res;
  int fd ;
  int current = HIGH;

  int  exact, contained;
  char str1[32];
  char str2[32];
  
  struct timeval t1, t2 ;
  int t ;
  int timing = 0;

  char buf [32] ;

  int count = 0;
  int value;
  int guess;
  // variables for command-line processing
  char str_in[20], str[20] = "some text";
  int verbose = 0, debug = 0, help = 0, opt_m = 0, opt_n = 0, opt_s = 0, unit_test = 0;
  int *result1;

  // -------------------------------------------------------
  // process command-line arguments

  // see: man 3 getopt for docu and an example of command line parsing
  { // see the CW spec for the intended meaning of these options
   int opt;
    while ((opt = getopt(argc, argv, "hvdus:")) != -1) {
      switch (opt) {
      case 'v':
	verbose = 1;
	break;
      case 'h':
	help = 1;
	break;
      case 'd':
	debug = 1;
	break;
      case 'u':
	unit_test = 1;
	break;
      case 's':
	opt_s = atoi(optarg); 
	break;
      default: /* '?' */
	fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
	exit(EXIT_FAILURE);
      }
    }
  }

  if (help) {
    fprintf(stderr, "MasterMind program, running on a Raspberry Pi, with connected LED, button and LCD display\n"); 
    fprintf(stderr, "Use the button for input of numbers. The LCD display will show the matches with the secret sequence.\n"); 
    fprintf(stderr, "For full specification of the program see: https://www.macs.hw.ac.uk/~hwloidl/Courses/F28HS/F28HS_CW2_2022.pdf\n"); 
    fprintf(stderr, "Usage: %s [-h] [-v] [-d] [-u <seq1> <seq2>] [-s <secret seq>]  \n", argv[0]);
    exit(EXIT_SUCCESS);
  }
  
  if (unit_test && optind >= argc-1) {
    fprintf(stderr, "Expected 2 arguments after option -u\n");
    exit(EXIT_FAILURE);
  }

  if (verbose && unit_test) {
    printf("1st argument = %s\n", argv[optind]);
    printf("2nd argument = %s\n", argv[optind+1]);
  }

  if (verbose) {
    fprintf(stdout, "Settings for running the program\n");
    fprintf(stdout, "Verbose is %s\n", (verbose ? "ON" : "OFF"));
    fprintf(stdout, "Debug is %s\n", (debug ? "ON" : "OFF"));
    fprintf(stdout, "Unittest is %s\n", (unit_test ? "ON" : "OFF"));
    if (opt_s)  fprintf(stdout, "Secret sequence set to %d\n", opt_s);
  }

  seq1 = (int*)malloc(seqlen*sizeof(int));
  seq2 = (int*)malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // check for -u option, and if so run a unit test on the matching function
  if (unit_test && argc > optind+1) { // more arguments to process; only needed with -u 
    strcpy(str_in, argv[optind]);
    opt_m = atoi(str_in);
    strcpy(str_in, argv[optind+1]);
    opt_n = atoi(str_in);
    // CALL a test-matches function; see testm.c for an example implementation
    readSeq(seq1, opt_m); // turn the integer number into a sequence of numbers
    readSeq(seq2, opt_n); // turn the integer number into a sequence of numbers
    if (verbose)
      fprintf(stdout, "Testing matches function with sequences %d and %d\n", opt_m, opt_n);
    result1 = countMatches(seq1, seq2);
    showMatches(result1, seq1, seq2, 1);
    exit(EXIT_SUCCESS);
  } else {
    /* nothing to do here; just continue with the rest of the main fct */
  }

  if (opt_s) { // if -s option is given, use the sequence as secret sequence
    if (theSeq==NULL)
      theSeq = (int*)malloc(seqlen*sizeof(int));
    readSeq(theSeq, opt_s);
    if (verbose) {
      fprintf(stderr, "Running program with secret sequence:\n");
      showSeq(theSeq);
    }
  }
  // -----------------------------------------------------------------------------
  if (geteuid () != 0)
    fprintf (stderr, "setup: Must be root. (Did you forget sudo?)\n") ;

  // init of guess sequence, and copies (for use in countMatches)
  attSeq = (int*) malloc(seqlen*sizeof(int));
  cpy1 = (int*)malloc(seqlen*sizeof(int));
  cpy2 = (int*)malloc(seqlen*sizeof(int));

  // -----------------------------------------------------------------------------
  // constants for RPi2
  gpiobase = 0x3F200000 ;

  // -----------------------------------------------------------------------------
  // memory mapping 
  // Open the master /dev/memory device

  if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
    return failure (FALSE, "setup: Unable to open /dev/mem: %s\n", strerror (errno)) ;

  // GPIO:
  gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpiobase) ;
  if ((int32_t)gpio == -1)
    return failure (FALSE, "setup: mmap (GPIO) failed: %s\n", strerror (errno)) ;

  // -------------------------------------------------------
  // Configuration of LED and BUTTON

  //BUTTON
  //Set GPIO PIN 19 to input
  pinMode(gpio, pinButton, INPUT); 

  //Red LED
  //Set GPIO PIN 5 to output
  pinMode(gpio, pin2LED2, OUTPUT); 

  //Green LED
  //Set GPIO PIN 13 to output
  pinMode(gpio, pinLED, OUTPUT); 
  
  // -----------------------------------------------------------------------------
  // Start of game
  printf("\tWelcome to Mastermind! Coded by Irfanuddin Syed\n");
  /* initialise the secret sequence */
  if (!opt_s)
    initSeq();
  if (debug)
    showSeq(theSeq);

  // optionally one of these 2 calls:
  // waitForEnter () ; 
  // waitForButton (gpio, pinButton) ;
  waitForEnter();

  // -----------------------------------------------------------------------------
  // +++++ main loop
  while (!found) {
    attempts++;
    printf("----------------------Move %d----------------------\n", attempts);
    /* ******************* */
    /* *  COMPLETE the code here  *                        */
    /* this needs to implement the main loop of the game:      */
    /* check for button presses and count them                 */
    /* store the input numbers in the sequence @attSeq@        */
    /* compute the match with the secret sequence, and         */
    /* show the result                                         */
    /* see CW spec for details                                 */
    /* ******************* */

    for(guess = 0; guess < 3; guess++)
    {
      printf("\nPress Button to guess\n"); 
      while(count < 3)
      {
        res = readButton(gpio,pinButton);
        if((BUTTON & 0xFFFFFFC0) == 0) 
        {
          value = LOW;			
          if ((res != 0) && (current == HIGH)) 
          {
            value = LOW;
            current = LOW;
            count ++;				
            timing = 0;
            printf("\tButton Press Detected\n");
          }
          else if ((res== 0) && (current == LOW)) 
          {
            value = HIGH;
            current = HIGH;
          }
        }
        else 
        {
          fprintf(stderr, "only supporting on-board pins\n");
        }
        timing ++;
	      delay(200);
        if(timing == 50) 
        {
          break;
        }
      }

      printf("Input Taken!\n");
      printf("Guess for color %d: %d\n", guess + 1, count);

      attSeq[guess] = count;
      blinkN(gpio, pin2LED2, 1);
      blinkN(gpio, pinLED, count); 
      count = 0;
      timing = 0;
      }

      printf("Answ%d: %d  %d  %d\n",attempts, attSeq[0], attSeq[1], attSeq[2]);

      
      result1 = countMatches(theSeq, attSeq);
      exact = result1[0];
      contained = result1[1];
      showMatches(result1, cpy1, cpy2,1);
      
      blinkN(gpio, pin2LED2, 1);
      blinkN(gpio, pinLED, exact); 
      blinkN(gpio, pin2LED2, 1); 
      blinkN(gpio, pinLED, contained); 
      if (exact == 3)
      {
        found = 1;
        break;
      }
      blinkN(gpio, pin2LED2, 3);//RED BLINK 3 TIMES (END OF ROUND)
    }

  if (found) 
  {
    digitalWrite(gpio, pin2LED2, HIGH);
    blinkN(gpio, pinLED, exact);
    printf("Game finished in %d moves\n", attempts);
  } 
  else 
  {
    fprintf(stdout, "Sequence not found\n");
  }
  return 0;
}