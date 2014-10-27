/*------------------------------------------------------------------------------\
| This code is for and HCS12 that is connected to: an ArbotiX on a Hexapod,
| Gun Trigger Servo, Ultrasonic Sensor, and Thermopile.  
| Actions: Turn slowly and read thermopile. When a heat source above ambient is
| detected, it will shoot the gun and walk towards the heat source. Using the
| ultrasonic sensor, it will approach the heat source until it is close. The gun
| will shoot again and the robot will turn around 180 degrees and search for
| another heat source.                                                 
\------------------------------------------------------------------------------*/

  #include <hidef.h>                  //Common defines and macros
  #include "derivative.h"             //Derivative-specific definitions

/* Serial Setup Start */
  #define SysClk 8000000              //HCS12 Crystal frequency
  #define sci_div=SysClk/(16*38400)   //Divisor rate

  unsigned char right_V;    //variables used for sending commands to hexapod
  unsigned char right_H;    //part of commander code
  unsigned char left_V;
  unsigned char left_H;
  unsigned char buttons;
  unsigned char zero;
  unsigned char checksum;

  void SCI0_Init(unsigned long Baud)
  {
  // initialize buffer
    //SCI0RXBuf.In=0;
    //SCI0RXBuf.Out=0;
  
    SCI0BD = (SysClk / 16) / Baud; /* calculate the SCI Baud register value */
                                   /* using the system clock rate */
    SCI0CR2 = SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, no interrupts */
    //SCI0CR2 = SCI0CR2_RIE_MASK + SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, only RX interrupt */
    //SCI0CR2 = SCI0CR2_TE_MASK + SCI0CR2_RE_MASK ; /* enable both the transmitter & receiver, no interrupt */
  }

  int putcharSCI0(char c)
  {
  	while (!(SCI0SR1&SCI0SR1_TDRE_MASK)) /* check TXbuffer is ready or not,TDRE */
  	; //OSTimeDly(1); /* wait here */
  	SCI0DRL=c; /* put the char in the TX */
  	return(c); /* return the character we sent */
  } /* end putchar */

  int getcharSCI0(void)
  {
    while (!(SCI0SR1&SCI0SR1_RDRF_MASK)) /* chec RX flag, has data or not...,RDRF */
    //OSTimeDly(1); /* no data wait here */
    return(SCI0DRL);
  } /* end getchar */

  void turnrobotcounter(void){   //Sends command for turning slowly counterclockwise
    right_V = (1023-511-512)/5 + 128;  //emulates analong sticks
    right_H = (444-512)/5 + 128;
    left_V = (1023-512-512)/5 + 128;
    left_H = (512-512)/5 + 128;
    checksum = 255 - ((right_V+right_H+left_V+left_H+buttons)%256);
    zero = 0x00;                 //part of commander code and Arbotix
                        
    putcharSCI0(0xFF);
    putcharSCI0(right_V);
    putcharSCI0(right_H);
    putcharSCI0(left_V);
    putcharSCI0(left_H);
    putcharSCI0(buttons);   
    putcharSCI0(zero);
    putcharSCI0(checksum);
    }
    
  void turnrobotcounter2(void){   //Sends command for turning slowly counterclockwise
    right_V = (1023-511-512)/5 + 128;  //emulates analong sticks
    right_H = (250-512)/5 + 128;
    left_V = (1023-512-512)/5 + 128;
    left_H = (512-512)/5 + 128;
    checksum = 255 - ((right_V+right_H+left_V+left_H+buttons)%256);
    zero = 0x00;                 //part of commander code and Arbotix
                        
    putcharSCI0(0xFF);
    putcharSCI0(right_V);
    putcharSCI0(right_H);
    putcharSCI0(left_V);
    putcharSCI0(left_H);
    putcharSCI0(buttons);   
    putcharSCI0(zero);
    putcharSCI0(checksum);
    }   
     
  void turnrobotclockwise(void){  //Sends command for turning slowly clockwise
    right_V = (1023-511-512)/5 + 128;
    right_H = (700-512)/5 + 128;
    left_V = (1023-512-512)/5 + 128;
    left_H = (512-512)/5 + 128;
    checksum = 255 - ((right_V+right_H+left_V+left_H+buttons)%256);
    zero = 0x00;
                        
    putcharSCI0(0xFF);
    putcharSCI0(right_V);
    putcharSCI0(right_H);
    putcharSCI0(left_V);
    putcharSCI0(left_H);
    putcharSCI0(buttons);   
    putcharSCI0(zero);
    putcharSCI0(checksum);
    }

  void moverobotforward(void){    //Sends command for moving forward
    right_V = (1023-511-512)/5 + 128;
    right_H = (512-512)/5 + 128;
    left_V = (1023-125-512)/5 + 128;
    left_H = (512-512)/5 + 128;
    checksum = 255 - ((right_V+right_H+left_V+left_H+buttons)%256);
    zero = 0x00;
 
    putcharSCI0(0xFF);
    putcharSCI0(right_V);
    putcharSCI0(right_H);
    putcharSCI0(left_V);
    putcharSCI0(left_H);
    putcharSCI0(buttons);   
    putcharSCI0(zero);
    putcharSCI0(checksum); 
    }
    
  void stopmoving(void){    //Sends command to stop moving
    right_V = 128;
    right_H = 128;
    left_V = 128;
    left_H = 128;
    checksum = 255 - ((right_V+right_H+left_V+left_H+buttons)%256);
    zero = 0x00;
 
    putcharSCI0(0xFF);
    putcharSCI0(right_V);
    putcharSCI0(right_H);
    putcharSCI0(left_V);
    putcharSCI0(left_H);
    putcharSCI0(buttons);   
    putcharSCI0(zero);
    putcharSCI0(checksum); 
    }

  void delayms(unsigned int count){   //Delays for 1 millisecond
    unsigned i,j;                     //int parameter defines #of msec
    for(i=0;i< count;i++) {
      for(j=0;j< 8000/6;j++) ;
      }
    }
/* Serial Setup End */

/* Thermopile Setup Start */
  #define Clck=0x47           /* recommended clock div -- Freescale Apnote 2318*/
  #define Thermal_ID=0xD0     /* Thermal pile default Slave address */
  void openI2C(char ibc, char i2c_ID);
  void SendSlaveID(char cx);
  char I2Ctemperature_read(char ID, char reg_addr, unsigned char temp[]);
  
  void openI2C(char ibc, char i2c_ID){
  	IBCR_IBEN = 1;	          /* enable I2C module */
  	IBFD = ibc;               /* configure I2C clock rate */
  	IBAD = i2c_ID;
  	IBCR_IBIE = 0;	          /* disable I2C interrupt */
  	IBCR_IBSWAI = 1;          /* disable I2C in wait mode */
    }

  void SendSlaveID(char cx){
    while(IBSR_IBB);          /* wait until I2C bus is idle */
    IBCR |= IBCR_TX_RX_MASK + IBCR_MS_SL_MASK; /* generate a start condition */
    IBDR = cx;                /* send out slace ID with R/W set to 0 */
    while(!(IBSR_IBIF));      /* wait for completion of transmission */
    IBSR_IBIF = 1;            /* clear IBIF flag by writing one*/
    }

    char I2Ctemperature_read(char ID, char reg_addr, unsigned char array[]){
    char dummy;               //Reads temp from thermopile
    int i;
     
    SendSlaveID(ID);
    if (IBSR_RXAK==1)         /* error no acknowledge */
       return -1;
    
    IBDR = reg_addr;          /* send out address of the location to be written */
    while(!(IBSR_IBIF));      /* wait for byte to be sent */
    IBSR_IBIF = 1;            /* clear the IBIF flag by writing 1 to it*/
    if (IBSR_RXAK==1)         /* error if EEPROM does not acknowledge */
       return -2;
    
    IBCR_RSTA=1;              /* generate a restart */
    IBDR = 0xD1;
    while(!(IBSR_IBIF));
    IBSR_IBIF = 1;            /* clear the IBIF flag */
    if (IBSR_RXAK==1)         /* error if EEPROM does not acknowledge */
       return -3;
    
    IBCR &= ~(IBCR_TX_RX_MASK + IBCR_TXAK_MASK); /* prepare to receive and ack */
    dummy=IBDR;               /* a dummy read to trigger 9 clock pulses */
    for (i=0;i<7; i++){
      while(!(IBSR_IBIF));
      IBSR_IBIF = 1;          /* clear the IBIF flag */
      array[i]=IBDR;
      }
      
    while(!(IBSR_IBIF));
    IBSR_IBIF = 1;            /* clear the IBIF flag */
    IBCR_TXAK=1;     
    array[7]=IBDR;
    while(!(IBSR_IBIF));
    IBSR_IBIF = 1;            /* clear the IBIF flag */
    IBCR_MS_SL = 0;           /* generate stop condition */
    array[8]=IBDR;
    return 0;                 /* normal write code */
    }
/* Thermopile Setup End */

/* Ultrasonic Setup Start */
  unsigned int edge1,diff,diff2;

  void delay_20us(){	  //Delays 20 micro seconds
    int i;              //perfect time for ultrasonic
  	for(i=1;i<=30;i++);
    }

  void captureultrasonic(void){
  	PORTA = 0;
  	PORTA = 1;
  	delay_20us();
  	PORTA = 0;
  	TFLG1 =0x02;          /*cleared C1F flag*/
  	while(!(TFLG1_C1F));  /*wait for the first edge*/
  	edge1 = TC1;
  	TFLG1 =0x02;          /*cleared C1F flag*/
  	while(!(TFLG1_C1F));  /*wait for the second edge*/
  	diff = TC1 - edge1;
    //dis = (diff/250)*2.54;
  	delayms(700);
  	PORTA = 0;
  	PORTA = 1;
  	delay_20us();
  	PORTA = 0;
  	TFLG1 =0x02;          /*cleared C1F flag*/
  	while(!(TFLG1_C1F));  /*wait for the first edge*/
  	edge1 = TC1;
  	TFLG1 =0x02;          /*cleared C1F flag*/
  	while(!(TFLG1_C1F));  /*wait for the second edge*/
  	diff2 = TC1 - edge1;
	  }
/* Ultrasonic Setup End */

/* PWM Setup Start */
  #define BIT0 0x01       /* easier to assign pins this way*/
  #define BIT1 0x02
  #define BIT2 0x04
  #define BIT3 0x08
  #define BIT4 0x10
  #define BIT5 0x20
  #define BIT6 0x40
  #define BIT7 0x80

  #define     ECLOCK     4000000    /* this is actually too large, and gets cut (not 16-bit) */                                
  #define     ServoPWMFREQ    40    /* I want 40mS frequency */
  #define     ServoSCLA 12          /* I have no idea */
  #define     ServoSCLB 12          /* I have no idea */

  #define ServoPWMEnable()  PWME|= BIT1   /* enable pulse on pin H1-20 */
  #define ServoPWMDisable() PWME&=~BIT1   /* disable pulse on pin H1-20 */
  #define magic_1ms 1333                  /* 1333 allows for 1mS delay */

  void InitPWM(void);                        /* Initalize PWM */
  void Shoot(void);                          /* shoot. actually shoot. just do it. */
  void PWM2servo(unsigned int SteeringPWM);  /* sets duty cycle */

  int limit_range(int uplimit,int lowlimit, int input);   /* tells the servo to turn */

/* InitPWM: sets up the PWM signal for the trigger pull servo */
  void InitPWM(void){       // init PWM
    PWMCAE  =0x00;          // set left aligned mode (default)
    PWMCTL  =0x0C+BIT7+BIT4;     // concatenate 8-bit PWM channels into 16-bits 0-1 BIT4, 6-7 BIT7  
    PWMCLK |=  BIT1;   // PWM 01 clock select,use clock SA,Trigger Pull
    PWMPRCLK=0x00;     // set clock A and B to 24 Mhz(Bus clock - prescaler setting) 
    PWMPOL|=BIT1;      // Output high duty

  // A, B  = 24000000 = 40HZ*25000*(2*SCLA)
  // SA,SB =  , Scale =12
  // SA=24M/(2*SCLA), SCLA=12
    PWMSCLA=4;         // SA= 8M/(2*SCLA)= 1MHz 
    PWMSCLB=4;         // SB= 8M/(2*SCLB)= 1MHz 
    PWMCNT01=0;        // clear pwmcnt0-1 - 16 bit registers are accessed by the high

  // PWM period 
    PWMPER01=25000;    // servo freq=1M/25K= 40Hz

  // set defult value for PWM Dutycycle
    PWMDTY01=1500 ;    // range 0-25000, 40Hz=25ms, 1ms=1000, 1 LSB= 0.001 ms, center servo
    }

  void Shoot(void){    //Shoot: rev the cannon, fire, release cannon */
    PORTB = 0xFF;     //start reving
    delayms(3000);    //attain maximum speed
    
    ServoPWMEnable();  /* enable the signal */
    PWM2servo(1800);   /* Pull Trigger */
    delayms(500);      /* delay 0.5 sec to allow dart to pass through */
   
    PWM2servo(900);    /* release trigger */
    delayms(500); 
    
    ServoPWMDisable(); /* disable the signal */
    PORTB =0 ;         // bit clear 
    }

  /* PWM2servo: sets up the actual PW for the trigger pull servo. */
  /*            Must be called for the servo to turn              */
  void PWM2servo(unsigned int SteeringPWM)  // Range 50 ~ 1900 uS
    { PWMDTY01=limit_range(1900,50,SteeringPWM);}  /* set */

  /* limit_range: makes sure you don't put in a stupid value for the trigger pull PW */
  int limit_range(int uplimit,int lowlimit, int input){
    if (input> uplimit) return uplimit;
    else if ( input < lowlimit ) return lowlimit;
    else return input;
    }
/* PWM Setup End */

 
//MAIN START 
  void main(void) {
  
    /*I2C*/
      int error;
      int i;
      int ameertest;
      unsigned char temp[9];
      openI2C(0x47,0xC0); // set master's address as slave to C0
    
    /*Serial*/
      SCI0_Init(38400);
    
    /*ultrasonic */
      DDRA = 0x0F;
    	PORTA = 0;
    	TSCR1 = 0x90;   /*enable TCNT and fast flag clear*/
    	TIOS &=0xFD;    /*select IC1 function*/
    	TSCR2 = 4;      /*set TCNT prescale factor to 16*/
    	TCTL4 = 0x0C;   /*select to capture the both edges of PT1*/
    	TFLG1 =0x02;    /*cleared C1F flag*/
  	
  	/*pwm*/
    	DDRB=0xFF;      // set all portB as output, to enable cannon 
      InitPWM();
  	
  	EnableInterrupts;

    //Search and Destroy
      for(;;) { 
      ameertest = 1;  
      turnrobotcounter();                            //Slowly turn Counter Clockwise
      delayms(30);                                   //Stabilize
      error = I2Ctemperature_read(0xD0, 0x01, temp); //Read Thermopile
    
      if(temp[4]>= temp[1]+5 || temp[3]>= temp[1]+5 || temp[3]>=temp[1]+5){       //if hottest temp is in the center, Tango Spotted
        stopmoving();     //stop turning
        
        Shoot();         //take the shot
        while(ameertest){
          
          moverobotforward();           //->Advance on target
          
          captureultrasonic();          //Check Distance from Target
          if(diff2<1700 && diff<1700){  //if target is now close
            stopmoving(); //stop advancing
            ameertest = 0;   //exit loop
             
            Shoot();                    //Execute target -> Tango down
            for(i=0;i<5000;i++){ turnrobotcounter2();} //Turn robot 180 degrees
            }                                         
          }
        }
  
         _FEED_COP(); /* feeds the dog */
      }//Keep Searching
      
  } /* loop forever  */
//MAIN END