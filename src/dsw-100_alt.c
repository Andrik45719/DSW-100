/*
600 - 20500 RPM
10 - 350 Hz
100 ms - 3 ms
6 steps per revolution

W5 min speed
W4 max speed
W3 open gap
W2 close gap
W1 manual open sensetivity

K1 left/right selector
K4 setup mode
*/


//#define _XTAL_FREQ 12000000
#define _XTAL_FREQ (12000000*4)

#define DSW_K1 (!PORTBbits.RB4)
#define DSW_K2 (!PORTBbits.RB5)
#define DSW_K3 (!PORTBbits.RB3)
#define DSW_K4 (!PORTBbits.RB2)

#define DSW_ADC_W1	3	//AN3
#define DSW_ADC_W2	2	//AN2
#define DSW_ADC_W3	1	//AN1
#define DSW_ADC_W4	4	//AN4
#define DSW_ADC_W5	0	//AN0

#define DSW_DOOR_SYNC_IN	PORTDbits.RD1
#define DSW_DOOR_SYNC_OUT	PORTCbits.RC0
#define DSW_RELAY		PORTCbits.RC1
#define DSW_HBRIDGE_ENABLE 	PORTCbits.RC2	//PWM!

#define DSW_FSI			PORTCbits.RC3
#define DSW_FSO			PORTCbits.RB1

#define DSW_HA 			PORTCbits.RC6
#define DSW_HB			PORTCbits.RC5
#define DSW_HC			PORTCbits.RC4


#define DSW_OSP1		PORTEbits.RE0
#define DSW_OSP2		PORTEbits.RE1
#define DSW_SEN			PORTEbits.RE2

#define DSW_MJ			PORTAbits.RA4


#pragma config FOSC = HS
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF     // Power-up Timer Enable
#pragma config MCLRE = ON      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

#include <xc.h>

//__EEPROM_DATA(0x82, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff);

#define PHASE_AL	0b01100000
#define PHASE_BL	0b11000000
#define PHASE_CL	0b10100000
#define PHASE_L_OFF	0b11100000
#define PHASE_AH	0b00100
#define PHASE_BH	0b01000
#define PHASE_CH	0b10000
#define PHASE_H_OFF	0b00000
/*
                                               HA HB HC
A8	1  0  1  0  1  0  00	cB	        0  0  1		2	Uw  wU	uW Uw
C4	1  1  0  0  0  1  00	bA              0  1  0         6       vW  vW  wV Wv
A4	1  0  1  0  0  1  00	cA              0  1  1         1       Uv  vU  uV Uv
70	0  1  1  1  0  0  00	aC              1  0  0         4       uV  uV  vU Vu
68	0  1  1  0  1  0  00    aB              1  0  1         3       Vw  wV  vW Vw
D0	1  1  0  1  0  0  00    bC              1  1  0         5       uW  uW  wU Wu

                                               HA HB HC
D0	1  1  0  1  0  0  00    bC	        0  0  1		2	Uw  wU
68	0  1  1  0  1  0  00    aB              0  1  0         6       vW  vW
70	0  1  1  1  0  0  00	aC              0  1  1         1       Uv  vU
A4	1  0  1  0  0  1  00	cA              1  0  0         4       uV  uV
C4	1  1  0  0  0  1  00	bA              1  0  1         3       Vw  wV
A8	1  0  1  0  1  0  00	cB              1  1  0         5       uW  uW
                                                        
*/


static unsigned char sector2phase[8] = {
(PHASE_L_OFF | PHASE_H_OFF),
(PHASE_AL |            PHASE_CH),
(           PHASE_BL | PHASE_CH),
(PHASE_AH | PHASE_BL           ),
(PHASE_AH |            PHASE_CL),
(           PHASE_BH | PHASE_CL),
(PHASE_AL | PHASE_BH           ),
(PHASE_L_OFF | PHASE_H_OFF)
};

static unsigned char hall2phase[8] = {
(PHASE_L_OFF | PHASE_H_OFF),                    //HA HB HC	sector			~HA ~HB ~HC
(PHASE_CL | PHASE_BH),				// 0  0  1	5     			  1   1   0    2	
(PHASE_BL | PHASE_AH),                          // 0  1  0	3                         1   0   1    6
(PHASE_CL | PHASE_AH),                          // 0  1  1	4                         1   0   0    1
(PHASE_AL | PHASE_CH),                          // 1  0  0	1                         0   1   1    4
(PHASE_AL | PHASE_BH),                          // 1  0  1	6                         0   1   0    3
(PHASE_BL | PHASE_CH),                          // 1  1  0	2                         0   0   1    5
(PHASE_L_OFF | PHASE_H_OFF)                     
};
/*
A c
 Bc
aB
a C
 bC
Ab
*/

static unsigned char hall2sector[8] = {0,5,3,4,1,6,2,0};

//static unsigned char phCW[8] = {0xE0, 0xA8, 0xC4, 0xA4, 0x70, 0x68, 0xD0, 0xE0};
//static unsigned char phCCW[8] = {0xE0, 0xD0, 0x68, 0x70, 0xA4, 0xC4, 0xA8, 0xE0};

#define PWM_MAX	255
inline void set_pwm(unsigned int pwm_cycle)
{
//  DC1B0 = (pwm_cycle & 1);
//  DC1B1 = (pwm_cycle >> 1) & 1;
//  CCPR1L = (pwm_cycle >> 2) & 0xff;
  if(pwm_cycle > PWM_MAX) pwm_cycle = PWM_MAX;
  CCPR1L = pwm_cycle & 0xff;
}

inline unsigned int get_adc(char adc_no)
{
  ADCON0bits.CHS = adc_no;
  ADCON0bits.ADON = 1;
  __delay_us(10);
  ADCON0bits.GO = 1;
  while(ADCON0bits.GO_nDONE);
  return (unsigned int)((ADRESH << 8) + ADRESL);
}

void init(void)
{
  ANSELA = 1 << _ANSELA_ANSA0_POSITION |
           1 << _ANSELA_ANSA1_POSITION |
           1 << _ANSELA_ANSA2_POSITION |
           1 << _ANSELA_ANSA3_POSITION |
           1 << _ANSELA_ANSA5_POSITION; 
  ANSELB = 0;
  ANSELE = 0; 
  ANSELD = 0;

  TRISC = 1 << _TRISC_TRISC3_POSITION |	//FSI
          1 << _TRISC_TRISC4_POSITION |	//HC
          1 << _TRISC_TRISC5_POSITION |	//HB
          1 << _TRISC_TRISC6_POSITION |	//HA
          1 << _TRISC_TRISC7_POSITION;	//?

  TRISB = 0 << _TRISB_TRISB1_POSITION |	//FSO
          1 << _TRISB_TRISB2_POSITION |	//K4
          1 << _TRISB_TRISB3_POSITION |	//K3
          1 << _TRISB_TRISB4_POSITION |	//K1
          1 << _TRISB_TRISB5_POSITION |	//K2
          1 << _TRISB_TRISB6_POSITION |	//?
          1 << _TRISB_TRISB7_POSITION;	//?

  TRISD = 1 << _TRISD_TRISD1_POSITION;	//sync in

  PORTD = 1 << _PORTD_RD1_POSITION;	//
  PORTC = 0;
  PORTA = 0;

  TRISA = 1 << _TRISA_TRISA0_POSITION |	//W5
          1 << _TRISA_TRISA1_POSITION |	//W3
          1 << _TRISA_TRISA2_POSITION |	//W2
          1 << _TRISA_TRISA3_POSITION |	//W1
          1 << _TRISA_TRISA4_POSITION |	//OPT MJ
          1 << _TRISA_TRISA5_POSITION;	//W4

  TRISE = 1 << _TRISE_TRISE0_POSITION |	//OPT OSP
          1 << _TRISE_TRISE1_POSITION |	//OPT OSP
          1 << _TRISE_TRISE2_POSITION;	//OPT SEN
  PORTE = 1 << _PORTE_RE0_POSITION |
          1 << _PORTE_RE1_POSITION |
          1 << _PORTE_RE2_POSITION;
  
  /*ADC*/
  ADCON1 = 1 << _ADCON1_ADFM_POSITION |	//ADFM 1
           2 << _ADCON1_ADCS_POSITION;	//FOSC/32


  /*PWM RC2 PIN*/
  CCP1CON = 0xC << _CCP1CON_CCP1M_POSITION |	//1100 = PWM mode: PxA, PxC active-high; PxB, PxD active-high
              3 << _CCP1CON_DC1B_POSITION;	//PWM Duty Cycle Least Significant bits
              
  CCPTMRS0 &= ~(0x00 << _CCPTMRS0_C1TSEL_POSITION);  //CCP1 is based off Timer2 in PWM mode
  /*PWM period = Fosc / ((PR2 + 1) * 4  * prescaler)*/
  PR2 = 0xBC;	//63kHz
  PR2 = 0xFF;   //46875Hz
  T2CONbits.T2CKPS = 0;				//Prescaler is 1
  PORTD = 0xE0;					//turn off BLDC
  TMR2ON = 1;
  
  /*TIMER 1*/
  T1CON = 3 << _T1CON_T1CKPS_POSITION |	//1:8
          0 << _T1CON_TMR1CS_POSITION;	//Fosc/4

#if 0
  T1CON |= (T1CKPS1 | T1CKPS0);
  T1CON &= ~(T1OSCEN | TMR1CS1 | TMR1CS0);		//Fosc/4
  TMR1H = 0x3a;		//30Hz
  TMR1L = 0x41;

//  PIE1 &= ~TMR1IE;
  PIE1 &= ~(1 << _PIE1_TMR1IE_POSITION);
  
  INTCON |= PEIE;
  OPTION_REG = 7;
  OPTION_REG = 1 << _OPTION_REG_PS0_POSITION |
               1 << _OPTION_REG_PS1_POSITION |
               1 << _OPTION_REG_PS2_POSITION;
#endif
}

//fout = fosc/(4*prescaler*(65536 - x)
void inline timer1_start_44ms(void)
{
  TMR1ON = 0;
  TMR1H = 0;
  TMR1L = 0;
  TMR1IF = 0;
  TMR1ON = 1;
}

/*
drive until stopped
steps 100 degree
0x861	2145
0x879	2170
--0x71a	1818
*/

#define DOOR_CLOSE	0
#define DOOR_OPEN	1

int drive(char direction, unsigned int steps)
{
  unsigned int nsteps, pwm_min, pwm_max, pwm_tmp, check_point, pwm_delta, gap;
  unsigned char hall_sensor_state, hall_sensor_state_prev;
  
  pwm_min = get_adc(DSW_ADC_W5) >> 2;
  pwm_max = get_adc(DSW_ADC_W4) >> 2;
  if(pwm_max > 0xf0) pwm_max = PWM_MAX;
  if(pwm_max < pwm_min) pwm_max = pwm_min;
  set_pwm(pwm_min);

  gap = get_adc((direction == DOOR_OPEN) ? DSW_ADC_W3 : DSW_ADC_W2);
  
  check_point = 0;
  if(steps)
  {
    if(steps > gap)
    {
      check_point = (steps - gap) / 3;
      pwm_delta = ((pwm_max - pwm_min) * 128) / check_point;	//fixed point math
    }
  }

  timer1_start_44ms();
  for(hall_sensor_state_prev = 0, nsteps = 0; ;)
  {
    hall_sensor_state = (PORTC >> 4) & 7;
    if(hall_sensor_state_prev != hall_sensor_state)
    {
      hall_sensor_state_prev = hall_sensor_state;
      if(DSW_K1) hall_sensor_state = 7 - hall_sensor_state;
      if(direction) hall_sensor_state = 7 - hall_sensor_state;
      PORTD = hall2phase[hall_sensor_state];
      nsteps++;

      if(check_point)
      {
        if(nsteps < check_point)		//accel
        {
          pwm_tmp = pwm_min + ((nsteps * pwm_delta) / 128);
        }
        else if(nsteps < check_point * 2)	//max
        {
          pwm_tmp = pwm_max;
        }
        else if(nsteps < check_point * 3)	//decel
        {
          pwm_tmp = pwm_max - ((nsteps - check_point * 2) * pwm_delta) / 128;
        }
        else		//rest of path - gap steps
        {
          pwm_tmp = pwm_min;
        }
      }
      else
      {
        pwm_tmp = get_adc(DSW_ADC_W5) >> 2;
      }
      if(pwm_tmp < pwm_min) pwm_tmp = pwm_min;
      set_pwm(pwm_tmp);

      timer1_start_44ms();
    }
    else
    {
      if(TMR1IF || (nsteps >> 12))
      {
        break;
      }
    }
  }
  if(!DSW_MJ)	//triggered 
  {
    set_pwm(1);	//hold door
  }
  else
  {
    PORTD = hall2phase[0];	//release
  }
  return (int)nsteps;
}

#define DOOR_STATE_UNKNOWN	0
#define DOOR_STATE_OPENED	1
#define DOOR_STATE_CLOSED	2

#define STATE_IDLE		0
#define STATE_OPEN		1
#define STATE_CLOSE		2
#define STATE_STABILIZE		3

#define EVENT_NOPE		0
#define EVENT_MJ		1
#define EVENT_DOOR_MOVE		2

int check_for_event(void)
{
  unsigned char hs, hsp, cnt, sens;
  cnt = 0;
  hsp = PORTC & (7 << 4);
  timer1_start_44ms();
  while(!TMR1IF)
  {
    if(!DSW_MJ)
    {
      return EVENT_MJ;
    }
    hs = PORTC & (7 << 4);
    if(hs != hsp)
    {
      hsp = hs;
      cnt++;
    }
  }
  sens = get_adc(DSW_ADC_W1) >> 8;
  return cnt > sens ? EVENT_DOOR_MOVE : EVENT_NOPE;
}

void stabilize(void)
{
  int i;
  for(i = 0; i < 20; )
  {
    if(check_for_event() == EVENT_NOPE) i++;
  }
}

#define DOOR_STATE_THRESHOLD	110

void main(void)
{
  int nsteps, whole_way, position, door_state, state, evt;
  
  init();

  whole_way = EEPROM_READ(0);
  whole_way |= (EEPROM_READ(1) << 8);

  if(DSW_K4 || whole_way == ~0)		//setup
  {
    whole_way = 0;
    nsteps = drive(DOOR_OPEN, 0);
    if(whole_way < nsteps) whole_way = nsteps;
    stabilize();
    nsteps = drive(DOOR_CLOSE, 0);
    if(whole_way < nsteps) whole_way = nsteps;
    stabilize();
    nsteps = drive(DOOR_OPEN, 0);
    if(whole_way < nsteps) whole_way = nsteps;
    
    EEPROM_WRITE(0, whole_way & 0xff);
    EEPROM_WRITE(1, (whole_way >> 8) & 0xff);
    
    while(DSW_K4)
    {
      DSW_RELAY = !DSW_RELAY;
      __delay_ms(250);
    }
  }

  stabilize();
  
  whole_way = EEPROM_READ(0);
  whole_way |= (EEPROM_READ(1) << 8);

  door_state = DOOR_STATE_UNKNOWN;
  state = STATE_IDLE;
  position = 0;
  while(1)
  {
    switch(state)
    {
      case STATE_IDLE:
       evt = check_for_event();
       if(evt == EVENT_MJ || evt == EVENT_DOOR_MOVE)
       {
         state = (door_state == DOOR_STATE_OPENED) ? STATE_CLOSE : STATE_OPEN;
       }
       break;
      case STATE_CLOSE:
       nsteps = position - 0;
       if(door_state == DOOR_STATE_UNKNOWN)
       {
         nsteps = 0;		//drive it slowly
         position = 0;
       }
       nsteps = drive(DOOR_CLOSE, (unsigned int) nsteps);
       position -= nsteps;
       if(position > DOOR_STATE_THRESHOLD)
       {
         //collision detected, reopen
         door_state = DOOR_STATE_CLOSED;
         state = STATE_OPEN;
       }
       else
       {
         position = 0;
         DSW_RELAY = 1;
         door_state = DOOR_STATE_CLOSED;
         state = STATE_STABILIZE;
       }
       break;
      case STATE_OPEN:
       DSW_RELAY = 0;
       nsteps = whole_way - position;
       if(door_state == DOOR_STATE_UNKNOWN)
       {
         nsteps = 0;		//drive it slowly
         position = 0;
       }

       nsteps = drive(DOOR_OPEN, (unsigned int) nsteps);
       position += nsteps;
       if((whole_way - position) > DOOR_STATE_THRESHOLD && 
          door_state != DOOR_STATE_UNKNOWN)
       {
         //collision detected
         door_state = DOOR_STATE_UNKNOWN;
       }
       else
       {
         door_state = DOOR_STATE_OPENED;
         position = whole_way;
       }
       state = STATE_STABILIZE;
       break;
      case STATE_STABILIZE:
       while(!DSW_MJ);		//wait until triggered
       PORTD = hall2phase[0];	//release BLDC
       stabilize();
       state = STATE_IDLE;
       break;
    }
  }
}
