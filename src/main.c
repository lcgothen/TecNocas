#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serial_printf.h"
#include "i2c.h"
#include "lcd1602.h"

//CORES
#define black 100
#define white 920

//********PINS***********

//encoders
#define ENC1_A 0 // B (D8)
#define ENC1_B 7 // D (D7)
#define ENC2_A 4 // B (D12)
#define ENC2_B 3 // B (D11)
#define BOTTOM 63535

//motores
#define AIN1 2 // B 
#define AIN2 1 // B
#define BIN1 4 // D
#define BIN2 3 // D
#define PWMA 5 // D
#define PWMB 6 // D

//sensores IV
#define IV1 0 // C
#define IV2 1 // C
#define IV3 2 // C
#define IV4 3 // C
#define IV5 4 // C

//************************

//velocidades base
#define BASEA 0 //entre 0 e 256
#define BASEB 0 //entre 0 e 256

//constantes
#define Kp 2.5 //2.2
#define Kd 7 //0.85
#define vbase 35 //35

uint8_t count = 4;
uint16_t sensx = 0;
uint8_t encA_1 = 0;
uint8_t encB_1 = 0;
uint8_t encA_2 = 0;
uint8_t encB_2 = 0;
uint32_t enc1[2]={0,0}, enc2[2]={0,0}, st_motorA=0, posA=0, st_motorB=0, posB=0;

int32_t pos;
int32_t prop, der, old_prop, error;

int laps=0;

void init_interrupts(void)
{
    //enable all interrupts
    SREG = 0b10000000;

    //enable pin change interrupts
    /*PCICR = (1<<PCIE2) | (1<<PCIE0);
    PCMSK2 = (1<<PCINT23);
    PCMSK0 = (1<<PCINT4) | (1<<PCINT3) | (1<<PCINT0);*/
}

void init_IO(void)
{
    //set IR sensors pins as inputs
    DDRC &= ~(1<<IV1);
    DDRC &= ~(1<<IV2);
    DDRC &= ~(1<<IV3);
    DDRC &= ~(1<<IV4);
    DDRC &= ~(1<<IV5);

    //set encoder pins as inputs
    DDRD &= ~(1<<ENC1_B);
    DDRB &= ~((1<<ENC1_A) | (1<<ENC2_A) | (1<<ENC2_B));

    //set motor pins
    DDRD |= (1<<PWMA) | (1<<PWMB);
    DDRB |= (1<<AIN1) | (1<<AIN2);
    DDRD |= (1<<BIN1) | (1<<BIN2);
    //set direction
    PORTB |= (1<<AIN1);
    PORTD |= (1<<BIN1);
    PORTB &= ~(1<<AIN2);
    PORTD &= ~(1<<BIN2);
    PORTD |= (1<<PWMA) | (1<<PWMB);

}

//***************************** ANALOG READ ************************************
void read_analog(uint8_t bit)
{
    ADMUX = bit;
    ADMUX = ADMUX | (1<<REFS0);
   // ADMUX &= ~(1<<ADLAR);
    ADCSRA = ADCSRA | (1<<ADSC);
}

void init_analog(void)
{
    //Definir Vref=AVcc
    ADMUX = ADMUX | (1<<REFS0);

    // Desativar buffer digital em PC0
    DIDR0 = DIDR0 | 0b00011111;

    // Pré-divisor em 128 e ativar ADC
    ADCSRA |= (7<<ADPS0) | (1<<ADEN) | (1<<ADIE);

    //start reading
    read_analog(count);
}

ISR(ADC_vect)
{
    sensx = ADC;
    count++;
    if(count<5)
    {
        read_analog(count);
    }
    else
    {
        count = 0;
        read_analog(count);
    }
}
//******************************************************************************

//******************************** PWM *******************************************
void init_pwm(void)
{
    TCCR0B = 0;
    TIFR0 |= (7<<TOV0);
    TCCR0A |= (1<<WGM00) | (1<<WGM01) | (1<<COM0A1) | (1<<COM0B1);
    TCNT0 = 0;
    OCR0A = BASEA;
    OCR0B = BASEB;
    TIMSK0 = 0;
    TCCR0B |= (1<<CS01) | (1<<CS00); //prescaler 64
}

void set_speed_A(float v)
{
    float aux;
    aux = v*255/100;
    OCR0A = aux;
}

void set_speed_B(float v)
{
    float aux;
    aux = v*255/100;
    OCR0B = aux;
}


//******************************************************************************

//*************************** FOLLOW LINE **************************************
void follow_line(int *IR_sensors)
{
    old_prop = prop;
    pos = (0*(int32_t)IR_sensors[0] + 1000*(int32_t)IR_sensors[1] + 2000*(int32_t)IR_sensors[2] + 3000*(int32_t)IR_sensors[3] + 4000*(int32_t)IR_sensors[4])/(20*((int32_t)IR_sensors[0]+(int32_t)IR_sensors[1]+(int32_t)IR_sensors[2]+(int32_t)IR_sensors[3]+(int32_t)IR_sensors[4]));
    prop = pos-100;
    der = prop-old_prop;


    error = prop*Kp + der*Kd;

  //printf(" error: %ld  pos: %ld\n", error, pos);

    if(error<10 && error>-10)
    {
        set_speed_A(vbase);
        set_speed_B(vbase);
    }
    
    else
    {
        if(vbase+error<0)
        {
            set_speed_A(0);

            if(vbase-error>100)
                set_speed_B(100);

            else
                set_speed_B(vbase-error);
        }

        else if(vbase-error<0)
        {
            set_speed_B(0);

            if(vbase+error>100)
                set_speed_A(100);

            else
                set_speed_A(vbase+error);
        }

        else
        {
            set_speed_A(vbase+error);
            set_speed_B(vbase-error);
        }
    }
}

void follow_line_left(int *IR_sensors)
{
    old_prop = prop;
    pos = (0*(int32_t)IR_sensors[0] + 1000*(int32_t)IR_sensors[1] + 2000*(int32_t)IR_sensors[2] + 3000*(int32_t)IR_sensors[3] + 4000*(int32_t)IR_sensors[4])/(20*((int32_t)IR_sensors[0]+(int32_t)IR_sensors[1]+(int32_t)IR_sensors[2]+(int32_t)IR_sensors[3]+(int32_t)IR_sensors[4]));
    prop = pos-100;
    der = prop-old_prop;


    error = prop*Kp + der*Kd;

    //printf(" error: %ld  pos: %ld\n", error, pos);
    

    if(error<10 && error>-10)
    {
        set_speed_A(vbase);
        set_speed_B(vbase);
    }
    
    else
    {
        if(vbase+error<0)
        {
            set_speed_A(0);

            if(vbase-error>100)
                set_speed_B(100);

            else
                set_speed_B(vbase-error);
        }

        else if(vbase-error<0)
        {
            set_speed_B(0);

            if(vbase+error>100)
                set_speed_A(100);

            else
                set_speed_A(vbase+error);
        }

        else
        {
            set_speed_A(vbase+error);
            set_speed_B(vbase-error);
        }
    }
}

void follow_line_right(int *IR_sensors)
{
    old_prop = prop;
    pos = (0*(int32_t)IR_sensors[0] + 1000*(int32_t)IR_sensors[1] + 2000*(int32_t)IR_sensors[2] + 3000*(int32_t)IR_sensors[3] + 4000*(int32_t)IR_sensors[4])/(20*((int32_t)IR_sensors[0]+(int32_t)IR_sensors[1]+(int32_t)IR_sensors[2]+(int32_t)IR_sensors[3]+(int32_t)IR_sensors[4]));
    prop = pos-100;
    der = prop-old_prop;


    error = prop*Kp + der*Kd;

    //printf(" error: %ld  pos: %ld\n", error, pos);

    //printf("1: %d  2: %d  3: %d  4: %d  5: %d\n", IR_sensors[0], IR_sensors[1], IR_sensors[2], IR_sensors[3], IR_sensors[4]);

    if((IR_sensors[0]==0 && IR_sensors[1]!=0 && IR_sensors[2]==0 && IR_sensors[3]!=0 && IR_sensors[4]==0))
    {
        set_speed_A(vbase);
        set_speed_B(vbase);
        laps++;
    }

    else if(IR_sensors[2]==0 && IR_sensors[3]==0 && IR_sensors[4]==0)
    {
        set_speed_A(5);
        set_speed_B(vbase+30);
    }

    else if((IR_sensors[0]==0 && IR_sensors[1]==0 && IR_sensors[2]==0) || (IR_sensors[0]==0 && IR_sensors[1]!=0 && IR_sensors[2]==0))
    {
        set_speed_A(5);
        set_speed_B(5);
    }

    else if(error<10 && error>-10)
    {
        set_speed_A(vbase);
        set_speed_B(vbase);
    }
    
    else
    {
        if(vbase+error<0)
        {
            set_speed_A(0);

            if(vbase-error>100)
                set_speed_B(100);

            else
                set_speed_B(vbase-error);
        }

        else if(vbase-error<0)
        {
            set_speed_B(0);

            if(vbase+error>100)
                set_speed_A(100);

            else
                set_speed_A(vbase+error);
        }

        else
        {
            set_speed_A(vbase+error);
            set_speed_B(vbase-error);
        }
    }
}

//*******************************************************************************

//***************************** SENSORES ****************************************
void read_sensors(int *IR_sensors)
{
    //update IR sensors information
    cli();
    if(count>0 && count<5)
    {    
        if(sensx>=white)
            IR_sensors[count-1] = 1000;
        else if(sensx<=black)
            IR_sensors[count-1] = 0; 
    }
    
    else if(count==0)
    {
        if(sensx>=white)
            IR_sensors[4] = 1000;
        else if(sensx<=black)
            IR_sensors[4] = 0;
    }
    sei();
}
//*******************************************************************************

//***************************** ENCODERS ****************************************
void init_timer1(void) //0.1 ms
{
    TCCR1B = 0;                    // Stop tc1
    TIFR1 |= (7<<OCF1A)|(1<<ICF1);  // Clear interruptions
    TCCR1A = 0;                     // normal mode
    TCNT1 = BOTTOM;                 // Load BOTTOM value (only count 2000)
    TIMSK1 = (1<<TOIE1);         // Enable overflow interrupt
    TCCR1B = 2;                   // Start TC1 (TP=8)
}

ISR(TIMER1_OVF_vect)
{
    //update encoders information
    if((PINB & (1<<0)) == 0)
        enc1[0] = 0;
    else
        enc1[0] = 1;
    if((PIND & (1<<7)) == 0)
        enc2[0] = 0;
    else
        enc2[0] = 1;
    if((PINB & (1<<4)) == 0)
        enc1[1] = 0;
    else
        enc1[1] = 1;
    if((PINB & (1<<3)) == 0)
        enc2[1] = 0;
    else
        enc2[1] = 1;

    //motor A state
    if(st_motorA==0 && enc1[0]==1 && enc2[0]==0)
    {
        st_motorA=2;
        posA++;
    }
    else if(st_motorA==0 && enc1[0]==0 && enc2[0]==1)
    {
        st_motorA=1;
        posA--;
    }
    else if(st_motorA==2 && enc1[0]==1 && enc2[0]==1)
    {
        st_motorA=3;
        posA++;
    }
    else if(st_motorA==2 && enc1[0]==0 && enc2[0]==0)
    {
        st_motorA=0;
        posA--;
    }
    else if(st_motorA==1 && enc1[0]==1 && enc2[0]==1)
    {
        st_motorA=3;
        posA--;
    }
    else if(st_motorA==1 && enc1[0]==0 && enc2[0]==0)
    {
        st_motorA=0;
        posA++;
    }
    else if(st_motorA==3 && enc1[0]==0 && enc2[0]==1)
    {
        st_motorA=1;
        posA++;
    }
    else if(st_motorA==3 && enc1[0]==1 && enc2[0]==0)
    {
        st_motorA=2;
        posA--;
    }

    //motor B state
    if(st_motorB==0 && enc1[1]==1 && enc2[1]==0)
    {
        st_motorB=2;
        posB--;
    }
    else if(st_motorB==0 && enc1[1]==0 && enc2[1]==1)
    {
        st_motorB=1;
        posB++;
    }
    else if(st_motorB==2 && enc1[1]==1 && enc2[1]==1)
    {
        st_motorB=3;
        posB--;
    }
    else if(st_motorB==2 && enc1[1]==0 && enc2[1]==0)
    {
        st_motorB=0;
        posB++;
    }
    else if(st_motorB==1 && enc1[1]==1 && enc2[1]==1)
    {
        st_motorB=3;
        posB++;
    }
    else if(st_motorB==1 && enc1[1]==0 && enc2[1]==0)
    {
        st_motorB=0;
        posB--;
    }
    else if(st_motorB==3 && enc1[1]==0 && enc2[1]==1)
    {
        st_motorB=1;
        posB--;
    }
    else if(st_motorB==3 && enc1[1]==1 && enc2[1]==0)
    {
        st_motorB=2;
        posB++;
    }
    TCNT1 = BOTTOM;                 // Load BOTTOM value (only count 2000)
}
//*******************************************************************************

int main(void)
{
    int IR_sensors[5];
    init_interrupts();
    init_IO();
    init_analog();
    printf_init();
    init_pwm();
    init_timer1();
    /*i2c_init();
    lcd1602_init();
    lcd1602_clear();*/
    
    while(1)
    {
        /*lcd1602_goto_xy(0,1);
        lcd1602_send_string("bla");*/

        read_sensors(IR_sensors);
        //printf("1: %d  2: %d  3: %d  4: %d  5: %d\n", IR_sensors[0], IR_sensors[1], IR_sensors[2], IR_sensors[3], IR_sensors[4]);
        //printf("posA: %ld   posB: %ld\n", posA, posB);
        //SEGUE LINHA
        follow_line(IR_sensors);
        //follow_line_right(IR_sensors);
    }
}  