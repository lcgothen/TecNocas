#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "serial_printf.h"

//CORES
#define black 150
#define white 850

//********PINS***********

//encoders
#define ENC1_A 0 // B (D8)
#define ENC1_B 7 // D (D7)
#define ENC2_A 4 // B (D12)
#define ENC2_B 3 // B (D11)

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

uint8_t count = 4;
uint16_t sensx = 0;
uint8_t encA_1 = 0;
uint8_t encB_1 = 0;
uint8_t encA_2 = 0;
uint8_t encB_2 = 0;

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

void set_speed_A(int v)
{
    OCR0A = v;
}

void set_speed_B(int v)
{
    OCR0B = v;
}


//******************************************************************************


int main(void)
{
    uint16_t IR_sensors[5];
    int enc1[2]={0,0}, enc2[2]={0,0}, st_motorA=0, posA=0, st_motorB=0, posB=0;
    init_interrupts();
    init_IO();
    init_analog();
    printf_init();
    init_pwm();
    
    while(1)
    {
        //update IR sensors information
        cli();
        if(count>0 && count<5)
            IR_sensors[count-1] = sensx;
        else if(count==0)
            IR_sensors[4] = sensx;
        sei();

        printf("1: %d  2: %d  3: %d  4: %d  5: %d\n", IR_sensors[0], IR_sensors[1], IR_sensors[2], IR_sensors[3], IR_sensors[4]);

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
        //printf("1: %d  2: %d  3: %d 4: %d\n", enc1[0], enc2[0], enc1[1], enc2[1]);

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
        //printf("posA: %d ", posA);

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
        //printf("posB: %d \n", posB);

        //SEGUE LINHA

        if(IR_sensors[2]<=black && IR_sensors[3]<=black && IR_sensors[1]>=white && IR_sensors[0]>=white && IR_sensors[4]>=white)
        {
            set_speed_B(100);
            set_speed_A(100;
            printf("1   ");
        }

        else if(IR_sensors[2]<=black && IR_sensors[3]>=black && IR_sensors[1]>=white && IR_sensors[0]>=white && IR_sensors[4]>=white))
        {
            set_speed_B(130);
            set_speed_A(80);
            printf("2   ");
        }

        else if(IR_sensors[2]<=black && IR_sensors[3]<=black && IR_sensors[1]>=white && IR_sensors[0]>=white && IR_sensors[4]<=black))
        {
            set_speed_B(80);
            set_speed_A(130);
            printf("3   ");
        }
        
        else if(IR_sensors[2]>black && (IR_sensors[0]<=black || IR_sensors[1]<=black))
        {
            set_speed_B(80);
            set_speed_A(130);
            printf("4   ");
        }

        else if(IR_sensors[2]>black && (IR_sensors[3]<=black || IR_sensors[4]<=black))
        {
            set_speed_B(110);
            set_speed_A(80);
            printf("5   ");
        }
        
        else if(IR_sensors[2]<=black  && IR_sensors[3]<=black  && IR_sensors[0]<=black)
        {
            set_speed_B(0);
            set_speed_A(100);
            printf("6   ");
        }

        else if(IR_sensors[2]<=black && IR_sensors[3]<=black && IR_sensors[4]<=black)
        {
            set_speed_B(100);
            set_speed_A(0);
            printf("7   ");
        }

        else
        {
            set_speed_B(0);
            set_speed_A(0);
            printf("8   ");
        }
    }

}   
