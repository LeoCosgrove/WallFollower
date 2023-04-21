// PWM.c
// Runs on MSP432
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
//***************************PWM_InitRL*******************************
// PWM outputs on P2.6, P2.7
// Inputs:  period (1.333us)
//          dutyRight
//          dutyLeft
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// period of P7.3 squarewave is 4*period*666.7ns
// P2.6=1 when timer equals TA0CCR3 on way down, P2.6=0 when timer equals TA0CCR3 on way up
// P2.7=1 when timer equals TA0CCR4 on way down, P2.7=0 when timer equals TA0CCR4 on way up
// Period of P2.6 is period*1.333us, duty cycle is dutyRight/period
// Period of P2.7 is period*1.333us, duty cycle is dutyLeft/period
void PWM_InitRL(uint16_t period, uint16_t dutyRight, uint16_t dutyLeft){

    P2->DIR |= 0xC0;          // P2.6, P2.7 output
    P2->SEL0 |= 0xC0;         // P2.6, P2.7 Timer0A functions
    P2->SEL1 &= ~0xC0;        // P2.6, P2.7 Timer0A functions
    TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode        //1.5MHz?
    TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A0->CCR[0] = period;       // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000;          // divide by 1
    TIMER_A0->CCTL[3] = 0x0040;      // CCR1 toggle/reset                           //CCTL 3 = P2.6
    TIMER_A0->CCR[3] = dutyRight;        // CCR1 duty cycle is duty1/period
    TIMER_A0->CCTL[4] = 0x0040;      // CCR2 toggle/reset                           //CCTL 3 = P2.6
    TIMER_A0->CCR[4] = dutyLeft;        // CCR2 duty cycle is duty2/period


}

//***************************PWM_Duty3*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  dutyRight
// Outputs: none
// period of P2.6 is 2*period*666.7ns, duty cycle is dutyRight/period
void PWM_DutyRight(uint16_t dutyRight) {

    if(dutyRight >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[3] = dutyRight;        // CCR1 duty cycle is duty1/period
  
}

//***************************PWM_Duty4*******************************
// change duty cycle of PWM output on P2.7
// Inputs:  dutyLeft
// Outputs: none
// period of P2.7 is 2*period*666.7ns, duty cycle is dutyLeft/period
void PWM_DutyLeft(uint16_t dutyLeft) {

    if(dutyLeft >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[4] = dutyLeft;        // CCR2 duty cycle is duty2/period

}
