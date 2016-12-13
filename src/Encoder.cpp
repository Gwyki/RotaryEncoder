#include "encoder.h"
#include "arduino.h"

// uncomment to allow manual ISR handling - so if you have other pinchange interrupts you can merge them
// a simple callback function is allowed for each encoder, though remember it is an ISR and should exit *quickly*
// think setting flags, incrementing menu position counter, etc...
// #define ENCODER_CUSTOM_ISR


// some common macros to help with quality of life
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

uint8_t Encoder::encIndex = 0;
Encoder *Encoder::encList[4];


Encoder::Encoder()
{  
}


void Encoder::begin(uint8_t pinA, uint8_t pinB)
{

  m_pinA = pinA;
  m_pinB = pinB;

  pinAsInputPullUp(m_pinA);
  pinAsInputPullUp(m_pinB);
  

  // sets the initial state of the encoder
  state = 0;
  state |= ((*pinOfPin(m_pinA) & pinMask(m_pinA)) >> (pinIndex(m_pinA)-1));
  state |= ((*pinOfPin(m_pinB) & pinMask(m_pinB)) >> (pinIndex(m_pinB)));
  // state is a 4 bit value where:
  // 1. The most significant 2 bits are the previous state of each encoder pin
  // 2. The least significant bits are the current state of each encoder pin
  // 3. These 4 bits map into a 16 element array that encode for each possible outcome, see: QUAD_ENC_DETENT[16] in encoder.h

  time1 = micros();
  time2 = 0;
  
  // setup the interrupt and pinchange masks

  PCICR |= ( 1 << digitalPinToPCICRbit(m_pinA) ) | ( 1 << digitalPinToPCICRbit(m_pinB) );  // enable the interrupt on the port the pins are on

  // find the correct interrupt mask register for the pin, and set it to only trigger for the set pin
  *digitalPinToPCMSK(m_pinA) |= ( 1 << digitalPinToPCMSKbit(m_pinA) );
  *digitalPinToPCMSK(m_pinB) |= ( 1 << digitalPinToPCMSKbit(m_pinB) );

  
  encList[encIndex] = this;
  encIndex++;

  // cap at 4 active encoders - somewhat arbitrary, only tested with two in non-trivial application
  if (encIndex > 3)
    encIndex = 3;
}

void Encoder::doInt()
{
    for (int i = 0; i < encIndex; ++i)
      encList[i] ->tick();
      
 }

void Encoder::tick()
{
  
  cli();

  uint8_t oldstate = state;
  state = (state << 2) & 0x0F;
  // see comments above on state variable and how it encodes what the encoder wheel is doing
     
  state |= ((*pinOfPin(m_pinA) & pinMask(m_pinA)) >> (pinIndex(m_pinA)-1));
  state |= ((*pinOfPin(m_pinB) & pinMask(m_pinB)) >> (pinIndex(m_pinB)));

//  if (oldstate == state)
//  {
//    sei();
//    return;
//  }

  // only do this when either pin goes high - high to high is one cycle (assumes detents - TODO: generalise this)

  if ( (state == 7) | (state == 11))
  {   
    unsigned long tmptime = micros();  
    time2 = tmptime - time1;  // holds micros between interrupts
    time1 = tmptime;
    
    // some very simple debouncing - *NOT* good enough for accurate acceleration gestures hardware is a must
    if (time2 < 10000)
      return;
    
    // we have successive actuations slower than the debounce but faster than the acceleration delay so accelerate
    velocity += m_accel;   // only count complete pulses for the velocity

    if (time2 > m_delay)
      velocity = 1;
  }

  int16_t oldcount = count;
  count += ( QUAD_ENC_DETENT[state] * (velocity) );
  // change the above line to reference QUAD_ENC_NODETENT if your encoder doesn't have a detent or you wish to track all the transitions.
  // also ensure that QUAD_ENC_NODETENT is defined in encoder.h - for memory reasons it is not by default
  
  if (count != oldcount)
  {
    m_changed = 1;
    

    if (m_callback)
      m_callback();
  }
  sei();
}

void Encoder::setCallback(void(*callback)())
{
  m_callback = callback;
}

int16_t Encoder::getCount()
{
  return count;  
}

bool Encoder::changed()
{
  return m_changed;
}

void Encoder::reset()
{
  m_changed = 0;
}

void Encoder::setAccel(uint8_t acc)
{
  m_accel = acc;
}

void Encoder::setDelay(uint16_t del)
{
  m_delay = del;
}

// potential issue here if you have other code making use of the PC_INT interrupts




#ifndef ENCODER_CUSTOM_ISR
ISR (PCINT1_vect)
{
  Encoder::doInt();
}

ISR (PCINT0_vect)
{
  Encoder::doInt();
}

ISR (PCINT2_vect)
{
  Encoder::doInt();
}
#endif
