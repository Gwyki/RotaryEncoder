#include "stdint.h"



class Encoder
{
  private:
 
  volatile int16_t count = 0;
  volatile int8_t velocity = 0;
  volatile uint32_t time1, time2;
  volatile uint8_t state;

  volatile bool m_changed = 0;

  uint8_t m_pinA, m_pinB;

  // the following arrays define the actions of each possible state of the two pins in the current and previous timestep
  // it is best to draw them out to really understand what is happening here
  //int8_t QUAD_ENC_NODETENT[16] = { 0, 1, -1, 0, -1, 0, 0, 1,  1, 0, 0, -1, 0, -1, 1, 0};
  int8_t QUAD_ENC_DETENT[16] = {  0, 0, 0, 0, 0, 0 ,0 ,1 ,0 ,0 ,0 ,-1 ,0 ,0 ,0 ,0 };
  // the version with the detent is the default - common ebay encoders have a detent that is actually after 4 cycles, so 
  // for each detent to return once a simple way is to modify the state array as above. 
  // This is not a runtime option to save memory
  
  uint16_t m_delay = 22000;
  uint8_t m_accel = 7;

  void (*m_callback)();
  
  public:
    Encoder();

    static void doInt();

    void begin(uint8_t pinA, uint8_t pinB);
    void tick();
    int16_t getCount();
    bool changed();
    void reset();
    void setAccel(uint8_t acc);
    void setDelay(uint16_t del);

    void setCallback( void(*callback)() );

    // static variables that hold the encoder references
    static Encoder *encList[4];
    static uint8_t encIndex;

};


