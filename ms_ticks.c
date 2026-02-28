volatile uint32_t msTickCount;
uint32_t UTIL_init( void )
{

    uint32_t stat;
    uint32_t ticks;

    /* Setup SysTick Timer for 1 msec interrupts  */
    ticks = CMU_ClockFreqGet( cmuClock_CORE ) / 1000;
    stat = SysTick_Config( ticks );

    return stat;
}
//--------------------------------------------------------------------------
void SysTick_Handler( void ) //1ms one time name can't change.
{
    static uint16_t i = 0;
    msTickCount++;
    i++;
    return;
}

//demo
void main()
{
  //some code here....
  UTIL_init(); 
  //code here....
}
