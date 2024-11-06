//Function to configure and enable the watchdog with a specified timeout
void init_watchdog(uint32 timeout_ms){
    WDT->RELOAD = (timeout_ms * SystemCoreClock ) / 1000;
    WDT->CTRL |= 0x01;
}

//Function to reset the countdown, keeping the system from resetting if called within the timeout period.
void kick_watchdog(void){
    WDT->RELOAD = WDT->RELOAD;
}
