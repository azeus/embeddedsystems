typedef struct {
    volatile uint32_t CTRL;     // Control register
    volatile uint32_t RELOAD;   // Reload value register
    volatile uint32_t STATUS;   // Status register
} WDT_TypeDef;

#define WDT_BASE    0x40003000
#define WDT         ((WDT_TypeDef*)WDT_BASE)

//Function to configure and enable the watchdog with a specified timeout
void init_watchdog(uint32 timeout_ms){
    WDT->RELOAD = (timeout_ms * SystemCoreClock ) / 1000;
    WDT->CTRL |= 0x01;
}

//Function to reset the countdown, keeping the system from resetting if called within the timeout period.
void kick_watchdog(void){
    WDT->RELOAD = WDT->RELOAD;
}
