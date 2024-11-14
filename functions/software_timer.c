volatile uint32_t ms_ticks = 0;

void SysTick_Handler(void) {
    ms_ticks++;
}

void init_systick(uint32_t ticks_per_ms) {
    SysTick->LOAD = ticks_per_ms - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;  // Enable, interrupt, use processor clock
}

bool check_timeout(uint32_t start_time, uint32_t timeout_ms) {
    return ((ms_ticks - start_time) >= timeout_ms);
}