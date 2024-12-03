// Hardware initialization example
void system_clock_init(void) {
    // Configure PLL and system clock
    RCC->CR |= RCC_CR_HSEON;                    // Enable HSE
    while(!(RCC->CR & RCC_CR_HSERDY));          // Wait for HSE ready

    RCC->PLLCFGR = (
        RCC_PLLCFGR_PLLSRC_HSE |               // Use HSE as PLL source
        (8 << RCC_PLLCFGR_PLLM_Pos) |          // PLLM = 8
        (336 << RCC_PLLCFGR_PLLN_Pos)          // PLLN = 336
    );

    RCC->CR |= RCC_CR_PLLON;                    // Enable PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));          // Wait for PLL ready
}