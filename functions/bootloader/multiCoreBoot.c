// Multi-core boot synchronization example
volatile uint32_t core_sync __attribute__((section(".shared_ram"))) = 0;

void core0_boot(void) {
    // Core 0 initialization
    system_clock_init();
    memory_init();

    // Signal core 1
    core_sync = 1;
    __DSB();

    // Wait for core 1 ready
    while (core_sync != 2);

    // Continue boot
    boot_continue();
}

void core1_boot(void) {
    // Wait for core 0 initialization
    while (core_sync != 1);

    // Core 1 initialization
    core1_peripherals_init();

    // Signal ready
    core_sync = 2;
    __DSB();
}