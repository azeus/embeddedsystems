// Application jump code
typedef void (*application_t)(void);

void jump_to_application(void) {
    // 1. Disable interrupts
    __disable_irq();

    // 2. Reset peripherals
    reset_peripherals();

    // 3. Set up vector table
    SCB->VTOR = APP_START_ADDRESS;

    // 4. Set up stack pointer
    vector_table_t* vectors = (vector_table_t*)APP_START_ADDRESS;
    __set_MSP(vectors->stack_pointer);

    // 5. Jump to application
    application_t app = (application_t)(vectors->reset_handler);
    app();
}