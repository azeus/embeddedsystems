#define DEBOUNCE_DELAY_MS  20
#define SAMPLE_COUNT       5

bool debounce_button(volatile uint32_t* port, uint8_t pin) {
    uint8_t count = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        if (READ_BIT(*port, pin)) {
            count++;
        }
        delay_ms(DEBOUNCE_DELAY_MS / SAMPLE_COUNT);
    }

    return (count > SAMPLE_COUNT/2);
}