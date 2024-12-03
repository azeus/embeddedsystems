// Example: Implementing a UART-style receiver
typedef struct {
    ShiftRegister sr;
    uint8_t bitCount;       // Count of received bits
    bool isReceiving;       // Currently receiving data
    uint8_t dataBits;       // Number of data bits (typically 8)
} UARTReceiver;

UARTReceiver initUARTReceiver(uint8_t dataBits) {
    UARTReceiver uart;
    uart.sr = initShiftRegister(dataBits);
    uart.bitCount = 0;
    uart.isReceiving = false;
    uart.dataBits = dataBits;
    return uart;
}

// Process incoming UART bit (returns true if byte complete)
bool processUARTBit(UARTReceiver* uart, bool bit) {
    // Start bit detection
    if (!uart->isReceiving && !bit) {
        uart->isReceiving = true;
        uart->bitCount = 0;
        return false;
    }

    // Data bits processing
    if (uart->isReceiving) {
        if (uart->bitCount < uart->dataBits) {
            shiftRight(&uart->sr, bit);
            uart->bitCount++;

            // Check if byte is complete
            if (uart->bitCount == uart->dataBits) {
                uart->isReceiving = false;
                return true;
            }
        }
    }

    return false;
}