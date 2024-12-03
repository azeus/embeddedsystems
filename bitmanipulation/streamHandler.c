// Define register bit positions
#define DATA_READY_BIT      (0)
#define DATA_OVERFLOW_BIT   (1)
#define DATA_ERROR_BIT      (2)

// Simulated hardware register (in real hardware, this would be a memory-mapped register)
volatile uint32_t* const STATUS_REGISTER = (volatile uint32_t*)0x40000000;

// Structure to handle incoming data stream
typedef struct {
    uint32_t assembledData;    // Final 32-bit value being assembled
    uint16_t inputRegister;    // 12-bit input register (using uint16_t for alignment)
    uint8_t bitsInRegister;    // Number of valid bits currently in input register
    uint8_t totalBitsReceived; // Total bits received for current 32-bit word
    bool isComplete;           // Flag indicating if 32-bit word is complete
} DataStreamHandler;

// Register manipulation functions
static inline void setRegisterBit(volatile uint32_t* reg, uint8_t bitPos) {
    *reg |= (1UL << bitPos);
}

static inline void clearRegisterBit(volatile uint32_t* reg, uint8_t bitPos) {
    *reg &= ~(1UL << bitPos);
}

static inline bool getRegisterBit(volatile uint32_t* reg, uint8_t bitPos) {
    return (*reg & (1UL << bitPos)) != 0;
}

// Initialize the data stream handler
DataStreamHandler initDataStreamHandler(void) {
    DataStreamHandler handler = {
        .assembledData = 0,
        .inputRegister = 0,
        .bitsInRegister = 0,
        .totalBitsReceived = 0,
        .isComplete = false
    };

    // Clear all status bits
    clearRegisterBit(STATUS_REGISTER, DATA_READY_BIT);
    clearRegisterBit(STATUS_REGISTER, DATA_OVERFLOW_BIT);
    clearRegisterBit(STATUS_REGISTER, DATA_ERROR_BIT);

    return handler;
}

// Process 12 bits of incoming data
void processInputRegister(DataStreamHandler* handler, uint16_t newBits) {
    // Clear data ready bit at start of new processing
    clearRegisterBit(STATUS_REGISTER, DATA_READY_BIT);

    // Mask to ensure only 12 bits are used
    const uint16_t MASK_12BIT = 0x0FFF;
    newBits &= MASK_12BIT;

    // Calculate how many bits we can use from this input
    uint8_t bitsToProcess = 12;
    uint8_t bitsRemaining = 32 - handler->totalBitsReceived;
    if (bitsToProcess > bitsRemaining) {
        bitsToProcess = bitsRemaining;
        // Set overflow bit if we have extra bits
        setRegisterBit(STATUS_REGISTER, DATA_OVERFLOW_BIT);
    }

    // Shift existing data left to make room for new bits
    handler->assembledData <<= bitsToProcess;

    // Add new bits to assembled data
    handler->assembledData |= (newBits >> (12 - bitsToProcess));

    // Update total bits received
    handler->totalBitsReceived += bitsToProcess;

    // Store remaining bits in input register if any
    if (bitsToProcess < 12) {
        handler->inputRegister = newBits & ((1 << (12 - bitsToProcess)) - 1);
        handler->bitsInRegister = 12 - bitsToProcess;
    } else {
        handler->inputRegister = 0;
        handler->bitsInRegister = 0;
    }

    // Check if we've completed a 32-bit word
    if (handler->totalBitsReceived >= 32) {
        handler->isComplete = true;
        // Set the data ready bit to signal completion
        setRegisterBit(STATUS_REGISTER, DATA_READY_BIT);
    }
}

// Reset handler for next 32-bit word while preserving any overflow bits
void resetHandler(DataStreamHandler* handler) {
    uint16_t savedInputReg = handler->inputRegister;
    uint8_t savedBits = handler->bitsInRegister;

    *handler = initDataStreamHandler();

    // Restore any overflow bits
    handler->inputRegister = savedInputReg;
    handler->bitsInRegister = savedBits;

    // If we have overflow bits, process them for the next word
    if (savedBits > 0) {
        handler->assembledData = savedInputReg;
        handler->totalBitsReceived = savedBits;
        clearRegisterBit(STATUS_REGISTER, DATA_OVERFLOW_BIT);
    }
}

// Example ISR for handling data ready condition
void __attribute__((interrupt)) DataReadyISR(void) {
    if (getRegisterBit(STATUS_REGISTER, DATA_READY_BIT)) {
        // Handle the completed data
        // Clear the interrupt flag
        clearRegisterBit(STATUS_REGISTER, DATA_READY_BIT);
    }
}