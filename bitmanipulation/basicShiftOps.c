// Structure to simulate a shift register with configurable size
typedef struct {
    uint32_t data;          // Current register contents
    uint8_t size;           // Size of register in bits
    uint32_t mask;          // Mask for valid bits
} ShiftRegister;

// Initialize a shift register of given size
ShiftRegister initShiftRegister(uint8_t size) {
    ShiftRegister sr;
    sr.size = size;
    sr.data = 0;
    sr.mask = (1UL << size) - 1;
    return sr;
}

// Left shift with new bit input (MSB first)
void shiftLeft(ShiftRegister* sr, bool newBit) {
    sr->data = ((sr->data << 1) | (newBit ? 1 : 0)) & sr->mask;
}

// Right shift with new bit input (LSB first)
void shiftRight(ShiftRegister* sr, bool newBit) {
    sr->data = ((sr->data >> 1) | (newBit ? (1UL << (sr->size - 1)) : 0));
}

// Circular left shift
void rotateLeft(ShiftRegister* sr) {
    bool msb = (sr->data >> (sr->size - 1)) & 1;
    sr->data = ((sr->data << 1) | msb) & sr->mask;
}

// Circular right shift
void rotateRight(ShiftRegister* sr) {
    bool lsb = sr->data & 1;
    sr->data = (sr->data >> 1) | (lsb << (sr->size - 1));
}
