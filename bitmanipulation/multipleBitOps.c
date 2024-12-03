uint8_t setMultipleBits(uint8_t num, uint8_t mask) {
    return num | mask;
}

uint8_t clearMultipleBits(uint8_t num, uint8_t mask) {
    return num & ~mask;
}