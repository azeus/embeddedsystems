uint8_t setBitField(uint8_t num, uint8_t value, uint8_t position, uint8_t width) {
    uint8_t mask = ((1 << width) - 1) << position;
    return (num & ~mask) | ((value << position) & mask);
}

uint8_t getBitField(uint8_t num, uint8_t position, uint8_t width) {
    return (num >> position) & ((1 << width) - 1);
}
