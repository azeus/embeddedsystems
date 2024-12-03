uint8_t setBit(uint8_t num, uint8_t position) {
    return num | (1 << position);
}

uint8_t clearBit(uint8_t num, uint8_t position) {
    return num & ~(1 << position);
}

uint8_t toggleBit(uint8_t num, uint8_t position) {
    return num ^ (1 << position);
}

bool getBit(uint8_t num, uint8_t position) {
    return (num >> position) & 1;
}
