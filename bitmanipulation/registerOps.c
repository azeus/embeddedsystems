void setRegisterBits(volatile uint32_t* reg, uint32_t mask) {
    *reg |= mask;
}

void clearRegisterBits(volatile uint32_t* reg, uint32_t mask) {
    *reg &= ~mask;
}

void toggleRegisterBits(volatile uint32_t* reg, uint32_t mask) {
    *reg ^= mask;
}
