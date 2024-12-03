uint32_t calculateCRC(uint32_t data, uint32_t polynomial, uint8_t width) {
    ShiftRegister sr = initShiftRegister(width);
    sr.data = data;

    for (int i = width - 1; i >= 0; i--) {
        bool msb = (sr.data >> (width - 1)) & 1;
        if (msb) {
            sr.data = (sr.data << 1) ^ polynomial;
        } else {
            sr.data = sr.data << 1;
        }
        sr.data &= sr.mask;
    }

    return sr.data;
}