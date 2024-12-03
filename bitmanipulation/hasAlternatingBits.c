// Check if bits are alternating
bool hasAlternatingBits(uint32_t num) {
    uint32_t temp = num ^ (num >> 1);
    return (temp & (temp + 1)) == 0;
}
