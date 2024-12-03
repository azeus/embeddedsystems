// Get lowest set bit
uint32_t getLowestSetBit(uint32_t num) {
    return num & (-num);
}