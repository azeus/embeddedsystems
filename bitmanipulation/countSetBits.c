// Count set bits (Brian Kernighan's Algorithm)
uint8_t countSetBits(uint8_t num) {
    uint8_t count = 0;
    while (num) {
        num &= (num - 1);
        count++;
    }
    return count;
}