// Check if number is power of 2
bool isPowerOfTwo(uint32_t num) {
    return num && !(num & (num - 1));
}
