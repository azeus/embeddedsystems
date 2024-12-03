int findRightmostSetBit(uint32_t num) {
    if (num == 0) return -1;
    return __builtin_ffs(num) - 1;  // GCC built-in function
}