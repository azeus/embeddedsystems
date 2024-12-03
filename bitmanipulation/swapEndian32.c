uint32_t swapEndian32(uint32_t num) {
    return ((num >> 24) & 0xFF) |
           ((num >> 8)  & 0xFF00) |
           ((num << 8)  & 0xFF0000) |
           ((num << 24) & 0xFF000000);
}