uint8_t check_endianness(void) {
    uint32_t num = 1;
    return (*(uint8_t *)&num == 1) ? LITTLE_ENDIAN : BIG_ENDIAN;
}