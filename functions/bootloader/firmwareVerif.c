// Firmware verification example
typedef struct {
    uint32_t crc;
    uint32_t version;
    uint32_t size;
} firmware_header_t;

bool verify_application(void) {
    firmware_header_t* header = (firmware_header_t*)APP_START_ADDRESS;

    // 1. Check size bounds
    if (header->size > MAX_FIRMWARE_SIZE) {
        return false;
    }

    // 2. Verify CRC
    uint32_t calculated_crc = calculate_crc32(
        (uint8_t*)(APP_START_ADDRESS + sizeof(firmware_header_t)),
        header->size
    );

    return (calculated_crc == header->crc);
}