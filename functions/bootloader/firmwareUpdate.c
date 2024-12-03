// Firmware update example
#define FIRMWARE_TEMP_ADDRESS  (FLASH_BASE + 0x20000)

typedef enum {
    UPDATE_SUCCESS = 0,
    UPDATE_ERR_SIZE,
    UPDATE_ERR_CRC,
    UPDATE_ERR_FLASH,
    UPDATE_ERR_VERIFY
} update_status_t;

update_status_t process_firmware_update(void) {
    firmware_header_t temp_header;

    // 1. Receive firmware header
    if (receive_uart_data(&temp_header, sizeof(firmware_header_t)) != sizeof(firmware_header_t)) {
        return UPDATE_ERR_SIZE;
    }

    // 2. Validate header
    if (temp_header.size > MAX_FIRMWARE_SIZE) {
        return UPDATE_ERR_SIZE;
    }

    // 3. Erase flash sectors
    if (!flash_erase_sectors(FIRMWARE_TEMP_ADDRESS, temp_header.size)) {
        return UPDATE_ERR_FLASH;
    }

    // 4. Receive and flash firmware
    uint32_t bytes_received = 0;
    uint8_t buffer[256];

    while (bytes_received < temp_header.size) {
        uint32_t chunk_size = receive_uart_data(buffer, sizeof(buffer));
        if (!flash_write(FIRMWARE_TEMP_ADDRESS + bytes_received, buffer, chunk_size)) {
            return UPDATE_ERR_FLASH;
        }
        bytes_received += chunk_size;
    }

    // 5. Verify CRC
    if (!verify_firmware_crc(FIRMWARE_TEMP_ADDRESS)) {
        return UPDATE_ERR_CRC;
    }

    // 6. Copy to final location
    if (!copy_firmware(FIRMWARE_TEMP_ADDRESS, APP_START_ADDRESS, temp_header.size)) {
        return UPDATE_ERR_FLASH;
    }

    return UPDATE_SUCCESS;
}