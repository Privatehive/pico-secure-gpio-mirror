#ifndef __FLASH_HELPER
#define __FLASH_HELPER

#include "hardware/flash.h"

#ifdef __cplusplus
extern "C" {
#endif

    // Call both functions if both cores are used
    void flash_init_core0();
    void flash_init_core1();

    // Use this only if a single core is used
    // you have to define PICO_FLASH_ASSUME_CORE1_SAFE=1 though
    void flash_init();

    // you can read up to FLASH_PAGE_SIZE bytes from the returned address. If there is nothing to read dest will point to NULL.
    void flash_read_page(const uint8_t ** dest);

    // writes a whole page of size FLASH_PAGE_SIZE
    void flash_write_page(const uint8_t * data);

    // you can write up to a size of FLASH_PAGE_SIZE
    void flash_write(const uint8_t * data, size_t count);

    // erase the whole flash sector
    void flash_erase();

#ifdef __cplusplus
}
#endif

#endif