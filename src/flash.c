#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "hardware/sync.h"
#include "pico/flash.h"
#include "pico/mem_ops.h"
#include "flash.h"

// Set the target offest to the last sector of flash. Must be aligned with FLASH_SECTOR_SIZE. Make sure to not overwrite the addresses where the program is located.
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

int first_empty_page, first_non_empty_page = -1;
const int num_pages = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;
volatile int32_t flash_init_done = 0;

void flash_init_core0() {

  flash_safe_execute_core_init();
  while (1) if (atomic_load(&flash_init_done) == 1) break;
  flash_init();
  atomic_store(&flash_init_done, 2);
}

void flash_init_core1() {

  flash_safe_execute_core_init();
  atomic_store(&flash_init_done, 1);
  while (1) if (atomic_load(&flash_init_done) == 2) break;
}

void flash_init() {

  for (unsigned int page = 0; page < num_pages; page++) {
    unsigned int *page_addr = (unsigned int *)(XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE));
    // Iterate the page and check if it is empty
    bool empty_page = true;
    for (unsigned int page_offset = 0; page_offset < FLASH_PAGE_SIZE / sizeof(int); page_offset++) {
      int *page_content = (int *)(page_addr + page_offset);
      if (*page_content != -1) {
        empty_page = false;
        break;
      }
    }
    if (empty_page) {
      first_empty_page = page;
      break;
    }
    first_non_empty_page = page;
  }
}

static void call_flash_range_erase(void *param) {

  uint32_t offset = (uint32_t)param;
  flash_range_erase(offset, FLASH_SECTOR_SIZE);
}

static void call_flash_range_program(void *param) {

  uint32_t offset = ((uintptr_t*)param)[0];
  const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
  flash_range_program(offset, data, FLASH_PAGE_SIZE);
}

void flash_read_page(const uint8_t ** dest) {

  if (first_non_empty_page < 0 || first_non_empty_page >= num_pages) {
    *dest = NULL;
  } else {
    const uint32_t mapped_flash_addr = (XIP_BASE + FLASH_TARGET_OFFSET + (first_non_empty_page * FLASH_PAGE_SIZE));
    *dest = (const uint8_t *)mapped_flash_addr;
  }
}

void flash_erase() {

  const int result = flash_safe_execute(call_flash_range_erase, (void*)FLASH_TARGET_OFFSET, UINT32_MAX);
  hard_assert(result == PICO_OK);
  first_empty_page = 0;
  first_non_empty_page = -1;
}

void flash_write(const uint8_t * data, size_t count) {

  uint8_t page[FLASH_PAGE_SIZE] = {0};
  memcpy(page, data, count);
  flash_write_page(page);
}

// data must be of size FLASH_PAGE_SIZE
void flash_write_page(const uint8_t * data) {

  if (first_empty_page < 0 || first_empty_page >= num_pages) {
    flash_erase();
  }

  const uint32_t flash_addr = FLASH_TARGET_OFFSET + (first_empty_page * FLASH_PAGE_SIZE);
  const uint32_t mapped_flash_addr = XIP_BASE +  FLASH_TARGET_OFFSET + (first_empty_page * FLASH_PAGE_SIZE);

  //printf("writing page to flash address %x (cpu address %x)", flash_addr, mapped_flash_addr);

  uintptr_t params[] = { flash_addr , (uintptr_t)data};
  const int result = flash_safe_execute(call_flash_range_program, params, UINT32_MAX);
  hard_assert(result == PICO_OK);

  first_non_empty_page = first_empty_page;
  first_empty_page += 1;
}
