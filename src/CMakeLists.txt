add_executable(blink_simple
        main.c
        flash.c
        portable8439.h
        portable8439.c
)

# pull in common dependencies
target_link_libraries(blink_simple pico_stdlib pico_status_led pico_multicore pico_rand pico_atomic)

#pico_enable_stdio_usb(blink_simple 1)
#pico_enable_stdio_uart(blink_simple 0)

set(INITIAL_KEY "0xe6;0x02;0xc9;0x20;0xcb;0x00;0xb6;0x92;0x40;0x16;0x00;0xce;0xdd;0x65;0x95;0x5c;0x24;0x32;0x61;0x2e;0xbb;0xc6;0xf8;0x5c;0x9f;0x2f;0x46;0x8a;0x2d;0x9c;0x71;0x74" CACHE STRING "The key that is used once for the first key exchange")

list(LENGTH INITIAL_KEY INITIAL_KEY_LENGTH)
if (INITIAL_KEY_LENGTH EQUAL 32)
    string(REPLACE ";" ", " INITIAL_KEY_DEFINE "${INITIAL_KEY}")
else ()
    message(FATAL_ERROR "INITIAL_KEY list must have a size of 32 entries (which equals 64 bit)")
endif ()

target_compile_definitions(blink_simple PRIVATE INITIAL_KEY=${INITIAL_KEY_DEFINE})

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(blink_simple)

pico_set_program_version(blink_simple "0.0.0")
pico_set_program_description(blink_simple "Mirrors the GPIO pins between two Raspberry Pi Picos via an encrypted protocol.")
pico_set_program_url(blink_simple "https://github.com/Privatehive/pico-secure-gpio-mirror")