/**
* Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "pico/status_led.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"
#include "pico/rand.h"
#include "hardware/gpio.h"
#include "flash.h"

#include "portable8439.h"

//----------------change if desired----------------
#define UART_ID uart1
#define BAUD_RATE 115200 // must match the spects of an rs-485
// https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_gpio_1autotoc_md0
#define UART_TX_PIN 4 // GPIO pin must support the selected uart interface selected via "UART_ID"
#define UART_RX_PIN 5 // GPIO pin must support the selected uart interface selected via "UART_ID"
#define GPIO_PRIMARY_SECONDARY_PIN 3 // GPIO pin either pulled to gnd or open. If pulled to ground the device will run as Primary. If open the device will run as Secondary.



//#define DBG_LOG
//-------------------------------------------------

#define UART_FIFO_SIZE 4 // not documented
#define MICROSECONDS 1
#define MILLISECONDS 1000
#define SECONDS (MILLISECONDS * 1000)

// | 4 Byte (magic number) | 1 Byte (message type) | 7 Byte (unused) | 12 Byte (nonce) | 32 Byte (encrypted payload) |
#define MESSAGE_MAGICK_NUMBER_SIZE 4
#define MESSAGE_HEADER_SIZE (MESSAGE_MAGICK_NUMBER_SIZE + 8 + RFC_8439_NONCE_SIZE) // 24
#define MESSAGE_PAYLOAD_SIZE 32
#define MESSAGE_PAYLOAD_ENCRYPTED_SIZE (MESSAGE_PAYLOAD_SIZE + RFC_8439_TAG_SIZE) // 48
#define MESSAGE_OVERALL_ENCRYPTED_SIZE (MESSAGE_HEADER_SIZE + MESSAGE_PAYLOAD_ENCRYPTED_SIZE) // 72

#ifdef NDEBUG
#undef DBG_LOG
#endif

// This key is used only once - when two devices perform the first handshake and exchange new keys. From then on this key is never used again.
uint8_t initial_key[RFC_8439_KEY_SIZE] = {INITIAL_KEY};
const uint8_t message_magic_number[MESSAGE_MAGICK_NUMBER_SIZE] = {0xff, 0x89, 0xff, 0x27};

#define UART_SYMBOL_SIZE_BIT 11 // start bit + data bits + stop bits + parity bit
const double baud_period_s = 1. / BAUD_RATE;
const uint32_t symbol_duration_us = 1 * UART_SYMBOL_SIZE_BIT * baud_period_s * 1000000;
const uint32_t clearance_delay_us = 2 * UART_SYMBOL_SIZE_BIT * baud_period_s * 1000000; // silence between two messages
const uint32_t message_tx_duration_us = 1 * MESSAGE_OVERALL_ENCRYPTED_SIZE * UART_SYMBOL_SIZE_BIT * baud_period_s *
                                        1000000; // time that elapses between two messages (from start to start)
const uint32_t message_interval_us = message_tx_duration_us + clearance_delay_us + symbol_duration_us;

pico_unique_board_id_t board_id;
volatile bool is_primary = false;
uint8_t internal_nonce[RFC_8439_NONCE_SIZE] = {0};

// stats
uint32_t message_read_desync_counter = 0;
uint32_t message_read_timeout_counter = 0;
uint32_t message_write_timeout_counter = 0;
uint32_t message_encrypt_error_counter = 0;
uint32_t message_decrypt_error_counter = 0;
uint32_t message_decode_error_counter = 0;
uint32_t missed_response_slot_counter = 0;

typedef enum MessageType {
    ERR = 0x00, // error indicating sth. went wrong - a message was not sent or received
    NOP = 0x01, // Primary/Secondary empty message no payload is expected
    KEY = 0x02, // Primary key exchange message
    ACK = 0x03, // Secondary key exchange acknowledge message
    CHG = 0x04, // Primary/Secondary io change message
} MessageType;

const uint8_t *get_shuffle_nonce() {
    uint32_t rand;
    rand = get_rand_32();
    memcpy(internal_nonce, &rand, 4);
    rand = get_rand_32();
    memcpy(internal_nonce + 4, &rand, 4);
    rand = get_rand_32();
    memcpy(internal_nonce + 8, &rand, 4);
    return internal_nonce;
}

void gen_rand_key(uint8_t key[RFC_8439_KEY_SIZE]) {

    uint64_t rand;
    rand = get_rand_64();
    memcpy(key, &rand, 8);
    rand = get_rand_64();
    memcpy(key + 8, &rand, 8);
    rand = get_rand_64();
    memcpy(key + 16, &rand, 8);
    rand = get_rand_64();
    memcpy(key + 24, &rand, 8);
}

void drain_uart_rx_fifo() {
    while (uart_is_readable(UART_ID)) uart_getc(UART_ID);
}

void print_stats() {
    printf(
        "----------------stats----------------\nmessage_read_desync_counter: %ld\nmessage_read_timeout_counter: %ld\nmessage_write_timeout_counter: %ld\nmessage_encrypt_error_counter: %ld\nmessage_decrypt_error_counter: %ld\nmessage_decode_error_counter: %ld\nmissed_response_slot_counter: %ld\n",
        atomic_load(&message_read_desync_counter), atomic_load(&message_read_timeout_counter),
        atomic_load(&message_write_timeout_counter), atomic_load(&message_encrypt_error_counter),
        atomic_load(&message_decrypt_error_counter), atomic_load(&message_decode_error_counter),
        atomic_load(&missed_response_slot_counter));
}

void reset_stats() {
    atomic_store(&message_read_desync_counter, 0);
    atomic_store(&message_read_timeout_counter, 0);
    atomic_store(&message_write_timeout_counter, 0);
    atomic_store(&message_encrypt_error_counter, 0);
    atomic_store(&message_decrypt_error_counter, 0);
    atomic_store(&message_decode_error_counter, 0);
    atomic_store(&missed_response_slot_counter, 0);
}

// Reads all data that is currently available in the uart fifo without waiting for new data. The fifo can hold up to UART_FIFO_SIZE
size_t read_uart_rx_fifo(uint8_t buf[UART_FIFO_SIZE]) {
    size_t bytes_read = 0;
    for (int i = 0; i < UART_FIFO_SIZE; i++) {
        if (uart_is_readable(UART_ID)) {
            buf[i] = uart_getc(UART_ID);
            bytes_read++;
        } else {
            break;
        }
    }
    return bytes_read;
}

// Ensures a complete message is read from uart. As soon as we return from this function and true is returned we are synced to the message stream. If false is retuned the sync is lost, and we are reading messages partially or not at all
bool read_raw_message(uint8_t msg[MESSAGE_OVERALL_ENCRYPTED_SIZE]) {
    bool is_timeout = false;
    const uint32_t timeout = message_interval_us + message_interval_us;
    unsigned offset = 0;
    const uint32_t time_start = time_us_32();
    do {
        if (uart_is_readable(UART_ID)) {
            msg[offset] = uart_getc(UART_ID);
            if (offset < MESSAGE_MAGICK_NUMBER_SIZE) {
                // check if magick number matches
                if (msg[offset] != message_magic_number[offset]) {
#ifdef DBG_LOG
                    printf("[D] invalid magick number detected - message desync\n");
#endif
                    drain_uart_rx_fifo();
                    // mismatch found, we are probably reading the message not from the beginning - return immediately so we have a chance to read the next message completely
                    atomic_fetch_add(&message_read_desync_counter, 1);
                    return false;
                }
            }
            offset++;
        }
        is_timeout = time_us_32() - time_start > timeout;
    } while (!is_timeout && offset < MESSAGE_OVERALL_ENCRYPTED_SIZE);
    if (is_timeout) {
#ifdef DBG_LOG
        printf("[D] read message timeout occurred\n");
#endif
        atomic_fetch_add(&message_read_timeout_counter, 1);
        drain_uart_rx_fifo();
    }
    return offset == MESSAGE_OVERALL_ENCRYPTED_SIZE;
}

bool write_raw_message(uint8_t msg[MESSAGE_OVERALL_ENCRYPTED_SIZE]) {
    bool is_timeout = false;
    const uint32_t timeout = message_tx_duration_us;
    unsigned offset = 0;
    const uint32_t time_start = time_us_32();
    do {
        uart_putc_raw(UART_ID, msg[offset]);
        offset++;
        is_timeout = time_us_32() - time_start > timeout;
    } while (!is_timeout && offset < MESSAGE_OVERALL_ENCRYPTED_SIZE);
#ifdef DBG_LOG
    printf("[D] wrote message within %lu us\n", time_us_32() - time_start);
#endif
    uart_tx_wait_blocking(UART_ID);
#ifdef DBG_LOG
    printf("[D] send message within %lu us\n", time_us_32() - time_start);
#endif
    if (is_timeout) {
#ifdef DBG_LOG
        printf("[D] write message timeout occurred\n");
#endif
        atomic_fetch_add(&message_write_timeout_counter, 1);
    }
    return offset == MESSAGE_OVERALL_ENCRYPTED_SIZE;
}

void encode_raw_message(const MessageType type, const uint8_t payload[static MESSAGE_PAYLOAD_SIZE],
                        uint8_t msg[static MESSAGE_OVERALL_ENCRYPTED_SIZE]) {
    const uint32_t time_start = time_us_32();
    memcpy(msg, message_magic_number, MESSAGE_MAGICK_NUMBER_SIZE);
    memcpy(msg + MESSAGE_MAGICK_NUMBER_SIZE, &type, 1);
    const uint8_t *nonce = get_shuffle_nonce();
    memcpy(msg + MESSAGE_HEADER_SIZE - RFC_8439_NONCE_SIZE, nonce, RFC_8439_NONCE_SIZE);
    const size_t cypher_size = portable_chacha20_poly1305_encrypt(msg + MESSAGE_HEADER_SIZE, initial_key, nonce, NULL,
                                                                  0, payload, MESSAGE_PAYLOAD_SIZE);
    if (cypher_size == -1u) {
        printf("[E] encryption of message payload failed\n");
        atomic_fetch_add(&message_encrypt_error_counter, 1);
    }
    hard_assert(cypher_size == MESSAGE_PAYLOAD_ENCRYPTED_SIZE);
#ifdef DBG_LOG
    printf("[D] message encoding took %lu us\n", time_us_32() - time_start);
#endif
}

MessageType decode_raw_message(const uint8_t msg[static MESSAGE_OVERALL_ENCRYPTED_SIZE],
                               uint8_t payload[static MESSAGE_PAYLOAD_SIZE]) {
    const uint32_t time_start = time_us_32();
    MessageType ret = ERR;
    if (memcmp(msg, message_magic_number, MESSAGE_MAGICK_NUMBER_SIZE) == 0) {
        MessageType type = msg[4];
        uint8_t nonce[RFC_8439_NONCE_SIZE] = {0};
        memcpy(nonce, msg + MESSAGE_HEADER_SIZE - RFC_8439_NONCE_SIZE, RFC_8439_NONCE_SIZE);
        const size_t plaintext_size = portable_chacha20_poly1305_decrypt(
            payload, initial_key, nonce, NULL, 0, msg + MESSAGE_HEADER_SIZE,
            MESSAGE_PAYLOAD_ENCRYPTED_SIZE);
        if (plaintext_size == -1u) {
            printf("[E] decryption of message payload failed\n");
            atomic_fetch_add(&message_decrypt_error_counter, 1);
        } else {
            ret = type;
        }
    } else {
        printf("[E] message does not begin with expected magic number\n");
        atomic_fetch_add(&message_decode_error_counter, 1);
    }
#ifdef DBG_LOG
    printf("[D] message decoding took %lu us\n", time_us_32() - time_start);
#endif
    return ret;
}

// used by Primary: Sends a single message via uart as Half duplex: only a one line (TX or RX) will be driven never both at the same time, so we can use a TTL to RS-485 adapter.
MessageType send_receive_msg(MessageType send_type, uint8_t payload[static MESSAGE_PAYLOAD_SIZE]) {
    MessageType ret = ERR;
    uint8_t raw_msg[MESSAGE_OVERALL_ENCRYPTED_SIZE] = {0};
    // Primary can transmit immediately after the clearance delay
    while (uart_is_readable_within_us(UART_ID, clearance_delay_us)) {
        // drain fifo
        drain_uart_rx_fifo();
    }
    encode_raw_message(send_type, payload, raw_msg);
    if (write_raw_message(raw_msg)) {
        if (read_raw_message(raw_msg)) {
            // we got a raw message from Secondary - decode message
            ret = decode_raw_message(raw_msg, payload);
        }
    }
    return ret;
}

// used by Secondary
MessageType receive_send_msg(MessageType send_type, uint8_t payload[static MESSAGE_PAYLOAD_SIZE]) {
    MessageType ret = ERR;
    uint8_t raw_msg[MESSAGE_OVERALL_ENCRYPTED_SIZE] = {0};
    // Secondary has to wait for Primary to finish transferring a complete message
    if (read_raw_message(raw_msg)) {
        const uint32_t time_start = time_us_32();
        // we got a raw message from Primary - decode message
        uint8_t tmp_payload[MESSAGE_PAYLOAD_SIZE] = {0};
        ret = decode_raw_message(raw_msg, tmp_payload);
        if (ret != ERR) {
            // Secondary write message
            encode_raw_message(send_type, payload, raw_msg);
            sleep_us(MAX((int32_t)symbol_duration_us - (int32_t)(time_us_32() - time_start), 0));
            // delay the response - some rs-485 may require some time switch from rx to tx.
            if (clearance_delay_us - time_us_32() - time_start > 0) {
                write_raw_message(raw_msg);
                memcpy(payload, tmp_payload, MESSAGE_PAYLOAD_SIZE);
            } else {
                printf("[E] Secondary missed the response slot\n");
                atomic_fetch_add(&missed_response_slot_counter, 1);
                ret = ERR;
            }
        }
    }
    return ret;
}

// blocks till the key exchange finished
void key_exchange(uint8_t key[RFC_8439_KEY_SIZE]) {
    MessageType result = ERR;
    if (is_primary) {
        uint8_t tmp_key[32] = {0};
        memcpy(tmp_key, key, RFC_8439_KEY_SIZE); // avoid override of key
        while (result != ACK) result = send_receive_msg(KEY, tmp_key);
        printf("[I] Secondary acknowledged new key\n");
    } else {
        while (result != KEY) result = receive_send_msg(ACK, key);
        printf("[I] Primary provided new key\n");
    }
}

void core1_entry() {

    flash_init_core1();

    flash_erase();

    const uint8_t *page;
    flash_read_page(&page);

    if (page) {
        // key exchange already finished - load key from flash
        memcpy(initial_key, page, RFC_8439_KEY_SIZE);
    } else {
        // perform key exchange
        uint8_t new_key[RFC_8439_KEY_SIZE] = {0};
        if (is_primary) gen_rand_key(new_key);
        key_exchange(new_key);
        // write key
        flash_write(new_key, RFC_8439_KEY_SIZE);
        memcpy(initial_key, new_key, RFC_8439_KEY_SIZE);
        if (is_primary) sleep_us(1 *SECONDS);
    }

    reset_stats();

    while (1) {
        // normal operation
        uint8_t dummy_payload[32] = {"hello"};
        if (is_primary) send_receive_msg(NOP, dummy_payload);
        else receive_send_msg(NOP, dummy_payload);
    }
}


int main() {
    stdio_init_all();

    pico_get_unique_board_id(&board_id);

    // enable uart
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    uart_init(UART_ID, BAUD_RATE);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_EVEN);
    uart_set_translate_crlf(UART_ID, false);

    // determine if Primary or Secondary
    gpio_init(GPIO_PRIMARY_SECONDARY_PIN);
    gpio_set_pulls(GPIO_PRIMARY_SECONDARY_PIN, true, false);
    gpio_set_dir(GPIO_PRIMARY_SECONDARY_PIN, GPIO_IN);

    sleep_us(10); // wait for pull up voltage to settle

    is_primary = !gpio_get(GPIO_PRIMARY_SECONDARY_PIN);

    if (is_primary) printf("[I] Running as Primary\n");
    else printf("[I] Running as Secondary\n");

    multicore_launch_core1(core1_entry);
    flash_init_core0();

    bool led_on = false;
    status_led_init();

    uint32_t time_start = time_us_32();

    while (1) {
        status_led_set_state(led_on);
        print_stats();
        led_on = !led_on;
        sleep_us(1*SECONDS);
    }
}
