set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13...3.27)

project(pico-secure-gpio-mirror)

pico_sdk_init()

add_subdirectory("src")
