cmake_minimum_required(VERSION 3.12)

project(STREAM_TEST VERSION 1.0 DESCRIPTION "test for streaming of LoRa data over mqtt")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -lstdc++)
add_definitions(-DLMIC_SX1276)

add_executable(dragino_lora_app main.cpp MQTTDataStreamer.cpp sx1276.cpp)

target_include_directories (dragino_lora_app PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/home/pi/paho.mqtt.cpp/src"
                            "/home/pi/boost")

target_link_directories(dragino_lora_app PUBLIC "/home/pi/boost/stage/lib")

target_link_libraries(dragino_lora_app PUBLIC wiringPi paho-mqttpp3 paho-mqtt3as boost_program_options pthread)