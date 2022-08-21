cmake_minimum_required(VERSION 3.12)

project(STREAM_TEST VERSION 1.0 DESCRIPTION "test for streaming of LoRa data over mqtt")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -lstdc++)
add_definitions(-DLMIC_SX1276)

add_executable(mqtt2LoRaConnector main.cpp MQTTDataStreamer.cpp sx1276.cpp)

target_include_directories (mqtt2LoRaConnector PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/paho.mqtt.cpp/src"
                            "/opt/libraries/boost")

target_link_directories(mqtt2LoRaConnector PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(mqtt2LoRaConnector PUBLIC wiringPi paho-mqttpp3 paho-mqtt3as boost_program_options pthread)




# tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
