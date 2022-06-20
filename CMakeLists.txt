cmake_minimum_required(VERSION 3.12)

project(STREAM_TEST VERSION 1.0 DESCRIPTION "test for streaming of LoRa data over mqtt")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -lstdc++)
add_definitions(-DLMIC_SX1276)

add_executable(rpi_lora main.cpp MQTTDataStreamer.cpp sx1276_old.cpp)

target_include_directories (rpi_lora PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/paho.mqtt.cpp/src"
                            "/opt/libraries/boost")

target_link_directories(rpi_lora PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(rpi_lora PUBLIC wiringPi paho-mqttpp3 paho-mqtt3as boost_program_options pthread)



add_executable(test_new_lib test_new_lib.cpp sx1276.cpp)

target_include_directories (test_new_lib PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/boost")

target_link_directories(test_new_lib PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(test_new_lib PUBLIC wiringPi boost_program_options pthread)




add_executable(TestRandomReceive ./tests/TestRandomReceive.cpp sx1276.cpp)

target_include_directories (TestRandomReceive PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/boost")

target_link_directories(TestRandomReceive PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestRandomReceive PUBLIC wiringPi boost_program_options pthread)



add_executable(TestSendPacket ./tests/TestSendPacket.cpp sx1276.cpp)

target_include_directories (TestSendPacket PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/boost")

target_link_directories(TestSendPacket PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestSendPacket PUBLIC wiringPi boost_program_options pthread)



add_executable(TestSendRecvPacket ./tests/TestSendRecvPacket.cpp MQTTDataStreamer.cpp sx1276.cpp sx1276_old.cpp)

target_include_directories (TestSendRecvPacket PUBLIC 
                            "${CMAKE_CURRENT_SOURCE_DIR}"
                            "/opt/libraries/paho.mqtt.cpp/src"
                            "/opt/libraries/boost")

target_link_directories(TestSendRecvPacket PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestSendRecvPacket PUBLIC wiringPi paho-mqttpp3 paho-mqtt3as boost_program_options pthread)

