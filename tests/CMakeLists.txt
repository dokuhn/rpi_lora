

add_executable(TestRandomReceive TestRandomReceive.cpp ../sx1276.cpp)

target_include_directories (TestRandomReceive PUBLIC
                            "../"
                            "/opt/libraries/boost")

target_link_directories(TestRandomReceive PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestRandomReceive PUBLIC wiringPi boost_program_options pthread)


####

add_executable(TestReceivePacket TestReceivePacket.cpp ../sx1276.cpp)

target_include_directories (TestReceivePacket PUBLIC
                            "../"
                            "/opt/libraries/boost")

target_link_directories(TestReceivePacket PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestReceivePacket PUBLIC wiringPi boost_program_options pthread)


####


add_executable(TestSendPacket TestSendPacket.cpp ../sx1276.cpp)

target_include_directories (TestSendPacket PUBLIC
                            "../"
                            "/opt/libraries/boost")

target_link_directories(TestSendPacket PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestSendPacket PUBLIC wiringPi boost_program_options pthread)


####


add_executable(TestSendRecvPacket TestSendRecvPacket.cpp ../MQTTDataStreamer.cpp ../sx1276.cpp)

target_include_directories (TestSendRecvPacket PUBLIC
                            "../"
                            "/opt/libraries/paho.mqtt.cpp/src"
                            "/opt/libraries/boost")

target_link_directories(TestSendRecvPacket PUBLIC "/opt/libraries/boost/stage/lib")

target_link_libraries(TestSendRecvPacket PUBLIC wiringPi paho-mqttpp3 paho-mqtt3as boost_program_options pthread)

