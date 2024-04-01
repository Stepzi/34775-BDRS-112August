cmake_minimum_required(VERSION 3.8)
project(raubase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED )
find_package(Threads REQUIRED)
#find_package(libgpiodcxx REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${dlib_INCLUDE_DIR})
execute_process(COMMAND uname -m RESULT_VARIABLE IS_OK OUTPUT_VARIABLE CPU1)
string(STRIP ${CPU1} CPU)
# works for Raspberry 3 and 4
if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
   message("# Is a RASPBERRY; CPU=${CPU} (Pi3=armv7l, pi4=aarch64)")
   #    set(EXTRA_CC_FLAGS " -mfpu=vfp -mfloat-abi=hard -march=armv6zk -mtune=arm1176jzf-s -DRASPBERRY_PI -D${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O2 -g0 -DRASPBERRY_PI -I/home/local/git/CLI11/include")
   #set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2 -DRASPBERRY_PI")
else()
   message("# Not a RASPBERRY; CPU=${CPU}")
   set(EXTRA_CC_FLAGS "-D${CPU} -O0 -g2")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic \
    -Wno-format-truncation -Wno-return-type \
    -std=c++20 ${EXTRA_CC_FLAGS}")
set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-pthread")


add_executable(raubase
      src/bplanIRTEST.cpp
      src/bplanCrossMission.cpp
      src/bplanGate.cpp
      src/bplan20.cpp
      src/bplan21.cpp
      src/bplan40.cpp
      src/bplan100.cpp
      src/bseesaw.cpp
      src/bmission0.cpp
      src/bracetrack.cpp
      src/baxe.cpp
      src/bplan101.cpp
      src/cedge.cpp
      src/bStairs.cpp
      src/cheading.cpp
      src/cmixer.cpp
      src/cmotor.cpp
      src/cservo.cpp
      src/main.cpp
      src/maruco.cpp
      src/medge.cpp
      src/mpose.cpp
      src/mgolfball.cpp
      src/scam.cpp
      src/sedge.cpp
      src/sencoder.cpp
      src/sgpiod.cpp
      src/simu.cpp
      src/sdist.cpp
      src/sjoylogitech.cpp
      src/spyvision.cpp
      src/sstate.cpp
      src/steensy.cpp
      src/upid.cpp
      src/uservice.cpp
      src/usocket.cpp
      src/utime.cpp
      )

if (${CPU} MATCHES "armv7l" OR ${CPU} MATCHES "aarch64")
  target_link_libraries(raubase ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod rt)
else()
  target_link_libraries(raubase ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} readline gpiod)
endif()

