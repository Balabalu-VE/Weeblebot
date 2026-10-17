cmake_minimum_required(VERSION 3.8)
project(camera_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

# Find Tesseract and Leptonica headers & libraries directly
find_path(TESSERACT_INCLUDE_DIR tesseract/baseapi.h)
find_library(TESSERACT_LIBRARY tesseract)

find_path(LEPTONICA_INCLUDE_DIR leptonica/allheaders.h)
find_library(LEPTONICA_LIBRARY lept)

add_executable(detect_room_sign src/detect_room_sign.cpp)
target_include_directories(detect_room_sign PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${TESSERACT_INCLUDE_DIRS}
  ${LEPTONICA_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(detect_room_sign
  ${OpenCV_LIBS}
  ${TESSERACT_LIBRARIES}
  ${LEPTONICA_LIBRARIES}
)

ament_target_dependencies(detect_room_sign
  rclcpp
  std_msgs
)

target_compile_features(detect_room_sign PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  detect_room_sign
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
