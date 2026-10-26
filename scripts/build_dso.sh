#!/usr/bin/env bash
# Збірка DSO (Direct Sparse Odometry) без вікна для Ubuntu 24.04 / WSL.
# Рецепт той самий, що в notebooks/slam_01_build_once.ipynb (Colab), тільки локально.
# Результат: ~/dso/build/bin/dso_dataset
set -e

# 1. Залежності
sudo apt-get update
sudo apt-get install -y git cmake build-essential pkg-config \
  libeigen3-dev libzip-dev libboost-system-dev libboost-thread-dev libopencv-dev

# 2. Код DSO
cd ~
rm -rf dso
git clone --depth 1 https://github.com/JakobEngel/dso.git
cd dso

# 3. Сумісність з OpenCV 4, GCC 13 і Boost 1.83
sed -i 's/-march=native/-march=x86-64-v2/g' CMakeLists.txt
grep -rl "CV_LOAD_IMAGE\|CV_BGR2GRAY" src | xargs -r sed -i \
  's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g; s/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g; s/CV_LOAD_IMAGE_COLOR/cv::IMREAD_COLOR/g; s/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g'
sed -i '1s/^/#include <boost\/bind\/placeholders.hpp>\nusing namespace boost::placeholders;\n/' src/util/IndexThreadReduce.h

# 4. Версія main без вікна Pangolin
cp src/main_dso_pangolin.cpp src/main_dso_headless.cpp
sed -i 's/.*viewer.*/\/\/ disabled viewer/g; s/.*PangolinDSOViewer.*/\/\/ disabled viewer/g; s/.*pangolin::.*/\/\/ disabled pangolin/g' src/main_dso_headless.cpp

# 5. Спрощений CMakeLists без GUI
#    (*_dummy.cpp виключені: інакше замість OpenCV-читача кадрів береться заглушка,
#     і DSO падає з Segmentation fault в Undistort)
cat > CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(dso)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2 -mpclmul -DRUN_NON_PARALLEL")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED libzip)
include_directories(${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIBZIP_INCLUDE_DIRS} src thirdparty/Sophus)
file(GLOB_RECURSE ALL_SRC "src/*.cpp")
set(DSO_CORE_SRC "")
foreach(f ${ALL_SRC})
  if(NOT f MATCHES "IOWrapper/Pangolin" AND NOT f MATCHES "main_" AND NOT f MATCHES "_dummy")
    list(APPEND DSO_CORE_SRC ${f})
  endif()
endforeach()
add_library(dso ${DSO_CORE_SRC})
target_link_libraries(dso ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LIBZIP_LIBRARIES})
add_executable(dso_dataset src/main_dso_headless.cpp)
target_link_libraries(dso_dataset dso ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LIBZIP_LIBRARIES})
EOF

# 6. Збірка (1 потік, щоб не закінчилась пам'ять)
mkdir -p build && cd build
cmake ..
make -j1 dso_dataset

echo
echo "Готово: $(pwd)/bin/dso_dataset"
