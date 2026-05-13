cmake_minimum_required(VERSION 3.10)
project(ArUcoTracker)

# Wymuszamy standard C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Szukamy biblioteki OpenCV w systemie (bez hardkodowania ścieżek!)
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION} found.")
else()
    message(FATAL_ERROR "OpenCV not found. Please set OpenCV_DIR.")
endif()

# Definiujemy plik wykonywalny (zakładam, że main.cpp jest w folderze src)
add_executable(ArUcoTracker src/main.cpp)

# Dołączamy nagłówki i linkujemy biblioteki
target_include_directories(ArUcoTracker PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ArUcoTracker PRIVATE ${OpenCV_LIBS})