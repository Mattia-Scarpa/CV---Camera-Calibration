# CMakeList.txt: progetto CMake. Includere l'origine e definire
# qui la logica specifica del progetto.

cmake_minimum_required (VERSION 3.8)

project ("calibration_main")

# Aggiungere l'origine all'eseguibile di questo progetto.

add_executable (calibration_main "src/main.cpp")

# Add tests and, if necessary, install destinations.
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
