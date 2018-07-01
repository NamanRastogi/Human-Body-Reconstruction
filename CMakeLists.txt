cmake_minimum_required(VERSION 3.5)
project(human_body_reconstruction)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug) # TODO: Remove later
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -w") # TODO: add -Wall and remove -w later
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

# Link to Ceres location
# set(Ceres_DIR "/net/voxel04/misc/me/naman/Softwares/lib/cmake/Ceres")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Link to C++ libraries
# include_directories("/net/voxel04/misc/me/naman/Softwares/include")
# link_directories("/net/voxel04/misc/me/naman/Softwares/lib")


set(SOURCE_FILES_LEARN
        learn.cpp
        global.hpp
        file_operations.hpp
        gl_operations.hpp
        scape_operations_learn.hpp)

add_executable(learn ${SOURCE_FILES_LEARN})
target_link_libraries(learn "-lGL -lGLU -lglut -lassimp -larmadillo -lpca")
target_link_libraries(learn ${CERES_LIBRARIES})


set(SOURCE_FILES_COMPLETE
        complete.cpp
        global.hpp
        file_operations.hpp
        gl_operations.hpp
        scape_operations_complete.hpp)

add_executable(complete ${SOURCE_FILES_COMPLETE})
target_link_libraries(complete "-lGL -lGLU -lglut -lassimp -larmadillo -lpca")
target_link_libraries(complete ${CERES_LIBRARIES})


add_executable(generate_skeleton_manually generate_skeleton_manually.cpp)
target_link_libraries(generate_skeleton_manually "-lGLU -lGL -lglut")


add_executable(generate_point_cloud generate_point_cloud.cpp)
target_link_libraries(generate_point_cloud "-lfltk_gl -lGLU -lGL -lfltk -lXrender -lXfixes -lXext -lXft -lfontconfig -ldl -lX11 -lassimp")
