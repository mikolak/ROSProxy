# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)


# Find required packages
FIND_PACKAGE(roscpp)
FIND_PACKAGE(std_msgs)

# Create an executable file from sources:
ADD_LIBRARY(FloatProxy SHARED ${files})


# ROS libraries
SET(ROS_LIBS ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})

# Link external libraries
TARGET_LINK_LIBRARIES(FloatProxy ${DisCODe_LIBRARIES} ${ROS_LIBS})



INSTALL_COMPONENT(FloatProxy)
