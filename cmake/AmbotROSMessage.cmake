# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



############### To include ROS ###############
find_package(Boost REQUIRED COMPONENTS
# regex
thread
filesystem   # 我的工程中只使用了 boost 的 filesystem 功能,因此这里只有一个组件
)
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
if(NOT roscpp_FOUND)
	message("Not found roscpp")
endif()
# use shell cmd to get ros version
execute_process(COMMAND rosversion -d
                WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                TIMEOUT 3
                RESULT_VARIABLE result_var
                OUTPUT_VARIABLE rosversion
                ERROR_VARIABLE error_var
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_STRIP_TRAILING_WHITESPACE)
include_directories(${Boost_INCLUDE_DIRS}
	"/opt/ros/${rosversion}/include/"
    )

############### End of including ROS ###############


include(FindOrFetch)


option(MUJOCO_SIMULATE_USE_SYSTEM_MUJOCO "Use installed MuJoCo version." OFF)

find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
if(NOT roscpp_FOUND)
	message("Not found roscpp")
endif()

set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_TESTS OFF)
set(MUJOCO_BUILD_PYTHON OFF)
set(MUJOCO_TEST_PYTHON_UTIL OFF)

findorfetch(
  USE_SYSTEM_PACKAGE
  MUJOCO_SIMULATE_USE_SYSTEM_MUJOCO
  PACKAGE_NAME
  ambot_msgs
  LIBRARY_NAME
  ambot_msgs
  GIT_REPO
  https://github.com/sunzhon/ambot_msgs.git
  GIT_TAG
  master
  TARGETS
  ambot_msgs
  EXCLUDE_FROM_ALL
)
set(AMBOT_MSGS_HEADER  "${ambot_msgs_SOURCE_DIR}/../../devel/include/")
message("ambot msgs ros: ${AMBOT_MSGS_HEADER}") 
#add_subdirectory(${ambot_msgs_SOURCE_DIR})
add_library(ambot_msgs ${ambot_msgs_SOURCE_DIR} )
set_target_properties(ambot_msgs PROPERTIES LINKER_LANGUAGE CXX)
#NOTE, to manually run make at ambot_msgs-build folder to generate .h files at build/devel/include/
#execute_process(COMMAND cd ${ambot_msgs_SOURCE_DIR}/../ambot_msgs-build/ && make)


execute_process(
    COMMAND echo "compile ambot_msgs"
    COMMAND make all
    COMMAND touch text.txt
    WORKING_DIRECTORY ${ambot_msgs_SOURCE_DIR}/../ambot_msgs-build/
    TIMEOUT 20
    RESULT_VARIABLE results
    OUTPUT_VARIABLE outputs
    ERROR_VARIABLE error_var
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
    )
message("results: ${results}, ${outputs}, ${error_var} end.")

#COMMAND rm -rf $<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework
#COMMAND cp -a $<TARGET_FILE_DIR:mujoco::mujoco>/../../../mujoco.framework
#        $<TARGET_FILE_DIR:simulate>/../Frameworks/
## Delete the symlink and the TBD, otherwise we can't sign and notarize.
#COMMAND rm -rf $<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework/mujoco.tbd
#COMMAND rm -rf
    #$<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework/Versions/A/libmujoco.dylib
    

include_directories(${AMBOT_MSGS_HEADER})

#target_compile_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
#target_link_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})

