cmake_minimum_required(VERSION 3.0.2)
project(chapter6_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES chapter6_tutorials
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/chapter6_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/chapter6_tutorials_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
find_package(PCL REQUIRED) 
include_directories(include${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

add_executable(pcl_create src/pcl_create.cpp)
target_link_libraries(pcl_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (pcd_write src/pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES})

add_executable (matrix_transform src/matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

add_executable (normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})

add_executable (pcl_test src/pcl_test.cpp)
target_link_libraries (pcl_test ${PCL_LIBRARIES})

add_executable (narf_feature_extraction src/narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})

add_executable (voxel_grid src/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (viewpcd src/viewpcd.cpp)
target_link_libraries (viewpcd ${PCL_LIBRARIES})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (cylinder_segmentation src/cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})

add_executable (correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})

add_executable (resampling src/resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (cly_seg src/cly_seg.cpp)
target_link_libraries (cly_seg ${PCL_LIBRARIES})

add_executable (cs src/cs.cpp)
target_link_libraries (cs ${PCL_LIBRARIES})


add_executable (global_hypothesis_verification src/global_hypothesis_verification.cpp)
target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})

add_executable (isiolate src/isiolate.cpp)
target_link_libraries (isiolate ${PCL_LIBRARIES})

add_executable (select src/select.cpp)
target_link_libraries (select ${PCL_LIBRARIES})

add_executable (extract_PFH src/extract_PFH.cpp)
target_link_libraries (extract_PFH ${PCL_LIBRARIES})
add_executable (extract_FPFH src/extract_FPFH.cpp)
target_link_libraries (extract_FPFH ${PCL_LIBRARIES})


add_executable (cluster_extraction src/cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})

add_executable (plot_test src/plot_test.cpp)
target_link_libraries (plot_test ${PCL_LIBRARIES})

add_executable (feature_match src/feature_match.cpp)
target_link_libraries (feature_match ${PCL_LIBRARIES})

add_executable (keypoint src/keypoint.cpp)
target_link_libraries (keypoint ${PCL_LIBRARIES})


add_executable (example_get_keypoints_indices src/example_get_keypoints_indices.cpp)
target_link_libraries (example_get_keypoints_indices ${PCL_LIBRARIES})

add_executable (compute_narf src/compute_narf.cpp)
target_link_libraries (compute_narf ${PCL_LIBRARIES})

add_executable (hypothesis_verification src/hypothesis_verification.cpp)
target_link_libraries (hypothesis_verification ${PCL_LIBRARIES})


add_executable (normal_distributions_transform src/normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})

# add_executable (conditional_euclidean_clustering src/conditional_euclidean_clustering.cpp)
# target_link_libraries (conditional_euclidean_clustering ${PCL_LIBRARIES})
# add_executable (feature_matching src/feature_matching.cpp)
# target_link_libraries (feature_matching ${PCL_LIBRARIES})
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chapter6_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
