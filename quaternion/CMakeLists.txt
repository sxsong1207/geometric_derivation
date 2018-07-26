add_executable(quaternion_test quaternion_test.cpp)
target_link_libraries(quaternion_test ${CERES_LIBRARIES} ${GLOG_LIBRARIES} ${GTEST_LIBRARIES})