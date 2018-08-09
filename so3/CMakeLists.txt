add_executable(so3_test so3_test.cpp)
target_link_libraries(so3_test ${CERES_LIBRARIES} ${GLOG_LIBRARIES} ${GTEST_LIBRARIES})