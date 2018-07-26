include(ExternalProject)

SET(THIRDPARTYLIB_DIR "${PROJECT_SOURCE_DIR}/3rdparty")

ExternalProject_Add(gflags_autobuild
        GIT_REPOSITORY https://github.com/gflags/gflags
        GIT_TAG 46f73f88b18aee341538c0dfc22b1710a6abedef
        SOURCE_DIR gflags_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_FLAGS=-fPIC
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DBUILD_SHARED_LIBS=ON
        -DBUILD_PACKAGING=OFF
        -DBUILD_TESTING=OFF
        ../../../gflags_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(glog_autobuild
        GIT_REPOSITORY https://github.com/google/glog.git
        GIT_TAG a6a166db069520dbbd653c97c2e5b12e08a8bb26
        SOURCE_DIR glog_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DBUILD_SHARED_LIBS=ON
        -DBUILD_TESTING=OFF
        ../../../glog_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(eigen_autobuild
        GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
        GIT_TAG dde02fceedfc1ba09d4d4f71a2b5dafcfcb85491
        SOURCE_DIR eigen_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        ../../../eigen_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(protobuf3_autobuild
        GIT_REPOSITORY https://github.com/google/protobuf.git
        GIT_TAG 106ffc04be1abf3ff3399f54ccf149815b287dd9
        SOURCE_DIR protobuf3_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -Dprotobuf_BUILD_TESTS=OFF
        -Dprotobuf_BUILD_EXAMPLES=OFF
        -DBUILD_SHARED_LIBS=ON
        ../../../protobuf3_src/cmake/
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(Sophus_autobuild
        GIT_REPOSITORY https://github.com/strasdat/Sophus.git
        GIT_TAG 13fb3288311485dc94e3226b69c9b59cd06ff94e
        SOURCE_DIR Sophus_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DBUILD_TESTS=OFF
        ../../../Sophus_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(vtk_autobuild
        URL https://gitlab.kitware.com/vtk/vtk/-/archive/v8.0.1/vtk-v8.0.1.tar.gz
        URL_MD5 09772c59d5b2fb759d77212a4dce1c20
        SOURCE_DIR vtk_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DVTK_FORBID_DOWNLOADS=ON
        -DBUILD_TESTING=OFF
        ../../../vtk_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(pcl_autobuild
        URL https://codeload.github.com/PointCloudLibrary/pcl/tar.gz/pcl-1.8.1
        URL_MD5 436704215670bb869ca742af48c749a9
        SOURCE_DIR pcl_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DWITH_DAVIDSDK=OFF
        -DWITH_DOCS=OFF
        -DWITH_DSSDK=OFF
        -DWITH_ENSENSO=OFF
        -DWITH_TUTORIALS=OFF
        -DWITH_RSSDK=OFF
        -DBUILD_SHARED_LIBS=ON
        ../../../pcl_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(Super4PCS_autobuild
        GIT_REPOSITORY https://github.com/nmellado/Super4PCS.git
        GIT_TAG 7971c7fab0ffcbe9a2a4a517c0211edc37ba7af8
        SOURCE_DIR Super4PCS_src
        PATCH_COMMAND git apply ${PROJECT_SOURCE_DIR}/cmake_modules/patches/super4pcs_log_patch.diff
        CONFIGURE_COMMAND cmake
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_STANDARD=11
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DBUILD_SHARED_LIBS=ON
        -DSUPER4PCS_COMPILE_TESTS=FALSE
        -DSUPER4PCS_COMPILE_DEMOS=FALSE
        -DIO_USE_OPENCV=FALSE
        -DENABLE_TIMING=FALSE
        ../../../Super4PCS_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

ExternalProject_Add(ceres_autobuild
        GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
        GIT_TAG facb199f3eda902360f9e1d5271372b7e54febe1
        SOURCE_DIR ceres_src
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND cmake
        -DGFLAGS=ON
        -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
        -DBUILD_DOCUMENTATION=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_TESTING=OFF
        -DCMAKE_INSTALL_PREFIX=${THIRDPARTYLIB_DIR}
        -DCMAKE_INSTALL_RPATH=${THIRDPARTYLIB_DIR}/lib
        -DBUILD_SHARED_LIBS=ON
        -DCMAKE_VERBOSE_MAKEFILE=ON
        -DCMAKE_BUILD_TYPE:STRING=Release
        -DCMAKE_CXX_FLAGS=-fPIC
        -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON
        -DCMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY=ON
        -DCMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY=ON
        ../../../ceres_src
        BUILD_COMMAND make -j${BUILD_3RDPARTY_THREADS}
        INSTALL_COMMAND make install -j${BUILD_3RDPARTY_THREADS}
        )

add_dependencies(glog_autobuild gflags_autobuild)

add_dependencies(ceres_autobuild eigen_autobuild gflags_autobuild glog_autobuild)

add_dependencies(pcl_autobuild eigen_autobuild vtk_autobuild)

add_custom_target(BuildThirdPartyLib
        DEPENDS
        gflags_autobuild
        glog_autobuild
        ceres_autobuild
        Sophus_autobuild
        )

add_custom_command(TARGET BuildThirdPartyLib
        POST_BUILD
        COMMAND  ${CMAKE_COMMAND} -E echo ${THIRD_PARTY_BUILT_VER} > ${THIRD_PARTY_BUILD_VER_FILE}
        COMMENT "Set built flag"
        )