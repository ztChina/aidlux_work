cmake_minimum_required(VERSION 3.12)
project(aidlite_cpp_samples)

set(CMAKE_BUILD_TYPE Release)

set(OPENCV_INCLUDE_DIR      /usr/include/opencv4)
set(OPENCV_LINK_DIR         "")
set(OPENCV_LIBS opencv_imgcodecs opencv_imgproc opencv_core) # 如果是静态库  需要注意先后顺序

set(AIDLITE_INCLUDE_DIR     /usr/local/include)
set(AIDLITE_LINK_DIR        /usr/local/lib)
set(AIDLITE_LIB aidlite)

function(func_generate_sample_exe sample_name)
    
    set(demo_name ${sample_name})

    file(GLOB src_files ${CMAKE_CURRENT_SOURCE_DIR}/${demo_name}.cpp)

    add_executable(${demo_name} ${src_files})

    target_compile_options(${demo_name} PRIVATE -std=c++11)
    target_include_directories(${demo_name} PUBLIC ${OPENCV_INCLUDE_DIR} ${AIDLITE_INCLUDE_DIR})
    target_link_directories(${demo_name} PUBLIC ${OPENCV_LINK_DIR} ${AIDLITE_LINK_DIR})
    target_link_libraries(${demo_name} PUBLIC ${AIDLITE_LIB} ${OPENCV_LIBS} pthread)
    message(STATUS "[CMAKEMSG] ${demo_name} need libraries is : ${AIDLITE_LIB} ${OPENCV_LIBS}")

endfunction()

set(SAMPLE_LIST qnn_yolov5_multi)

FOREACH(sample ${SAMPLE_LIST})
    message("prepare to generate cpp sample : ${sample}")

    func_generate_sample_exe(${sample})
ENDFOREACH(sample)