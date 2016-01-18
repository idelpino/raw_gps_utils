## corner extraction test
#IF(faramotics_FOUND)
#    ADD_EXECUTABLE(test_extract_corners test_extract_corners.cpp)
#    TARGET_LINK_LIBRARIES(test_extract_corners ${GLUT_glut_LIBRARY} ${pose_state_time_LIBRARIES} ${faramotics_LIBRARIES} ${PROJECT_NAME})
#ENDIF(faramotics_FOUND)

## test
ADD_EXECUTABLE(test_data test_data.cpp)
TARGET_LINK_LIBRARIES(test_data ${PROJECT_NAME})

