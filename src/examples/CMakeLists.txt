## corner extraction test
#IF(faramotics_FOUND)
#    ADD_EXECUTABLE(test_extract_corners test_extract_corners.cpp)
#    TARGET_LINK_LIBRARIES(test_extract_corners ${GLUT_glut_LIBRARY} ${pose_state_time_LIBRARIES} ${faramotics_LIBRARIES} ${PROJECT_NAME})
#ENDIF(faramotics_FOUND)

## test
ADD_EXECUTABLE(test_data test_data.cpp)
TARGET_LINK_LIBRARIES(test_data ${PROJECT_NAME})

ADD_EXECUTABLE(test_ecef2lla test_ecef2lla.cpp)
TARGET_LINK_LIBRARIES(test_ecef2lla ${PROJECT_NAME})

ADD_EXECUTABLE(test_transform test_transform.cpp)
TARGET_LINK_LIBRARIES(test_transform ${PROJECT_NAME})

