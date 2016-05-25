# Build example 1
ADD_EXECUTABLE(example_1 example_1.c)
TARGET_LINK_LIBRARIES(example_1 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example 2
ADD_EXECUTABLE(example_2 example_2.c)
TARGET_LINK_LIBRARIES(example_2 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example 3
ADD_EXECUTABLE(example_3 example_3.cpp)
TARGET_LINK_LIBRARIES(example_3 ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})

# Build example 4
ADD_EXECUTABLE(example_4 example_4.cpp)
TARGET_LINK_LIBRARIES(example_4 ${PROJECT_NAME}_LIB ${PCL_LIBRARIES})

# Build Factory restore
ADD_EXECUTABLE(FactoryRestore FactoryRestore.c)
TARGET_LINK_LIBRARIES(FactoryRestore ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES})
