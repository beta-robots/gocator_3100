# Build example 1
ADD_EXECUTABLE(example_1 example_1.c)
TARGET_LINK_LIBRARIES(example_1 GoSdk kApi)
add_dependencies(example_1 GoSdk kApi)

# Build example 2
ADD_EXECUTABLE(example_2 example_2.c)
TARGET_LINK_LIBRARIES(example_2 GoSdk kApi)
add_dependencies(example_2 GoSdk kApi)

# Build example 3
ADD_EXECUTABLE(example_3 example_3.cpp)
TARGET_LINK_LIBRARIES(example_3 GoSdk kApi ${PCL_LIBRARIES})
add_dependencies(example_3 GoSdk kApi)

# Build example 4
ADD_EXECUTABLE(example_4 example_4.cpp)
TARGET_LINK_LIBRARIES(example_4 ${PROJECT_NAME}_LIB ${PCL_LIBRARIES})
add_dependencies(example_4 ${PROJECT_NAME}_LIB)

# Build example_5_factory_restore
ADD_EXECUTABLE(example_5_factory_restore example_5_factory_restore.c)
TARGET_LINK_LIBRARIES(example_5_factory_restore GoSdk kApi)

# Build example_6_health_temperature
ADD_EXECUTABLE(example_6_health_temperature example_6_health_temperature.cpp)
TARGET_LINK_LIBRARIES(example_6_health_temperature ${PROJECT_NAME}_LIB ${PCL_LIBRARIES})

# Build Factory restore
ADD_EXECUTABLE(FactoryRestore FactoryRestore.c)
TARGET_LINK_LIBRARIES(FactoryRestore GoSdk kApi)
add_dependencies(FactoryRestore GoSdk kApi ${PCL_LIBRARIES})
