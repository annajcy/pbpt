
set(TEST_SRCS
    test_bxdf.cpp
    test_microfacet_model.cpp
)

foreach(SRC ${TEST_SRCS})
    get_filename_component(TEST_NAME ${SRC} NAME_WE)
    add_executable(${TEST_NAME} ${SRC})
    target_link_libraries(${TEST_NAME} PRIVATE pbpt GTest::gtest_main)
    gtest_discover_tests(${TEST_NAME})
endforeach()
