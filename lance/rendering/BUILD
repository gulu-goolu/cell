cc_library(
    name = "rendering",
    srcs = ["vk_api.cc"],
    hdrs = ["vk_api.h"],
    visibility = ["//visibility:public"],
    deps = ["@vulkan_headers"],
)

cc_test(
    name = "unittests",
    srcs = ["vk_api_test.cc"],
    deps = [
        ":rendering",
        "@com_google_googletest//:gtest_main",
    ],
)
