cc_library(
    name = "rendering",
    srcs = [
        "device.cc",
        "vk_api.cc",
    ],
    hdrs = [
        "device.h",
        "vk_api.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//lance/core",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@vulkan_headers",
    ],
)

cc_test(
    name = "unittests",
    srcs = ["vk_api_test.cc"],
    deps = [
        ":rendering",
        "@com_google_googletest//:gtest_main",
    ],
)
