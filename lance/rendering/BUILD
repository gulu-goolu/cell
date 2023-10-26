cc_library(
    name = "rendering",
    srcs = [
        "device.cc",
        "render_graph.cc",
        "shader_compiler.cc",
        "vk_api.cc",
    ],
    hdrs = [
        "device.h",
        "render_graph.h",
        "shader_compiler.h",
        "vk_api.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//lance/core",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
        "@glslang",
        "@vulkan_headers",
    ],
)

cc_test(
    name = "unittests",
    srcs = [
        "compiler_test.cc",
        "device_test.cc",
        "vk_api_test.cc",
    ],
    linkstatic = True,
    deps = [
        ":rendering",
        "@com_google_googletest//:gtest_main",
    ],
)
