cc_library(
    name = "platform",
    srcs = ["app.cc"],
    hdrs = ["app.h"],
    implementation_deps = [
        "//lance/core",
        "@glfw",
    ],
    linkopts = select({
        "@platforms//os:windows": [
            "-DEFAULTLIB:user32.lib",
            "-DEFAULTLIB:advapi32.lib",
            "-DEFAULTLIB:gdi32.lib",
            "-DEFAULTLIB:shell32.lib",
        ],
        "//conditions:default": [],
    }),
    deps = ["@com_google_absl//absl/status"],
)

cc_test(
    name = "app_test",
    srcs = ["app_test.cc"],
    deps = [":platform"],
)
