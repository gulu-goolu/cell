load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "glfw",
    generate_args = ["-DGLFW_BUILD_DOCS=0"],
    lib_source = ":srcs",
    out_static_libs = select(
        {
            "@platforms//os:windows": ["glfw3.lib"],
            "@platforms//os:linux": ["libglfw3.a"],
        },
    ),
    visibility = ["//visibility:public"],
)
