cc_library(
    name = "scene",
    srcs = [
        "gltf_loader.cc",
        "mesh.cc",
        "scene.cc",
    ],
    hdrs = [
        "gltf_loader.h",
        "mesh.h",
        "scene.h",
    ],
    implementation_deps = [
        "@com_github_nlohmann_json//:json",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//lance/core",
        "//lance/rendering",
    ],
)
