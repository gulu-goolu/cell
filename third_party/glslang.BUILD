load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "glslang",
    generate_args = ["-DENABLE_OPT=0"],
    lib_source = ":srcs",
    out_static_libs = [
        "glslang.lib",
        "OGLCompiler.lib",
        "OSDependent.lib",
        "glslang-default-resource-limits.lib",
        "GenericCodeGen.lib",
        "SPIRV.lib",
        "SPVRemapper.lib",
        "HLSL.lib",
        "MachineIndependent.lib",
    ],
    visibility = ["//visibility:public"],
)
