load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "srcs",
    srcs = glob(["**"]),
)

cmake(
    name = "glslang",
    generate_args = ["-DENABLE_OPT=0"],
    lib_source = ":srcs",
    out_static_libs = select(
        {
            "@platforms//os:windows": [
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
            "@platforms//os:linux": [
                "libglslang.a",
                "libOGLCompiler.a",
                "libOSDependent.a",
                "libglslang-default-resource-limits.a",
                "libGenericCodeGen.a",
                "libSPIRV.a",
                "libSPVRemapper.a",
                "libHLSL.a",
                "libMachineIndependent.a",
            ],
        },
    ),
    visibility = ["//visibility:public"],
)
