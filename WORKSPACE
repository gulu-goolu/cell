load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "vulkan_headers",
    build_file = "//third_party:vulkan_headers.BUILD",
    strip_prefix = "Vulkan-Headers-1.3.261",
    urls = ["https://github.com/KhronosGroup/Vulkan-Headers/archive/refs/tags/v1.3.261.tar.gz"],
)

http_archive(
    name = "bazel_skylib",
    sha256 = "f7be3474d42aae265405a592bb7da8e171919d74c16f082a5457840f06054728",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.2.1/bazel-skylib-1.2.1.tar.gz",
        "https://github.com/bazelbuild/bazel-skylib/releases/download/1.2.1/bazel-skylib-1.2.1.tar.gz",
    ],
)

http_archive(
    name = "com_google_absl",
    strip_prefix = "abseil-cpp-20230125.1",
    urls = ["https://github.com/abseil/abseil-cpp/archive/refs/tags/20230125.1.tar.gz"],
)

http_archive(
    name = "hedron_compile_commands",
    strip_prefix = "bazel-compile-commands-extractor-7082b8e31e6a4fe612394f0d7f19236fc75f8c85",

    # Replace the commit hash in both places (below) with the latest, rather than using the stale one here.
    # Even better, set up Renovate and let it do the work for you (see "Suggestion: Updates" in the README).
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/7082b8e31e6a4fe612394f0d7f19236fc75f8c85.tar.gz",
    # When you first run this tool, it'll recommend a sha256 hash to put here with a message like: "DEBUG: Rule 'hedron_compile_commands' indicated that a canonical reproducible form can be obtained by modifying arguments sha256 = ..."
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

http_archive(
    name = "com_google_googletest",
    strip_prefix = "googletest-release-1.11.0",
    urls = [
        "https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz",
    ],
)

http_archive(
    name = "com_github_google_glog",
    sha256 = "21bc744fb7f2fa701ee8db339ded7dce4f975d0d55837a97be7d46e8382dea5a",
    strip_prefix = "glog-0.5.0",
    urls = ["https://github.com/google/glog/archive/v0.5.0.zip"],
)

http_archive(
    name = "com_github_gflags_gflags",  # 2018-11-11T21:30:10Z
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
)

http_archive(
    name = "rules_foreign_cc",
    strip_prefix = "rules_foreign_cc-3a85c822bf8bd44ca427c27407e838fdecd6bc86",
    url = "https://github.com/bazelbuild/rules_foreign_cc/archive/3a85c822bf8bd44ca427c27407e838fdecd6bc86.zip",
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

# This sets up some common toolchains for building targets. For more details, please see
# https://bazelbuild.github.io/rules_foreign_cc/0.10.1/flatten.html#rules_foreign_cc_dependencies
rules_foreign_cc_dependencies()

http_archive(
    name = "glslang",
    build_file = "//third_party:glslang.BUILD",
    strip_prefix = "glslang-13.1.1",
    urls = ["https://github.com/KhronosGroup/glslang/archive/refs/tags/13.1.1.tar.gz"],
)

new_local_repository(
    name = "renderdoc_app",
    build_file = "//third_party:renderdoc_app.BUILD",
    path = "third_party/renderdoc_app",
)

http_archive(
    name = "com_github_nlohmann_json",
    build_file = "//third_party:nlohmann_json.BUILD",
    strip_prefix = "json-3.11.2",
    urls = ["https://github.com/nlohmann/json/archive/refs/tags/v3.11.2.tar.gz"],
)
