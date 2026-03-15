"""Module extensions for vendored libraries without their own MODULE.bazel."""

load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

def _local_vendor_impl(_mctx):
    new_local_repository(
        name = "pico_libfixmath",
        path = "firmware/libs/pico-libfixmath",
        build_file_content = """
load("@rules_cc//cc:defs.bzl", "cc_library")
cc_library(
    name = "fixmath",
    srcs = glob(["src/*.c"]),
    hdrs = glob(["include/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
""",
    )

local_vendor = module_extension(
    implementation = _local_vendor_impl,
)
