load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "front_logic_v1.elf",
    srcs = glob(["src/**/*.c", "include/**/*.h"]),
    additional_linker_inputs = ["linker.ld"],
    copts = ["-std=c11", "-Os"],
    includes = ["include"],
    linkopts = [
        "-T$(location linker.ld)",
        "-Wl,--gc-sections",
    ],
    local_defines = ["PCB_REV=1"],
    target_compatible_with = select({
        "@pico-sdk//bazel/constraint:host": ["@platforms//:incompatible"],
        "//conditions:default": [],
    }),
    deps = [
        "//firmware/libs/r2bus",
        "@pico-sdk//src/rp2_common/hardware_dma",
        "@pico-sdk//src/rp2_common/hardware_pio",
        "@pico-sdk//src/rp2_common/hardware_pwm",
        "@pico-sdk//src/rp2_common/hardware_spi",
        "@pico-sdk//src/rp2_common/pico_rand",
        "@pico-sdk//src/rp2_common/pico_stdio_usb",
        "@pico-sdk//src/rp2_common/pico_stdlib",
    ],
)
