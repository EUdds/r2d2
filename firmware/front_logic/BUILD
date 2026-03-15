load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "front_logic_v1.elf",
    srcs = [
        "src/logic_lights.c",
        "src/main.c",
        "src/r2bus_receiver.c",
        "src/screen.c",
        "src/status_led.c",
        "src/tasks.c",
    ] + glob(["include/**/*.h"]),
    additional_linker_inputs = ["linker.ld"],
    copts = ["-std=c11", "-Os"],
    includes = ["include"],
    linkopts = [
        "-T$(location linker.ld)",
        "-Wl,--gc-sections",
    ],
    local_defines = ["PCB_REV=1"],
    target_compatible_with = ["@pico-sdk//bazel/constraint:rp2040"],
    deps = [
        "//firmware/libs/r2bus",
        "//firmware/libs/st7789",
        "@pico-sdk//src/rp2_common/hardware_dma",
        "@pico-sdk//src/rp2_common/hardware_pio",
        "@pico-sdk//src/rp2_common/hardware_pwm",
        "@pico-sdk//src/rp2_common/hardware_spi",
        "@pico-sdk//src/rp2_common/pico_rand",
        "@pico-sdk//src/rp2_common/pico_stdio_usb",
        "@pico-sdk//src/rp2_common/pico_stdlib",
    ],
)
