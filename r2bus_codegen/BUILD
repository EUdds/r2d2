cc_library(
    name = "r2bus_codegen",
    srcs = glob(["*.c"]),
    hdrs = glob(["*.h"]),
    copts = ["-O2", "-Wall", "-Wno-unused-function"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "r2bus_heartbeat_sample",
    srcs = ["sample_heartbeat.c"],
    deps = [":r2bus_codegen"],
)
