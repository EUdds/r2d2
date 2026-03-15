load("@rules_cc//cc/private/toolchain:unix_cc_toolchain_config.bzl", "cc_toolchain_config")

AARCH64_INCLUDE_PATHS = [
    "/usr/lib/gcc-cross/aarch64-linux-gnu/9/../../../../aarch64-linux-gnu/include/c++/9",
    "/usr/lib/gcc-cross/aarch64-linux-gnu/9/../../../../aarch64-linux-gnu/include/c++/9/aarch64-linux-gnu",
    "/usr/lib/gcc-cross/aarch64-linux-gnu/9/../../../../aarch64-linux-gnu/include/c++/9/backward",
    "/usr/lib/gcc-cross/aarch64-linux-gnu/9/include",
    "/usr/lib/gcc-cross/aarch64-linux-gnu/9/../../../../aarch64-linux-gnu/include",
    "/usr/aarch64-linux-gnu/include",
    "/usr/include",
]

AARCH64_TOOL_PATHS = {
    "gcc": "aarch64-gcc.sh",
    "g++": "aarch64-g++.sh",
    "ld": "aarch64-ld.sh",
    "ar": "aarch64-ar.sh",
    "cpp": "aarch64-cpp.sh",
    "gcov": "aarch64-gcov.sh",
    "nm": "aarch64-nm.sh",
    "objcopy": "aarch64-objcopy.sh",
    "objdump": "aarch64-objdump.sh",
    "strip": "aarch64-strip.sh",
    "dwp": "aarch64-dwp.sh",
}

def aarch64_linux_toolchain_config(name):
    cc_toolchain_config(
        name = name,
        abi_libc_version = "glibc",
        abi_version = "aarch64-linux-gnu",
        builtin_sysroot = "",
        compile_flags = [
            "-no-canonical-prefixes",
            "--sysroot=/usr/aarch64-linux-gnu",
        ],
        conly_flags = [],
        cpu = "aarch64",
        coverage_compile_flags = [],
        coverage_link_flags = [],
        cxx_builtin_include_directories = AARCH64_INCLUDE_PATHS,
        cxx_flags = ["-no-canonical-prefixes"],
        dbg_compile_flags = [],
        host_system_name = "local",
        link_flags = [
            "-no-canonical-prefixes",
            "-B/usr/aarch64-linux-gnu/lib",
            "-L/usr/aarch64-linux-gnu/lib",
            "-L/usr/lib/gcc-cross/aarch64-linux-gnu/9",
            "-Wl,-rpath-link,/usr/aarch64-linux-gnu/lib",
        ],
        link_libs = ["-lstdc++", "-lm"],
        opt_compile_flags = [],
        opt_link_flags = [],
        supports_start_end_lib = False,
        target_libc = "glibc",
        target_system_name = "aarch64-linux-gnu",
        tool_paths = AARCH64_TOOL_PATHS,
        toolchain_identifier = "aarch64-linux-gnu-gcc",
        compiler = "gcc",
        unfiltered_compile_flags = [],
    )
