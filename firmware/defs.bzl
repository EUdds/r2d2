"""Firmware build macros."""

load("@bazel_tools//tools/build_defs/cc:action_names.bzl", "OBJ_COPY_ACTION_NAME")
load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cpp_toolchain", "use_cc_toolchain")

def _platform_transition_impl(settings, attr):
    return {"//command_line_option:platforms": [attr.platform]}

_platform_transition = transition(
    implementation = _platform_transition_impl,
    inputs = [],
    outputs = ["//command_line_option:platforms"],
)

def _platform_firmware_impl(ctx):
    files = depset(transitive = [t[DefaultInfo].files for t in ctx.attr.targets])
    return [DefaultInfo(files = files)]

platform_firmware = rule(
    doc = """Wraps a firmware filegroup target and fixes its build platform.

    Allows `bazel build //firmware:front_logic_v1` without --platforms.
    """,
    implementation = _platform_firmware_impl,
    attrs = {
        "targets": attr.label_list(cfg = _platform_transition),
        "platform": attr.string(mandatory = True),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)

def _pico_hex_impl(ctx):
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    objcopy = cc_common.get_tool_for_action(
        feature_configuration = feature_configuration,
        action_name = OBJ_COPY_ACTION_NAME,
    )
    ctx.actions.run(
        inputs = depset(
            direct = [ctx.file.src],
            transitive = [cc_toolchain.all_files],
        ),
        executable = objcopy,
        outputs = [ctx.outputs.out],
        arguments = ["-O", "ihex", ctx.file.src.path, ctx.outputs.out.path],
    )

pico_hex = rule(
    implementation = _pico_hex_impl,
    attrs = {
        "src": attr.label(allow_single_file = True, mandatory = True),
        "out": attr.output(mandatory = True),
    },
    fragments = ["cpp"],
    toolchains = use_cc_toolchain(),
)

def pico_binary(name, **kwargs):
    """Declares cc_binary + uf2 genrule + hex rule + filegroup.

    Name convention: "<module>/<rev>/elf"
    Produces sibling targets:
      <module>/<rev>/uf2
      <module>/<rev>/hex
      <module>/<rev>        (filegroup of all three)
    """
    cc_binary(name = name, **kwargs)

    base = name.rsplit("/", 1)[0]  # e.g. "front_logic/1"
    stem = base.replace("/", "_")  # e.g. "front_logic_1" (for filenames)

    native.genrule(
        name = base + "/uf2",
        srcs = [":" + name],
        outs = [stem + ".uf2"],
        cmd = "$(execpath @picotool//:picotool) uf2 convert --quiet -t elf $(SRCS) $@",
        tools = ["@picotool//:picotool"],
        target_compatible_with = kwargs.get("target_compatible_with", []),
    )

    pico_hex(
        name = base + "/hex",
        src = ":" + name,
        out = stem + ".hex",
        target_compatible_with = kwargs.get("target_compatible_with", []),
    )

    native.filegroup(
        name = base,
        srcs = [
            ":" + name,
            ":" + base + "/uf2",
            ":" + base + "/hex",
        ],
        visibility = kwargs.get("visibility", ["//visibility:public"]),
    )
