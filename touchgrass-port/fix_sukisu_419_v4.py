from pathlib import Path

base = Path("kernel/KernelSU/kernel")

allowlist = base / "policy/allowlist.c"
text = allowlist.read_text()
original = "        fallthrough;\n    case 3:"
fixed = "        /* fall through */\n    case 3:"
if original in text:
    allowlist.write_text(text.replace(original, fixed, 1))
elif fixed not in text:
    raise SystemExit("Unrecognized allowlist fall-through source")

sulog = base / "sulog/event.c"
text = sulog.read_text()
original = "#define USER_ARG_NULL user_arg_null_ptr()"
fixed = "#define USER_ARG_NULL (*user_arg_null_ptr())"
if original in text:
    sulog.write_text(text.replace(original, fixed, 1))
elif fixed not in text:
    raise SystemExit("Unrecognized sulog null-argument source")

# The builtin branch correctly excludes SELinux-hide below Linux 5.10, but
# its runtime still calls two lifecycle helpers unconditionally. Guard those
# calls rather than backporting a feature that depends on newer SELinux state
# internals absent from Samsung's Linux 4.19 tree.
runtime = base / "runtime/ksud.c"
text = runtime.read_text()
post_call = "    ksu_selinux_hide_handle_post_fs_data();"
post_guard = (
    "#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)\n"
    "    ksu_selinux_hide_handle_post_fs_data();\n"
    "#endif /* SELinux-hide requires Linux 5.10+ */"
)
if post_call in text and post_guard not in text:
    text = text.replace(post_call, post_guard, 1)

second_call = "            ksu_selinux_hide_handle_second_stage();"
second_guard = (
    "#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)\n"
    "            ksu_selinux_hide_handle_second_stage();\n"
    "#endif /* SELinux-hide requires Linux 5.10+ */"
)
while second_call in text:
    text = text.replace(second_call, second_guard, 1)
runtime.write_text(text)

# Verify that no SELinux-hide implementation was forced into the 4.19 unity
# build and all runtime calls are protected.
unity = base / "ksu.c"
unity_text = unity.read_text()
if '#include "feature/selinux_hide.c"\n#endif' not in unity_text:
    raise SystemExit("Unexpected SukiSU SELinux-hide unity source state")
if "SELinux-hide requires Linux 5.10+" not in runtime.read_text():
    raise SystemExit("SELinux-hide runtime guard verification failed")
if "/* fall through */" not in allowlist.read_text():
    raise SystemExit("Allowlist compatibility verification failed")
if "#define USER_ARG_NULL (*user_arg_null_ptr())" not in sulog.read_text():
    raise SystemExit("Sulog compatibility verification failed")

print("Verified SukiSU Ultra Linux 4.19 source compatibility")
print("Guarded unsupported SELinux-hide runtime calls")
