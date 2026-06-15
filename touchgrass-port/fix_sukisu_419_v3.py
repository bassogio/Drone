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

# The builtin branch is a unity build. It excludes selinux_hide from kernels
# below 5.10, but the ksud runtime still calls its lifecycle functions. The
# implementation itself contains version-specific compatibility paths, so
# include and initialize it on this 4.19 kernel.
unity = base / "ksu.c"
text = unity.read_text()
replacements = {
    '#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)\n#include "feature/selinux_hide.h"\n#endif':
        '#include "feature/selinux_hide.h"',
    '#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)\n#include "feature/selinux_hide.c"\n#endif':
        '#include "feature/selinux_hide.c"',
    '#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)\n    ksu_selinux_hide_init();\n#endif':
        '    ksu_selinux_hide_init();',
}
for original, fixed in replacements.items():
    if original in text:
        text = text.replace(original, fixed, 1)
    elif fixed not in text:
        raise SystemExit("Unrecognized SukiSU selinux-hide unity source")
unity.write_text(text)

if "/* fall through */" not in allowlist.read_text():
    raise SystemExit("Allowlist compatibility verification failed")
if "#define USER_ARG_NULL (*user_arg_null_ptr())" not in sulog.read_text():
    raise SystemExit("Sulog compatibility verification failed")
final_unity = unity.read_text()
for required in (
    '#include "feature/selinux_hide.h"',
    '#include "feature/selinux_hide.c"',
    '    ksu_selinux_hide_init();',
):
    if required not in final_unity:
        raise SystemExit("SELinux-hide unity integration verification failed")

print("Verified SukiSU Ultra Linux 4.19 source compatibility")
print("Enabled SELinux-hide implementation in the builtin unity build")
