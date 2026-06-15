from pathlib import Path

base = Path("kernel/KernelSU/kernel")

allowlist = base / "policy/allowlist.c"
text = allowlist.read_text()
original_fallthrough = "        fallthrough;\n    case 3:"
fixed_fallthrough = "        /* fall through */\n    case 3:"
if original_fallthrough in text:
    allowlist.write_text(text.replace(original_fallthrough, fixed_fallthrough, 1))
elif fixed_fallthrough not in text:
    raise SystemExit("Unrecognized allowlist fall-through source")

sulog = base / "sulog/event.c"
text = sulog.read_text()
original_null = "#define USER_ARG_NULL user_arg_null_ptr()"
fixed_null = "#define USER_ARG_NULL (*user_arg_null_ptr())"
if original_null in text:
    sulog.write_text(text.replace(original_null, fixed_null, 1))
elif fixed_null not in text:
    raise SystemExit("Unrecognized sulog null-argument source")

kbuild = base / "Kbuild"
text = kbuild.read_text()
selinux_object = "kernelsu-objs += feature/selinux_hide.o"
if selinux_object not in text:
    anchor = "kernelsu-objs += feature/sucompat.o\n"
    if anchor in text:
        text = text.replace(anchor, anchor + selinux_object + "\n", 1)
    else:
        text = text.rstrip() + "\n" + selinux_object + "\n"
    kbuild.write_text(text)

if fixed_fallthrough not in allowlist.read_text():
    raise SystemExit("Allowlist compatibility verification failed")
if fixed_null not in sulog.read_text():
    raise SystemExit("Sulog compatibility verification failed")
if selinux_object not in kbuild.read_text():
    raise SystemExit("SELinux-hide object verification failed")

print("Verified SukiSU Ultra Linux 4.19 source compatibility")
