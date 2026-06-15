from pathlib import Path

base = Path("kernel/KernelSU/kernel")

allowlist = base / "policy/allowlist.c"
text = allowlist.read_text()
old = "        fallthrough;\n    case 3:"
new = "        /* fall through */\n    case 3:"
if old not in text:
    raise SystemExit("SukiSU fallthrough compatibility anchor not found")
allowlist.write_text(text.replace(old, new, 1))

sulog = base / "sulog/event.c"
text = sulog.read_text()
old = "#define USER_ARG_NULL user_arg_null_ptr()"
new = "#define USER_ARG_NULL (*user_arg_null_ptr())"
if old not in text:
    raise SystemExit("SukiSU sulog compatibility anchor not found")
sulog.write_text(text.replace(old, new, 1))

# The pinned builtin branch contains the SELinux-hide implementation and
# calls it from ksud integration, but its Kbuild omits the object. Include it
# so the two public lifecycle functions are available at final linking.
kbuild = base / "Kbuild"
text = kbuild.read_text()
obj_line = "kernelsu-objs += feature/selinux_hide.o"
if obj_line not in text:
    anchor = "kernelsu-objs += feature/sucompat.o\n"
    if anchor in text:
        text = text.replace(anchor, anchor + obj_line + "\n", 1)
    else:
        text = text.rstrip() + "\n" + obj_line + "\n"
    kbuild.write_text(text)

print("Applied SukiSU Ultra Linux 4.19 compatibility fixes")
print("Included feature/selinux_hide.o in the builtin Kbuild")
