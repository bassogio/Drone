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

print("Applied SukiSU Ultra Linux 4.19 compatibility fixes")
