from pathlib import Path

path = Path("kernel/fs/notify/fdinfo.c")
text = path.read_text()
old = "out_seq_printf:\n\t\t/*"
new = "out_seq_printf:\n\t\t;\n\t\t/*"

if old not in text:
    raise SystemExit("fdinfo SUSFS label anchor not found")

path.write_text(text.replace(old, new, 1))
print("Applied fdinfo label fix")
