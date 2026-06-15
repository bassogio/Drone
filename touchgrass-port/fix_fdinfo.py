from pathlib import Path
import re

path = Path("kernel/fs/notify/fdinfo.c")
text = path.read_text()

# The SUSFS v1.5.5 patch leaves a label immediately before a conditional
# preprocessor boundary. After preprocessing, the next token is a C
# declaration, which is not a statement and is rejected by this kernel's
# compiler mode. Insert a null statement after the optional #endif block.
pattern = re.compile(
    r"(out_seq_printf:\n(?:[ \t]*#endif\n)?)(?![ \t]*;)",
    flags=re.MULTILINE,
)

updated, count = pattern.subn(r"\1\t\t;\n", text, count=1)
if count != 1:
    raise SystemExit("fdinfo SUSFS label anchor not found or already ambiguous")

path.write_text(updated)
print("Applied fdinfo label fix")
