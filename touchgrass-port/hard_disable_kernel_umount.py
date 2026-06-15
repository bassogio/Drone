from pathlib import Path
import re

path = Path("kernel/KernelSU/kernel/feature/kernel_umount.c")
text = path.read_text()

text, count = re.subn(
    r"static bool ksu_kernel_umount_enabled = true;",
    "static bool ksu_kernel_umount_enabled = false;",
    text,
    count=1,
)
if count != 1:
    raise SystemExit("kernel umount default state anchor not found")

text, count = re.subn(
    r"void __init ksu_kernel_umount_init\(void\)\n\{.*?\n\}\n\nvoid __exit ksu_kernel_umount_exit\(void\)\n\{.*?\n\}",
    "void __init ksu_kernel_umount_init(void)\n{\n\tksu_kernel_umount_enabled = false;\n}\n\nvoid __exit ksu_kernel_umount_exit(void)\n{\n}",
    text,
    count=1,
    flags=re.DOTALL,
)
if count != 1:
    raise SystemExit("kernel umount init/exit anchor not found")

path.write_text(text)
print("Kernel Umount hard-disabled and feature handler hidden")
