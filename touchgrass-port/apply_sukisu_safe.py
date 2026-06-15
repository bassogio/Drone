from pathlib import Path
import re

root = Path("kernel")
sukisu = root / "KernelSU" / "kernel"

# Keep the first SukiSU hardware test deliberately conservative:
# no SUSFS, no KPM, no adb-root, and no kernel-level automatic unmount.
defconfig = root / "arch/arm64/configs/a52xq_defconfig"
lines = defconfig.read_text().splitlines()

symbols = [
    "CONFIG_KSU",
    "CONFIG_KSU_DEBUG",
    "CONFIG_KSU_FEATURE_ADBROOT",
    "CONFIG_KSU_DISABLE_MANAGER",
    "CONFIG_KSU_DISABLE_POLICY",
    "CONFIG_KSU_SUSFS",
    "CONFIG_KPM",
]

cleaned = []
for line in lines:
    if any(line == f"{symbol}=y" or line == f"{symbol}=m" or line == f"# {symbol} is not set" for symbol in symbols):
        continue
    if line.startswith("CONFIG_KSU_SUSFS_") or line.startswith("# CONFIG_KSU_SUSFS_"):
        continue
    if line.startswith("CONFIG_KSU_MANUAL_HOOK") or line.startswith("# CONFIG_KSU_MANUAL_HOOK"):
        continue
    if line.startswith("CONFIG_KSU_KPROBES_HOOK") or line.startswith("# CONFIG_KSU_KPROBES_HOOK"):
        continue
    cleaned.append(line)

cleaned += [
    "CONFIG_KSU=y",
    "# CONFIG_KSU_DEBUG is not set",
    "# CONFIG_KSU_FEATURE_ADBROOT is not set",
    "# CONFIG_KSU_DISABLE_MANAGER is not set",
    "# CONFIG_KSU_DISABLE_POLICY is not set",
    "# CONFIG_KPM is not set",
    "# CONFIG_KSU_SUSFS is not set",
]
defconfig.write_text("\n".join(cleaned) + "\n")

# Permanently disable and hide the Kernel Umount feature. This preserves the
# behavior of the last known-good TouchGrass build and prevents the manager
# from re-enabling it.
umount = sukisu / "feature/kernel_umount.c"
text = umount.read_text()
text, count = re.subn(
    r"static bool ksu_kernel_umount_enabled = true;",
    "static bool ksu_kernel_umount_enabled = false;",
    text,
    count=1,
)
if count != 1:
    raise SystemExit("SukiSU kernel umount default anchor not found")

text, count = re.subn(
    r"void __init ksu_kernel_umount_init\(void\)\n\{.*?\n\}\n\nvoid __exit ksu_kernel_umount_exit\(void\)\n\{.*?\n\}",
    "void __init ksu_kernel_umount_init(void)\n{\n\tksu_kernel_umount_enabled = false;\n}\n\nvoid __exit ksu_kernel_umount_exit(void)\n{\n}",
    text,
    count=1,
    flags=re.DOTALL,
)
if count != 1:
    raise SystemExit("SukiSU kernel umount init/exit anchor not found")
umount.write_text(text)

# TouchGrass already contains the standard manual filesystem/input hooks.
# Add the public reboot hook needed by the modern built-in userspace installer.
reboot = root / "kernel/reboot.c"
text = reboot.read_text()
if "ksu_handle_sys_reboot" not in text:
    marker = "SYSCALL_DEFINE4(reboot, int, magic1, int, magic2, unsigned int, cmd,"
    declaration = (
        "#ifdef CONFIG_KSU\n"
        "extern int ksu_handle_sys_reboot(int magic1, int magic2, unsigned int cmd, void __user **arg);\n"
        "#endif\n\n"
    )
    if marker not in text:
        raise SystemExit("reboot syscall declaration anchor not found")
    text = text.replace(marker, declaration + marker, 1)

    body = "\tchar buffer[256];\n\tint ret = 0;\n"
    replacement = (
        body
        + "\n#ifdef CONFIG_KSU\n"
        + "\tksu_handle_sys_reboot(magic1, magic2, cmd, &arg);\n"
        + "#endif\n"
    )
    if body not in text:
        raise SystemExit("reboot syscall body anchor not found")
    text = text.replace(body, replacement, 1)
    reboot.write_text(text)

print("Applied conservative SukiSU Ultra built-in profile")
print("  SUSFS: disabled")
print("  KPM: disabled")
print("  ADB root: disabled")
print("  Kernel Umount: permanently disabled and hidden")
