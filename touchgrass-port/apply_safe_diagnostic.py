from pathlib import Path
from shutil import copy2

kernel = Path("kernel")
reference = Path("ksun-reference/drivers/kernelsu")
ksu = kernel / "KernelSU/kernel"

# Restore KernelSU-Next's original app-process and unmount behavior.
copy2(reference / "feature/kernel_umount.c", ksu / "feature/kernel_umount.c")
copy2(reference / "hook/setuid_hook.c", ksu / "hook/setuid_hook.c")

# Disable SUSFS features that alter Zygote-spawned app behavior or app paths.
defconfig = kernel / "arch/arm64/configs/a52xq_defconfig"
lines = defconfig.read_text().splitlines()

disable = [
    "CONFIG_KSU_SUSFS_SUS_PATH",
    "CONFIG_KSU_SUSFS_SUS_KSTAT",
    "CONFIG_KSU_SUSFS_TRY_UMOUNT",
    "CONFIG_KSU_SUSFS_AUTO_ADD_TRY_UMOUNT_FOR_BIND_MOUNT",
    "CONFIG_KSU_SUSFS_AUTO_ADD_SUS_BIND_MOUNT",
    "CONFIG_KSU_SUSFS_OPEN_REDIRECT",
]

for symbol in disable:
    lines = [
        line
        for line in lines
        if line != f"{symbol}=y" and line != f"# {symbol} is not set"
    ]
    lines.append(f"# {symbol} is not set")

defconfig.write_text("\n".join(lines) + "\n")

print("Restored original KernelSU app-process/unmount path")
print("Disabled Zygote-sensitive SUSFS features:")
for symbol in disable:
    print(f"  {symbol}=n")
