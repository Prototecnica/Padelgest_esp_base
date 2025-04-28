import subprocess
from SCons.Script import Import

Import("env")

def get_branch_prefix():
    try:
        branch = (
            subprocess
            .check_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])
            .strip()
            .decode("utf-8")
        )
        return branch[:2]
    except Exception:
        return "??"

def has_dirty_changes():
    try:
        status = (
            subprocess
            .check_output(["git", "status", "--porcelain"])
            .strip()
        )
        return bool(status)
    except Exception:
        return False

def get_git_version():
    prefix = get_branch_prefix()
    try:
        commit = (
            subprocess
            .check_output(["git", "rev-parse", "--short=7", "HEAD"])
            .strip()
            .decode("utf-8")
        )
    except Exception:
        commit = "unknown"
    version = f"{prefix}{commit}"
    if has_dirty_changes():
        version += "*"
    return version

firmware_version = get_git_version()
print(f"Generated firmware version: {firmware_version}")

env.Append(
    BUILD_FLAGS=[f"-D FIRMWARE_VERSION=\"\\\"{firmware_version}\\\"\""]
)