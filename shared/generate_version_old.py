import subprocess
from SCons.Script import Import

Import("env")

def get_git_hash():
    try:
        # Get the first 7 characters of the current commit hash
        commit_hash = subprocess.check_output(["git", "rev-parse", "--short=7", "HEAD"]).strip().decode("utf-8")
        return commit_hash
    except Exception:
        return "unknown"  # Fallback if Git info is unavailable

# Generate the firmware version
firmware_version = get_git_hash()
print(f"Generated firmware version: {firmware_version}")

# Append the firmware version as a build flag with quotes
env.Append(BUILD_FLAGS=[f"-D FIRMWARE_VERSION=\"\\\"{firmware_version}\\\"\""])