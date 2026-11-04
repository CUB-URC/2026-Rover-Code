"""Configure, build and run the C++ tests. Works the same on Windows, Mac and Linux.

    python tools/run_cpp_tests.py

Uses CMake from PATH, or the one installed with `pip install cmake`.
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"


def find_tool(name: str) -> str:
    try:
        import cmake  # pip-installed CMake
        candidate = Path(cmake.CMAKE_BIN_DIR) / (name + (".exe" if os.name == "nt" else ""))
        if candidate.exists():
            return str(candidate)
    except ImportError:
        pass
    found = shutil.which(name)
    if not found:
        sys.exit(f"Could not find {name}. Run: python -m pip install cmake")
    return found


def run(cmd):
    print(">", " ".join(cmd), flush=True)
    result = subprocess.run(cmd, cwd=ROOT)
    if result.returncode != 0:
        sys.exit(result.returncode)


def main():
    cmake, ctest = find_tool("cmake"), find_tool("ctest")
    run([cmake, "-S", str(ROOT), "-B", str(BUILD), "-DROVER_BUILD_PYTHON=OFF"])
    run([cmake, "--build", str(BUILD), "--config", "Debug"])
    run([ctest, "--test-dir", str(BUILD), "-C", "Debug", "--output-on-failure"])


if __name__ == "__main__":
    main()
