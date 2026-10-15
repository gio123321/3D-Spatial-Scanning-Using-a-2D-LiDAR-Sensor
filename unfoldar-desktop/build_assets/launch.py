"""PyInstaller entry point with writable streams and uncaught-exception logging."""
import os
import sys
import traceback

# Native dependencies need writable streams in a windowed build.
_null = "NUL" if sys.platform == "win32" else "/dev/null"
if sys.stdout is None or not hasattr(sys.stdout, "write"):
    sys.stdout = open(_null, "w")
if sys.stderr is None or not hasattr(sys.stderr, "write"):
    sys.stderr = open(_null, "w")

# Preserve uncaught tracebacks in a file when no console is available.
_log_dir = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "UnfoLDAR", "logs")
os.makedirs(_log_dir, exist_ok=True)
_log_path = os.path.join(_log_dir, "crash.log")


def _log_crash(exc_type, exc_value, exc_tb) -> None:
    try:
        with open(_log_path, "a", encoding="utf-8") as handle:
            traceback.print_exception(exc_type, exc_value, exc_tb, file=handle)
    except OSError:
        pass
    sys.__excepthook__(exc_type, exc_value, exc_tb)


sys.excepthook = _log_crash

from unfoldar.app import main

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        _log_crash(*sys.exc_info())
        raise
