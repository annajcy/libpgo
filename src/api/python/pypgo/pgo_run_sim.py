import argparse
import os
from pathlib import Path
import sys
from typing import Optional
import pypgo


def _run_sim(config_path: str, deterministic: Optional[bool]) -> None:
    print(pypgo.run_sim_from_config(config_path, deterministic=deterministic))


def _run_sim_with_stdout_redirect(config_path: str, deterministic: Optional[bool], log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # Redirect process stdout/stderr (fd=1/2) so native pypgo logging is captured as well.
    sys.stdout.flush()
    sys.stderr.flush()
    saved_stdout_fd = os.dup(1)
    saved_stderr_fd = os.dup(2)
    try:
        with log_path.open("w", encoding="utf-8") as log_file:
            os.dup2(log_file.fileno(), 1)
            os.dup2(log_file.fileno(), 2)
            _run_sim(config_path, deterministic)
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(saved_stdout_fd, 1)
        os.dup2(saved_stderr_fd, 2)
        os.close(saved_stdout_fd)
        os.close(saved_stderr_fd)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run simulation from a config JSON file.")
    parser.add_argument("config", help="Path to simulation config JSON")
    parser.add_argument(
        "--deterministic",
        action="store_true",
        help="Force deterministic mode (single-threaded) regardless of config file setting.",
    )
    parser.add_argument(
        "--save-log",
        action="store_true",
        help="Save run stdout and stderr to a .log file next to the config file.",
    )

    args = parser.parse_args()

    deterministic = True if args.deterministic else None
    if args.save_log:
        log_path = Path(args.config).with_suffix(".log")
        _run_sim_with_stdout_redirect(args.config, deterministic, log_path)
    else:
        _run_sim(args.config, deterministic)
