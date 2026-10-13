"""Logging utilities for the autonomous navigation framework."""

import logging
import sys
from typing import Optional


def setup_logger(
    name: str = "autonomous_navigation",
    level: str = "INFO",
    log_file: Optional[str] = None,
) -> logging.Logger:
    """Configures and returns a standardized logger.

    Args:
        name: Name of the logger.
        level: Logging level string ('DEBUG', 'INFO', 'WARNING', 'ERROR').
        log_file: Optional filepath to write log output.

    Returns:
        Configured logging.Logger instance.
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))

    if not logger.handlers:
        formatter = logging.Formatter(
            fmt="[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

        if log_file:
            file_handler = logging.FileHandler(log_file, encoding="utf-8")
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

    return logger
