# backend/logger.py
import logging
import logging.handlers
import os
from datetime import datetime

LOG_ROOT = "/tmp/Autoscope_v1_logs"

def get_logger(name: str = "autoscope") -> logging.Logger:
    """
    Returns a pre-configured logger that writes to
    /tmp/Autoscope_v1_logs/<YYYY-MM-DD>/logfile_<HHMMSS>.log
    Logs are kept indefinitely (no automatic deletion).
    """
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    logger.setLevel(logging.DEBUG)

    # ---- daily folder -------------------------------------------------
    today = datetime.now().strftime("%Y-%m-%d")
    day_dir = os.path.join(LOG_ROOT, today)
    os.makedirs(day_dir, exist_ok=True)

    # ---- file name with start-time ------------------------------------
    start_ts = datetime.now().strftime("%H%M%S")
    log_file = os.path.join(day_dir, f"logfile_{start_ts}.log")

    # ---- normal file handler (no rotation or deletion) ----------------
    handler = logging.FileHandler(log_file)
    formatter = logging.Formatter(
        "%(asctime)s | %(name)s | %(levelname)-8s | %(threadName)s | %(message)s"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    # ---- also print to console ----------------------------------------
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)

    return logger

