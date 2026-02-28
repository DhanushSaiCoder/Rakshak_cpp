import os
import logging
from logging.handlers import TimedRotatingFileHandler

class CombinedRotatingFileHandler(TimedRotatingFileHandler):
    def __init__(self, filename, when='midnight', interval=1, backupCount=7, maxBytes=5*1024*1024, encoding=None):
        super().__init__(filename, when=when, interval=interval, backupCount=backupCount, encoding=encoding)
        self.maxBytes = maxBytes

    def shouldRollover(self, record):
        if super().shouldRollover(record):
            return 1
        if self.stream is None:
            self.stream = self._open()
        self.stream.flush()
        if os.stat(self.baseFilename).st_size >= self.maxBytes:
            return 1
        return 0

def setup_combined_logger(module_name, log_dir="logs", log_file="application.log", max_bytes=5*1024*1024, backup_count=7):
    os.makedirs(log_dir, exist_ok=True)
    file_path = os.path.join(log_dir, log_file)

    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s [%(levelname)s] [%(module)s] %(message)s')

    combined_handler = CombinedRotatingFileHandler(
        file_path, when='midnight', interval=1, backupCount=backup_count, maxBytes=max_bytes, encoding='utf-8'
    )
    combined_handler.setFormatter(formatter)

    if not logger.handlers:
        logger.addHandler(combined_handler)

    return logger
