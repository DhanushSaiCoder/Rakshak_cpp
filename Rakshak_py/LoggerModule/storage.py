import shutil
from .global_config import settings

class SystemStorage:
    
    def __init__(self, threshold_value: int = settings._STORAGE_THRESHOLD_PERCENT):
        
        self.threshold = threshold_value

    def checking(self):
        usage = shutil.disk_usage("/")
        used_percentage = (usage.used / usage.total) * 100

        if used_percentage >= self.threshold:
            raise RuntimeError(
                f"Insufficient disk space (used={used_percentage:.2f}%)"
            )

        return True

