import yaml
import math
from pathlib import Path
from typing import Union


KEY_MAP = {
    "STORAGE_THRESHOLD_PERCENT": ("storage", "threshold_percent"),
    "LOG_DIRECTORY_MAX_SIZE_MB": ("storage", "max_directory_size_mb"),
    "MAX_DIRECTORY_WARNING_THRESHOLD": ("storage", "max_dir_warning_threshold"),

    "LOG_DIRECTORY": ("logger", "directory"),
    "MAX_FILE_SIZE_MB": ("logger", "max_file_size_mb"),
    "QUEUE_SIZE": ("logger", "queue_size"),
    "DEFAULT_FILE_TYPE": ("logger", "default_file_type"),
    "DEFAULT_COMPRESS": ("logger", "default_compress"),
    "ENCODER": ("logger", "encoder"),

    "XLSX_MAX_ROWS": ("xlsxconfig", "rows"),
}

TYPE_MAP = {
    "STORAGE_THRESHOLD_PERCENT": int,
    "LOG_DIRECTORY_MAX_SIZE_MB": (int, float),
    "MAX_DIRECTORY_WARNING_THRESHOLD": (int, str),

    "LOG_DIRECTORY": str,
    "MAX_FILE_SIZE_MB": (int, float),
    "QUEUE_SIZE": int,
    "DEFAULT_FILE_TYPE": str,
    "DEFAULT_COMPRESS": bool,
    "ENCODER": bool,

    "XLSX_MAX_ROWS": int,
}


class LoggerConfig:
    def __init__(self, path=None):
        try:
            # ---------------- PATH RESOLUTION ----------------
            if path is None:
                # Calculate path relative to THIS python file
                current_dir = Path(__file__).resolve().parent
                self._path = current_dir / "config.yaml"
            else:
                self._path = Path(path)

            # ---------------- LOAD CONFIG ----------------
            with open(self._path, "r") as f:
                data = yaml.safe_load(f)

            # ---------------- STORAGE ----------------
            self._STORAGE_THRESHOLD_PERCENT = int(data["storage"]["threshold_percent"])
            self._LOG_DIRECTORY_MAX_SIZE_MB = float(data["storage"]["max_directory_size_mb"])
            raw_warning = data["storage"]["max_dir_warning_threshold"]

            # ---------------- LOGGER -----------------
            self._LOG_DIRECTORY = Path(data["logger"]["directory"])
            self._MAX_FILE_SIZE_MB = float(data["logger"]["max_file_size_mb"])
            self._QUEUE_SIZE = int(data["logger"]["queue_size"])
            self._DEFAULT_FILE_TYPE = data["logger"]["default_file_type"]
            self._DEFAULT_COMPRESS = bool(data["logger"]["default_compress"])
            self._ENCODER = bool(data["logger"]["encoder"])

            # ---------------- XLSX -------------------
            self._XLSX_MAX_ROWS = int(data["xlsxconfig"]["rows"])

            # ---------------- DERIVED ----------------
            self._MAX_FILES_POSSIBLE = self._max_files_possible()

            if raw_warning == "auto":
                self._MAX_DIRECTORY_WARNING_THRESHOLD = self._auto_warning_threshold()
                self._WARNING_MODE = "auto"
            else:
                self._MAX_DIRECTORY_WARNING_THRESHOLD = int(raw_warning)
                self._WARNING_MODE = "manual"

            self._validate()

            # print(
            #     f"[Config] Loaded from {self._path}\n"
            #     f"[Config] max_files={self.MAX_FILES_POSSIBLE}, "
            #     f"warning={self.MAX_DIRECTORY_WARNING_THRESHOLD}% "
            #     f"(mode={self._WARNING_MODE})"
            # )

        except Exception as e:
            print(f"Exception in LoggerConfig (__init__): {e}")


    # =====================================================
    # AUTO WARNING LOGIC
    # =====================================================
    def _max_files_possible(self) -> int:
        try:
            if self._LOG_DIRECTORY_MAX_SIZE_MB <= 0 or self._MAX_FILE_SIZE_MB <= 0:
                raise RuntimeError(
                    "Invalid config: max_directory_size_mb and max_file_size_mb must be > 0"
                )

            files = math.floor(
                self._LOG_DIRECTORY_MAX_SIZE_MB / self._MAX_FILE_SIZE_MB
            )

            if files < 1:
                raise RuntimeError(
                    "Invalid config: directory too small to hold even one file"
                )

            return files
        except Exception as e:
            print(f"Exception in LoggerConfig (_max_files_possible): {e}")

    def _auto_warning_threshold(self) -> int:
        try:
            files = self._MAX_FILES_POSSIBLE

            if files < 2:
                raise RuntimeError(
                    "Invalid config: Warning threshold cannot be 100%"
                )

            if files == 2:
                return 50
            elif files == 3:
                return 66
            elif files == 4:
                return 75
            else:
                return 70
            
        except Exception as e:
            print(f"Exception in LoggerConfig (_auto_warning_threshold): {e}")
           
           

    # =====================================================
    # VALIDATION (FAIL-FAST)
    # =====================================================
    def _validate(self):
        try:
            if not (0 < self._STORAGE_THRESHOLD_PERCENT <= 100):
                raise RuntimeError(
                "Invalid STORAGE_THRESHOLD_PERCENT (must be 1–100)"
            )

            if not (0 < self._MAX_DIRECTORY_WARNING_THRESHOLD < 100):
                raise RuntimeError(
                    "Invalid MAX_DIRECTORY_WARNING_THRESHOLD (must be 1–99)"
                )

            if self._MAX_DIRECTORY_WARNING_THRESHOLD >= 100:
                raise RuntimeError(
                    "Warning threshold cannot be 100%"
                )
        except Exception as e:
            print(f"Exception in LoggerConfig (_validate): {e}")
            

    # =====================================================
    # UPDATE CONFIG (AUTO WARNING IS LOCKED)
    # =====================================================
    def update_config(self, updates: dict):
        try:
            # self._path is now an absolute Path object, so this works safely
            with open(self._path, "r") as f:
                data = yaml.safe_load(f)

            raw_warning = data["storage"]["max_dir_warning_threshold"]

            for key, value in updates.items():
                
                if (
                    key == "MAX_DIRECTORY_WARNING_THRESHOLD"
                    and raw_warning == "auto"
                ):
                    print(
                        "[WARNING] MAX_DIRECTORY_WARNING_THRESHOLD is auto-managed "
                        "and cannot be changed manually."
                    )
                    continue

                if key not in KEY_MAP:
                    raise KeyError(f"Unknown config key: {key}")

                expected = TYPE_MAP[key]
                if not isinstance(value, expected):
                    raise TypeError(
                        f"{key} must be {expected}, got {type(value).__name__}"
                    )

                section, field = KEY_MAP[key]
                data[section][field] = value
            
            if raw_warning == "auto":
                data["storage"]["max_dir_warning_threshold"] = "auto"
            
            # Create temp file
            tmp = self._path.with_suffix(".tmp")
            
            with open(tmp, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)

            # Atomic replace
            tmp.replace(self._path)
            
            # Re-initialize to reload values
            self.__init__(self._path)

        except Exception as e:
            print(f"Exception in LoggerConfig (update_config): {e}")

    @property
    def LOG_DIRECTORY(self):
        return self._LOG_DIRECTORY

    @LOG_DIRECTORY.setter
    def LOG_DIRECTORY(self, value: str):
        if not isinstance(value, str):
            raise TypeError("LOGDIRECTORY must be string")

        self.update_config({"LOG_DIRECTORY": value})

    @property
    def LOG_DIRECTORY_MAX_SIZE_MB(self) ->  Union[int, float]:
        return self._LOG_DIRECTORY_MAX_SIZE_MB

    @LOG_DIRECTORY_MAX_SIZE_MB.setter
    def LOG_DIRECTORY_MAX_SIZE_MB(self, value: Union[int, float]):
        if not isinstance(value, int):
            raise TypeError("LOGDIRECTORYMAXSIZEMB must be int")
        self.update_config({"LOG_DIRECTORY_MAX_SIZE_MB": value})

    @property
    def MAX_FILE_SIZE_MB(self) -> Union[int, float]:  
        return self._MAX_FILE_SIZE_MB

    @MAX_FILE_SIZE_MB.setter
    def MAX_FILE_SIZE_MB(self, value: Union[int, float]):
        if not isinstance(value, (int, float)):
            raise TypeError("MAXFILESIZEMB must be int or float")
        self.update_config({"MAX_FILE_SIZE_MB": value})

    @property
    def QUEUE_SIZE(self) -> int:
        return self._QUEUE_SIZE

    @QUEUE_SIZE.setter
    def QUEUE_SIZE(self, value: int):
        if not isinstance(value, int):
            raise TypeError("QUEUESIZE must be int")
        self.update_config({"QUEUE_SIZE": value})


    @property
    def XLSX_MAX_ROWS(self) -> int:
        return self._XLSX_MAX_ROWS

    @XLSX_MAX_ROWS.setter
    def XLSX_MAX_ROWS(self, value: int):
        if not isinstance(value, int):
            raise TypeError("XLSXMAXROWS must be int")
        self.update_config({"XLSX_MAX_ROWS": value})


    @property
    def STORAGE_THRESHOLD_PERCENT(self) -> int:
        return self._STORAGE_THRESHOLD_PERCENT

    @STORAGE_THRESHOLD_PERCENT.setter
    def STORAGE_THRESHOLD_PERCENT(self, value: int):
        if not isinstance(value, int):
            raise TypeError("STORAGETHRESHOLDPERCENT must be int")
        self.update_config({"STORAGE_THRESHOLD_PERCENT": value})



try:
    settings = LoggerConfig()
except Exception as e:
    print(f"[FATAL CONFIG ERROR] {e}")
    raise SystemExit(1)