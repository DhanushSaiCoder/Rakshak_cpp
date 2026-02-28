import math
import datetime
import gzip
import shutil
from pathlib import Path
from .global_config import settings

class FileManager:

    def __init__(self, file_type: str, 
                log_directory: Path = None, 
                max_file_size_mb: int = None, 
                dir_max_size_mb: int = None, 
                compress: bool = False,
                file_keyword: str = None):
        
        try:
            self.file_type = file_type.lstrip(".")
            self.log_dir = log_directory or settings._LOG_DIRECTORY
            self.max_file_size = (max_file_size_mb or settings._MAX_FILE_SIZE_MB) * 1024 * 1024
            self.dir_max_size = (dir_max_size_mb or settings._LOG_DIRECTORY_MAX_SIZE_MB) * 1024 * 1024
            self.compress = compress
            self.max_uncompressed_files = int(settings._LOG_DIRECTORY_MAX_SIZE_MB / settings._MAX_FILE_SIZE_MB)

            self.warning_bytes = (
                settings._LOG_DIRECTORY_MAX_SIZE_MB *
                settings._MAX_DIRECTORY_WARNING_THRESHOLD / 100 *
                1024 * 1024
            )
            self.flag = True
            self.file_keyword = file_keyword


            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.current_file = self._new_log_file()
            self.gz_files_to_delete = 0
            self.gz_deleted_size = 0
        except Exception as e:
            raise RuntimeError(f"Exception from FileManager (__init__): {e}") from e

    def _new_log_file(self):
        try:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            if self.file_keyword == None:
                path = self.log_dir / f"log_{ts}.{self.file_type}"
            else:
                path = self.log_dir / f"{self.file_keyword}_log_{ts}.{self.file_type}"
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path.touch(exist_ok=False)
            return path
        except Exception as e:
            raise RuntimeError(f"Exception from FileManager (_new_log_file): {e}") from e




    def directory_size(self):
        # total_size = 0

        # for f in self.log_dir.iterdir():
        #     if f.is_file():
        #         file_size = f.stat().st_size
        #         total_size += file_size

        # return total_size

        return sum(f.stat().st_size for f in self.log_dir.iterdir() if f.is_file())
    
    #=================================== COMPRESSOR ======================================
    def compress_worker(self, compress_file):
        
        try:
            gz_path = compress_file.with_suffix(compress_file.suffix + ".gz")

            with open(compress_file, "rb") as f_in, gzip.open(gz_path, "wb") as f_out:
                shutil.copyfileobj(f_in, f_out)
                compress_file.unlink()
        except Exception as e:
            raise RuntimeError(f"Exception from FileManager (compress_worker): {e}") from e

    #=================================== GZ_SORTER ========================================
    def gz_files_sort(self):
        try:
            files = sorted(
                (f for f in self.log_dir.rglob("*.gz") if f.is_file()),
                key=lambda f: f.stat().st_mtime
            )

            return files
        except Exception as e:
            raise RuntimeError(f"Exception from FileManager (gz_files_sort): {e}") from e
            

    #================================= COMPRESSOR LOGS ====================================
    def compress_logs(self):

        try:
            if not self.compress:
                dir_size = self.directory_size()
                all_files = [
                    f for f in self.log_dir.iterdir()
                    if f.is_file()
                ]
                if dir_size >= self.warning_bytes:
                    print(f"[WARNING] {self.log_dir} size ({dir_size/1e6:.2f} MB) exceeds warning limit!")

                if self.file_type == "xlsx":
                    if dir_size >= self.dir_max_size:
                        raise RuntimeError(f"Max Dir Size Hit {dir_size/1e6:.2f} MB. Logger Stopped!")
                    else:
                        return 
                
                elif len(all_files) > self.max_uncompressed_files:
                    raise RuntimeError(f"Max Dir Size Hit {dir_size/1e6:.2f} MB. Logger Stopped!")

                return
            
            if self.file_type == "xlsx":
                all_files = [
                    f for f in self.log_dir.iterdir()
                    if f.is_file() and f.suffix == f".{self.file_type}"
                ]

                all_files.sort(key=lambda f: f.stat().st_mtime, reverse=True)

                files_to_keep = 5 

                if self.flag and all_files:

                    oldest_file = all_files[-1]
                    

                    reference_size = oldest_file.stat().st_size
                    

                    if reference_size == 0: reference_size = 1024 * 1024 

                    self.max_file_size = reference_size


                    limit_bytes = self.dir_max_size
                    files_to_keep = math.floor(limit_bytes / reference_size)
                    

                    files_to_keep = max(files_to_keep, 1)



                    for old_file in all_files[files_to_keep:]:
                        self.compress_worker(old_file)

                    dir_size = self.directory_size()
                    
                    if dir_size >= self.warning_bytes:
                        print(f"[WARNING] {self.log_dir} size ({dir_size/1e6:.2f} MB) exceeds warning limit!")

                
                        
                    if dir_size >= self.dir_max_size:

                        gz_files = sorted(
                            (
                                f for f in self.log_dir.iterdir()
                                if f.is_file() and f.suffix == ".gz"
                            ),
                            key=lambda f: f.stat().st_mtime  
                        )

                        self.gz_files_to_delete = self.max_file_size - self.current_file.stat().st_size
                    
                        for old_gz in gz_files:
                            size = old_gz.stat().st_size
                            self.gz_deleted_size += size
                            old_gz.unlink()
                            print(f"[Logger] Deleted {old_gz} to free space.")
                            dir_size -= size
                            # print(f"gz_files_to_delete: {self.gz_files_to_delete} and current_file: {self.current_file.stat().st_size} and gz_deleted_size: {self.gz_deleted_size}")
                            
                            if self.gz_deleted_size >= self.gz_files_to_delete:
                                raise RuntimeError(
                                f"[CRITICAL] Logging stopped: {self.log_dir} exceeds "
                                f"{self.dir_max_size // (1024 * 1024)} MB."
                            )

                            if dir_size < self.dir_max_size:
                                break
                    
                    return


            all_files = [
                f for f in self.log_dir.iterdir()
                if f.is_file() and f.suffix == f".{self.file_type}"
            ]

            all_files.sort(key=lambda f: f.stat().st_mtime, reverse=True)

            for old_file in all_files[self.max_uncompressed_files:]:
                self.compress_worker(old_file)

            dir_size = self.directory_size()


            if dir_size >= self.warning_bytes:
                print(
                    f"[WARNING] {self.log_dir} exceeds "
                    f"{self.warning_bytes / 1000000} MB!"
                )

            if dir_size >= self.dir_max_size:

                gz_files = sorted(
                    (
                        f for f in self.log_dir.iterdir()
                        if f.is_file() and f.suffix == ".gz"
                    ),
                    key=lambda f: f.stat().st_mtime  
                )

                self.gz_files_to_delete = self.max_file_size - self.current_file.stat().st_size
            
                for old_gz in gz_files:
                    size = old_gz.stat().st_size
                    self.gz_deleted_size += size
                    old_gz.unlink()
                    print(f"[Logger] Deleted {old_gz} to free space.")
                    dir_size -= size
                    # print(f"gz_files_to_delete: {self.gz_files_to_delete} and current_file: {self.current_file.stat().st_size} and gz_deleted_size: {self.gz_deleted_size}")
                    
                    if self.gz_deleted_size >= self.gz_files_to_delete:
                        raise RuntimeError(
                        f"[CRITICAL] Logging stopped: {self.log_dir} exceeds "
                        f"{self.dir_max_size // (1024 * 1024)} MB."
                    )

                    if dir_size < self.dir_max_size:
                        break
        except Exception as e:
            raise RuntimeError(f"Exception from FileManager (compress_logs): {e}") from e
            
    