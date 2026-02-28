import multiprocessing as mp
from multiprocessing import shared_memory
import numpy as np

class SharedFrame:
    def __init__(self, name=None, shape=(1080,1920, 3), dtype=np.uint8):
        self.shape = shape
        self.dtype = dtype
        self.size = np.prod(shape) * np.dtype(dtype).itemsize
 
        if name is None:
            self.shm = shared_memory.SharedMemory(create=True, size=self.size)
            self.name = self.shm.name
        else:
            self.shm = shared_memory.SharedMemory(name=name)
            self.name = name
 
        self.array = np.ndarray(self.shape, dtype=self.dtype, buffer=self.shm.buf)
 
    def write(self, frame):
        np.copyto(self.array, frame)
 
    def read(self):
        return self.array.copy()
 
    def close(self):
        self.shm.close()
 
    def unlink(self):
        self.shm.unlink()
