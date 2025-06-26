
import threading

class SafeDict:
    def __init__(self):
        self._lock = threading.Lock()
        self._dict = dict()

    def get(self, key, default=None):
        with self._lock:
            return self._dict.get(key, default)

    def add(self, key, value):
        with self._lock:
            self._dict[key] = value  # 直接赋值，覆盖已有key的值

    def remove(self, key):
        with self._lock:
            self._dict.pop(key, None)

    def items(self):
        with self._lock:
            return list(self._dict.items())
