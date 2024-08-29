from threading import Lock

class ThreadingDict:
    def __init__(self) -> None:
        self.data = dict()
        self.lock = Lock()
    
    def update(self, k, v):
        with self.lock:
            self.data[k] = v
    
    def read(self, k):
        with self.lock:
            return self.data[k]

    def read_and_flip(self, k, v):
        valid = False
        with self.lock:
            if self.data.get(k, not v) == v:
                valid = True
                self.data[k] = not v
        return valid
    
    def clear(self):
        with self.lock:
            self.data = dict()
    
    def items(self):
        with self.lock:
            c = dict(self.data)
        return c
    
    def items_and_clear(self):
        with self.lock:
            c = dict(self.data)
            self.data = dict()
        return c
    
