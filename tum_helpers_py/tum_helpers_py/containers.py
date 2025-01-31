from typing import Callable

class function_queue:
    def __init__(self) -> None:
        self.functions = []

    def push_back(self, func: Callable) -> None:
        self.functions.append(func)

    def call(self) -> None:
        for func in self.functions:
            func()

    def get_function_queue(self) -> Callable:
        return self.call
