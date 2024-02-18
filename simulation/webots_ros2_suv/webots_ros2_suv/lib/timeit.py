from functools import wraps
import time
import inspect

def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        # first item in the args, ie `args[0]` is `self`
        if hasattr(args[0], "_logger"):
            args[0]._logger.info(f'Function {func.__name__} took {total_time:.4f} seconds')
        else:
            args[0].log(f'Function {func.__name__} from {type(args[0]).__name__} took {total_time:.4f} seconds')
        return result
    return timeit_wrapper