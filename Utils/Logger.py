import os
import csv
import json
from datetime import datetime
from threading import Lock
import numpy as np

class Logger:
    _instance = None
    _lock = Lock()

    def __new__(cls, log_dir=".", log_file=None, csv_header=None, float_precision=2):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._init(log_dir, log_file, csv_header, float_precision)
            return cls._instance

    def _init(self, log_dir, log_file, csv_header, float_precision):
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)

        if log_file is None:
            date_str = datetime.now().strftime("%Y-%m-%d")
            log_file = f"log_{date_str}.csv"

        self.log_path = os.path.join(log_dir, log_file)

        self._csv_header = csv_header or []
        self._csv_header.insert(0, "time")
        self._written_header = False
        self._float_precision = float_precision
        self.rows_printed = 0

        self._file = open(self.log_path, "a", newline='', encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self._csv_header)

        self._cache = []  # 缓存结构：[(args, kwargs)]
        self._cache_lock = Lock()

    def _format_value(self, value):
        # 支持numpy.ndarray转换成列表后处理
        if isinstance(value, float):
            return f"{value:.{self._float_precision}f}"
        elif isinstance(value, np.ndarray):
            value = value.tolist()
            return json.dumps(self._format_complex(value), ensure_ascii=False)
        elif isinstance(value, (list, tuple, dict)):
            try:
                formatted = self._format_complex(value)
                return json.dumps(formatted, ensure_ascii=False)
            except Exception:
                return str(value)
        else:
            return str(value)

    def _format_complex(self, value):
        if isinstance(value, float):
            return round(value, self._float_precision)
        elif isinstance(value, dict):
            return {k: self._format_complex(v) for k, v in value.items()}
        elif isinstance(value, (list, tuple)):
            return [self._format_complex(v) for v in value]
        else:
            return value

    def log(self, *args, **kwargs):
        with self._cache_lock:
            formatted_args = tuple(self._format_value(arg) for arg in args)
            formatted_kwargs = {k: self._format_value(v) for k, v in kwargs.items()}
            self._cache.append((formatted_args, formatted_kwargs))

    def flush(self, total_rows):
        with self._cache_lock:
            if not self._cache:
                return

            if not self._written_header:
                self._writer.writeheader()
                self._written_header = True

            self.rows_printed = 0
            timestamp = datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
            print(timestamp)
            self.rows_printed += 1
            merged_row = {}
            merged_row["time"] = timestamp

            for args, kwargs in self._cache:
                arg_strs = [str(arg) for arg in args]
                kwarg_strs = [f"{k}={v}" for k, v in kwargs.items()]
                print(", ".join(arg_strs + kwarg_strs))
                self.rows_printed += 1
                for k, v in kwargs.items():
                    if k in self._csv_header:
                        merged_row[k] = v
            if merged_row:
                self._writer.writerow(merged_row)
            for i in range(total_rows - self.rows_printed):
                print("")

            self._file.flush()
            self._cache.clear()

    def close(self):
        self.flush()
        self._file.close()

    def __del__(self):
        try:
            self.close()
        except:
            pass
