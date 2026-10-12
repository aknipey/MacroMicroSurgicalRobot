"""CSV logging that never blocks the caller.

Callbacks put rows on a queue; a background thread writes them to disk, so a
slow disk cannot stall the ROS executor that is receiving 500-1000 Hz data.
"""

import csv
import queue
import threading

_CLOSE = object()


class CsvStream:
    """One CSV file fed from a queue by its own writer thread."""

    def __init__(self, path, header):
        self.path = path
        self.rows = 0
        self._queue = queue.SimpleQueue()
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._thread = threading.Thread(target=self._run, name=f"csv:{path}", daemon=True)
        self._thread.start()

    def write(self, row):
        self.rows += 1
        self._queue.put(row)

    def close(self):
        """Flush the queue and close the file (waits for the writer thread)."""
        self._queue.put(_CLOSE)
        self._thread.join()

    def _run(self):
        try:
            while True:
                row = self._queue.get()
                if row is _CLOSE:
                    break
                self._writer.writerow(row)
        finally:
            self._file.close()
