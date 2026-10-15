from __future__ import annotations

import threading
import traceback
from collections.abc import Callable
from typing import Any

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal


class JobSignals(QObject):
    progress = Signal(float, str)
    result = Signal(object)
    error = Signal(str)
    finished = Signal()


class Worker(QRunnable):
    def __init__(self, operation: Callable[..., Any]) -> None:
        super().__init__()
        self.operation = operation
        self.signals = JobSignals()
        self.cancel_event = threading.Event()

    def cancel(self) -> None:
        self.cancel_event.set()

    def run(self) -> None:
        try:
            result = self.operation(
                progress=lambda value, message: self.signals.progress.emit(float(value), str(message)),
                cancelled=self.cancel_event.is_set,
            )
            if self.cancel_event.is_set(): raise InterruptedError("Operation cancelled")
            self.signals.result.emit(result)
        except InterruptedError:
            self.signals.error.emit("Operation cancelled")
        except Exception:
            self.signals.error.emit(traceback.format_exc())
        finally:
            self.signals.finished.emit()


class JobQueue(QObject):
    busy_changed = Signal(bool)
    progress = Signal(float, str)
    result = Signal(object)
    error = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.pool = QThreadPool(self)
        self.pool.setMaxThreadCount(1)
        self.current: Worker | None = None

    def start(self, operation: Callable[..., Any]) -> bool:
        if self.current is not None:
            return False
        worker = Worker(operation)
        self.current = worker
        worker.signals.progress.connect(self.progress)
        worker.signals.result.connect(self.result)
        worker.signals.error.connect(self.error)
        worker.signals.finished.connect(lambda: self._finished(worker))
        self.busy_changed.emit(True)
        self.pool.start(worker)
        return True

    def cancel(self) -> None:
        if self.current:
            self.current.cancel()

    def _finished(self,worker=None) -> None:
        if worker is not None and self.current is not worker:return
        if self.current is None:return
        self.current = None
        self.busy_changed.emit(False)

