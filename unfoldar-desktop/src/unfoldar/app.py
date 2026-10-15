from __future__ import annotations

import os
import sys
from pathlib import Path

_APP_ICON = Path(__file__).parent / "resources" / "app_icon.svg"


def main() -> int:
    os.environ.setdefault("QT_API", "pyside6")
    from PySide6.QtGui import QIcon
    from PySide6.QtWidgets import QApplication
    from .viewer import MainWindow

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("UnfoLDAR Desktop")
    app.setOrganizationName("UnfoLDAR")
    if _APP_ICON.exists():
        app.setWindowIcon(QIcon(str(_APP_ICON)))
    window = MainWindow()
    screen = app.primaryScreen()
    if screen:
        available = screen.availableGeometry()
        window.resize(
            min(window.width(), available.width() - 32),
            min(window.height(), available.height() - 64),
        )
    window.show()
    return app.exec()

