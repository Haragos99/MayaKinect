"""Model bar inside the panel: one click switches the character, also while streaming."""

from PySide6 import QtCore, QtWidgets

from ..character import characters
from .icons import model_button

ICON_SIZE = 64
COLUMNS = 2


class ModelBar(QtWidgets.QWidget):
    """Grid of checkable thumbnail buttons; the active model is highlighted."""

    model_selected = QtCore.Signal(object)   # ModelInfo

    def __init__(self, parent=None):
        super().__init__(parent)
        self._group = QtWidgets.QButtonGroup(self)
        self._group.setExclusive(True)
        self._grid = QtWidgets.QGridLayout(self)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setSpacing(4)
        self._empty = QtWidgets.QLabel("No models yet")
        self._empty.setStyleSheet("color: gray;")

    def refresh(self, models):
        """Rebuild the buttons for `models` and highlight the active one."""
        for button in self._group.buttons():
            self._group.removeButton(button)
            button.deleteLater()
        self._grid.removeWidget(self._empty)
        self._empty.hide()

        if not models:
            self._grid.addWidget(self._empty, 0, 0)
            self._empty.show()
            return

        for i, model in enumerate(models):
            button = model_button(model, ICON_SIZE, checked=characters.is_active(model), parent=self)
            button.clicked.connect(lambda _=False, m=model: self.model_selected.emit(m))
            self._group.addButton(button)
            self._grid.addWidget(button, i // COLUMNS, i % COLUMNS)
