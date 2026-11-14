"""Pop-up dialog: pick a character model by clicking its picture."""

from PySide6 import QtCore, QtWidgets

from ..character import characters, models as model_lib
from .icons import model_button

ICON_SIZE = 128
COLUMNS = 3


class ModelPickerDialog(QtWidgets.QDialog):
    """Grid of image buttons, one per model. After exec() == Accepted, `selected` is the ModelInfo."""

    def __init__(self, models, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Character")
        self.setModal(True)
        self.selected = None

        layout = QtWidgets.QVBoxLayout(self)

        if not models:
            layout.addWidget(QtWidgets.QLabel("No models found in the models folder.\nUse Browse... to pick an OBJ file."))

        # --- Grid of model buttons in a scroll area
        grid_widget = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_widget)
        grid.setSpacing(8)
        for i, model in enumerate(models):
            button = model_button(model, ICON_SIZE, checked=characters.is_active(model), parent=grid_widget)
            if characters.find_group(model):
                button.setToolTip(button.toolTip() + "\n(already in the scene, switches instantly)")
            button.clicked.connect(lambda _=False, m=model: self._pick(m))
            grid.addWidget(button, i // COLUMNS, i % COLUMNS)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(grid_widget)
        scroll.setMinimumSize(COLUMNS * (ICON_SIZE + 30) + 20, ICON_SIZE + 80)
        layout.addWidget(scroll)

        # --- Browse / Cancel
        buttons = QtWidgets.QHBoxLayout()
        browse = QtWidgets.QPushButton("Browse...")
        browse.clicked.connect(self._browse)
        cancel = QtWidgets.QPushButton("Cancel")
        cancel.clicked.connect(self.reject)
        buttons.addWidget(browse)
        buttons.addStretch()
        buttons.addWidget(cancel)
        layout.addLayout(buttons)

    def _pick(self, model):
        self.selected = model
        self.accept()

    def _browse(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Import OBJ character", "", "OBJ files (*.obj)")
        if path:
            self._pick(model_lib.from_path(path))
