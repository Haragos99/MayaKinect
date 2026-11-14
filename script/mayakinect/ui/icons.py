"""Thumbnails for model buttons, with a placeholder when a model has no image."""

from PySide6 import QtCore, QtGui


def placeholder_pixmap(text, size):
    """Grey square with the model name written on it."""
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(QtGui.QColor(70, 70, 70))
    painter = QtGui.QPainter(pixmap)
    painter.setPen(QtGui.QColor(220, 220, 220))
    font = painter.font()
    font.setPointSize(max(7, size // 10))
    painter.setFont(font)
    margin = size // 10
    painter.drawText(
        QtCore.QRect(margin, margin, size - 2 * margin, size - 2 * margin),
        QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap,
        text,
    )
    painter.end()
    return pixmap


def model_icon(model, size):
    """QIcon from the model's thumbnail, or a placeholder."""
    if model.thumb is not None:
        pixmap = QtGui.QPixmap(str(model.thumb))
        if not pixmap.isNull():
            return QtGui.QIcon(pixmap.scaled(size, size, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
    return QtGui.QIcon(placeholder_pixmap(model.display_name, size))


def model_button(model, icon_size, checked=False, parent=None):
    """Checkable tool button: thumbnail with the model name under it."""
    from PySide6 import QtWidgets

    button = QtWidgets.QToolButton(parent)
    button.setIcon(model_icon(model, icon_size))
    button.setIconSize(QtCore.QSize(icon_size, icon_size))
    button.setText(model.display_name)
    button.setToolTip(str(model.obj))
    button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
    button.setCheckable(True)
    button.setChecked(checked)
    button.setAutoRaise(True)
    button.setStyleSheet(
        "QToolButton { padding: 4px; border: 2px solid transparent; border-radius: 4px; }"
        "QToolButton:hover { border-color: #5a5a5a; }"
        "QToolButton:checked { border-color: #4ea3e0; background-color: rgba(78, 163, 224, 40); }"
    )
    return button
