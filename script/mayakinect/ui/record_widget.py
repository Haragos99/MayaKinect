"""Recording controls for the Kinect panel: Record button with countdown, takes list.

The widget only shows state and emits signals; the panel runs the Maya commands.
"""

from PySide6 import QtCore, QtWidgets

COUNTDOWN_SECONDS = 3

_RECORD_STYLE = (
    "QPushButton { font-weight: bold; }"
    "QPushButton:checked { background-color: #b02a2a; color: white; }"
)


class RecordWidget(QtWidgets.QWidget):
    countdown_tick = QtCore.Signal(int)     # 3, 2, 1 during the countdown, 0 when it ends/cancels
    record_started = QtCore.Signal()        # countdown finished: start recording now
    record_stopped = QtCore.Signal()        # user pressed Record again while recording
    save_requested = QtCore.Signal(str)     # take name
    load_requested = QtCore.Signal(str)     # take file path
    delete_requested = QtCore.Signal(str)   # take file path

    def __init__(self, parent=None):
        super().__init__(parent)
        self._recording = False
        self._remaining = 0
        self._countdown = QtCore.QTimer(self)
        self._countdown.setInterval(1000)
        self._countdown.timeout.connect(self._tick)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Record + status
        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setCheckable(True)
        self.record_btn.setMinimumHeight(40)
        self.record_btn.setStyleSheet(_RECORD_STYLE)
        self.record_btn.clicked.connect(self._on_record_clicked)
        layout.addWidget(self.record_btn)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setStyleSheet("color: gray;")
        layout.addWidget(self.status_label)

        # Take name + save
        name_row = QtWidgets.QHBoxLayout()
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText("Take name")
        self.save_btn = QtWidgets.QPushButton("Save Take")
        self.save_btn.clicked.connect(lambda: self.save_requested.emit(self.name_edit.text().strip()))
        name_row.addWidget(self.name_edit)
        name_row.addWidget(self.save_btn)
        layout.addLayout(name_row)

        self.autosave_check = QtWidgets.QCheckBox("Auto-save take when recording stops")
        layout.addWidget(self.autosave_check)

        # Saved takes
        layout.addWidget(QtWidgets.QLabel("Saved takes:"))
        self.takes_list = QtWidgets.QListWidget()
        self.takes_list.setMaximumHeight(120)
        self.takes_list.itemDoubleClicked.connect(lambda _item: self._emit_selected(self.load_requested))
        layout.addWidget(self.takes_list)

        take_row = QtWidgets.QHBoxLayout()
        self.load_btn = QtWidgets.QPushButton("Load Take")
        self.load_btn.clicked.connect(lambda: self._emit_selected(self.load_requested))
        self.delete_btn = QtWidgets.QPushButton("Delete")
        self.delete_btn.clicked.connect(lambda: self._emit_selected(self.delete_requested))
        take_row.addWidget(self.load_btn)
        take_row.addWidget(self.delete_btn)
        layout.addLayout(take_row)

    # ------------------------------------------------------------ state from the panel

    @property
    def counting_down(self):
        return self._countdown.isActive()

    def set_recording(self, recording):
        """Show recording on/off (also used to reset the button after a failed start)."""
        self._recording = recording
        self.record_btn.blockSignals(True)
        self.record_btn.setChecked(recording)
        self.record_btn.setText("■ Stop Recording" if recording else "● Record")
        self.record_btn.blockSignals(False)

    def set_status(self, text):
        self.status_label.setText(text)

    def set_take_name(self, name):
        self.name_edit.setText(name)

    def set_takes(self, infos):
        """Fill the list from takes.list_takes() metadata."""
        self.takes_list.clear()
        for info in infos:
            frames = int(info.get("end", 0)) - int(info.get("start", 0)) + 1
            fps = float(info.get("fps", 30.0)) or 30.0
            text = "{}   {:.1f} s   {}".format(info.get("name", "?"), frames / fps, info.get("date", ""))
            item = QtWidgets.QListWidgetItem(text)
            item.setData(QtCore.Qt.UserRole, info.get("path"))
            self.takes_list.addItem(item)

    def cancel_countdown(self):
        if self._countdown.isActive():
            self._countdown.stop()
            self.countdown_tick.emit(0)
        if not self._recording:
            self.set_recording(False)

    # ------------------------------------------------------------ internals

    def _on_record_clicked(self, checked):
        if self._recording:
            self.record_stopped.emit()
        elif self._countdown.isActive():
            self.cancel_countdown()          # second click during the countdown cancels it
        elif checked:
            self._remaining = COUNTDOWN_SECONDS
            self.record_btn.setText("Cancel ({})".format(self._remaining))
            self.countdown_tick.emit(self._remaining)
            self._countdown.start()

    def _tick(self):
        self._remaining -= 1
        if self._remaining > 0:
            self.record_btn.setText("Cancel ({})".format(self._remaining))
            self.countdown_tick.emit(self._remaining)
            return
        self._countdown.stop()
        self.countdown_tick.emit(0)
        self.record_started.emit()

    def _emit_selected(self, signal):
        item = self.takes_list.currentItem()
        if item is not None:
            signal.emit(item.data(QtCore.Qt.UserRole))
