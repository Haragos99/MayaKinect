import maya.cmds as cmds
import ctypes
import os
import sys
from enum import Enum
from PySide6 import QtWidgets, QtGui, QtCore
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin

# Make the mayakinect package importable (loadKinect.py does this too). The fallback is
# for running this file with exec() from the Script Editor, where __file__ is not set.
try:
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _SCRIPT_DIR = r"C:\Users\Geri\Documents\Projects\CG\MayaKinect\script"
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from mayakinect.anim import takes
from mayakinect.character import characters, models as model_lib
from mayakinect.ui.model_bar import ModelBar
from mayakinect.ui.model_picker import ModelPickerDialog
from mayakinect.ui.record_widget import RecordWidget

RECORD_FPS = 30.0   # Kinect v1 skeleton rate


class Mode(Enum):
    IDLE = "Idle"             # nothing streaming, no take on the skeleton
    LIVE = "Live"             # streaming, skeleton follows the Kinect
    RECORDING = "Recording"   # streaming and collecting a take
    REVIEW = "Review"         # stream stopped, take keyed on the skeleton (play the timeline)


# Which controls are enabled in which mode (PLAN.md C3). Models can always be switched.
_ENABLED = {
    "start":  {Mode.IDLE, Mode.REVIEW},
    "stop":   {Mode.LIVE, Mode.RECORDING},
    "record": {Mode.LIVE, Mode.RECORDING},
    "filter": {Mode.LIVE, Mode.RECORDING},
    "save":   {Mode.REVIEW},
    "takes":  {Mode.IDLE, Mode.REVIEW},
}


class KinectMayaPanel(MayaQWidgetDockableMixin, QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(KinectMayaPanel, self).__init__(parent=parent)
        self.setWindowTitle("Kinect Control Panel")
        self.resize(900, 620) # Slightly wider to accommodate the sidebar

        # --- Main Layout (Horizontal Splitter) ---
        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.main_layout.addWidget(self.splitter)

        # --- Left Side: Controls (scrollable, the column got long) ---
        self.control_widget = QtWidgets.QWidget()
        self.control_layout = QtWidgets.QVBoxLayout(self.control_widget)
        self.control_layout.setAlignment(QtCore.Qt.AlignTop)
        self.control_scroll = QtWidgets.QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setWidget(self.control_widget)
        self.control_scroll.setMinimumWidth(240)

        self.mode_label = QtWidgets.QLabel()
        self.start_btn = QtWidgets.QPushButton("Start Stream")
        self.stop_btn = QtWidgets.QPushButton("Stop Stream")
        self.import_btn = QtWidgets.QPushButton("Import Model")
        self.rotation_filter_checkbox = QtWidgets.QCheckBox("Rotation filter")
        self.rotation_filter_checkbox.setChecked(True)

        # Styling buttons for visibility
        self.start_btn.setMinimumHeight(40)
        self.stop_btn.setMinimumHeight(40)
        self.import_btn.setMinimumHeight(40)

        self.control_layout.addWidget(QtWidgets.QLabel("<b>Kinect Controls</b>"))
        self.control_layout.addWidget(self.mode_label)
        self.control_layout.addWidget(self.start_btn)
        self.control_layout.addWidget(self.stop_btn)
        self.control_layout.addWidget(self.rotation_filter_checkbox)

        # Recording: Record button with countdown, takes
        self.control_layout.addSpacing(10)
        self.control_layout.addWidget(QtWidgets.QLabel("<b>Recording</b>"))
        self.record_widget = RecordWidget()
        self.control_layout.addWidget(self.record_widget)

        # Character models: click to switch (also while streaming)
        self.control_layout.addSpacing(10)
        self.control_layout.addWidget(QtWidgets.QLabel("<b>Characters</b>"))
        self.control_layout.addWidget(self.import_btn)
        self.model_bar = ModelBar()
        self.control_layout.addWidget(self.model_bar)

        # --- Right Side: Image Feed ---
        self.image_container = QtWidgets.QWidget()
        self.image_layout = QtWidgets.QVBoxLayout(self.image_container)
        self.image_label = QtWidgets.QLabel("Kinect Standby...")
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        # Background color for the feed area
        self.image_label.setStyleSheet("background-color: black; color: white;")
        self.image_layout.addWidget(self.image_label)

        # Add widgets to splitter
        self.splitter.addWidget(self.control_scroll)
        self.splitter.addWidget(self.image_container)
        self.splitter.setStretchFactor(1, 4) # Give more space to the image

        # --- Kinect Logic Setup ---
        self.width = 640
        self.height = 480
        self.channels = 4
        self.buffer_size = self.width * self.height * self.channels
        self.image_buffer = ctypes.create_string_buffer(self.buffer_size)
        self.memory_address = str(ctypes.addressof(self.image_buffer))

        # Recording state
        self.mode = Mode.IDLE
        self.take_saved = True          # False while a recorded take exists only in the scene
        self.countdown_value = 0        # 3-2-1 shown over the camera image
        self.record_clock = QtCore.QElapsedTimer()

        # Timer setup (Starts active by default)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        # Button Connections
        self.start_btn.clicked.connect(self.start_stream)
        self.stop_btn.clicked.connect(self.stop_stream)
        self.import_btn.clicked.connect(self.import_model)
        self.rotation_filter_checkbox.toggled.connect(self.set_rotation_filter)
        self.model_bar.model_selected.connect(self.switch_model)
        self.record_widget.countdown_tick.connect(self.on_countdown_tick)
        self.record_widget.record_started.connect(self.start_recording)
        self.record_widget.record_stopped.connect(self.stop_recording)
        self.record_widget.save_requested.connect(self.save_take)
        self.record_widget.load_requested.connect(self.load_take)
        self.record_widget.delete_requested.connect(self.delete_take)

        self.refresh_models()
        self.refresh_takes()
        self.record_widget.set_take_name(takes.next_take_name())
        # A take already on the skeleton (e.g. reopened scene) starts in Review
        self.take_saved = not takes.mocap_has_keys()
        self.set_mode(Mode.REVIEW if takes.mocap_has_keys() else Mode.IDLE)

    #  modes

    def set_mode(self, mode):
        self.mode = mode
        enabled = {name: mode in modes for name, modes in _ENABLED.items()}
        self.start_btn.setEnabled(enabled["start"])
        self.stop_btn.setEnabled(enabled["stop"])
        self.record_widget.record_btn.setEnabled(enabled["record"])
        self.rotation_filter_checkbox.setEnabled(enabled["filter"])
        self.record_widget.save_btn.setEnabled(enabled["save"])
        self.record_widget.name_edit.setEnabled(enabled["save"] or mode == Mode.LIVE)
        for w in (self.record_widget.load_btn, self.record_widget.delete_btn, self.record_widget.takes_list):
            w.setEnabled(enabled["takes"])

        colors = {Mode.IDLE: "gray", Mode.LIVE: "#3fa33f", Mode.RECORDING: "#d03030", Mode.REVIEW: "#4ea3e0"}
        self.mode_label.setText('Mode: <b style="color:{}">{}</b>'.format(colors[mode], mode.value))

    # stream

    def start_stream(self):
        if self.mode not in (Mode.IDLE, Mode.REVIEW) or self.timer.isActive():
            return

        # Keys on the skeleton would fight with the live data: save or discard them first
        if takes.mocap_has_keys() and not self.resolve_current_take("Start the live stream"):
            return

        # Try to init hardware
        try:
            started = cmds.runMayaKinect()
        except Exception:
            self.image_label.setText("MayaKinect plugin not loaded! Run loadKinect.py first.")
            return
        if not started:
            self.image_label.setText("No Kinect found. Check the sensor and its power supply.")
            return

        self.rotation_filter_checkbox.setChecked(cmds.setKinectRotationFilter())
        self.timer.start(33)
        self.set_mode(Mode.LIVE)
        print("Kinect Stream Started")

    def set_rotation_filter(self, enabled):
        cmds.setKinectRotationFilter(int(enabled))

    def stop_stream(self):
        if self.mode == Mode.RECORDING:
            self.stop_recording()          # saves the keys, then stops the stream
            return
        self.record_widget.cancel_countdown()
        self._close_stream()
        self.set_mode(Mode.REVIEW if takes.mocap_has_keys() else Mode.IDLE)

    def _close_stream(self):
        if self.timer.isActive():
            self.timer.stop()
            self.image_label.setText("Stream Paused")
            cmds.closeKinect()
            print("Kinect Stream Stopped")

    def update_frame(self):
        if self.timer.isActive():
            # Call C++ command
            success = cmds.updateMayaKinect(self.memory_address,  str(self.buffer_size))

            if success:
                q_img = QtGui.QImage(
                    self.image_buffer,
                    640, 480,
                    QtGui.QImage.Format_RGB32
                )

                # Use SmoothTransformation if user resizes the window
                pixmap = QtGui.QPixmap.fromImage(q_img)
                scaled_pixmap = pixmap.scaled(
                    self.image_label.size(),
                    QtCore.Qt.KeepAspectRatio,
                    QtCore.Qt.SmoothTransformation
                )
                self.draw_overlay(scaled_pixmap)
                self.image_label.setPixmap(scaled_pixmap)

            if self.mode == Mode.RECORDING:
                self.record_widget.set_status(self.recording_text())

    def recording_text(self):
        seconds = self.record_clock.elapsed() / 1000.0
        return "REC {:02d}:{:02d}  \u00B7  ~{} f".format(int(seconds) // 60, int(seconds) % 60, int(seconds * RECORD_FPS))

    def draw_overlay(self, pixmap):
        """Countdown number or REC indicator on top of the camera image."""
        if self.countdown_value <= 0 and self.mode != Mode.RECORDING:
            return
        painter = QtGui.QPainter(pixmap)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if self.countdown_value > 0:
            font = painter.font()
            font.setPointSize(max(24, pixmap.height() // 4))
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QColor(255, 255, 255, 230))
            painter.drawText(pixmap.rect(), QtCore.Qt.AlignCenter, str(self.countdown_value))
        else:
            radius = max(6, pixmap.height() // 40)
            painter.setBrush(QtGui.QColor(220, 30, 30))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPoint(radius * 3, radius * 3), radius, radius)
            font = painter.font()
            font.setPointSize(max(10, pixmap.height() // 30))
            font.setBold(True)
            painter.setFont(font)
            painter.setPen(QtGui.QColor(255, 255, 255))
            painter.drawText(QtCore.QPoint(radius * 5, int(radius * 3.6)), self.recording_text())
        painter.end()

    # recording

    def on_countdown_tick(self, value):
        self.countdown_value = value
        self.record_widget.set_status("Recording starts in {}...".format(value) if value > 0 else "")

    def start_recording(self):
        if self.mode != Mode.LIVE:
            self.record_widget.set_recording(False)
            return

        # Keys are placed in scene frames: the scene must run at the Kinect rate
        if abs(takes.scene_fps() - RECORD_FPS) > 1e-3:
            takes.set_scene_fps(RECORD_FPS)
            print("Scene frame rate set to {:g} fps for recording".format(RECORD_FPS))

        try:
            cmds.startKinectRecording(startFrame=1, framesPerSecond=RECORD_FPS)
        except RuntimeError as e:
            cmds.warning("Could not start recording: {}".format(e))
            self.record_widget.set_recording(False)
            self.record_widget.set_status("Recording failed to start")
            return

        self.record_clock.start()
        self.record_widget.set_recording(True)
        self.set_mode(Mode.RECORDING)
        print("Kinect recording started")

    def stop_recording(self):
        if self.mode != Mode.RECORDING:
            return

        frames = None
        try:
            frames = cmds.stopKinectRecording()
        except RuntimeError as e:
            cmds.warning("Could not save the recording: {}".format(e))

        self._close_stream()
        self.record_widget.set_recording(False)

        if frames:
            cmds.playbackOptions(minTime=1, maxTime=frames, animationStartTime=1, animationEndTime=frames)
            cmds.currentTime(1)
            self.take_saved = False
            self.record_widget.set_take_name(takes.next_take_name())
            self.record_widget.set_status("Recorded {} frames ({:.1f} s)".format(frames, frames / RECORD_FPS))
            if self.record_widget.autosave_check.isChecked():
                self.save_take(self.record_widget.name_edit.text().strip())
        else:
            self.record_widget.set_status("Nothing recorded")

        self.set_mode(Mode.REVIEW if takes.mocap_has_keys() else Mode.IDLE)

    # takes

    def resolve_current_take(self, action):
        """Before replacing the take on the skeleton: ask to save it if needed, then clear it.
        Returns False if the user cancelled."""
        if not self.take_saved:
            answer = QtWidgets.QMessageBox.question(
                self, "Unsaved take",
                "The recorded take is not saved.\n{} anyway?".format(action),
                QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Save,
            )
            if answer == QtWidgets.QMessageBox.Cancel:
                return False
            if answer == QtWidgets.QMessageBox.Save and not self.save_take(self.record_widget.name_edit.text().strip()):
                return False
        takes.clear_mocap_keys()
        self.take_saved = True
        return True

    def save_take(self, name):
        name = name or takes.next_take_name()
        if not takes.valid_name(name):
            QtWidgets.QMessageBox.warning(self, "Save Take", "Use only letters, digits, _ and - in the take name.")
            return False

        overwrite = False
        if takes.take_path(name).exists():
            answer = QtWidgets.QMessageBox.question(self, "Save Take", "'{}' already exists. Overwrite it?".format(name))
            if answer != QtWidgets.QMessageBox.Yes:
                return False
            overwrite = True

        try:
            path = takes.save_take(name, overwrite=overwrite)
        except Exception as e:
            cmds.warning("Could not save take: {}".format(e))
            return False

        self.take_saved = True
        self.record_widget.set_status("Saved {}".format(path.name))
        self.record_widget.set_take_name(takes.next_take_name())
        self.refresh_takes()
        print("Take saved: {}".format(path))
        return True

    def load_take(self, path):
        if self.mode not in (Mode.IDLE, Mode.REVIEW) or not path:
            return
        if takes.mocap_has_keys() and not self.resolve_current_take("Load another take"):
            return
        try:
            info = takes.load_take(path)
        except Exception as e:
            cmds.warning("Could not load take: {}".format(e))
            return
        self.take_saved = True
        self.record_widget.set_status("Loaded {} ({} - {})".format(info["name"], info["start"], info["end"]))
        self.set_mode(Mode.REVIEW)
        print("Take loaded: {}".format(path))

    def delete_take(self, path):
        if not path:
            return
        name = os.path.splitext(os.path.basename(path))[0]
        answer = QtWidgets.QMessageBox.question(self, "Delete Take", "Delete the take file '{}'?".format(name))
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            takes.delete_take(path)
        except OSError as e:
            cmds.warning("Could not delete take: {}".format(e))
        self.refresh_takes()
        self.record_widget.set_take_name(takes.next_take_name())

    def refresh_takes(self):
        self.record_widget.set_takes(takes.list_takes())

    # characters

    def available_models(self):
        """Models in the models folder, plus models imported with Browse... that are in the scene."""
        found = model_lib.discover()
        known = {characters.model_key(m.obj) for m in found}
        for path, _ in characters.cached():
            if characters.model_key(path) not in known:
                found.append(model_lib.from_path(path))
        return found

    def refresh_models(self):
        self.model_bar.refresh(self.available_models())

    def import_model(self):
        dialog = ModelPickerDialog(self.available_models(), parent=self)
        if dialog.exec() == QtWidgets.QDialog.Accepted and dialog.selected is not None:
            self.switch_model(dialog.selected)

    def switch_model(self, model):
        # The first switch to a model imports and skins it, which takes a moment
        first_import = characters.find_group(model) is None
        if first_import:
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            characters.switch_to(model)
            print("Character: " + model.display_name)
        except Exception as e:
            cmds.warning("Could not switch to {}: {}".format(model.display_name, e))
        finally:
            if first_import:
                QtWidgets.QApplication.restoreOverrideCursor()
        self.refresh_models()


    def closeEvent(self, event):
        if self.mode == Mode.RECORDING:
            self.stop_recording()          # never lose a take by closing the panel
        if hasattr(self, 'timer'):
            self.timer.stop()
        cmds.closeKinect()
        super(KinectMayaPanel, self).closeEvent(event)

# Launch logic
def show_kinect_ui():
    global kinect_ui
    try:
        kinect_ui.close()
    except:
        pass
    kinect_ui = KinectMayaPanel()
    kinect_ui.show(dockable=True)

show_kinect_ui()
