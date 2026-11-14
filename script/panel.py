import maya.cmds as cmds
import ctypes
import os
import sys
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

from mayakinect.character import characters, models as model_lib
from mayakinect.ui.model_bar import ModelBar
from mayakinect.ui.model_picker import ModelPickerDialog


class KinectMayaPanel(MayaQWidgetDockableMixin, QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(KinectMayaPanel, self).__init__(parent=parent)
        self.setWindowTitle("Kinect Control Panel")
        self.resize(800, 520) # Slightly wider to accommodate the sidebar

        # --- Main Layout (Horizontal Splitter) ---
        self.main_layout = QtWidgets.QHBoxLayout(self)
        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.main_layout.addWidget(self.splitter)

        # --- Left Side: Controls ---
        self.control_widget = QtWidgets.QWidget()
        self.control_layout = QtWidgets.QVBoxLayout(self.control_widget)
        self.control_layout.setAlignment(QtCore.Qt.AlignTop)

        self.start_btn = QtWidgets.QPushButton("Start Stream")
        self.stop_btn = QtWidgets.QPushButton("Stop Stream")
        self.import_btn = QtWidgets.QPushButton("Import Model")
        self.rotation_filter_checkbox = QtWidgets.QCheckBox("Rotation filter")
        self.rotation_filter_checkbox.setChecked(True)
        self.rotation_filter_checkbox.setEnabled(False)
        
        # Styling buttons for visibility
        self.start_btn.setMinimumHeight(40)
        self.stop_btn.setMinimumHeight(40)
        self.import_btn.setMinimumHeight(40)

        self.control_layout.addWidget(QtWidgets.QLabel("<b>Kinect Controls</b>"))
        self.control_layout.addWidget(self.start_btn)
        self.control_layout.addWidget(self.stop_btn)
        self.control_layout.addWidget(self.import_btn)
        self.control_layout.addWidget(self.rotation_filter_checkbox)

        # Character models: click to switch (also while streaming)
        self.control_layout.addSpacing(10)
        self.control_layout.addWidget(QtWidgets.QLabel("<b>Characters</b>"))
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
        self.splitter.addWidget(self.control_widget)
        self.splitter.addWidget(self.image_container)
        self.splitter.setStretchFactor(1, 4) # Give more space to the image

        # --- Kinect Logic Setup ---
        self.width = 640
        self.height = 480
        self.channels = 4
        self.buffer_size = self.width * self.height * self.channels
        self.image_buffer = ctypes.create_string_buffer(self.buffer_size)
        self.memory_address = str(ctypes.addressof(self.image_buffer))



        # Timer setup (Starts active by default)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        # Button Connections
        self.start_btn.clicked.connect(self.start_stream)
        self.stop_btn.clicked.connect(self.stop_stream)
        self.import_btn.clicked.connect(self.import_model)
        self.rotation_filter_checkbox.toggled.connect(self.set_rotation_filter)
        self.model_bar.model_selected.connect(self.switch_model)

        self.refresh_models()

    def start_stream(self):
        if not self.timer.isActive():
            self.timer.start(33)

            # Try to init hardware
            try:
                cmds.runMayaKinect()
                self.rotation_filter_checkbox.setChecked(cmds.setKinectRotationFilter())
                self.rotation_filter_checkbox.setEnabled(True)
            except Exception as e:
                self.image_label.setText("Plugin not found or Kinect disconnected!")

            print("Kinect Stream Started")

    def set_rotation_filter(self, enabled):
        cmds.setKinectRotationFilter(int(enabled))

    def stop_stream(self):
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
                self.image_label.setPixmap(scaled_pixmap)

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