import maya.cmds as cmds
import ctypes
from PySide6 import QtWidgets, QtGui, QtCore
from maya.app.general.mayaMixin import MayaQWidgetDockableMixin



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
        
        # Styling buttons for visibility
        self.start_btn.setMinimumHeight(40)
        self.stop_btn.setMinimumHeight(40)
        self.import_btn.setMinimumHeight(40)

        self.control_layout.addWidget(QtWidgets.QLabel("<b>Kinect Controls</b>"))
        self.control_layout.addWidget(self.start_btn)
        self.control_layout.addWidget(self.stop_btn)
        self.control_layout.addWidget(self.import_btn)
        
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
        self.buffer_size = 640 * 480 * 4
        self.image_buffer = ctypes.create_string_buffer(self.buffer_size)
        self.memory_address = str(ctypes.addressof(self.image_buffer))



        # Timer setup (Starts active by default)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)

        # Button Connections
        self.start_btn.clicked.connect(self.start_stream)
        self.stop_btn.clicked.connect(self.stop_stream)
        self.import_btn.clicked.connect(self.import_model)

    def start_stream(self):
        if not self.timer.isActive():
            self.timer.start(33)

            # Try to init hardware
            try:
                cmds.runMayaKinect()
            except Exception as e:
                self.image_label.setText("Plugin not found or Kinect disconnected!")

            print("Kinect Stream Started")

    def stop_stream(self):
        if self.timer.isActive():
            self.timer.stop()
            self.image_label.setText("Stream Paused")
            cmds.closeKinect()
            print("Kinect Stream Stopped")

    def update_frame(self):
        if self.timer.isActive():
            # Call C++ command
            success = cmds.updateMayaKinect(self.memory_address)
            
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

    def import_model(self):
        cmds.importCharacter(r"C:/Users/Geri/Documents/Projects/CG/MayaKinect/tpose.obj")
        print("Import Character")


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