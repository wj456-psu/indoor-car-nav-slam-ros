import signal
import subprocess
import sys
import time
from pathlib import Path

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
HOSTNAME = "nanorobotv2.local"
USER_NAME = "punch2"
WS_PATH = Path(f"/home/{USER_NAME}/catkin_ws")

# Directory containing your map files
MAPS_DIRECTORY = Path(f"{WS_PATH}/src/robot_navigation/maps")

# Directory containing your map files
RVIZCONFIG_DIRECTORY = Path(f"{WS_PATH}/src/rviz_config/rviz")

class RosLauncherApp(QWidget):
    def __init__(self):
        super().__init__()
        self.running_proc = []
        self.load_map_from_robot()
        self.initUI()

    def load_map_from_robot(self):
        subprocess.run(
            ["rm", "-rf", f"/home/{USER_NAME}/catkin_ws/src/robot_navigation/maps/*"]
        )
        subprocess.run([f"/home/{USER_NAME}/load_map_nnrb.exp", HOSTNAME, USER_NAME])

    def run_local_command(self, command: str):
        proc = subprocess.Popen(
            ["gnome-terminal", "--disable-factory", "--", *command.split(" ")]
        )
        self.running_proc.append(proc)

    def run_remote_command(self, command: str):
        proc = subprocess.Popen(
            [
                "gnome-terminal",
                "--disable-factory",
                "--",
                f"/home/{USER_NAME}/login_nnrb.exp",
                HOSTNAME,
                command,
            ]
        )
        self.running_proc.append(proc)

    def initUI(self):
        left_layout_button_height = 90
        # Main Horizontal Layout
        main_layout = QHBoxLayout()

        # Left Layout (for navigation buttons)
        left_layout = QVBoxLayout()

        # Upper Middle Layout (for SLAM-related buttons)
        upper_middle_layout = QVBoxLayout()

        # Lower Middle Layout (for map selection and navigation button)
        lower_middle_layout = QVBoxLayout()

        # Upper Right Layout (for RVIZ-related buttons)
        upper_right_layout = QVBoxLayout()

        # Lower Right Layout (for Explore-related buttons)
        lower_right_layout = QVBoxLayout()

        # Right Layout Combining Upper Right and Lower Right)
        right_layout = QVBoxLayout()
        right_layout.addLayout(upper_right_layout)
        right_layout.addLayout(lower_right_layout)

        # Middle Layout (Combining Upper Middle and Lower Middle)
        middle_layout = QVBoxLayout()
        middle_layout.addLayout(upper_middle_layout)
        middle_layout.addLayout(lower_middle_layout)

        # Add all the major layouts to the main layout
        main_layout.addLayout(left_layout)
        main_layout.addLayout(middle_layout)
        main_layout.addLayout(right_layout)

        # Set the main layout to the window
        self.setLayout(main_layout)
        self.setWindowTitle("Scalable ROS Launcher")

        # ====================== Button UI ======================

        # Button 1 (Navigation with RVIZ)
        self.button1 = QPushButton("Quick Navigation", self)
        self.button1.setFixedSize(150, left_layout_button_height)
        self.button1.clicked.connect(self.on_click_nav_rviz)

        # SLAM Method Dropdown
        self.slam_method_combo = QComboBox(self)
        self.slam_method_combo.addItem("gmapping")
        self.slam_method_combo.addItem("cartographer")
        # Add more SLAM methods as needed

        # Button 2 (Start SLAM)
        self.button2 = QPushButton("Start SLAM", self)
        self.button2.clicked.connect(self.on_click_start_slam)

        # Button 3 (Start Explore Lite)
        self.button3 = QPushButton("Start Explore Lite", self)
        self.button3.clicked.connect(self.on_click_start_explore_lite)

        # Button 4 (Start move_base)
        self.button4 = QPushButton("Stop all", self)
        self.button4.clicked.connect(self.on_click_stop_running_proc)

        # Button 9 (Start Rviz)
        self.button9 = QPushButton("Start Rviz", self)
        self.button9.clicked.connect(self.on_click_start_rviz)

        # MAP Dropdown
        self.selected_maps = QComboBox(self)
        self.populate_maps()

        # RVIZ Dropdown
        self.selected_rvizconfig = QComboBox(self)
        self.populate_rvizconfig()

        # Button 5 (Start Navigation)
        self.button5 = QPushButton("Start Navigation", self)
        self.button5.clicked.connect(self.on_click_start_nav)

        # SLAM with Explore button
        self.slam_with_other_button = QPushButton("Quick SLAM", self)
        self.slam_with_other_button.setFixedSize(150, left_layout_button_height)
        self.slam_with_other_button.clicked.connect(self.on_click_start_combined)

        # ====================== Button Layout ======================

        # Create QLabel widgets to display topic information
        label_topic2 = QLabel("SLAM", self)
        label_topic3 = QLabel("Navigation", self)
        label_topic5 = QLabel("Rviz", self)
        label_topic6 = QLabel("Explore", self)

        # Add label_topic labels to their respective layout
        upper_middle_layout.addWidget(label_topic2)
        lower_middle_layout.addWidget(label_topic3)
        upper_right_layout.addWidget(label_topic5)
        lower_right_layout.addWidget(label_topic6)

        # Button Layout
        left_layout.addWidget(self.button1)  # Nav_rviz button
        left_layout.addWidget(self.slam_with_other_button)  # SLAM with other
        lower_middle_layout.addWidget(self.selected_maps)
        upper_middle_layout.addWidget(self.slam_method_combo)  # DropBox SLAM Metthod
        upper_middle_layout.addWidget(self.button2)  # Start SLAM
        upper_right_layout.addWidget(self.selected_rvizconfig)  # Dropbox Rviz config
        upper_right_layout.addWidget(self.button9)  # Rviz config
        lower_right_layout.addWidget(self.button4)  # Move_base button
        lower_right_layout.addWidget(self.button3)  # Explore Lite button
        lower_middle_layout.addWidget(self.selected_maps)
        lower_middle_layout.addWidget(self.button5)  # Navigation button

        # Set main layout
        self.setLayout(main_layout)
        self.setWindowTitle("ROS Launcher")

    def on_click_start_combined(self):
        # Set selected_rvizconfig to "slam.rviz"
        self.selected_rvizconfig.setCurrentText("slam.rviz")

        self.on_click_start_slam()
        self.on_click_start_explore_lite()
        time.sleep(8)
        self.on_click_start_rviz()
        time.sleep(3)
        # python3 Record_script2.py
        self.run_local_command(f"python3 /home/{USER_NAME}/Desktop/Record_script3.py")

    # Start Navigation with RVIZ Button
    def on_click_nav_rviz(self):
        # Set selected_rvizconfig to "navigation.rviz"
        self.selected_rvizconfig.setCurrentText("navigation.rviz")

        self.on_click_start_nav()
        self.on_click_start_rviz()
        

    def populate_maps(self):
        # Check if the directory exists
        if MAPS_DIRECTORY.is_dir():
            # Get a list of all files in the directory
            map_files = [
                entry.name
                for entry in MAPS_DIRECTORY.iterdir()
                if entry.is_file() and entry.name.endswith(".yaml")
            ]

            # Add each map file as an item to the ComboBox
            for map_file in map_files:
                self.selected_maps.addItem(map_file)

    def populate_rvizconfig(self):
        # Check if the directory exists
        if RVIZCONFIG_DIRECTORY.is_dir():
            # Get a list of all files in the directory
            config = [
                entry.name
                for entry in RVIZCONFIG_DIRECTORY.iterdir()
                if entry.is_file() and entry.name.endswith(".rviz")
            ]

            # Add each map file as an item to the ComboBox
            for config in config:
                self.selected_rvizconfig.addItem(config)

    # ====================== Button Function ======================

    def on_click_start_rviz(self):
        selected_rvizconfig = self.selected_rvizconfig.currentText()
        selected_rvizconfig = selected_rvizconfig.rstrip(
            ".rviz"
        )  # Remove ".rviz" if it exists
        command = (
            f"roslaunch rviz_config launch_rviz.launch mode:={selected_rvizconfig}"
        )
        self.run_local_command(command)

    def on_click_start_nav(self):
        selected_map = self.selected_maps.currentText()
        command = f"roslaunch robot_navigation robot_navigation.launch map_file:={selected_map}"
        self.run_remote_command(command)

    def on_click_start_slam(self):
        selected_slam_method = self.slam_method_combo.currentText()
        command = f"roslaunch robot_navigation robot_slam_laser.launch planner:=dwa slam_methods:={selected_slam_method}"
        self.run_remote_command(command)

    def on_click_start_explore_lite(self):
        command = "roslaunch explore_lite explore.launch"
        self.run_remote_command(command)

    def on_click_stop_running_proc(self):
        for proc in self.running_proc:
            proc.kill()
        self.running_proc = []


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = RosLauncherApp()
    ex.show()

    def sigint_handler(*args):
        app.quit()

    signal.signal(signal.SIGINT, sigint_handler)
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    sys.exit(app.exec())
