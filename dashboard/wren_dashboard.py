import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QCheckBox, QWidget, QComboBox, QLabel, QPushButton, QFrame, QFileDialog, QMessageBox
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
from wren_manager import WrenManager, TimeSeries
import time


class WrenDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.event_lines = []
        self.setWindowTitle("Wren Dashboard")
        self.setGeometry(100, 100, 1000, 600)
        self.wren = WrenManager()

        # Initialize default data for the plots
        self.altitude = TimeSeries()
        self.velocity = TimeSeries()
        self.acceleration = TimeSeries()

        # Create the figure and primary plot
        self.figure, self.ax_altitude = plt.subplots()
        self.canvas = FigureCanvas(self.figure)

        # Initial plot with multiple y-axes
        self.altitude_line, = self.ax_altitude.plot(self.altitude.x, self.altitude.y, 'b-', label="Altitude")
        self.ax_altitude.set_ylabel("Altitude (m)", color='b')

        # Create a second y-axis for velocity
        self.ax_velocity = self.ax_altitude.twinx()
        self.velocity_line, = self.ax_velocity.plot(self.velocity.x, self.velocity.y, 'g-', label="Velocity")
        self.ax_velocity.set_ylabel("Velocity (m/s)", color='g')

        # Create a third y-axis for acceleration (shared x-axis)
        self.ax_acceleration = self.ax_altitude.twinx()
        self.ax_acceleration.spines['right'].set_position(('outward', 60))
        self.acceleration_line, = self.ax_acceleration.plot(self.acceleration.x, self.acceleration.y, 'r-', label="Acceleration")
        self.ax_acceleration.set_ylabel("Acceleration (m/s²)", color='r')

        self.ax_altitude.set_title("Altitude, Velocity, and Acceleration")

        # Create the left-side control panel
        control_panel = QVBoxLayout()

        # Buttons for file and USB operations
        self.open_file_button = QPushButton("Open File")
        self.open_usb_button = QPushButton("Open USB")
        self.open_file_button.clicked.connect(self.open_file_dialog)  # Connect the button to the file dialog
        self.open_usb_button.clicked.connect(self.open_usb)  # Connect the button to the file dialog
        control_panel.addWidget(self.open_file_button)
        control_panel.addWidget(self.open_usb_button)

        # USB Panel (hidden by default)
        self.usb_panel = QFrame()
        usb_layout = QVBoxLayout()
        self.usb_update_timer = QTimer(self)
        self.usb_update_timer.timeout.connect(self.update_usb_status)
        self.usb_update_timer.start(500)
        self.usb_status_label = QLabel("")
        # usb_font = self.usb_status_label.font()
        # usb_font.setPointSize(12)  # Set font size to 12 points (or adjust as needed)
        # self.usb_status_label.setFont(usb_font)
        # self.usb_status_label.setFixedWidth(200)

        self.read_memory_button = QPushButton("Read Memory")
        self.read_memory_button.clicked.connect(self.read_memory_handler)
        self.clear_memory_button = QPushButton("Clear Memory")
        self.clear_memory_button.clicked.connect(self.clear_memory_handler)

        # Add widgets to the USB panel layout
        usb_layout.addWidget(self.usb_status_label)
        usb_layout.addWidget(self.read_memory_button)
        usb_layout.addWidget(self.clear_memory_button)
        usb_layout.addStretch()

        self.usb_panel.setLayout(usb_layout)

        # Add USB panel to control panel
        control_panel.addWidget(self.usb_panel)


        # Add a spacer to push everything up in the control panel
        control_panel.addStretch()

        # Frame for the control panel layout
        control_panel_widget = QFrame()
        control_panel_widget.setLayout(control_panel)

        # Create the main layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(control_panel_widget)

        # Add plot and toolbar on the right
        plot_layout = QVBoxLayout()
        self.toolbar = NavigationToolbar2QT(self.canvas, self)
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)

        # Group toggle buttons and data source dropdown horizontally and center them
        checkbox_layout = QHBoxLayout()  # Keep as horizontal layout
        checkbox_layout.setSpacing(10)  # Reduce spacing between buttons

        # Data source dropdown (leftmost in the group)
        self.data_source_dropdown = QComboBox()
        checkbox_layout.addWidget(self.data_source_dropdown)

        # Add checkboxes to layout
        self.checkbox_altitude = QCheckBox("Show Altitude")
        self.checkbox_altitude.setChecked(True)
        self.checkbox_altitude.stateChanged.connect(self.toggle_altitude)

        self.checkbox_velocity = QCheckBox("Show Velocity")
        self.checkbox_velocity.setChecked(True)
        self.checkbox_velocity.stateChanged.connect(self.toggle_velocity)

        self.checkbox_acceleration = QCheckBox("Show Acceleration")
        self.checkbox_acceleration.setChecked(True)
        self.checkbox_acceleration.stateChanged.connect(self.toggle_acceleration)

        # Add checkboxes to the layout after the dropdown
        checkbox_layout.addWidget(self.checkbox_altitude)
        checkbox_layout.addWidget(self.checkbox_velocity)
        checkbox_layout.addWidget(self.checkbox_acceleration)

        # Add spacing around checkbox group
        checkbox_layout.addStretch(1)  # Add stretch to push them leftwards

        # Now center the entire checkbox group
        centered_layout = QHBoxLayout()
        centered_layout.addStretch(1)
        centered_layout.addLayout(checkbox_layout)
        centered_layout.addStretch(1)

        plot_layout.addLayout(centered_layout)

        main_layout.addLayout(plot_layout)

        # Central widget
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        self.data_source_dropdown.currentIndexChanged.connect(self.dropdown_handler)

    def open_file_dialog(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Flash Files (*.bin)")
        if file_name:
            print(f"File selected: {file_name}")
            self.open_file(file_name)


    def read_memory_handler(self):
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Memory To File", "", "Binary Files (*.bin)")
        if not file_name:
            return  # User canceled the dialog
        buf = self.wren.read_flash()
        f = open(file_name, "wb")
        f.write(buf)


    def clear_memory_handler(self):
        confirm = QMessageBox(self)
        confirm.setIcon(QMessageBox.Warning)
        confirm.setWindowTitle("Confirm Clear Memory")
        confirm.setText("Are you sure you want to clear the memory?")
        confirm.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        confirm.setDefaultButton(QMessageBox.No)

        # Show the dialog and get the user's response
        response = confirm.exec_()
        if response == QMessageBox.Yes:
            self.wren.clear_flash()

    def open_file(self, path):
        self.wren.open_file(path)
        self.data_source_dropdown.clear()
        for i, split in enumerate(self.wren.splits):
            self.data_source_dropdown.addItem(f"Flight {i + 1}")

        if self.wren.splits:
            self.update_plot_data(self.wren.splits[-1])

    def open_usb(self, path):
        self.wren.open_usb()

    def update_usb_status(self):
        if self.wren.usb_is_open:
            self.wren.fetch_status()
            status = "USB connected\n"
            status += f"time: {self.wren.time:.1f}\n"
            status += f"battery: {self.wren.battery_percent:.0f}%\n"
            status += f"memory used: {self.wren.flash_used:.0f}%\n"
            status += f"battery volt: {self.wren.battery_voltage:.2f}\n"
            status += f"acceleration: {self.wren.acceleration:.2f}\n"
            status += f"altitude: {self.wren.altitude:.1f}\n"
            self.usb_status_label.setText(status)
        else:
            self.usb_status_label.setText("USB disconnected")


    def toggle_altitude(self):
        """Toggle visibility of the altitude line and its axis."""
        is_checked = self.checkbox_altitude.isChecked()
        self.altitude_line.set_visible(is_checked)
        if is_checked:
            self.ax_altitude.spines['left'].set_visible(True)
            self.ax_altitude.yaxis.set_visible(True)
        else:
            self.ax_altitude.spines['left'].set_visible(False)
            self.ax_altitude.yaxis.set_visible(False)
        self.canvas.draw()

    def toggle_velocity(self):
        """Toggle visibility of the velocity line and its axis."""
        is_checked = self.checkbox_velocity.isChecked()
        self.velocity_line.set_visible(is_checked)
        if is_checked:
            self.ax_velocity.spines['right'].set_visible(True)
            self.ax_velocity.yaxis.set_visible(True)
        else:
            self.ax_velocity.spines['right'].set_visible(False)
            self.ax_velocity.yaxis.set_visible(False)
        self.canvas.draw()

    def toggle_acceleration(self):
        """Toggle visibility of the acceleration line and its axis."""
        is_checked = self.checkbox_acceleration.isChecked()
        self.acceleration_line.set_visible(is_checked)
        if is_checked:
            self.ax_acceleration.spines['right'].set_visible(True)
            self.ax_acceleration.yaxis.set_visible(True)
        else:
            self.ax_acceleration.spines['right'].set_visible(False)
            self.ax_acceleration.yaxis.set_visible(False)
        self.canvas.draw()

    def dropdown_handler(self):
        """Handles changes in the dropdown selection and updates the plot with the corresponding split."""
        selected_item = self.data_source_dropdown.currentText()

        try:
            split_number = int(''.join(filter(str.isdigit, selected_item)))
            split_data = self.wren.splits[split_number - 1]
        except (ValueError, IndexError) as e:
            print(f"Error parsing split number or accessing split: {e}")
            return

        # Update the plot data
        self.update_plot_data(split_data)


    def update_plot_data(self, data):
        """Updates the plot data based on the selected data source."""
        self.altitude = data["altitude"]
        self.velocity = data["velocity"]
        self.acceleration = data["acceleration"]

        # Update each plot line with the new data
        self.altitude_line.set_data(self.altitude.x, self.altitude.y)
        self.velocity_line.set_data(self.velocity.x, self.velocity.y)
        self.acceleration_line.set_data(self.acceleration.x, self.acceleration.y)

        # Update axes limits and refresh the canvas
        self.ax_altitude.relim()
        self.ax_altitude.autoscale_view()
        self.ax_velocity.relim()
        self.ax_velocity.autoscale_view()
        self.ax_acceleration.relim()
        self.ax_acceleration.autoscale_view()

        self.canvas.draw()