import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QCheckBox, QWidget, QComboBox, QLabel, QPushButton, QFrame, QFileDialog
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT

class PlotWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.event_lines = []
        self.setWindowTitle("Interactive Plot with Events")
        self.setGeometry(100, 100, 1000, 600)

        # Initialize default data for the plots
        self.time = np.linspace(0, 10, 100)
        self.altitude = np.sin(self.time) * 100
        self.velocity = np.cos(self.time) * 50
        self.acceleration = np.sin(self.time / 2) * 10

        # Create the figure and primary plot
        self.figure, self.ax_altitude = plt.subplots()
        self.canvas = FigureCanvas(self.figure)

        # Initial plot with multiple y-axes
        self.altitude_line, = self.ax_altitude.plot(self.time, self.altitude, 'b-', label="Altitude")
        self.ax_altitude.set_ylabel("Altitude (m)", color='b')

        # Create a second y-axis for velocity
        self.ax_velocity = self.ax_altitude.twinx()
        self.velocity_line, = self.ax_velocity.plot(self.time, self.velocity, 'g-', label="Velocity")
        self.ax_velocity.set_ylabel("Velocity (m/s)", color='g')

        # Create a third y-axis for acceleration (shared x-axis)
        self.ax_acceleration = self.ax_altitude.twinx()
        self.ax_acceleration.spines['right'].set_position(('outward', 60))
        self.acceleration_line, = self.ax_acceleration.plot(self.time, self.acceleration, 'r-', label="Acceleration")
        self.ax_acceleration.set_ylabel("Acceleration (m/s²)", color='r')

        self.ax_altitude.set_title("Altitude, Velocity, and Acceleration with Events")

        # Create event markers
        self.events = [2, 5, 7]  # Default event times for Source 1
        self.plot_events()

        # Create the left-side control panel
        control_panel = QVBoxLayout()

        # Buttons for file and USB operations
        self.open_file_button = QPushButton("Open File")
        self.open_usb_button = QPushButton("Open USB")  # Renamed
        self.open_file_button.clicked.connect(self.open_file_dialog)  # Connect the button to the file dialog
        control_panel.addWidget(self.open_file_button)
        control_panel.addWidget(self.open_usb_button)

        # Labels for additional information (now placed below the buttons)
        self.info_label = QLabel("Time: 0 s\nVoltage: 12.0 V\nFlash Memory: 1.2 GB")
        control_panel.addWidget(self.info_label)

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
        self.data_source_dropdown.addItem("Source 1")
        self.data_source_dropdown.addItem("Source 2")
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

        # Connect data source selection to update function
        self.data_source_dropdown.currentIndexChanged.connect(self.update_plot_data)

    def open_file_dialog(self):
        """Open a file dialog and select a file."""
        file_name, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Text Files (*.txt);;All Files (*)")
        if file_name:
            print(f"File selected: {file_name}")
            # You can implement logic here to read the selected file
            # For example, load data for plotting or other actions

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

    def plot_events(self):
        """Plot events as vertical lines."""
        for event in self.events:
            event_line = self.ax_altitude.axvline(x=event, color='black', linestyle='--', label=f'Event at t={event}s')
            self.event_lines.append(event_line)

    def update_plot_data(self):
        """Updates the plot data based on the selected data source."""
        selected_source = self.data_source_dropdown.currentText()

        if selected_source == "Source 1":
            self.time = np.linspace(0, 10, 100)
            self.altitude = np.sin(self.time) * 100
            self.velocity = np.cos(self.time) * 50
            self.acceleration = np.sin(self.time / 2) * 10
            self.events = [2, 5, 7]  # Events for Source 1
            self.info_label.setText("Time: 1.0 s\nVoltage: 12.5 V\nFlash Memory: 1.0 GB")

        elif selected_source == "Source 2":
            self.time = np.linspace(0, 20, 200)
            self.altitude = np.sin(self.time / 2) * 120
            self.velocity = np.cos(self.time / 2) * 55
            self.acceleration = np.cos(self.time / 4) * 15
            self.events = [4, 8, 12]  # Events for Source 2
            self.info_label.setText("Time: 2.0 s\nVoltage: 13.0 V\nFlash Memory: 0.8 GB")

        # Update each plot line with the new data
        self.altitude_line.set_data(self.time, self.altitude)
        self.velocity_line.set_data(self.time, self.velocity)
        self.acceleration_line.set_data(self.time, self.acceleration)

        # Clear previous event lines and re-plot them
        for event_line in self.event_lines:
            event_line.remove()  # Remove the event lines from the plot
        self.event_lines.clear()  # Clear the event lines list

        # Plot new events
        self.plot_events()

        # Update axes limits and refresh the canvas
        self.ax_altitude.relim()
        self.ax_altitude.autoscale_view()
        self.ax_velocity.relim()
        self.ax_velocity.autoscale_view()
        self.ax_acceleration.relim()
        self.ax_acceleration.autoscale_view()

        self.canvas.draw()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PlotWindow()
    window.show()
    sys.exit(app.exec_())