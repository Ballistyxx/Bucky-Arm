import sys
import os
import json
import numpy as np
import requests
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QSlider, QPushButton, QGroupBox, QGridLayout, 
                            QLineEdit, QSplitter, QDialog, QFormLayout, QDoubleSpinBox,
                            QTabWidget, QMessageBox, QSpinBox)
from PyQt5.QtCore import Qt, QUrl, QTimer, QSettings, pyqtSignal
from PyQt5.QtGui import QVector3D
from PyQt5.QtWebEngineWidgets import QWebEngineView
import pyqtgraph.opengl as gl
import pyqtgraph as pg

class Joint:
    def __init__(self, id, name, min_angle=-90, max_angle=90, length=1.0, initial_angle=0, gear_ratio=1.0):
        self.id = id
        self.name = name
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle = initial_angle
        self.length = length
        self.gear_ratio = gear_ratio  # Motor rotations per joint rotation
        self.children = []
        
    def add_child(self, child):
        self.children.append(child)
        return child

class RobotArmModel:
    def __init__(self):
        # Create joint hierarchy based on the 5-axis robot arm
        # Default position is standing straight up (shoulder and elbow at 90 degrees)
        self.base = Joint(0, "Base", min_angle=-180, max_angle=180, length=0.5, initial_angle=0, gear_ratio=1.0)
        self.shoulder = self.base.add_child(Joint(1, "Shoulder", min_angle=-90, max_angle=90, length=1.5, initial_angle=90, gear_ratio=2.0))
        self.elbow = self.shoulder.add_child(Joint(2, "Elbow", min_angle=-90, max_angle=90, length=1.2, initial_angle=0, gear_ratio=2.0))
        self.wrist_pitch = self.elbow.add_child(Joint(3, "Wrist Pitch", min_angle=-90, max_angle=90, length=0.8, initial_angle=0, gear_ratio=1.5))
        self.wrist_roll = self.wrist_pitch.add_child(Joint(4, "Wrist Roll", min_angle=-180, max_angle=180, length=0.4, initial_angle=0, gear_ratio=1.0))
        
        self.joints = [self.base, self.shoulder, self.elbow, self.wrist_pitch, self.wrist_roll]
        
        # End effector position in cartesian coordinates
        self.end_effector_pos = np.zeros(3)
        
        # Track if position has changed since last update
        self.position_changed = False
        
    def update_joint_angle(self, joint_id, angle):
        if joint_id >= len(self.joints):
            return 0
            
        joint = self.joints[joint_id]
        
        # Check if angle is actually changing
        if joint.angle != angle:
            joint.angle = max(joint.min_angle, min(joint.max_angle, angle))
            self.position_changed = True
            
            # Update end effector position
            self._update_end_effector_position()
        
        return joint.angle
    
    def _update_end_effector_position(self):
        positions = self.get_joint_positions()
        # The last position is the end effector
        if positions:
            self.end_effector_pos = positions[-1][1]
    
    def update_end_effector(self, target_pos):
        """Update joint angles to position end effector at the target position using inverse kinematics"""
        # Simple implementation of inverse kinematics for a 5-axis arm
        # This is a basic approach and could be improved with more sophisticated IK algorithms
        
        # Store original angles in case we need to revert
        original_angles = [joint.angle for joint in self.joints]
        
        # Check if target position is the same as current position
        current_pos = self.end_effector_pos
        if np.linalg.norm(target_pos - current_pos) < 0.01:
            return True  # No change needed
            
        # Set position changed flag
        self.position_changed = True
        
        # Simple approach: use Jacobian transpose method (very simplified)
        # In a real application, you would use a proper IK library or algorithm
        
        # Set learning rate and number of iterations
        learning_rate = 0.01
        max_iterations = 100
        
        for iteration in range(max_iterations):
            current_pos = self.end_effector_pos
            error = target_pos - current_pos
            
            # If we're close enough, stop iterating
            if np.linalg.norm(error) < 0.01:
                break
                
            # Very simplified "Jacobian transpose" approach
            # Adjust each joint angle based on the error
            for i, joint in enumerate(self.joints):
                # The influence of each joint decreases as we move down the chain
                influence = 1.0 / (i + 1)
                delta = learning_rate * influence * np.linalg.norm(error)
                
                # Try moving in positive direction
                joint.angle += delta
                self._update_end_effector_position()
                new_error_pos = np.linalg.norm(target_pos - self.end_effector_pos)
                
                # If it made things worse, try negative direction
                if new_error_pos > np.linalg.norm(error):
                    joint.angle -= 2 * delta
                    self._update_end_effector_position()
                    new_error_pos = np.linalg.norm(target_pos - self.end_effector_pos)
                    
                    # If still worse, revert the change
                    if new_error_pos > np.linalg.norm(error):
                        joint.angle += delta
                        self._update_end_effector_position()
        
        # Check if we found a reasonable solution
        final_error = np.linalg.norm(target_pos - self.end_effector_pos)
        if final_error > 0.5:  # If error is too large, revert to original angles
            for i, angle in enumerate(original_angles):
                self.joints[i].angle = angle
            self._update_end_effector_position()
            self.position_changed = False  # No change was made
            return False
            
        return True
        
    def reset_position_changed_flag(self):
        """Reset the position changed flag after an update is sent to the robot"""
        self.position_changed = False
        
    def get_joint_angles(self):
        """Return the current joint angles as a list"""
        return [joint.angle for joint in self.joints]
        
    def set_joint_angles(self, angles):
        """Set all joint angles from a list"""
        changed = False
        for i, angle in enumerate(angles):
            if i < len(self.joints) and self.joints[i].angle != angle:
                self.joints[i].angle = max(self.joints[i].min_angle, min(self.joints[i].max_angle, angle))
                changed = True
                
        if changed:
            self.position_changed = True
            self._update_end_effector_position()
            return True
        return False
        
    def get_joint_positions(self):
        positions = []
        self._calculate_joint_positions(self.base, np.identity(4), positions)
        return positions
        
    def _calculate_joint_positions(self, joint, transform_matrix, positions):
        # Apply rotation for this joint
        rotation = np.identity(4)
        if joint.id == 0:  # Base rotation (around Z)
            c = np.cos(np.radians(joint.angle))
            s = np.sin(np.radians(joint.angle))
            rotation[:3, :3] = np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])
        else:  # All other joints rotate around X axis
            c = np.cos(np.radians(joint.angle))
            s = np.sin(np.radians(joint.angle))
            rotation[:3, :3] = np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ])
            
        # Apply translation for this joint
        translation = np.identity(4)
        if joint.id == 0:  # Base
            translation[2, 3] = joint.length  # Z translation for base
        else:
            # For all other joints, translate along the Y axis
            translation[1, 3] = joint.length
            
        # Apply the transformations
        local_transform = transform_matrix @ rotation @ translation
        
        # Add the joint position to our list
        joint_pos = local_transform @ np.array([0, 0, 0, 1])
        positions.append((joint.id, joint_pos[:3]))
        
        # Process children
        for child in joint.children:
            self._calculate_joint_positions(child, local_transform, positions)

class RobotArmVisualizer(gl.GLViewWidget):
    # Signal when the user interacts with the visualization
    position_selected = pyqtSignal(np.ndarray)
    
    def __init__(self, arm_model):
        super().__init__()
        self.arm_model = arm_model
        self.setCameraPosition(distance=10, elevation=30, azimuth=45)
        
        # Add a grid
        grid = gl.GLGridItem()
        grid.scale(1, 1, 1)
        self.addItem(grid)
        
        # Create items for joints and links
        self.joint_items = []
        for _ in range(len(arm_model.joints)):
            joint_item = gl.GLMeshItem(meshdata=self._create_sphere(0.1), color=(1, 0, 0, 1))
            self.addItem(joint_item)
            self.joint_items.append(joint_item)
            
        self.link_items = []
        for _ in range(len(arm_model.joints)):
            link_item = gl.GLLinePlotItem(color=(0, 1, 0, 1), width=3)
            self.addItem(link_item)
            self.link_items.append(link_item)
        
        # Add end effector target marker
        self.target_marker = gl.GLMeshItem(meshdata=self._create_sphere(0.15), color=(0, 0, 1, 0.5))
        self.target_marker.setVisible(False)
        self.addItem(self.target_marker)
        
        # Update visualization
        self.update_visualization()
        
        # Interaction state
        self.ik_mode = False
        
    def _create_sphere(self, radius):
        md = gl.MeshData.sphere(rows=10, cols=10, radius=radius)
        return md
        
    def update_visualization(self):
        # Get joint positions
        positions = self.arm_model.get_joint_positions()
        positions = [(joint_id, pos) for joint_id, pos in positions]
        
        # Update joint positions
        for joint_id, pos in positions:
            if joint_id < len(self.joint_items):
                self.joint_items[joint_id].resetTransform()
                self.joint_items[joint_id].translate(pos[0], pos[1], pos[2])
            
        # Update links
        for i in range(len(positions) - 1):
            current_id, current_pos = positions[i]
            next_ids = [item[0] for item in positions[i+1:]]
            if current_id + 1 in next_ids and current_id < len(self.link_items):
                next_idx = next_ids.index(current_id + 1) + i + 1
                next_id, next_pos = positions[next_idx]
                
                pts = np.array([current_pos, next_pos])
                self.link_items[current_id].setData(pos=pts)
                
    def set_ik_mode(self, enabled):
        """Enable or disable inverse kinematics mode"""
        self.ik_mode = enabled
        
    def set_target_position(self, position):
        """Set the target position for the end effector"""
        self.target_marker.resetTransform()
        self.target_marker.translate(position[0], position[1], position[2])
        self.target_marker.setVisible(True)
                
    def mousePressEvent(self, event):
        if self.ik_mode and event.button() == Qt.LeftButton:
            # Get mouse position in world coordinates
            pos = self.mapToView(event.pos())
            if pos is not None:
                self.position_selected.emit(np.array(pos))
        else:
            super().mousePressEvent(event)
        
    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        
    def wheelEvent(self, event):
        super().wheelEvent(event)
        
    def mapToView(self, pos):
        # Convert mouse position to 3D world coordinates
        # This is a simplified version that projects onto the XY plane at Z=0
        # A more sophisticated version would use raycasting
        
        # Get the screen position
        x, y = pos.x(), pos.y()
        
        # Get the view matrix
        view = self.viewMatrix()
        
        # Create a point at cursor position on the near plane
        near_point = self.unprojectPoint(QVector3D(x, y, 0))
        
        # Create a point at cursor position on the far plane
        far_point = self.unprojectPoint(QVector3D(x, y, 1))
        
        # Create a ray from the near point to the far point
        ray_direction = far_point - near_point
        
        # Find intersection with the z=0 plane
        if abs(ray_direction.z()) > 1e-6:  # Avoid division by zero
            t = -near_point.z() / ray_direction.z()
            intersection = near_point + t * ray_direction
            return [intersection.x(), intersection.y(), 0]
        
        return None

class ConnectionSettingsWidget(QWidget):
    connection_changed = pyqtSignal(str, str)
    
    def __init__(self, arm_ip="192.168.1.100", camera_ip="192.168.1.101"):
        super().__init__()
        
        self.arm_ip_input = QLineEdit(arm_ip)
        self.camera_ip_input = QLineEdit(camera_ip)
        
        layout = QFormLayout()
        layout.addRow("Robot Arm IP:", self.arm_ip_input)
        layout.addRow("Camera IP:", self.camera_ip_input)
        
        connect_btn = QPushButton("Connect")
        connect_btn.clicked.connect(self.apply_settings)
        layout.addRow("", connect_btn)
        
        self.setLayout(layout)
        
    def apply_settings(self):
        arm_ip = self.arm_ip_input.text()
        camera_ip = self.camera_ip_input.text()
        self.connection_changed.emit(arm_ip, camera_ip)
        
    def get_ips(self):
        return self.arm_ip_input.text(), self.camera_ip_input.text()

class AngleInputWidget(QWidget):
    valueChanged = pyqtSignal(float)
    
    def __init__(self, initial_value=0, min_val=-180, max_val=180):
        super().__init__()
        
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Create the spin box for direct input
        self.spin_box = QDoubleSpinBox()
        self.spin_box.setRange(min_val, max_val)
        self.spin_box.setValue(initial_value)
        self.spin_box.setSuffix("°")
        self.spin_box.setFixedWidth(70)
        self.spin_box.valueChanged.connect(self._spin_box_changed)
        
        layout.addWidget(self.spin_box)
        self.setLayout(layout)
        
    def _spin_box_changed(self, value):
        self.valueChanged.emit(value)
        
    def setValue(self, value):
        self.spin_box.setValue(value)
        
    def value(self):
        return self.spin_box.value()

class GearRatioWidget(QWidget):
    valueChanged = pyqtSignal(float)
    
    def __init__(self, initial_value=1.0, joint_id=0):
        super().__init__()
        
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        
        self.joint_id = joint_id
        self.label = QLabel(f"Gear Ratio:")
        
        self.spin_box = QDoubleSpinBox()
        self.spin_box.setRange(0.1, 10.0)
        self.spin_box.setValue(initial_value)
        self.spin_box.setSingleStep(0.1)
        self.spin_box.setDecimals(2)
        self.spin_box.setFixedWidth(70)
        self.spin_box.valueChanged.connect(self._value_changed)
        
        layout.addWidget(self.label)
        layout.addWidget(self.spin_box)
        layout.addStretch(1)
        
        self.setLayout(layout)
        
    def _value_changed(self, value):
        self.valueChanged.emit(value)
        
    def setValue(self, value):
        self.spin_box.setValue(value)
        
    def value(self):
        return self.spin_box.value()

class InverseKinematicsWidget(QWidget):
    update_position = pyqtSignal(np.ndarray)
    toggle_ik_mode = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        
        layout = QVBoxLayout()
        
        # Create input fields for X, Y, Z coordinates
        coord_layout = QFormLayout()
        
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-10, 10)
        self.x_input.setValue(0)
        self.x_input.setSingleStep(0.1)
        self.x_input.setDecimals(2)
        
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-10, 10)
        self.y_input.setValue(3.0)  # Start with a reasonable Y value
        self.y_input.setSingleStep(0.1)
        self.y_input.setDecimals(2)
        
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(-10, 10)
        self.z_input.setValue(1.0)  # Start with a reasonable Z value
        self.z_input.setSingleStep(0.1)
        self.z_input.setDecimals(2)
        
        coord_layout.addRow("X:", self.x_input)
        coord_layout.addRow("Y:", self.y_input)
        coord_layout.addRow("Z:", self.z_input)
        
        # Create buttons
        button_layout = QHBoxLayout()
        
        self.update_btn = QPushButton("Update End Effector Position")
        self.update_btn.clicked.connect(self._update_position)
        
        self.enable_ik_mode_btn = QPushButton("Enable 3D Positioning")
        self.enable_ik_mode_btn.setCheckable(True)
        self.enable_ik_mode_btn.toggled.connect(self._toggle_ik_mode)
        
        button_layout.addWidget(self.update_btn)
        button_layout.addWidget(self.enable_ik_mode_btn)
        
        # Add layouts to main layout
        layout.addLayout(coord_layout)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
    def _update_position(self):
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()
        self.update_position.emit(np.array([x, y, z]))
        
    def _toggle_ik_mode(self, enabled):
        self.toggle_ik_mode.emit(enabled)
        
    def set_position(self, position):
        """Update the input fields with current end effector position"""
        self.x_input.setValue(position[0])
        self.y_input.setValue(position[1])
        self.z_input.setValue(position[2])

class CameraSettingsWidget(QWidget):
    settings_changed = pyqtSignal(str, int, str)
    
    def __init__(self, camera_ip="192.168.1.101", port=81, stream_path="/stream"):
        super().__init__()
        
        layout = QFormLayout()
        
        self.camera_ip_input = QLineEdit(camera_ip)
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(port)
        self.stream_path_input = QLineEdit(stream_path)
        
        layout.addRow("Camera IP:", self.camera_ip_input)
        layout.addRow("Port:", self.port_input)
        layout.addRow("Stream Path:", self.stream_path_input)
        
        apply_btn = QPushButton("Apply Camera Settings")
        apply_btn.clicked.connect(self.apply_settings)
        layout.addRow("", apply_btn)
        
        self.setLayout(layout)
        
    def apply_settings(self):
        camera_ip = self.camera_ip_input.text()
        port = self.port_input.value()
        stream_path = self.stream_path_input.text()
        self.settings_changed.emit(camera_ip, port, stream_path)

class BuckyArmController(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Load settings
        self.settings = QSettings("BuckyArm", "Controller")
        self.arm_ip = self.settings.value("arm_ip", "192.168.1.100")
        self.camera_ip = self.settings.value("camera_ip", "192.168.1.101")
        self.camera_port = int(self.settings.value("camera_port", 81))
        self.camera_stream_path = self.settings.value("camera_stream_path", "/stream")
        
        # Robot arm model
        self.arm_model = RobotArmModel()
        
        # Load saved joint angles if they exist
        self.load_joint_angles()
        
        # Set up the UI
        self.init_ui()
        
        # Timer for periodic updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_visualization)
        self.update_timer.start(100)  # Update every 100ms
        
        # Camera loading state
        self.camera_tab_active = False
        
    def init_ui(self):
        self.setWindowTitle("Bucky Arm Controller")
        self.setGeometry(100, 100, 1300, 900)
        
        # Main layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        
        # Left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)
        
        # Right panel for visualization and camera
        right_panel = QTabWidget()
        
        # Create a splitter to allow resizing
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([400, 900])
        main_layout.addWidget(splitter)
        
        # Connection settings
        connection_group = QGroupBox("Connection Settings")
        self.connection_widget = ConnectionSettingsWidget(self.arm_ip, self.camera_ip)
        self.connection_widget.connection_changed.connect(self.update_connection)
        
        connection_layout = QVBoxLayout()
        connection_layout.addWidget(self.connection_widget)
        connection_group.setLayout(connection_layout)
        left_layout.addWidget(connection_group)
        
        # Joint controls
        joints_group = QGroupBox("Joint Controls")
        joints_layout = QGridLayout()
        
        self.joint_sliders = []
        self.joint_values = []
        self.gear_ratio_widgets = []
        
        # Headers
        joints_layout.addWidget(QLabel("Joint"), 0, 0)
        joints_layout.addWidget(QLabel("Control"), 0, 1, 1, 3)
        joints_layout.addWidget(QLabel("Angle"), 0, 4)
        joints_layout.addWidget(QLabel("Value"), 0, 5)
        joints_layout.addWidget(QLabel("Speed"), 0, 6)
        joints_layout.addWidget(QLabel("Gear Ratio"), 0, 7)
        
        for i, joint in enumerate(self.arm_model.joints):
            row = i + 1  # Start from row 1 due to headers
            
            # Joint label
            joints_layout.addWidget(QLabel(f"{joint.name}:"), row, 0)
            
            # Forward button
            fwd_btn = QPushButton("Forward")
            fwd_btn.setFixedWidth(80)
            fwd_btn.clicked.connect(lambda checked, j=i: self.control_motor(j, "forward"))
            joints_layout.addWidget(fwd_btn, row, 1)
            
            # Backward button
            back_btn = QPushButton("Backward")
            back_btn.setFixedWidth(80)
            back_btn.clicked.connect(lambda checked, j=i: self.control_motor(j, "backward"))
            joints_layout.addWidget(back_btn, row, 2)
            
            # Stop button
            stop_btn = QPushButton("Stop")
            stop_btn.setFixedWidth(80)
            stop_btn.clicked.connect(lambda checked, j=i: self.control_motor(j, "stop"))
            joints_layout.addWidget(stop_btn, row, 3)
            
            # Joint angle slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(joint.min_angle)
            slider.setMaximum(joint.max_angle)
            slider.setValue(joint.angle)
            slider.valueChanged.connect(lambda value, j=i: self.update_joint_angle_from_slider(j, value))
            joints_layout.addWidget(slider, row, 4)
            self.joint_sliders.append(slider)
            
            # Value input
            value_input = AngleInputWidget(joint.angle, joint.min_angle, joint.max_angle)
            value_input.valueChanged.connect(lambda value, j=i: self.update_joint_angle_from_input(j, value))
            joints_layout.addWidget(value_input, row, 5)
            self.joint_values.append(value_input)
            
            # Speed slider
            speed_slider = QSlider(Qt.Horizontal)
            speed_slider.setMinimum(100)
            speed_slider.setMaximum(2000)
            speed_slider.setValue(500)
            speed_slider.valueChanged.connect(lambda value, j=i: self.set_speed(j, value))
            joints_layout.addWidget(speed_slider, row, 6)
            
            # Gear ratio input
            gear_ratio = GearRatioWidget(joint.gear_ratio, i)
            gear_ratio.valueChanged.connect(lambda value, j=i: self.update_gear_ratio(j, value))
            joints_layout.addWidget(gear_ratio, row, 7)
            self.gear_ratio_widgets.append(gear_ratio)
            
        joints_group.setLayout(joints_layout)
        left_layout.addWidget(joints_group)
        
        # Inverse Kinematics control
        ik_group = QGroupBox("Inverse Kinematics")
        self.ik_widget = InverseKinematicsWidget()
        self.ik_widget.update_position.connect(self.set_end_effector_position)
        self.ik_widget.toggle_ik_mode.connect(self.toggle_ik_mode)
        
        ik_layout = QVBoxLayout()
        ik_layout.addWidget(self.ik_widget)
        ik_group.setLayout(ik_layout)
        left_layout.addWidget(ik_group)
        
        # Action buttons
        actions_group = QGroupBox("Actions")
        actions_layout = QHBoxLayout()
        
        update_btn = QPushButton("Update Position")
        update_btn.clicked.connect(self.send_position_to_robot)
        actions_layout.addWidget(update_btn)
        
        reset_btn = QPushButton("Reset Position")
        reset_btn.clicked.connect(self.reset_position)
        actions_layout.addWidget(reset_btn)
        
        actions_group.setLayout(actions_layout)
        left_layout.addWidget(actions_group)
        
        # Add spacer at the bottom
        left_layout.addStretch(1)
        
        # 3D visualization tab
        viz_widget = QWidget()
        viz_layout = QVBoxLayout()
        viz_widget.setLayout(viz_layout)
        
        self.arm_visualizer = RobotArmVisualizer(self.arm_model)
        self.arm_visualizer.position_selected.connect(self.handle_position_selected)
        viz_layout.addWidget(self.arm_visualizer)
        
        viz_instruction = QLabel("Use mouse to manipulate view: Left-click to rotate, Right-click to pan, Scroll to zoom")
        viz_layout.addWidget(viz_instruction)
        
        right_panel.addTab(viz_widget, "3D Visualization")
        
        # Camera feed tab
        camera_widget = QWidget()
        camera_layout = QVBoxLayout()
        camera_widget.setLayout(camera_layout)
        
        self.camera_view = QWebEngineView()
        # We'll load the camera feed only when the tab is selected
        camera_layout.addWidget(self.camera_view)
        
        right_panel.addTab(camera_widget, "Camera Feed")
        
        # Camera settings tab
        camera_settings_widget = QWidget()
        camera_settings_layout = QVBoxLayout()
        camera_settings_widget.setLayout(camera_settings_layout)
        
        self.camera_settings = CameraSettingsWidget(
            self.camera_ip, 
            self.camera_port, 
            self.camera_stream_path
        )
        self.camera_settings.settings_changed.connect(self.update_camera_settings)
        
        camera_settings_layout.addWidget(self.camera_settings)
        
        right_panel.addTab(camera_settings_widget, "Camera Settings")
        
        # Connect tab changed signal
        right_panel.currentChanged.connect(self.on_tab_changed)
        
        # Load previously saved joint gear ratios if they exist
        self.load_gear_ratios()
        
        # Update sliders and input fields to match loaded joint angles
        for i, joint in enumerate(self.arm_model.joints):
            if i < len(self.joint_sliders) and i < len(self.joint_values):
                self.joint_sliders[i].setValue(int(joint.angle))
                self.joint_values[i].setValue(joint.angle)
        
        # Update IK widget with initial end effector position
        self.ik_widget.set_position(self.arm_model.end_effector_pos)
        
        # Show the window
        self.show()
        
    def update_joint_angle_from_slider(self, joint_id, angle):
        # Update the arm model
        actual_angle = self.arm_model.update_joint_angle(joint_id, angle)
        
        # Update the value input without triggering signals
        self.joint_values[joint_id].blockSignals(True)
        self.joint_values[joint_id].setValue(actual_angle)
        self.joint_values[joint_id].blockSignals(False)
        
        # Update IK widget with new end effector position
        self.ik_widget.set_position(self.arm_model.end_effector_pos)
        
        # Update the visualization
        self.arm_visualizer.update_visualization()
        
    def update_joint_angle_from_input(self, joint_id, angle):
        # Update the arm model
        actual_angle = self.arm_model.update_joint_angle(joint_id, angle)
        
        # Update the slider without triggering signals
        self.joint_sliders[joint_id].blockSignals(True)
        self.joint_sliders[joint_id].setValue(int(actual_angle))
        self.joint_sliders[joint_id].blockSignals(False)
        
        # Update IK widget with new end effector position
        self.ik_widget.set_position(self.arm_model.end_effector_pos)
        
        # Update the visualization
        self.arm_visualizer.update_visualization()
        
    def update_gear_ratio(self, joint_id, value):
        if joint_id < len(self.arm_model.joints):
            self.arm_model.joints[joint_id].gear_ratio = value
            self.save_gear_ratios()
            
    def save_gear_ratios(self):
        """Save gear ratios to settings"""
        ratios = [joint.gear_ratio for joint in self.arm_model.joints]
        self.settings.setValue("gear_ratios", json.dumps(ratios))
        
    def load_gear_ratios(self):
        """Load gear ratios from settings"""
        ratios_json = self.settings.value("gear_ratios", None)
        if ratios_json:
            try:
                ratios = json.loads(ratios_json)
                for i, ratio in enumerate(ratios):
                    if i < len(self.arm_model.joints):
                        self.arm_model.joints[i].gear_ratio = ratio
                        self.gear_ratio_widgets[i].setValue(ratio)
            except:
                pass  # If there's an error loading, use defaults
        
    def control_motor(self, motor_id, direction):
        if motor_id >= len(self.arm_model.joints):
            return
            
        try:
            url = f"http://{self.arm_ip}/{direction}{motor_id}"
            requests.get(url, timeout=1)
        except requests.exceptions.RequestException as e:
            print(f"Error controlling motor {motor_id}: {e}")
            QMessageBox.warning(self, "Connection Error",
                f"Could not connect to robot arm at {self.arm_ip}.\nPlease check the IP address and connection.")
            
    def set_speed(self, motor_id, speed):
        if motor_id >= len(self.arm_model.joints):
            return
            
        try:
            url = f"http://{self.arm_ip}/setSpeed{motor_id}?value={speed}"
            requests.get(url, timeout=1)
        except requests.exceptions.RequestException as e:
            print(f"Error setting speed for motor {motor_id}: {e}")
            
    def update_connection(self, arm_ip, camera_ip):
        """Update connection settings"""
        self.arm_ip = arm_ip
        self.camera_ip = camera_ip
        
        # Save settings
        self.settings.setValue("arm_ip", arm_ip)
        self.settings.setValue("camera_ip", camera_ip)
        
        # Update camera view
        self.update_camera_view()
        
        QMessageBox.information(self, "Connection Updated", 
            f"Connected to:\nRobot Arm: {arm_ip}\nCamera: {camera_ip}")
            
    def update_camera_view(self):
        """Update the camera view with the current camera IP"""
        if self.camera_tab_active:
            camera_url = f"http://{self.camera_ip}:{self.camera_port}{self.camera_stream_path}"
            self.camera_view.setUrl(QUrl(camera_url))
        else:
            # If tab is not active, load a blank page to save bandwidth
            self.camera_view.setUrl(QUrl("about:blank"))
            
    def update_camera_settings(self, camera_ip, port, stream_path):
        """Update camera settings"""
        self.camera_ip = camera_ip
        self.camera_port = port
        self.camera_stream_path = stream_path
        
        # Save settings
        self.settings.setValue("camera_ip", camera_ip)
        self.settings.setValue("camera_port", port)
        self.settings.setValue("camera_stream_path", stream_path)
        
        # Update camera view if tab is active
        self.update_camera_view()
        
        QMessageBox.information(self, "Camera Settings Updated", 
            f"Camera settings updated to:\nIP: {camera_ip}\nPort: {port}\nStream Path: {stream_path}")
            
    def on_tab_changed(self, index):
        """Handle tab change events"""
        tab_widget = self.sender()
        if tab_widget:
            tab_text = tab_widget.tabText(index)
            self.camera_tab_active = (tab_text == "Camera Feed")
            self.update_camera_view()
        
    def send_position_to_robot(self):
        """Send current joint positions to the robot"""
        # Check if position actually changed since last update
        if not self.arm_model.position_changed:
            QMessageBox.information(self, "No Change", "Robot position has not changed since last update.")
            return
            
        # Get the current joint angles from the model
        for i, joint in enumerate(self.arm_model.joints):
            if i >= len(self.arm_model.joints):
                continue
                
            # Calculate motor rotations based on gear ratio
            motor_rotations = joint.angle * joint.gear_ratio
            
            # First stop the motor
            self.control_motor(i, "stop")
            
            # Then set the direction and start the motor
            if motor_rotations > 0:
                self.control_motor(i, "forward")
            else:
                self.control_motor(i, "backward")
                
        # Save the current joint angles
        self.save_joint_angles()
        
        # Reset the position changed flag
        self.arm_model.reset_position_changed_flag()
                
    def reset_position(self):
        """Reset all joint angles to default upright position"""
        default_angles = [0, 90, 90, 0, 0]  # Standing upright
        for i, angle in enumerate(default_angles):
            if i < len(self.arm_model.joints) and i < len(self.joint_sliders) and i < len(self.joint_values):
                self.arm_model.update_joint_angle(i, angle)
                self.joint_sliders[i].setValue(angle)
                self.joint_values[i].setValue(angle)
        
        # Update IK widget with new end effector position
        self.ik_widget.set_position(self.arm_model.end_effector_pos)
        
        # Update the visualization
        self.arm_visualizer.update_visualization()
        
        # Save the current joint angles
        self.save_joint_angles()
        
    def save_joint_angles(self):
        """Save joint angles to settings"""
        angles = self.arm_model.get_joint_angles()
        self.settings.setValue("joint_angles", json.dumps(angles))
        
    def load_joint_angles(self):
        """Load joint angles from settings"""
        angles_json = self.settings.value("joint_angles", None)
        if angles_json:
            try:
                angles = json.loads(angles_json)
                self.arm_model.set_joint_angles(angles)
            except:
                # If there's an error loading, use default upright position
                default_angles = [0, 90, 90, 0, 0]
                self.arm_model.set_joint_angles(default_angles)
        
    def set_end_effector_position(self, position):
        """Set the end effector position using inverse kinematics"""
        # Set target position marker
        self.arm_visualizer.set_target_position(position)
        
        # Apply inverse kinematics
        success = self.arm_model.update_end_effector(position)
        
        if success:
            # Update sliders and inputs
            for i, joint in enumerate(self.arm_model.joints):
                if i < len(self.joint_sliders) and i < len(self.joint_values):
                    self.joint_sliders[i].setValue(int(joint.angle))
                    self.joint_values[i].setValue(joint.angle)
        else:
            QMessageBox.warning(self, "Inverse Kinematics", "Could not reach the target position.")
            
        # Update the visualization
        self.arm_visualizer.update_visualization()
        
    def toggle_ik_mode(self, enabled):
        """Toggle inverse kinematics mode"""
        self.arm_visualizer.set_ik_mode(enabled)
        
    def handle_position_selected(self, position):
        """Handle when a position is selected in the 3D view"""
        self.ik_widget.set_position(position)
        self.set_end_effector_position(position)
        
    def update_visualization(self):
        """Periodic update of the visualization"""
        self.arm_visualizer.update_visualization()
        
    def closeEvent(self, event):
        """Save settings when closing the application"""
        self.save_gear_ratios()
        self.save_joint_angles()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = BuckyArmController()
    sys.exit(app.exec_())