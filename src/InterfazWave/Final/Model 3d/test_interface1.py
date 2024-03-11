import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from pyqtgraph.opengl import GLViewWidget, MeshData, GLMeshItem
from stl import mesh
from PyQt5.QtCore import QTimer, QThread
import serial_comm as my_serial

yaw, roll, pitch = 0, 0, 0

def animate_mesh():
    global yaw, roll, pitch
    rotate_mesh(yaw, roll, pitch)

def rotate_mesh(yaw_angle, roll_angle, pitch_angle):
    global mesh
    mesh.resetTransform()  # Reset the transformation
    mesh.rotate((yaw_angle), 0, 1, 0)  # Apply yaw rotation
    mesh.rotate((pitch_angle), 1, 0, 0) 
    mesh.rotate((roll_angle), 0, 0, 1)
    

def update_yaw(value):
    global yaw
    yaw = value
    animate_mesh()

def update_roll(value):
    global roll
    roll = value
    animate_mesh()

def update_pitch(value):
    global pitch
    pitch = value
    animate_mesh()

class WorkerThread(QThread):
    def run(self):
        serial_connector = my_serial.SerialObj(115200)
        az = "COM5"
        serial_connector.connect(az)
        while True:
            if serial_connector.is_connect():
                
                try:
                    data_string=serial_connector.get_data().decode('utf-8').replace('\r\n','')
                    
                    data_array=data_string.split(',')
                    print(data_array)
                    yaw=float(data_array[6])
                    pitch=float(data_array[7])
                    roll=float(data_array[8])
                    update_pitch(pitch)
                    update_roll(roll)
                    update_yaw(yaw)
                except:
                    pass
                
            
        print("Processing complete")

if True:
    app = QApplication(sys.argv)

    # Setup UI for controlling yaw, roll, and pitch
    # 3D View
    view = GLViewWidget(rotationMethod='quaternion')
    view.setCameraPosition(distance=500) 
    view.pan(0, 0, 0) 
    view.opts['panning'] = False 
    view.opts['rotationMode'] = 'cube'

    stl_mesh = mesh.Mesh.from_file('src\plane.stl')

    points = stl_mesh.points.reshape(-1, 3)
    faces = np.arange(points.shape[0]).reshape(-1, 3)

    mesh_data = MeshData(vertexes=points, faces=faces)
    mesh = GLMeshItem(meshdata=mesh_data, smooth=True, drawFaces=False, drawEdges=True, edgeColor=(1, 1, 0, 1))
    view.addItem(mesh)

    # Show window and 3D view
    view.show()

    # Start worker thread
    worker_thread = WorkerThread()
    worker_thread.start()

    sys.exit(app.exec())