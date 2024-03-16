import folium
import io
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore, QtWebChannel, QtGui
import geocoder
import json
from folium import plugins
from stl import mesh
from pyqtgraph.opengl import GLViewWidget, MeshData, GLMeshItem
import numpy as np
import serial_comm as my_serial
import sys

yaw, roll, pitch = 0, 0, 0

def animate_mesh():
    global yaw, roll, pitch
    rotate_mesh(yaw, roll, pitch)

def rotate_mesh(yaw_angle, roll_angle, pitch_angle):
    meshIem.resetTransform()  # Reset the transformation
    meshIem.rotate((yaw_angle), 0, 1, 0)  # Apply yaw rotation
    meshIem.rotate((pitch_angle), 1, 0, 0) 
    meshIem.rotate((roll_angle), 0, 0, 1)
    

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
        # az = "COM5"
        serialForConnect = sys.argv[1]
        serial_connector.connect(serialForConnect)
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
        #     print('Funciona')



class Ui_MainWindow(object):
    def setupUi(self, MainWindow):

        #=============================================================
        try:
            myjsonFile = open('currentData.json', 'r')
        except:
            open("currentData.json", "x")
            myjsonFile = open('currentData.json', 'r')

        df = myjsonFile.read()
        if(df != ''):

            # serial_connector = my_serial.SerialObj(115200)
            # serialForConnect = sys.argv[1]
            # serial_connector.connect(serialForConnect)
            # data_string=serial_connector.get_data().decode('utf-8').replace('\r\n','')
            # data_array=data_string.split(',')
            if(json.loads(df)[0][2] != 'Posicion En Tiempo Real'):
                # self.marker_coord = [[float(data_array[9]), float(data_array[10]), 'Posicion En Tiempo Real'] ,*json.loads(df)]
                self.marker_coord = [[4.706739812511032, -74.15178166325258, 'Posicion En Tiempo Real'] ,*json.loads(df)]

            else:
                self.marker_coord = [*json.loads(df)]


        data = io.BytesIO()

        if len(self.marker_coord) > 0 :
            self.m = folium.Map(location=[self.marker_coord[0][0],self.marker_coord[0][1]], zoom_start=50)
        marker_radius = 20
        for index, i in enumerate(self.marker_coord):
            if(index == 0):
                folium.Marker(
                    location = [i[0], i[1]],
                    icon = folium.Icon(color='black', icon='home'),
                    tooltip=f'{i[2]}',
                    ).add_to(self.m)
            else:
                folium.vector_layers.Circle(
                location=[i[0], i[1]],
                tooltip=f"Radio de {marker_radius} metros",
                radius=marker_radius,
                color="red",
                fill=True,
                fill_color="blue"
                    ).add_to(self.m)
                folium.Marker(
                    location = [i[0], i[1]],
                    icon = folium.Icon(color='red'),
                    tooltip=f'Punto de control: {i[2]}',
                    ).add_to(self.m)
            if(index < len(self.marker_coord)-1):
                folium.PolyLine(
                    [[self.marker_coord[index][0],self.marker_coord[index][1]], [self.marker_coord[index+1][0],self.marker_coord[index+1][1]]],
                    tooltip=f"Trayecto: {self.marker_coord[index][2]} --> {self.marker_coord[index+1][2]}",
                    color= 'black',
                    weight= 10,
                    opacity= 0.5
                ).add_to(self.m)
            #lines
        if(len(self.marker_coord) > 2):
            folium.PolyLine(
                [[self.marker_coord[0][0],self.marker_coord[0][1]], [self.marker_coord[-1][0],self.marker_coord[-1][1]]],
                tooltip='Trayectoria de regreso',
                color= 'red',
                weight= 10,
                opacity= 0.5
            ).add_to(self.m)
        elif(len(self.marker_coord) == 2):
                folium.PolyLine(
                [[self.marker_coord[0][0],self.marker_coord[0][1]], [self.marker_coord[-1][0],self.marker_coord[-1][1]]],
                tooltip='Trayectoria de ida y vuelta',
                color= 'purple',
                weight= 10,
                opacity= 0.5
            ).add_to(self.m)
                
        roundnum = "function(num) {return L.Util.formatNum(num, 5);};"
        folium.plugins.LocateControl().add_to(self.m)

        plugins.MousePosition(position='topright', separator=' | ', prefix="Position:",lat_formatter=roundnum, lng_formatter=roundnum).add_to(self.m)
        self.m.add_child(folium.LatLngPopup())

        self.m.save(data, close_file=False)
        self.webView = QtWebEngineWidgets.QWebEngineView()

        self.webView.setHtml(data.getvalue().decode())
    
        #=============================================================

        MainWindow.setObjectName("InterfazWave")
        MainWindow.resize(862, 556)
        MainWindow.setStyleSheet("color: #FFF; border: none")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.iconOnly = QtWidgets.QWidget(self.centralwidget)
        self.iconOnly.setMaximumSize(QtCore.QSize(81, 16777215))
        self.iconOnly.setObjectName("iconOnly")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.iconOnly)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.Ruta1 = QtWidgets.QPushButton(self.iconOnly)
        self.Ruta1.setMinimumSize(QtCore.QSize(40, 40))
        self.Ruta1.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(".\\img/map-marker (1).png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon.addPixmap(QtGui.QPixmap(".\\img/map-marker.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.Ruta1.setIcon(icon)
        self.Ruta1.setIconSize(QtCore.QSize(20, 20))
        self.Ruta1.setCheckable(True)
        self.Ruta1.setAutoExclusive(True)
        self.Ruta1.setObjectName("Ruta1")
        self.verticalLayout.addWidget(self.Ruta1)
        self.Modelo1 = QtWidgets.QPushButton(self.iconOnly)
        self.Modelo1.setMinimumSize(QtCore.QSize(40, 40))
        self.Modelo1.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(".\\img/settings-sliders (1).png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon1.addPixmap(QtGui.QPixmap(".\\img/settings-sliders.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.Modelo1.setIcon(icon1)
        self.Modelo1.setIconSize(QtCore.QSize(20, 20))
        self.Modelo1.setCheckable(True)
        self.Modelo1.setAutoExclusive(True)
        self.Modelo1.setObjectName("Modelo1")
        self.verticalLayout.addWidget(self.Modelo1)
        self.Metricas1 = QtWidgets.QPushButton(self.iconOnly)
        self.Metricas1.setMinimumSize(QtCore.QSize(40, 40))
        self.Metricas1.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(".\\img/cubes (1).png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon2.addPixmap(QtGui.QPixmap(".\\img/cubes.png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.Metricas1.setIcon(icon2)
        self.Metricas1.setIconSize(QtCore.QSize(20, 20))
        self.Metricas1.setCheckable(True)
        self.Metricas1.setAutoExclusive(True)
        self.Metricas1.setObjectName("Metricas1")
        self.verticalLayout.addWidget(self.Metricas1)
        self.verticalLayout_3.addLayout(self.verticalLayout)
        spacerItem = QtWidgets.QSpacerItem(20, 331, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.horizontalLayout_5.addWidget(self.iconOnly)
        self.fullMenu = QtWidgets.QWidget(self.centralwidget)
        self.fullMenu.setMinimumSize(QtCore.QSize(100, 0))
        self.fullMenu.setMaximumSize(QtCore.QSize(100, 16777215))
        self.fullMenu.setObjectName("fullMenu")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.fullMenu)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.Ruta2 = QtWidgets.QPushButton(self.fullMenu)
        # self.Ruta2.setStyleSheet("text-align:center;")
        self.Ruta2.setMinimumSize(QtCore.QSize(40, 40))
        self.Ruta2.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.Ruta2.setIcon(icon)
        self.Ruta2.setIconSize(QtCore.QSize(20, 20))
        self.Ruta2.setCheckable(True)
        self.Ruta2.setAutoExclusive(True)
        self.Ruta2.setObjectName("Ruta2")
        self.verticalLayout_2.addWidget(self.Ruta2)
        self.Modelo2 = QtWidgets.QPushButton(self.fullMenu)
        self.Modelo2.setMinimumSize(QtCore.QSize(40, 40))
        self.Modelo2.setIcon(icon1)
        self.Modelo2.setIconSize(QtCore.QSize(20, 20))
        self.Modelo2.setCheckable(True)
        self.Modelo2.setAutoExclusive(True)
        self.Modelo2.setObjectName("Modelo2")
        self.verticalLayout_2.addWidget(self.Modelo2)
        self.Metricas2 = QtWidgets.QPushButton(self.fullMenu)
        self.Metricas2.setMinimumSize(QtCore.QSize(40, 40))
        self.Metricas2.setIcon(icon2)
        self.Metricas2.setIconSize(QtCore.QSize(20, 20))
        self.Metricas2.setCheckable(True)
        self.Metricas2.setAutoExclusive(True)
        self.Metricas2.setObjectName("Metricas2")
        self.verticalLayout_2.addWidget(self.Metricas2)
        self.verticalLayout_4.addLayout(self.verticalLayout_2)
        spacerItem1 = QtWidgets.QSpacerItem(20, 331, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem1)
        self.horizontalLayout_5.addWidget(self.fullMenu)
        self.verticalLayout_11 = QtWidgets.QVBoxLayout()
        self.verticalLayout_11.setObjectName("verticalLayout_11")
        self.horizontalLayout_5.addLayout(self.verticalLayout_11)
        self.Content = QtWidgets.QWidget(self.centralwidget)
        self.Content.setObjectName("Content")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.Content)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.Menu = QtWidgets.QPushButton(self.Content)
        self.Menu.setMinimumSize(QtCore.QSize(30, 30))
        self.Menu.setMaximumSize(QtCore.QSize(30, 30))
        self.Menu.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(".\\img/apps.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        # icon3.addPixmap(QtGui.QPixmap(".\\img/apps (1).png"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        self.Menu.setIcon(icon3)
        self.Menu.setIconSize(QtCore.QSize(20, 20))
        self.Menu.setCheckable(True)
        self.Menu.setObjectName("Menu")
        self.verticalLayout_5.addWidget(self.Menu)
        self.stackedWidget = QtWidgets.QStackedWidget(self.Content)
        self.stackedWidget.setObjectName("stackedWidget")
        self.pageRuta = QtWidgets.QWidget()
        self.pageRuta.setMouseTracking(False)
        self.pageRuta.setObjectName("pageRuta")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.pageRuta)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.widget = self.webView
        # self.widget = QtWidgets.QWidget(self.pageRuta)
        self.widget.setMinimumSize(QtCore.QSize(0, 250))
        self.widget.setObjectName("widget")
        self.verticalLayout_9.addWidget(self.widget)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.AgregarParada = QtWidgets.QPushButton(self.pageRuta)
        self.AgregarParada.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.AgregarParada.setObjectName("AgregarParada")
        self.horizontalLayout.addWidget(self.AgregarParada)
        self.EditarParada = QtWidgets.QPushButton(self.pageRuta)
        self.EditarParada.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.EditarParada.setObjectName("EditarParada")
        self.horizontalLayout.addWidget(self.EditarParada)
        self.EliminarParada = QtWidgets.QPushButton(self.pageRuta)
        self.EliminarParada.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.EliminarParada.setObjectName("EliminarParada")
        self.horizontalLayout.addWidget(self.EliminarParada)
        self.Subir = QtWidgets.QPushButton(self.pageRuta)
        self.Subir.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.Subir.setObjectName("Subir")
        self.horizontalLayout.addWidget(self.Subir)
        self.Bajar = QtWidgets.QPushButton(self.pageRuta)
        self.Bajar.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.Bajar.setObjectName("Bajar")
        self.horizontalLayout.addWidget(self.Bajar)
        self.ActualizarDatos = QtWidgets.QPushButton(self.pageRuta)
        self.ActualizarDatos.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.ActualizarDatos.setMinimumSize(QtCore.QSize(115, 0))
        self.ActualizarDatos.setObjectName("ActualizarDatos")
        self.horizontalLayout.addWidget(self.ActualizarDatos)
        self.GuardaDatos = QtWidgets.QPushButton(self.pageRuta)
        self.GuardaDatos.setStyleSheet('border: 1px solid #bf9b6f; padding: 5px')
        self.GuardaDatos.setMinimumSize(QtCore.QSize(100, 0))
        self.GuardaDatos.setObjectName("GuardaDatos")
        self.horizontalLayout.addWidget(self.GuardaDatos)
        self.verticalLayout_9.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout()
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_3 = QtWidgets.QLabel(self.pageRuta)
        self.label_3.setObjectName("label_3")
        #---------
        self.label_3.setStyleSheet("color: #FFF")
        self.verticalLayout_8.addWidget(self.label_3)
        self.IndexList = QtWidgets.QListWidget(self.pageRuta)
        self.IndexList.setMaximumSize(QtCore.QSize(12000000, 16777215))
        self.IndexList.setObjectName("IndexList")
        self.verticalLayout_8.addWidget(self.IndexList)
        self.horizontalLayout_2.addLayout(self.verticalLayout_8)
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label = QtWidgets.QLabel(self.pageRuta)
        self.label.setObjectName("label")
        #---------
        self.label.setStyleSheet("color: #FFF")
        self.verticalLayout_7.addWidget(self.label)
        self.LatitudList = QtWidgets.QListWidget(self.pageRuta)
        self.LatitudList.setMaximumSize(QtCore.QSize(12000000, 12000000))
        self.LatitudList.setObjectName("LatitudList")
        self.verticalLayout_7.addWidget(self.LatitudList)
        self.horizontalLayout_2.addLayout(self.verticalLayout_7)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.label_2 = QtWidgets.QLabel(self.pageRuta)
        self.label_2.setObjectName("label_2")
        #---------
        self.label_2.setStyleSheet("color: #FFF")
        self.verticalLayout_6.addWidget(self.label_2)
        self.LongitudList = QtWidgets.QListWidget(self.pageRuta)
        self.LongitudList.setMaximumSize(QtCore.QSize(12000000, 12000000))
        self.LongitudList.setObjectName("LongitudList")
        self.verticalLayout_6.addWidget(self.LongitudList)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_9.addLayout(self.horizontalLayout_2)
        self.stackedWidget.addWidget(self.pageRuta)
        self.pageMetricas = QtWidgets.QWidget()
        self.pageMetricas.setObjectName("pageMetricas")
        self.label_4 = QtWidgets.QLabel(self.pageMetricas)
        self.label_4.setGeometry(QtCore.QRect(0, 0, 51, 21))
        self.label_4.setObjectName("label_4")
        self.stackedWidget.addWidget(self.pageMetricas)
        self.pageModelo = QtWidgets.QWidget()
        self.pageModelo.setObjectName("pageModelo")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.pageModelo)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        # self.openGLWidget = QtWidgets.QOpenGLWidget(self.pageModelo)
        # self.openGLWidget.setObjectName("openGLWidget")

        #===================================================================================

        view = GLViewWidget(rotationMethod='quaternion')
        view.setCameraPosition(distance=500) 
        view.pan(0, 0, 0) 
        view.opts['panning'] = False 
        view.opts['rotationMode'] = 'cube'
        stl_mesh = mesh.Mesh.from_file('plane.STL')
        points = stl_mesh.points.reshape(-1, 3)
        faces = np.arange(points.shape[0]).reshape(-1, 3)
        mesh_data = MeshData(vertexes=points, faces=faces)
        global meshIem
        meshIem = GLMeshItem(meshdata=mesh_data, smooth=True, drawFaces=False, drawEdges=True, edgeColor=(1, 1, 0, 1))
        view.addItem(meshIem)
        # view.show()
        worker_thread = WorkerThread()
        worker_thread.start()
        self.horizontalLayout_3.addWidget(view)

        #===================================================================================

        # self.horizontalLayout_3.addWidget(self.openGLWidget)
        self.stackedWidget.addWidget(self.pageModelo)
        self.verticalLayout_5.addWidget(self.stackedWidget)
        self.horizontalLayout_5.addWidget(self.Content)
        MainWindow.setCentralWidget(self.centralwidget)

        for i in self.marker_coord:
            self.LatitudList.insertItem(0,str(i[0]))
            self.LongitudList.insertItem(0,str(i[1]))
            self.IndexList.insertItem(0,str(i[2]))


        self.LatitudList.setStyleSheet("background-color: #BF9B6F")
        self.LongitudList.setStyleSheet("background-color: #BF9B6F")
        self.IndexList.setStyleSheet("background-color: #BF9B6F")
        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(1)
        self.LatitudList.setCurrentRow(-1)
        self.Metricas2.toggled['bool'].connect(self.Metricas1.setChecked) # type: ignore
        self.Modelo2.toggled['bool'].connect(self.Modelo1.setChecked) # type: ignore
        self.Metricas1.toggled['bool'].connect(self.Metricas2.setChecked) # type: ignore
        self.Ruta1.toggled['bool'].connect(self.Ruta2.setChecked) # type: ignore
        self.Ruta2.toggled['bool'].connect(self.Ruta1.setChecked) # type: ignore
        self.Modelo1.toggled['bool'].connect(self.Modelo2.setChecked) # type: ignore
        self.Menu.toggled['bool'].connect(self.iconOnly.setVisible) # type: ignore
        self.Menu.toggled['bool'].connect(self.fullMenu.setHidden) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(MainWindow)



    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.Ruta2.setText(_translate("MainWindow", " Ruta"))
        self.Modelo2.setText(_translate("MainWindow", " Metricas"))
        self.Metricas2.setText(_translate("MainWindow", " Modelo"))
        self.AgregarParada.setText(_translate("MainWindow", "Agregar"))
        self.EditarParada.setText(_translate("MainWindow", "Editar"))
        self.EliminarParada.setText(_translate("MainWindow", "Eliminar"))
        self.Subir.setText(_translate("MainWindow", "Subir"))
        self.Bajar.setText(_translate("MainWindow", "Bajar"))
        self.ActualizarDatos.setText(_translate("MainWindow", "Actualizar Pocision"))
        self.GuardaDatos.setText(_translate("MainWindow", "Guardar Datos"))
        self.label_3.setText(_translate("MainWindow", "Elemento:"))
        self.label.setText(_translate("MainWindow", "Latitud:"))
        self.label_2.setText(_translate("MainWindow", "Longitud:"))
        self.label_4.setText(_translate("MainWindow", "Metricas"))
