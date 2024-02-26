import sys
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QVBoxLayout
from PyQt5.QtCore import pyqtSlot, QFile, QTextStream
from PyQt5.QtWebEngineWidgets import QWebEngineView

from sidebar_ui import Ui_MainWindow

import io
import folium

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.iconOnly.hide()
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.pushButton.setChecked(True)

        layout = QVBoxLayout()
        self.setLayout(layout)

        coordinate = (37.8199286, -122.4782551)
        m = folium.Map(
        	zoom_start=13,
        	location=coordinate
        )

        # save map data to data object
        data = io.BytesIO()
        m.save(data, close_file=False)

        webView = QWebEngineView()
        webView.setHtml(data.getvalue().decode())
        layout.addWidget(webView)

    ## Change QPushButton Checkable status when stackedWidget index changed
    def on_stackedWidget_currentChanged(self, index):
        btn_list = self.ui.iconOnly.findChildren(QPushButton) \
                    + self.ui.fullMenu.findChildren(QPushButton)
        
        for btn in btn_list:
            if index in [5, 6]:
                btn.setAutoExclusive(False)
                btn.setChecked(False)
            else:
                btn.setAutoExclusive(True)
            
    ## functions for changing menu page
    def on_pushButton_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(0)
    
    def on_pushButton_5_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(0)

    def on_pushButton_2_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    def on_pushButton_6_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(1)

    def on_pushButton_3_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(2)

    def on_pushButton_7_toggled(self):
        self.ui.stackedWidget.setCurrentIndex(2)

if __name__ == "__main__":
    app = QApplication(sys.argv)

    ## loading style file
    # with open("style.qss", "r") as style_file:
    #     style_str = style_file.read()
    # app.setStyleSheet(style_str)

    ## loading style file, Example 2
    style_file = QFile("style.qss")
    style_file.open(QFile.ReadOnly | QFile.Text)
    style_stream = QTextStream(style_file)
    app.setStyleSheet(style_stream.readAll())


    window = MainWindow()
    window.show()

    sys.exit(app.exec())

# import folium
# import io
# from PyQt5.QtWidgets import *
# from PyQt5.QtCore import *
# from PyQt5.QtGui import *
# from PyQt5 import QtWidgets, QtWebEngineWidgets
# import geocoder

# class MianApp(QtWidgets.QWidget):
#     def __init__(self, parent=None):
#         QtWidgets.QWidget.__init__(self, parent)
#         self.layout = QtWidgets.QVBoxLayout()
#         m = folium.Map(
#             title='coastlines',
#             zoom_start=100)

#         data = io.BytesIO()
#         g = geocoder.ip('me')
#         coordinates = g.latlng
#         if coordinates is not None:
#             latitude, longitude = coordinates
#             m = folium.Map(location=[latitude, longitude], zoom_start=100)
#         else:
#             m = folium.Map(location=[4.7086033306706145, -74.06116161374081], zoom_start=100)


#         marker_coord = [[4.706954992957293, -74.06297268202088], [4.707710271226791, -74.05265169756495]]
#         # marker_coord = []
#         marker_radius = 20
#         #Mapa
        
#         for index, i in enumerate(marker_coord):
#             if(index == 0):
#                 folium.Marker(
#                     location = [i[0], i[1]],
#                     icon = folium.Icon(color='red'),
#                     tooltip='Partida/Llegada',
#                     ).add_to(m)
#             else:
#                 folium.vector_layers.Circle(
#                 location=i,
#                 tooltip=f"Radio de {marker_radius} metros",
#                 radius=marker_radius,
#                 color="red",
#                 fill=True,
#                 fill_color="blue"
#                     ).add_to(m)
#                 folium.Marker(
#                     location = i,
#                     color = 'green',
#                     tooltip=f'Estacion #{index}',
#                     ).add_to(m)
#             #lines
#         if(len(marker_coord) > 2):
#             folium.PolyLine(
#                 marker_coord,
#                 tooltip='Trayectoria de ida',
#                 color= 'blue',
#                 weight= 10,
#                 opacity= 0.5
#             ).add_to(m)
#             folium.PolyLine(
#                 [marker_coord[0], marker_coord[-1]],
#                 tooltip='Trayectoria de regreso',
#                 color= 'red',
#                 weight= 10,
#                 opacity= 0.5
#             ).add_to(m)
#         elif(len(marker_coord) == 2):
#                 folium.PolyLine(
#                 [marker_coord[0], marker_coord[-1]],
#                 tooltip='Trayectoria de ida y vuelta',
#                 color= 'purple',
#                 weight= 10,
#                 opacity= 0.5
#             ).add_to(m)
#         m.save(data, close_file=False)
#         webView = QtWebEngineWidgets.QWebEngineView()
#         webView.setHtml(data.getvalue().decode())
#         self.layout.addWidget(webView)
#         self.setLayout(self.layout)

# if __name__ == '__main__':
#     app = QApplication([])
#     window = MianApp()
#     window.show()
#     app.exec_()