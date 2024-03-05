from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWebEngineWidgets import QWebEngineView
import folium
import geocoder

class MianApp(QMainWindow):
    def __init__(self, parent=None,*args):
        super(MianApp, self).__init__(parent=parent)

        
        g = geocoder.ip('me')
        coordinates = g.latlng
        if coordinates is not None:
            latitude, longitude = coordinates
            m = folium.Map(location=[latitude, longitude], zoom_start=100)
        else:
            m = folium.Map(location=[4.7086033306706145, -74.06116161374081], zoom_start=100)


        marker_coord = [[4.706954992957293, -74.06297268202088], [4.707710271226791, -74.05265169756495]]
        # marker_coord = []
        marker_radius = 20
        #Mapa
        
        for index, i in enumerate(marker_coord):
            if(index == 0):
                folium.Marker(
                    location = [i[0], i[1]],
                    icon = folium.Icon(color='red'),
                    tooltip='Partida/Llegada',
                    ).add_to(m)
            else:
                folium.vector_layers.Circle(
                location=i,
                tooltip=f"Radio de {marker_radius} metros",
                radius=marker_radius,
                color="red",
                fill=True,
                fill_color="blue"
                    ).add_to(m)
                folium.Marker(
                    location = i,
                    color = 'green',
                    tooltip=f'Estacion #{index}',
                    ).add_to(m)
            #lines
        if(len(marker_coord) > 2):
            folium.PolyLine(
                marker_coord,
                tooltip='Trayectoria de ida',
                color= 'blue',
                weight= 10,
                opacity= 0.5
            ).add_to(m)
            folium.PolyLine(
                [marker_coord[0], marker_coord[-1]],
                tooltip='Trayectoria de regreso',
                color= 'red',
                weight= 10,
                opacity= 0.5
            ).add_to(m)
        elif(len(marker_coord) == 2):
                folium.PolyLine(
                [marker_coord[0], marker_coord[-1]],
                tooltip='Trayectoria de ida y vuelta',
                color= 'purple',
                weight= 10,
                opacity= 0.5
            ).add_to(m)
        self = QWebEngineView()
        self.setFixedSize(600,400)
        self.setWindowTitle('Wave project')
        self.setHtml(m.get_root().render())
        self.show()





if __name__ == '__main__':
    app = QApplication([])
    window = MianApp()
    window.show()
    app.exec_()

