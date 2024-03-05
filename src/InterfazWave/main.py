import folium

if __name__ == '__main__':
    
    #Variables
    map_filepath = "folium-map.html"
    center_coord = [4.7086033306706145, -74.06116161374081]
    marker_coord = [[4.7086033306706145, -74.06116161374081], [4.706954992957293, -74.06297268202088], [4.707710271226791, -74.05265169756495]]
    marker_radius = 20
    #Mapa
    vmap = folium.Map(center_coord, zoom_start=100)

    for index, i in enumerate(marker_coord):
        if(index == 0):
            folium.Marker(
                location = i,
                icon = folium.Icon(color='red'),
                tooltip='Partida/Llegada',
                ).add_to(vmap)
        else:
            folium.vector_layers.Circle(
            location=i,
            tooltip=f"Radio de {marker_radius} metros",
            radius=marker_radius,
            color="red",
            fill=True,
            fill_color="blue"
                ).add_to(vmap)
            folium.Marker(
                location = i,
                color = 'green',
                tooltip=f'Estacion #{index}',
                ).add_to(vmap)

    #lines
    folium.PolyLine(
        marker_coord,
        tooltip='Trayectoria de ida',
        color= 'blue',
        weight= 10,
        opacity= 0.5
    ).add_to(vmap)

    folium.PolyLine(
        [marker_coord[0], marker_coord[-1]],
        tooltip='Trayectoria de regreso',
        color= 'red',
        weight= 10,
        opacity= 0.5
    ).add_to(vmap)

    vmap.save(map_filepath)