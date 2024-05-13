from PyQt5 import QtCore, QtWidgets, QtWebEngineWidgets
from folium.plugins import Draw
import folium, io, sys, json


def javaScriptConsoleMessage(level, msg, line, sourceID):
    try:
        geojson = json.loads(msg)
        if 'geometry' in geojson and 'coordinates' in geojson['geometry']:
            coordinates = geojson['geometry']['coordinates']
            if isinstance(coordinates[0], list):
                # Marker coordinates are nested lists in GeoJSON, so we need to flatten them
                coordinates = [item for sublist in coordinates for item in sublist]
            coordinates = coordinates[::-1]
            print("Marker coordinates:", coordinates)
    except Exception as e:
        print("Error parsing JavaScript console message:", e)

if __name__ == '__main__': 
    app = QtWidgets.QApplication(sys.argv)

    m = folium.Map(location=[59.092086890317404, 5.910674107222455], zoom_start=19)

    draw = Draw(
       draw_options={
          'polyline':False,
          'rectangle':False,
          'polygon':False,
          'circle':False,
          'marker':True,
          'circlemarker':False},
       edit_options={'edit':True})
    m.add_child(draw)

    data = io.BytesIO()
    m.save(data, close_file=False)

    view = QtWebEngineWidgets.QWebEngineView()
    page = QtWebEngineWidgets.QWebEnginePage(view)
    page.javaScriptConsoleMessage = javaScriptConsoleMessage
    view.setPage(page)
    view.setHtml(data.getvalue().decode())
    view.show()
    sys.exit(app.exec_())
