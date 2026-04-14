# This function exports a flight path as a KML fil
import search_leg

def save_kml(input, filename="path.kml", name="Path"):
    """
    coords: list of (lon, lat)
    Writes a simple KML file that Google Earth can display as a path.
    Altitude is set to 0 for all points.
    """
    # Build coordinate lines: "lon,lat,0"
    coord_text = ""
    for item in input:
        if isinstance(item, tuple):
            coord_text +=  f"{item[1]},{item[0]},0 "
        else:
            coord_text += f"{item.start_pos[1]},{item.start_pos[0]},0 "

    kml_text = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name}</name>

    <Style id="pathStyle">
      <LineStyle>
        <color>ff0000ff</color>
        <width>3</width>
      </LineStyle>
    </Style>

    <Placemark>
      <name>{name}</name>
      <styleUrl>#pathStyle</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>
          {coord_text.strip()}
        </coordinates>
      </LineString>
    </Placemark>

  </Document>
</kml>
"""

    with open(filename, "w", encoding="utf-8") as f:
        f.write(kml_text)

    print(f"Wrote {filename}")
