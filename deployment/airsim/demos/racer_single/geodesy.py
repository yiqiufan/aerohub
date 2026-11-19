import math

def ned_to_geo(origin, north, east, down):
    semimajor = 6378137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = math.radians(origin["latitude"])
    scale = math.sqrt(1 - eccentricity_squared * math.sin(latitude) ** 2)
    east_radius = semimajor / scale + origin["altitude"]
    north_radius = semimajor * (1 - eccentricity_squared) / scale**3 + origin["altitude"]
    return {
        "latitude": origin["latitude"] + math.degrees(north / north_radius),
        "longitude": origin["longitude"] + math.degrees(east / (east_radius * math.cos(latitude))),
        "altitude": origin["altitude"] - down,
    }

