import pymap3d as pm

class GNSSConverter:
    def __init__(self, origin_lat, origin_lon, origin_alt):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_alt = origin_alt

    def to_local_enu(self, lat, lon, alt):
        """
        Convert geodetic coordinates to ENU (local frame) using pymap3d.
        """
        xEast, yNorth, zUp = pm.geodetic2enu(
            lat, lon, alt,
            self.origin_lat, self.origin_lon, self.origin_alt
        )
        return xEast, yNorth, zUp