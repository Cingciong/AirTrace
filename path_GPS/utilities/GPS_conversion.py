import numpy as np

class GPSPosition:
    def __init__(self):
        # Constants
        self.A = 6378137.0
        self.F_INV = 298.257223563
        self.F = 1 / self.F_INV
        self.E2 = 2 * self.F - self.F ** 2

    def get_radii(self, lat_rad):
        #calculate curv radii
        sin_lat = np.sin(lat_rad)
        den = 1 - self.E2 * (sin_lat ** 2)

        m = (self.A * (1 - self.E2)) / np.power(den, 1.5)
        n = self.A / np.sqrt(den)
        return m, n

    @staticmethod
    def rotate_to_ned(dx, dy, yaw_rad):
        #convert sx, dy to NED by yaw, dy - forward, dx - right
        dn = dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)
        de = dx * np.sin(yaw_rad) - dy * np.cos(yaw_rad)
        return dn, de

    def calculate_path(self, dx_list, dy_list, yaw_list, start_lat, start_lon):
        #calculate path in GPS coords
        current_lat_rad = np.radians(start_lat)
        current_lon_rad = np.radians(start_lon)

        #init pos in deg
        lat = [start_lat]
        lon = [start_lon]

        for dx, dy, yaw in zip(dx_list, dy_list, yaw_list):
            dn, de = self.rotate_to_ned(dx, dy, yaw)

            m, n = self.get_radii(current_lat_rad)

            d_lat_rad = dn / m
            d_lon_rad = de / (n * np.cos(current_lat_rad))

            current_lat_rad += d_lat_rad
            current_lon_rad += d_lon_rad

            lat.append(np.degrees(current_lat_rad))
            lon.append(np.degrees(current_lon_rad))

        return np.array(lat), np.array(lon)