from datetime import datetime, timedelta
from pyproj import CRS, Transformer


class TypeSender:
    USPACE_MANAGER = "uspace_manager"
    USPACE_CLIENT = "uspace_client"
    OPERATOR_UAV = "operator_uav"
    OPERATOR_VERTIPORT = "operator_vertiport"
    FLIGHTPLAN_GENERATOR = "flight_plan_generator"
    COMMAND_GENERATOR = "command_generator"
    SINGLE_UAV = "single_uav"

class TypeMessage:
    EXTENSSION_ON_OFF = "extension_on_off"
    CMD_FP_REQUEST = "cmd_fp_request"
    USPACE = "uspace"

class RequestState:
    CANCELLED = "Cancelled"
    PENDING = "Pending"
    IN_PROGRESS = "In progress"
    COMPLETED = "Completed"

class UAVStatus:
    IDLE = "idle"
    BUSY = "busy"
    OUT_OF_SERVICE = "out_of_service"

class TimeManager:
    def __init__(self):
        self.start_real_time = None
        self.time_offset = 0.0
        self.current_sim_time = 0.0
        self.current_real_time = None

    def start(self):
        self.start_real_time = datetime.now()

    def stop(self):
        self.start_real_time = None
        self.time_offset = 0.0
        self.current_sim_time = 0.0
        self.current_real_time = None

    def pause(self):
        self.start_pause_time = datetime.now()

    def resume(self):
        end_pause_time = datetime.now()
        pause_duration = (end_pause_time - self.start_pause_time).total_seconds()
        self.time_offset += pause_duration

    def real_to_sim(self, real_time=None):
        if real_time is None:   time = datetime.now()
        else:                   time = real_time
        
        sim_time = (time - self.start_real_time).total_seconds() - self.time_offset
        return sim_time
    
    def sim_to_real(self, sim_time):
        real_time = self.start_real_time + timedelta(seconds=(sim_time + self.time_offset))
        return real_time

class GeospatialManager:
    def __init__(self, lon_origin:float, lat_origin:float, alt_origin:float=0.0):
        # 1. Define the local topocentric coordinate system based on the origin
        # crs_local = CRS.from_proj4(f"+proj=topocentric +ellps=WGS84 +lat_0={lat_origin} +lon_0={lon_origin} +h_0={alt_origin}")

        # 2. Define the WGS84 coordinate system
        # crs_wgs84 = CRS.from_epsg(4979)

        pipeline = (
            f"+proj=pipeline "
            f"+step +proj=topocentric +inv +ellps=WGS84 "
            f"+lon_0={lon_origin} +lat_0={lat_origin} +h_0={alt_origin} "
            f"+step +inv +proj=cart +ellps=WGS84"
        )

        # 3. Create a transformer to convert between the two coordinate systems.
        #    Note: always_xy=True ensures that the transformer expects (lon, lat) order for geographic coordinates.
        # self.transformer = Transformer.from_crs(crs_local, crs_wgs84, always_xy=True)
        self.transformer = Transformer.from_pipeline(pipeline)
    
    def geo_to_sim(self, lon, lat, alt):
        x, y, z = self.transformer.transform(lon, lat, alt, direction="INVERSE")
        return x, y, z
    
    def sim_to_geo(self, x, y, z):
        lon, lat, alt = self.transformer.transform(x, y, z)
        return lon, lat, alt
    
class GeoConverter:
    def __init__(self):
        self.origin = None
        self.transformer = None
 
    def set_origin(self, epsg_origen=25830, epsg_destino=4326):
        self.origin = (epsg_origen, epsg_destino)
        self.transformer = Transformer.from_crs(epsg_origen, epsg_destino, always_xy=True)
   
    def get_UTM_from_idx(self, total_rows, array_idx_x, array_idx_y, x_origin, y_origin, cell_size):
        """
        Versión vectorizada con NumPy para máxima velocidad.
        """
       
        # Asegurarnos de que son arrays de numpy
        # cols = np.array(array_idx_x)
        # rows = np.array(array_idx_y)
       
        # Cálculo directo a todos los elementos a la vez
        x_utm = x_origin + (array_idx_x * cell_size) + (cell_size / 2)
        y_utm = (y_origin + (total_rows * cell_size)) - (array_idx_y * cell_size) - (cell_size / 2)
       
        # Devuelve listas (o puedes dejarlo como arrays si prefieres)
        return x_utm, y_utm
   
    def get_lat_lon_from_UTM(self, x_utm, y_utm):
        """
        Convierte arrays de coordenadas UTM (X, Y) a Latitud y Longitud.
        Versión vectorizada con pyproj para máxima velocidad.
       
        Parámetros:
        - x_utm: lista o numpy array con las coordenadas X.
        - y_utm: lista o numpy array con las coordenadas Y.
        - epsg_origen: Código EPSG de tu UTM.
                    25830 = ETRS89 / UTM zone 30N (El oficial en España peninsular).
                    32630 = WGS 84 / UTM zone 30N (Si el origen era GPS estándar).
        - epsg_destino: 4326 es el estándar WGS84 para Latitud/Longitud.
        """
        # 1. Asegurarnos de que son arrays de NumPy
        # x_arr = np.array(x_utm)
        # y_arr = np.array(y_utm)
       
        # 3. Transformación vectorizada (pyproj procesa los arrays enteros a la vez)
        lon, lat = self.transformer.transform(x_utm, y_utm)
       
        # 4. Devolvemos como listas (o como arrays si prefieres quitar el .tolist())
        return lat, lon
   
    def get_UTM_from_lat_lon(self, lat, lon):
        """
        Convierte arrays de Latitud y Longitud a coordenadas UTM (X, Y).
        Versión vectorizada con pyproj para máxima velocidad.
       
        Parámetros:
        - lat: lista o numpy array con las latitudes.
        - lon: lista o numpy array con las longitudes.
        - epsg_origen: 4326 es el estándar WGS84 para Latitud/Longitud.
        - epsg_destino: Código EPSG de tu UTM (Ej: 25830 para UTM 30N ETRS89).
        """
        # 1. Asegurarnos de que son arrays de NumPy
        # lat_arr = np.array(lat)
        # lon_arr = np.array(lon)
       
        # 3. Transformación vectorizada
        # ¡ATENCIÓN AL ORDEN AQUÍ! Como usamos always_xy=True,
        # a pyproj hay que pasarle PRIMERO la longitud (eje X) y LUEGO la latitud (eje Y).
        x_utm, y_utm = self.transformer.transform(lon, lat, direction="INVERSE")
       
        # 4. Devolvemos las coordenadas UTM
        return x_utm, y_utm
   