import serial
import threading
import time
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl

# --- Configuración del puerto seirial ---
portName = "COM6"  # Cambia esto a tu puerto serial
baudRate = 115200

# Variables globales
distance = 0.0
displacement = 0.0
data_lock = threading.Lock()
reference_distance = None
cube_position = [0, 0, 0]  # Posición inicial del cubo [x, y, z]

# Buffer para suavizado de movimiento
position_buffer = []
buffer_size = 5

def serial_thread():
    global distance, displacement, reference_distance, position_buffer
    
    # Intentar conectar a varios puertos si el principal falla
    ser = None
    puertos_a_probar = ["COM6", "COM5", "COM4", "COM3", "COM7"]
    
    for puerto in puertos_a_probar:
        try:
            ser = serial.Serial(puerto, baudRate, timeout=1)
            print(f"Conectado a {puerto} a {baudRate} baudios")
            break
        except serial.SerialException as e:
            print(f"No se pudo abrir el puerto {puerto}: {e}")
    
    if ser is None:
        print("No se pudo conectar a ningún puerto. Verifica las conexiones.")
        return

    buffer = ""
    while True:
        try:
            # Leer datos disponibles
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer += data.decode('utf-8', errors='ignore')
                
                # Procesar líneas completas
                if '\n' in buffer:
                    lines = buffer.split('\n')
                    buffer = lines[-1]  # Mantener la última línea incompleta
                    
                    for line in lines[:-1]:  # Procesar todas las líneas completas
                        line = line.strip()
                        if ',' in line:
                            try:
                                dist_str, displ_str = line.split(',')
                                dist = float(dist_str)
                                displ = float(displ_str)
                                
                                # Validar datos
                                if 2 <= dist <= 400:  # Rango válido del HC-SR04
                                    with data_lock:
                                        distance = dist
                                        displacement = displ
                                        
                                        # Calcular nueva posición del cubo (solo en el eje Z)
                                        new_z_position = -displacement * 5  # Factor de escala para visualización
                                        
                                        # Añadir al buffer para suavizado
                                        position_buffer.append(new_z_position)
                                        if len(position_buffer) > buffer_size:
                                            position_buffer.pop(0)
                            except ValueError:
                                pass  # Ignorar líneas mal formateadas
        except Exception as e:
            print(f"Error leyendo serial: {e}")
            time.sleep(0.1)

def crear_cubo():
    # 8 vértices del cubo
    vertices = np.array([
        [-25, -25, -25], # 0
        [ 25, -25, -25], # 1
        [ 25,  25, -25], # 2
        [-25,  25, -25], # 3
        [-25, -25,  25], # 4
        [ 25, -25,  25], # 5
        [ 25,  25,  25], # 6
        [-25,  25,  25], # 7
    ])
    # 12 caras triangulares (2 por cada cara del cubo)
    faces = np.array([
        [0, 1, 2], [0, 2, 3],     # back
        [4, 5, 6], [4, 6, 7],     # front
        [0, 1, 5], [0, 5, 4],     # bottom
        [2, 3, 7], [2, 7, 6],     # top
        [1, 2, 6], [1, 6, 5],     # right
        [0, 3, 7], [0, 7, 4],     # left
    ])
    meshdata = gl.MeshData(vertexes=vertices, faces=faces)
    cubo = gl.GLMeshItem(meshdata=meshdata, smooth=False, drawEdges=True, 
                         edgeColor=(0,0,0,1), drawFaces=True, 
                         color=(0.5,0.7,1,1))
    cubo.setGLOptions('opaque')
    return cubo

def crear_plano_referencia():
    # Crear un plano de referencia
    z = np.zeros((2, 2))
    p1 = gl.GLSurfacePlotItem(z=z, shader='shaded', color=(0.5, 0.5, 0.5, 0.8))
    p1.scale(100, 100, 1)
    p1.translate(-50, -50, 0)
    return p1

def crear_ejes():
    eje_x = gl.GLLinePlotItem(pos=np.array([[0,0,0],[100,0,0]]), color=(1,0,0,1), width=2, antialias=True)
    eje_y = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,100,0]]), color=(0,1,0,1), width=2, antialias=True)
    eje_z = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,100]]), color=(0,0,1,1), width=2, antialias=True)
    return eje_x, eje_y, eje_z

class Ventana3D(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Visualización HC-SR04 - ESP32')
        self.setStyleSheet("background-color: white;")

        # Layout principal vertical
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(5)

        # Widget GL
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 400
        self.gl_widget.opts['elevation'] = 30
        self.gl_widget.opts['azimuth'] = 45
        self.gl_widget.setMinimumSize(800, 500)
        self.gl_widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        main_layout.addWidget(self.gl_widget, stretch=10)

        # Overlay para etiquetas
        overlay = QtWidgets.QWidget(self.gl_widget)
        overlay.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        overlay.setStyleSheet("background: transparent;")
        overlay.setGeometry(10, 10, 300, 80)

        overlay_layout = QtWidgets.QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(0, 0, 0, 0)
        overlay_layout.setSpacing(2)

        self.label_distance = QtWidgets.QLabel("Distancia: 0.00 cm")
        self.label_displacement = QtWidgets.QLabel("Desplazamiento: 0.00 cm")
        self.label_fps = QtWidgets.QLabel("FPS: 0")

        for label in (self.label_distance, self.label_displacement, self.label_fps):
            label.setStyleSheet("color: black; font-size: 16px; background: rgba(255,255,255,180); padding: 2px; border-radius: 3px;")
            overlay_layout.addWidget(label)

        # Crear objetos 3D
        self.cubo = crear_cubo()
        self.plano = crear_plano_referencia()
        self.eje_x, self.eje_y, self.eje_z = crear_ejes()
        
        # Añadir objetos a la escena
        self.gl_widget.addItem(self.cubo)
        self.gl_widget.addItem(self.plano)
        self.gl_widget.addItem(self.eje_x)
        self.gl_widget.addItem(self.eje_y)
        self.gl_widget.addItem(self.eje_z)

        # Variables para cálculo de FPS
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0

        # Timer para actualizar la visualización
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(33)  # ~30 FPS para mejor rendimiento

    def resizeEvent(self, event):
        self.findChild(QtWidgets.QWidget).move(10, 10)
        super().resizeEvent(event)

    def actualizar(self):
        global distance, displacement, position_buffer
        
        # Calcular FPS
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = current_time
        
        # Actualizar posición del cubo con suavizado
        with data_lock:
            if position_buffer:
                # Calcular promedio para suavizar el movimiento
                avg_z_position = sum(position_buffer) / len(position_buffer)
                
                # Actualizar posición del cubo (solo en el eje Z)
                self.cubo.resetTransform()
                self.cubo.translate(0, 0, avg_z_position)
                
                # Actualizar etiquetas
                self.label_distance.setText(f"Distancia: {distance:.2f} cm")
                self.label_displacement.setText(f"Desplazamiento: {displacement:.2f} cm")
                self.label_fps.setText(f"FPS: {self.current_fps}")

if __name__ == '__main__':
    # Configurar prioridad normal para el proceso Python
    try:
        import psutil
        p = psutil.Process()
        p.nice(psutil.NORMAL_PRIORITY_CLASS)  # Prioridad normal para evitar sobrecarga
    except:
        print("No se pudo establecer prioridad para el proceso")
    
    # Iniciar hilo serial
    hilo_serial = threading.Thread(target=serial_thread, daemon=True)
    hilo_serial.start()
    
    # Iniciar aplicación Qt
    app = QtWidgets.QApplication([])
    ventana = Ventana3D()
    ventana.resize(900, 600)
    ventana.show()
    app.exec_()
