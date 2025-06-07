import serial
import threading
import time
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
import pyqtgraph.opengl as gl

# --- Configuración del puerto serial ---
portName = "COM6"  # Cambia esto a tu puerto serial
baudRate = 115200

# Variables globales para cuaterniones y ángulos de Euler
q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
pitch, roll, yaw = 0.0, 0.0, 0.0

# Variables para filtro complementario adicional en Python
pitch_filtered, roll_filtered, yaw_filtered = 0.0, 0.0, 0.0
alpha = 0.95  # Factor de filtro complementario para Python

# Variable para control de datos recibidos
data_lock = threading.Lock()

# Variables para tiempo y rendimiento
last_time = time.time()
fps_counter = 0
last_fps_time = time.time()
current_fps = 0

# Variables para detección de errores
error_count = 0
max_errors = 10
last_valid_data = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def serial_thread():
    global q0, q1, q2, q3, pitch, roll, yaw
    global pitch_filtered, roll_filtered, yaw_filtered, last_time
    global error_count, last_valid_data
    
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
                    line, buffer = buffer.split('\n', 1)
                    valores = line.strip().split(',')
                    
                    # Validar datos
                    if len(valores) >= 7:
                        try:
                            new_values = [float(v) for v in valores[:7]]
                            
                            # Verificar si los valores son razonables
                            if abs(new_values[0]) <= 1.0 and abs(new_values[1]) <= 1.0 and \
                               abs(new_values[2]) <= 1.0 and abs(new_values[3]) <= 1.0:
                                
                                with data_lock:
                                    # Actualizar cuaterniones
                                    q0, q1, q2, q3 = new_values[:4]
                                    
                                    # Obtener ángulos del sensor (ya filtrados en Arduino)
                                    pitch_raw, roll_raw, yaw_raw = new_values[4:7]
                                    
                                    # Aplicar filtro complementario adicional en Python
                                    current_time = time.time()
                                    dt = current_time - last_time
                                    if dt <= 0: dt = 0.01
                                    last_time = current_time
                                    
                                    # Inicializar valores si es la primera vez
                                    if pitch_filtered == 0.0 and roll_filtered == 0.0 and yaw_filtered == 0.0:
                                        pitch_filtered = pitch_raw
                                        roll_filtered = roll_raw
                                        yaw_filtered = yaw_raw
                                    else:
                                        # Filtro complementario adicional
                                        pitch_filtered = alpha * pitch_filtered + (1 - alpha) * pitch_raw
                                        roll_filtered = alpha * roll_filtered + (1 - alpha) * roll_raw
                                        yaw_filtered = alpha * yaw_filtered + (1 - alpha) * yaw_raw
                                    
                                    # Actualizar valores finales
                                    pitch = pitch_filtered
                                    roll = roll_filtered
                                    yaw = yaw_filtered
                                    
                                    # Guardar últimos valores válidos
                                    last_valid_data = [q0, q1, q2, q3, pitch, roll, yaw]
                                    error_count = 0
                        except ValueError:
                            error_count += 1
                            if error_count <= max_errors:
                                print(f"Error de conversión en datos: {valores}")
                            
                            # Usar últimos valores válidos si hay muchos errores
                            if error_count > max_errors:
                                with data_lock:
                                    q0, q1, q2, q3, pitch, roll, yaw = last_valid_data
        except Exception as e:
            print(f"Error leyendo serial: {e}")
            time.sleep(0.01)

def crear_cubo():
    # 8 vértices del cubo
    vertices = np.array([
        [-50, -50, -50], # 0
        [ 50, -50, -50], # 1
        [ 50,  50, -50], # 2
        [-50,  50, -50], # 3
        [-50, -50,  50], # 4
        [ 50, -50,  50], # 5
        [ 50,  50,  50], # 6
        [-50,  50,  50], # 7
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
    cubo = gl.GLMeshItem(meshdata=meshdata, smooth=False, drawEdges=True, edgeColor=(0,0,0,1), drawFaces=True, color=(1,1,1,1))
    cubo.setGLOptions('opaque')
    return cubo

def crear_ejes():
    eje_x = gl.GLLinePlotItem(pos=np.array([[0,0,0],[100,0,0]]), color=(1,0,0,1), width=3, antialias=True)
    eje_y = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,100,0]]), color=(0,1,0,1), width=3, antialias=True)
    eje_z = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,100]]), color=(0,0,1,1), width=3, antialias=True)
    return eje_x, eje_y, eje_z

def quaternion_to_matrix(q0, q1, q2, q3):
    # Invertir q1 y q2 para corregir la orientación
    q1 = -q1
    q2 = -q2
    
    return np.array([


        [1 - 2*q2*q2 - 2*q3*q3, 2*q1*q2 - 2*q0*q3,     2*q1*q3 + 2*q0*q2,     0],
        [2*q1*q2 + 2*q0*q3,     1 - 2*q1*q1 - 2*q3*q3, 2*q2*q3 - 2*q0*q1,     0],
        [2*q1*q3 - 2*q0*q2,     2*q2*q3 + 2*q0*q1,     1 - 2*q1*q1 - 2*q2*q2, 0],
        [0,                     0,                     0,                     1]
    ])

class Ventana3D(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Visualización MPU9250 - ESP32')
        self.setStyleSheet("background-color: white;")

        # Layout principal vertical
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(5)

        # Widget GL
        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.opts['distance'] = 400
        self.gl_widget.setMinimumSize(800, 500)
        self.gl_widget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        main_layout.addWidget(self.gl_widget, stretch=10)

        # Overlay para etiquetas
        overlay = QtWidgets.QWidget(self.gl_widget)
        overlay.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        overlay.setStyleSheet("background: transparent;")
        overlay.setGeometry(10, 10, 300, 120)

        overlay_layout = QtWidgets.QVBoxLayout(overlay)
        overlay_layout.setContentsMargins(0, 0, 0, 0)
        overlay_layout.setSpacing(2)

        self.label_roll = QtWidgets.QLabel("Roll: 0.00°")
        self.label_pitch = QtWidgets.QLabel("Pitch: 0.00°")
        self.label_yaw = QtWidgets.QLabel("Yaw: 0.00°")
        self.label_fps = QtWidgets.QLabel("FPS: 0")

        for label in (self.label_roll, self.label_pitch, self.label_yaw, self.label_fps):
            label.setStyleSheet("color: black; font-size: 16px; background: rgba(255,255,255,180); padding: 2px; border-radius: 3px;")
            overlay_layout.addWidget(label)

        # Crear cubo y ejes
        self.cubo = crear_cubo()
        self.eje_x, self.eje_y, self.eje_z = crear_ejes()
        self.gl_widget.addItem(self.cubo)
        self.gl_widget.addItem(self.eje_x)
        self.gl_widget.addItem(self.eje_y)
        self.gl_widget.addItem(self.eje_z)

        # Timer para actualizar la visualización
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.actualizar)
        self.timer.start(33)  # ~30 FPS

    def resizeEvent(self, event):
        self.findChild(QtWidgets.QWidget).move(10, 10)
        super().resizeEvent(event)
 
    def actualizar(self):
        global q0, q1, q2, q3, pitch, roll, yaw
        global fps_counter, last_fps_time, current_fps
        
        # Calcular FPS
        fps_counter += 1
        current_time = time.time()
        if current_time - last_fps_time >= 1.0:
            current_fps = fps_counter
            fps_counter = 0
            last_fps_time = current_time
        
        # Actualizar visualización
        with data_lock:
            mat = quaternion_to_matrix(q0, q1, q2, q3)
            
        from PyQt5.QtGui import QMatrix4x4
        m = QMatrix4x4(*mat.T.flatten())
        self.cubo.setTransform(m)
        
        # Actualizar etiquetas
        self.label_roll.setText(f"Roll: {roll:.2f}°")
        self.label_pitch.setText(f"Pitch: {pitch:.2f}°")
        self.label_yaw.setText(f"Yaw: {yaw:.2f}°")
        self.label_fps.setText(f"FPS: {current_fps}")

    def keyPressEvent(self, event):
        if event.text().lower() == 'r':
            try:
                ser = serial.Serial(portName, baudRate, timeout=1)
                ser.write(b'r')
                ser.close()
            except Exception as e:
                print(f"Error enviando comando reset: {e}")

if __name__ == '__main__':
    # Configurar prioridad alta para el proceso Python (solo en Windows)
    try:
        import psutil
        p = psutil.Process()
        p.nice(psutil.HIGH_PRIORITY_CLASS)
    except:
        print("No se pudo establecer prioridad alta para el proceso")
    
    # Iniciar hilo serial
    hilo_serial = threading.Thread(target=serial_thread, daemon=True)
    hilo_serial.start()
    
    # Iniciar aplicación Qt
    app = QtWidgets.QApplication([])
    ventana = Ventana3D()
    ventana.resize(900, 600)
    ventana.show()
    app.exec_()