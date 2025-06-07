// Código optimizado para ESP32 con HC-SR04 para mediciones precisas// Incluye detección de cambios de distancia y activación de zumbador
#include <RunningMedian.h>
// Definición de pines
const int trigPin = 5;
const int echoPin = 18;
const int buzzerPin = 16;  // Pin para el zumbador
// Constantes para cálculos
#define SOUND_SPEED 0.0343 // Velocidad del sonido en cm/us
#define SAMPLES 5          // Número de muestras para el filtro de mediana
#define DISTANCE_THRESHOLD 4.0  // Umbral de cambio en cm
#define DURATION_THRESHOLD 3000 // Duración mínima en ms (3 segundos)
// Variables para medición
RunningMedian distances = RunningMedian(SAMPLES);
float filteredDistance = 0;
float lastValidDistance = 0;
float referenceDistance = 0;
float initialDistance = 0;
bool referenceSet = false;
bool initialDistanceSet = false;
// Variables para detección de cambios
bool changeDetected = false;
unsigned long changeStartTime = 0;
bool buzzerActive = false;
// Variables para tiempo
unsigned long lastMeasurementTime = 0;
const int measurementInterval = 50; // 50ms = 20Hz
// Variables para control del zumbador
unsigned long buzzerStartTime = 0;
const int buzzerDuration = 2000; // Duración del sonido del zumbador en ms

#include "Simple_MPU6050.h"
#include <Wire.h>
#define MPU6050_DEFAULT_ADDRESS     0x68 // address pin low (GND), default for InvenSense evaluation board
// Definir pines I2C para ESP32
#define SDA_PIN 21
#define SCL_PIN 22
// Pin del zumbador
#define BUZZER_PIN 16
Simple_MPU6050 mpu;
float mag[3]; // Storage location for magnetometer readings
byte MagDataReady = 0;
// Variables para control de inclinación
float initialPitch = 0.0;
float initialRoll = 0.0;
bool initialSet = false;
unsigned long thresholdTime = 0;
bool buzzerOn = false;
//***************************************************************************************
//******************              Callback Function                **********************
//***************************************************************************************
// See mpu.on_FIFO(print_Values); in the Setup Loop
void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat) {
  Quaternion q;
  VectorFloat gravity;
  float ypr[3] = { 0, 0, 0 };
  float xyz[3] = { 0, 0, 0 };
  mpu.GetQuaternion(&q, quat);
  mpu.GetGravity(&gravity, &q);
  mpu.GetYawPitchRoll(ypr, &q, &gravity);
  mpu.ConvertToDegrees(ypr, xyz);
  // Guardar ángulos iniciales
  if (!initialSet) {
    initialPitch = xyz[1];
    initialRoll = xyz[2];
    initialSet = true;
    Serial.println("Angulos iniciales establecidos");
  }
  // Verificar umbral de 4 grados
  float pitchDiff = abs(xyz[1] - initialPitch);
  float rollDiff = abs(xyz[2] - initialRoll);
  if (pitchDiff > 8.0 || rollDiff > 8.0) {
    if (thresholdTime == 0) {
      thresholdTime = millis(); // Iniciar temporizador
    } else if (millis() - thresholdTime > 3000 && !buzzerOn) {
      digitalWrite(BUZZER_PIN, HIGH); // Activar zumbador
      buzzerOn = true;
      Serial.println("Zumbador activado!");
    }
  } else {
    thresholdTime = 0; // Resetear temporizador
    if (buzzerOn) {
      digitalWrite(BUZZER_PIN, LOW); // Desactivar zumbador
      buzzerOn = false;
      Serial.println("Zumbador desactivado");
    }
  }
  // Formato para Python: q0,q1,q2,q3,pitch,roll,yaw
  Serial.print(q.w, 2);
  Serial.print(",");
  Serial.print(q.x, 2);
  Serial.print(",");
  Serial.print(q.y, 2);
  Serial.print(",");
  Serial.print(q.z, 2);
  Serial.print(",");
  Serial.print(xyz[1], 2); // pitch
  Serial.print(",");
  Serial.print(xyz[2], 2); // roll
  Serial.print(",");
  Serial.println(xyz[0], 2); // yaw
}


void setup() {
  Serial.begin(115200);
  // Configurar pines
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  // Asegurar que los pines comiencen en estado correcto
  digitalWrite(trigPin, LOW);
  digitalWrite(buzzerPin, LOW);
  // Esperar a que el sensor se estabilice
  delay(500);
  Serial.println("HC-SR04 inicializado. Enviando datos en formato: distancia,desplazamiento");
  Serial.println("Esperando medición inicial de referencia...");

  // initialize serial communication
  Serial.begin(115200);
  // Inicializar pin del zumbador
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  // Inicializar I2C con los pines específicos del ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz para comunicación más rápida
  Serial.println(F("Iniciando MPU6050/MPU9250 con ESP32..."));
  // Setup the MPU
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(20);  // Aumentado a 100Hz para mejor respuesta
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); //Sets the address of the MPU.
  Serial.println(F("Calibrando MPU..."));
  mpu.CalibrateMPU();                // Calibrates the MPU.
  Serial.println(F("Cargando imagen DMP..."));
  mpu.load_DMP_Image();              // Loads the DMP image into the MPU and finish configuration.
  mpu.on_FIFO(print_Values);         // Set callback function that is triggered when FIFO Data is retrieved
  Serial.println(F("¡Configuración completada!"));


}

void loop() {
  // Limitar la frecuencia de medición
  if (millis() - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = millis();
    // Obtener medición filtrada
    float currentDistance = getFilteredDistance();
    // Establecer distancia inicial (primera medición válida)
    if (!initialDistanceSet && currentDistance > 0) {
      initialDistance = currentDistance;
      initialDistanceSet = true;
      Serial.println("Distancia inicial establecida: " + String(initialDistance) + " cm");
    }
    // Si es la primera medición válida, establecerla como referencia
    if (!referenceSet && currentDistance > 0) {
      referenceDistance = currentDistance;
      referenceSet = true;
      Serial.println("Distancia de referencia establecida: " + String(referenceDistance) + " cm");
    }
    // Verificar cambios de distancia respecto a la inicial
    if (initialDistanceSet) {
      checkDistanceChange(currentDistance);
    }
    // Calcular desplazamiento relativo a la referencia
    float displacement = 0;
    if (referenceSet) {
      displacement = referenceDistance - currentDistance;
    }
    // Enviar datos en formato: distancia,desplazamiento
    Serial.print(currentDistance, 2);
    Serial.print(",");
    Serial.println(displacement, 2);
    // Controlar el zumbador
    manageBuzzer();
  }
  // Pequeña pausa para estabilidad del sistema
  delay(1);

  // Polling del FIFO buffer con un intervalo más corto para mejor respuesta
  static unsigned long FIFO_DelayTimer;
  if ((millis() - FIFO_DelayTimer) >= (50)) { // 9ms en lugar de 99ms para una respuesta más rápida
    if (mpu.dmp_read_fifo(false)) FIFO_DelayTimer = millis(); 
  }
  
  // Leer datos del magnetómetro si está disponible (MPU9250)
  if (mpu.akm_addr > 0) {
    if (mpu.readMagData(mag)) MagDataReady = 1;
  }



}

void checkDistanceChange(float currentDistance) {
  // Verificar si hay un cambio significativo respecto a la distancia inicial
  float distanceDifference = abs(currentDistance - initialDistance);
  if (distanceDifference > DISTANCE_THRESHOLD) {
    // Si no se había detectado un cambio, iniciar el temporizador
    if (!changeDetected) {
      changeDetected = true;
      changeStartTime = millis();
      Serial.println("Cambio de distancia detectado. Iniciando temporizador...");
    } else {
      // Verificar si el cambio se ha mantenido por más de 3 segundos
      if (millis() - changeStartTime > DURATION_THRESHOLD && !buzzerActive) {
        activateBuzzer();
        Serial.println("Cambio mantenido por más de 3 segundos. Activando zumbador.");
      }
    }
  } else {
    // Si la distancia vuelve al rango normal, resetear la detección
    if (changeDetected) {
      changeDetected = false;
      Serial.println("Distancia normalizada. Reseteando detección de cambios.");
    }
  }
}

void activateBuzzer() {
  buzzerActive = true;
  buzzerStartTime = millis();
  // Activar zumbador con tono específico
  tone(buzzerPin, 1000); // Frecuencia de 1000 Hz
  Serial.println("Zumbador activado");
}
void manageBuzzer() {
  // Controlar la duración del zumbador
  if (buzzerActive && (millis() - buzzerStartTime > buzzerDuration)) {
    noTone(buzzerPin);
    buzzerActive = false;
    Serial.println("Zumbador desactivado");
  }
}

float getFilteredDistance() {
  // Realizar múltiples mediciones para el filtro
  for (int i = 0; i < SAMPLES; i++) {
    float rawDistance = measureDistance();
    // Solo considerar mediciones válidas (dentro del rango del sensor)
    if (rawDistance >= 2 && rawDistance <= 400) {
      distances.add(rawDistance);
      lastValidDistance = rawDistance;
    } else if (lastValidDistance > 0) {
      // Si la medición es inválida pero tenemos una válida anterior, usar esa
      distances.add(lastValidDistance);
    }
    // Pequeña pausa entre mediciones
    delay(10);
  }
  // Obtener la mediana de las mediciones (filtra valores atípicos)
  filteredDistance = distances.getMedian();
  return filteredDistance;
}
float measureDistance() {
  // Generar pulso de 10us en el pin TRIG
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Medir duración del pulso en el pin ECHO
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout de 30ms
  // Verificar si se recibió una respuesta válida
  if (duration == 0) {
    return -1; // Indicar medición fallida
  }
  // Calcular distancia en cm
  float distance = duration * SOUND_SPEED / 2;
  // Aplicar compensación de temperatura (aproximada)
  // Asumiendo temperatura ambiente de 20°C
  float tempCompensation = 1.0; // Factor de compensación
  distance = distance * tempCompensation;
  return distance;
}
