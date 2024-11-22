#!/usr/bin/python3
import cv2
from keras.models import load_model
import numpy as np
import time
import RPi.GPIO as GPIO

import mysql.connector

from signal import signal, SIGTERM, SIGHUP, pause
from time import sleep
from threading import Thread
from gpiozero import DistanceSensor
import requests
from datetime import datetime
import base64
#mqtt
import paho.mqtt.client as mqtt

# Configuración del broker MQTT
mqtt_broker = "localhost"  # Dirección IP del broker MQTT
mqtt_port = 1883  # Puerto del broker MQTT
# Conexión al broker MQTT
mqtt_client = mqtt.Client()
mqtt_client.connect(mqtt_broker, mqtt_port, 60)

#configurar servos
# Usar las variables de entorno PIGPIO_ADDR y PIGPIO_PORT


GPIO.setmode(GPIO.BCM)

GPIO.cleanup()

# Definir el pin GPIO al que está conectado el control del motor
motor_pin1 = 23
motor_pin2 = 24
motor_pin3 = 25

# Configurar el pin GPIO como salida
GPIO.setup(motor_pin1, GPIO.OUT)
#GPIO.output(motor_pin1, GPIO.LOW) 

GPIO.setup(motor_pin2, GPIO.OUT)
#GPIO.output(motor_pin2, GPIO.LOW)  

GPIO.setup(motor_pin3, GPIO.OUT)
#GPIO.output(motor_pin3, GPIO.LOW) 



# Crear un objeto PWM para controlar el motor
p1 = GPIO.PWM(motor_pin1, 50)  # Frecuencia de PWM: 50 Hz
p2 = GPIO.PWM(motor_pin2, 50)  # Frecuencia de PWM: 50 Hz
p3 = GPIO.PWM(motor_pin3, 50)  # Frecuencia de PWM: 50 Hz

SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500

#motor de paso
# Define los pines del controlador ULN2003 que están conectados al Raspberry Pi
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# Define la secuencia de pasos para el motor
seq = [
    [0, 0, 0, 1],
    [0, 0, 1, 1],
    [0, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 0, 0],
    [1, 1, 0, 0],
    [1, 0, 0, 0],
    [1, 0, 0, 1]
]

# Inicializa los pines GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def insertar_inferencia(clase, confidencia):
    # Configura la conexión a la base de datos MySQL
    db_config = {
        "host": "localhost",
        "user": "root",
        "password": "rootleon",
        "database": "db_peruberry"
    }

    try:
        # Conecta a la base de datos
        conn = mysql.connector.connect(**db_config)

        # Crea un cursor para ejecutar comandos SQL
        cursor = conn.cursor()

        # Inserta datos en la tabla
        insert_data_query = """
        INSERT INTO inferencia (fecha,clase,confidencia) VALUES (now(), %s, %s)
        """

        data = (clase,float(confidencia))
        cursor.execute(insert_data_query, data)
 
        # Guarda los cambios en la base de datos.
        
        conn.commit()

    except mysql.connector.Error as err:
        print("Error: {}".format(err))

    finally:
        if 'conn' in locals() and conn.is_connected():
            cursor.close()
            conn.close()

def rotate_90_degrees(direction):
    for _ in range(128):  # 512 pasos son necesarios para girar 360 grados
        for i in range(8):
            for pin in range(4):
                GPIO.output([IN1, IN2, IN3, IN4][pin], seq[i][pin])
            time.sleep(0.001)  # Puedes ajustar el tiempo de espera si es necesario
    if direction == 'CW':
        time.sleep(1)  # Espera un segundo para evitar que el motor se mueva involuntariamente
    elif direction == 'CCW':
        time.sleep(1)

def map(value, inMin, inMax, outMin, outMax):
    return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin


def setAngle(angle,p):      
    angle = max(0, min(180, angle))
    pulse_width = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE)
    pwm = map(pulse_width, 0, 20000, 0, 100)
    p.ChangeDutyCycle(pwm)

def on_connect(client, userdata, flags, rc):
    print("Conectado al broker MQTT")
    client.subscribe("compuerta1")  # Suscribirse al topic 
    client.subscribe("compuerta2")  # Suscribirse al topic 
    client.subscribe("compuerta3")  # Suscribirse al topic 

def on_message(client, userdata, msg):
    global mensajes1
    global mensajes2
    global mensajes3
    global mensajes4
   
    print(f"Mensaje recibido: {msg.topic} - {msg.payload}")

def envioAPI(equipo, fecha, confidencia, estado, imagen):
    # Crear el JSON con los datos a enviar
    
    confidencia = float(confidencia)
    
    data = {
        "equipo": equipo,
        "fecha": fecha,
        "confidencia": confidencia,
        "estado": estado,
        "imagen": imagen
    }

    # URL del servicio al que se enviarán los datos
    url = 'https://www.kingbot.pe/app_aguaymanto/APIaguaymanto.php'

    # Encabezados de la solicitud
    headers = {
        "Content-Type": "application/json",
        "User-Agent": "MyApp/1.0"  # Puedes modificar el User-Agent si es necesario
    }

    # Hacer la solicitud POST
    try:
        response = requests.post(url, json=data, headers=headers)
        response.raise_for_status()  # Levanta un error si la respuesta contiene un status code 4xx o 5xx
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"An error occurred: {e}")

def frame_to_base64(frame):
    
    #frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    # Codificar la imagen en formato JPEG
    ret, buffer = cv2.imencode('.jpg', frame)

    # Verificar si la codificación fue exitosa
    if not ret:
        raise Exception("Error al codificar la imagen.")

    # Convertir la imagen a Base64
    img_base64 = base64.b64encode(buffer).decode('utf-8')

    return img_base64
    
#datos inferencia
# Disable scientific notation for clarity
np.set_printoptions(suppress=True)
# Cargar el modelo
model = load_model('/home/calero/pi/PROY_AGUAYMANTO/modelo_clasificacion_v2/keras_model.h5',compile=False)

# Cargar las etiquetas
class_names = open('/home/calero/pi/PROY_AGUAYMANTO/modelo_clasificacion_v2/labels.txt', "r").readlines()

#contador
i=0

def safe_exit(signum, frame):
    exit(1)



def read_clasificador():
    global i
    while True:
        #giro 90 grados
            rotate_90_degrees('CCW')
            #Configurar la cámara
            camera = cv2.VideoCapture(0)
            #realizar inferencia
            if(camera.isOpened()):
                i=i+1
                ret, frame = camera.read()
                
                cv2.imshow("Webcam Image", frame)
                cv2.waitKey(0)
               
                # Guardar la imagen

                # Leer la imagen almacenada
                # imagen_guardada = cv2.imread('inferencia2.jpg')
                
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 

                image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
              
      
                
                cv2.imwrite("inferencia.jpg", image)
                
                # Make the image a numpy array and reshape it to the models input shape.
                image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
                
                
                # Normalize the image array
                image = (image / 127.5) - 1

                # Realizar la inferencia
                predictions = model.predict(image)

                print("Predicciones por clase:")
                for i, probabilidad in enumerate(predictions[0]):
                 print(f"Clase {i}: {probabilidad}")
    
                # Obtener la clase con mayor probabilidad
                class_index = np.argmax(predictions)

                # Obtener el nombre de la clase
                class_name = class_names[class_index]

                  # Obtener la probabilidad de la clase predicha
                confidence = predictions[0][class_index]
                            
                print("*******************************************")            
                print("Class:", class_name[2:], end="")
                print("Confidence Score:", str(np.round(confidence * 100))[:-2], "%")
                print(confidence)
                print("*******************************************") 
                
                #insertar_inferencia(class_name,confidence)
                
                imagen64= frame_to_base64(frame)
                ahora=datetime.now()
                fecharegistro=ahora.strftime('%Y-%m-%d %H:%M:%S')
                envioAPI("EQUIPO 01",fecharegistro,confidence,class_name[2:],imagen64)

                if(confidence>0.95):
                                        
                    if class_name[0]=="0":
                        print("--girar a bandeja APTOS--")
                        mqtt_client.publish("apto", "1")
                         # Girar el motor 90 grados
                        
                        setAngle(50,p1) 
                        time.sleep(1)
                        # Detener el motor
                                             
            
                    elif class_name[0]=="1":
                        print("--girar a bandeja DEFECTOS--")
                        mqtt_client.publish("defecto", "1")
                        setAngle(0,p1) 
                        time.sleep(1)
                       
                                    
                camera.release()
                print("fin de ciclo")
                
        
    sleep(0.1)

try:
    signal(SIGTERM, safe_exit)
    signal(SIGHUP, safe_exit)

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect(mqtt_broker, mqtt_port)
    mqtt_client.loop_start()
    
    p1.start(0) 
    p2.start(0) 
    p3.start(0) 
    
    reader = Thread(target=read_clasificador, daemon=True)
    reader.start()

    pause()

except KeyboardInterrupt:
    pass

finally:
    reading = False
    reader.join()
    sensor.close()
