#Aqui poneis el Docstring que querais
"""
   Este codigo esta sacado del siguiente enlace https://www.printables.com/model/818975-compact-robot-arm-arduino-3d-printed/files.
Está diseñado para controlar un brazo robótico con servomotores y un controlador PWM PCA9685, que se conecta a una placa Jetson. El brazo incluye varios servomotores para mover sus articulaciones (hombro, codo, muñeca, base y garra) y potenciómetros que permiten el control en tiempo real de estos servos. Además, un botón controla la apertura y cierre de la garra.
Requisitos:
- Jetson.GPIO: Control de los pines GPIO en la Jetson.
- adafruit_pca9685: Comunicación con el controlador PWM PCA9685.
- adafruit_servokit: Gestión de servomotores con la librería Adafruit.
- time: Introducción de retrasos y configuración del sistema.
Funcionamiento:
- El script configura los servos mediante el controlador PWM PCA9685, que gestiona las señales PWM necesarias para controlar la posición de cada motor.
- Los potenciómetros se leen a través de los pines GPIO y se mapean a un rango de valores para ajustar el ángulo de los servos de las articulaciones (hombro, codo, muñeca, base).
- El valor del potenciómetro se convierte en un ancho de pulso PWM que controla el movimiento de los servos.
- Un botón conectado al pin GPIO 15 controla la garra del brazo. Cuando el botón no está presionado, la garra se cierra; cuando está presionado, la garra se abre.
Funciones principales:
- moveMotor(controlIn, motorOut): Lee el valor de un potenciómetro conectado a un pin GPIO y ajusta el movimiento del servo correspondiente según la posición del potenciómetro.
- El script entra en un bucle infinito donde se ajustan constantemente las posiciones de los servos según los valores de los potenciómetros y se controla la garra según el estado del botón.
Parámetros de configuración:
- MIN_PULSE_WIDTH: Ancho de pulso mínimo para los servos (650 microsegundos).
- MAX_PULSE_WIDTH: Ancho de pulso máximo para los servos (2350 microsegundos).
- FREQUENCY: Frecuencia de actualización de la señal PWM (50 Hz).
"""

#import wire
#import Adafruit_PWMServoDriver
import board 
import busio
import Jetson.GPIO as GPIO
import adatafruit_pca9685
import time
i2c = busio.I2C(board.SCL, board.SDA)


#Declaro variables globales
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
FREQUENCY = 50


#Instancio el Driver del controlador de servos
#Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
pwm = adafruit_pca9685.PCA9685(i2c)
kit = Servokit(channels=16)
potWrist = GPIO.input(11)
potElbow = GPIO.input(13)                      #//Assign Potentiometers to pins on Arduino Uno
potShoulder = GPIO.input(15)
potBase = GPIO.input(17)
#Configuro el SetUP
time.sleep(5)    #<-- So I have time to get controller to starting position
pca.frecuency = FREQUENCY
GPIO.setmode(GPIO.BOARD) 
hand = adafruit_motor.servo.Servo(0)    
wrist = adafruit_motor.servo.Servo(1)
elbow = adafruit_motor.servo.Servo(2)
shoulder = adafruit_motor.servo.Servo(3) 
base = adafruit_motor.servo.Servo(4)
potWrist = adafruit_motor.servo.Servo(5)
potElbow = adafruit_motor.servo.Servo(6)
potShoulder = adafruit_motor.servo.Servo(7)
potBase = adafruit_motor.servo.Servo(8)

pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)        #Set Gripper to 90 degrees (Close Gripper) X en Jetson
pwm.begin()
GPIO.setup(7, GPIO.IN)  #channel tiene que ser un pin valido en jetson 


def moveMotor(controlIn, motorOut):
  """
    Descripción de la función MoveMotor(controlIN, motorOUT):

    Parámetros:
      controlIN (int): Pin GPIO designado para leer el valor del potenciómetro mediante el pin especificado.
      motorOUT (int): Pin GPIO utilizado para emitir la señal PWM que controla el motor.
    
    Valor de retorno:
      La función retorna la posicion del robot en funcion del valor del potenciometro.

  """
  pulse_wide, pulse_width, potVal = -7
  
#  potVal = analogRead(controlIn);                                                  //Read value of Potentiometer
  potval = GPIO.input(controlIN)
  pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096)               #//Map Potentiometer position to Motor
  
#  pwm.setPWM(motorOut, 0, pulse_width);
  pwm = GPIO.PW(motorOut, 0, pulse_width)
 

  while (True):
   moveMotor(potWrist, wrist)                              
   moveMotor(potElbow, elbow)                   #//Assign Motors to corresponding Potentiometers                                                    
   moveMotor(potShoulder, shoulder)
   moveMotor(potBase, base)
   pushButton = GPIO.input(7)
   if(pushButton == GPIO.LOW):
  
     pwm.setPWM(hand, 0, 180)                            #//Keep Gripper closed when button is not pressed
     print("Grab")
  
  else:
  
    pwm.setPWM(hand, 0, 90)                             #//Open Gripper when button is pressed
    print("Release")

  GPIO.cleanup()




