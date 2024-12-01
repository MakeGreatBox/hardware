# Importing Libraries
from gpiozero import PWMOutputDevice, LED, Button, Buzzer, TonalBuzzer, DigitalInputDevice
from time import sleep
from gpiozero.tones import Tone
from threading import Thread
from datetime import datetime
from gpiozero import DistanceSensor
import drivers
import paho.mqtt.client as mqtt
import random
import time

# Define pins for devices
motor = PWMOutputDevice(18)        # Motor PWM pin
green_led = LED(12)                 # Green LED pin
red_led = LED(16)                   # Red LED pin
blue_led = LED(21)                  # Blue LED pin
yellow_led = LED(20)                # Yellow LED pin
buzzer = TonalBuzzer(23)
safty_button = Button(22)
start_button = Button(25)
safty_detector = DigitalInputDevice(13)  
display = drivers.Lcd()
processedBoxes = 0

# MQTT Configuration
mqtt_broker = "localhost"
mqtt_port = 1883
mqtt_topics = ["machine/start", "machine/stop", "machine/emergencyStop", "machine/machineConsume",
               "machine/boxcount", "machine/settings", "machine/velocity"]

# States
warmingUp = False
warmedUp = False
machineStarted = False
machineStopped = False
machineEmergencyStopped = False

# Variables
machineVelocity = 0.2
lcdVelocity = 1

# MQTT Client
client = mqtt.Client()

# Settings
machineSettings = {
    "dimensions": {"length": 0, "width": 0, "height": 0},
    "velocity": 0,
}

# Global signal tracking
start_signal_received = False
stop_signal_received = False
reset_signal_received = False
# def checkDistance():
#     if distance.distance <= 1:
#         processEmergencyStop()
#         print("Distance is less than 10cm. Emergency stop.")
#         return True

# Consum dummy data generator
# Function to simulate the consumption of a servo motor
def simulate_servo_motor_consumption():
    # Assume idle consumption of 0.01A (10mA), and up to 0.5A (500mA) under load
    idle_consumption = 0.01  # 10mA in Amps
    max_consumption = 0.5    # 500mA in Amps
    current_consumption = random.uniform(idle_consumption, max_consumption)
    return current_consumption
# Function to simulate the Raspberry Pi consumption
def simulate_raspberry_pi_consumption():
    # Raspberry Pi consumption range between 2W to 4W
    min_power = 2.0  # 2W
    max_power = 4.0  # 4W
    current_power = random.uniform(min_power, max_power)
    return current_power
# Function to simulate LED consumption
def simulate_led_consumption(led_count):
    # Each LED consumes around 20mA
    consumption_per_led = 0.02  # 20mA in Amps
    total_consumption = led_count * consumption_per_led
    return total_consumption
# Function to simulate Buzzer consumption
def simulate_buzzer_consumption():
    # Buzzer typically uses about 50mA
    buzzer_consumption = 0.05  # 50mA in Amps
    return buzzer_consumption
# Function to process the velocity to the screen
def procesVelocityToScrren():
    global lcdVelocity
    cases = {
        0.2: 1,
        0.25: 2,
        0.27: 3
    }
    lcdVelocity = cases.get(machineVelocity, 0)  # Default to 0 if no match found
# Function to process the emergency stop
def checkSaftySensor():
    global machineStarted
    if safty_detector.value == 0:
        motor.off()
        processEmergencyStop()
# Buzzer Start Sequence
def startBuzzer():
    
    buzzer.play(Tone("A4"))
    sleep(1)
    buzzer.stop()
    sleep(2)
    buzzer.play(Tone("A4"))
    sleep(2)
    buzzer.stop()
    sleep(1)
    buzzer.play(Tone("A4"))
    sleep(2)
    buzzer.stop()
    sleep(1)
    buzzer.play(Tone("A4"))
    sleep(2)
    buzzer.stop()
# MQTT message handler
def on_message(client, userdata, message):
    global processedBoxes
    topic = message.topic
    payload = message.payload.decode()

    if topic == "machine/start" and payload.lower() == "true":
        processStart()
    elif topic == "machine/boxcount":
        processedBoxes = int(payload)
    elif topic == "machine/warmingUp" and payload.lower() == "true":
        warmupMachine()        
    elif topic == "machine/stop" and payload.lower() == "true":
        processStop()
        print("Stop signal received!")
    # elif topic == "machine/settings":
    #     processSetings()
    elif topic == "machine/velocity":
        value = float(payload)
        procesVelocity(value)
  

def procesVelocity(value):
    global machineVelocity
    machineVelocity = value
                       # Clear the display of any data

def processStart():
    global machineStarted
    global machineStopped
    global machineEmergencyStopped
    if warmedUp and not machineStarted and not machineEmergencyStopped:
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("Starting Machine:", 1)
        machineStopped = False
        red_led.off()
        blue_led.off()
        green_led.on()
        # startBuzzer()
        motor.value = machineVelocity
        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print("Machine started at: ", start_time)
        machineStarted = True
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("MACHINE STARTED:", 1)
        client.publish("machine/start", start_time)
    elif not machineEmergencyStopped:
        warmupMachine()

def procesManualStart():
    global machineStarted, machineStopped
    if not machineStarted and not machineEmergencyStopped:
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("Starting Machine:", 1)
        machineStopped = False
        machineStarted = True
        blue_led.off()
        green_led.on()
        startBuzzer()
        motor.value = machineVelocity
        start_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print("Machine started at: ", start_time)
        client.publish("machine/start", start_time)
    elif not machineEmergencyStopped:
        warmupMachine()

def cyclingLedsWarmingLedWithBuzzer():
    while warmingUp:
        green_led.on()
        sleep(1)
        red_led.on()
        sleep(1)
        yellow_led.on()
        sleep(1)
        blue_led.on()
        # buzzer.play(Tone("A4"))
        sleep(2)
        green_led.off()
        red_led.off()
        yellow_led.off()
        blue_led.off()
        # buzzer.stop()
        sleep(1)

def processEmergencyStop():
    global machineEmergencyStopped, machineStarted
    if machineEmergencyStopped:
        machineEmergencyStopped = False
        blue_led.on()
        red_led.off()
        yellow_led.off()
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("STOP OUT:", 1)
        client.publish("machine/emergencyStop", False)
    else:
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("EMERGENY STOP", 1)
        machineEmergencyStopped = True
        machineStarted = False
        red_led.on()
        yellow_led.on()
        # buzzer.play(Tone("A4"))
        motor.off()
        sleep(2)
        buzzer.stop()
        print("Machine emergency stopped.")
        emergenyTime = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        client.publish("machine/emergencyStop", True)

def warmUpMotorLoop():
    while warmingUp:
        motor.value = 0.25
        sleep(5)
        motor.value = 0.2
        sleep(5)
        motor.value = 0.01
        sleep(5)
        motor.value = 0.018
        sleep(5)
        motor.value = 0
        sleep(5)


def warmupMachine():
    global warmingUp
    global warmedUp
    global machineStarted
    if not machineStarted:
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("WARMING UP...", 1)
        print("Warming up the machine...")
        warmingUp = True
        led_thread = Thread(target=cyclingLedsWarmingLedWithBuzzer)
        motor_thread = Thread(target=warmUpMotorLoop)
        led_thread.start()
        motor_thread.start()
        sleep(10)
        warmingUp = False
        led_thread.join()
        motor_thread.join()
        client.publish("machine/warmedUp", True)
        blue_led.on()
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("WARM COMPLETE:", 1)
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("WARMED UP", 1)
        display.lcd_display_string("READY TO START", 2)
        print("Warm-up complete")
        print("Machine ready to start.")
        warmedUp = True

    else:
        print("Machine is already started. No need to warm up.")

def processStop():
    global machineStarted, machineStopped
    if machineStarted:
        display.lcd_clear()                                # Clear the display of any data
        display.lcd_display_string("STOP MACHINE:", 1)
        machineStarted = False
        machineStopped = True
        green_led.off()
        red_led.on()
        motor.off()
        stop_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print("Machine stopped at: ", stop_time)
        client.publish("machine/stop", stop_time)
    else:
        print("Machine is already stopped.")

def waitingForSignals():
    client.connect(mqtt_broker, mqtt_port, 60)
    for topic in mqtt_topics:
        client.subscribe(topic)

    client.on_message = on_message
    client.loop_start()

def procesBlueButton():
    global machineStarted
    if machineStarted:
        processStop()
    else:
        processStart()
def sendData():
    # Simulate power consumption
    machine_consumption = simulate_servo_motor_consumption()
    raspberry_consumption = simulate_raspberry_pi_consumption()
    led_consumption = simulate_led_consumption(4)  # Assume 4 LEDs are on
    buzzer_consumption = simulate_buzzer_consumption()

    # Total consumption
    total_consumption = machine_consumption + raspberry_consumption + led_consumption + buzzer_consumption
    # Send the data to the MQTT broker
    client.publish("machine/machineConsume", total_consumption)


def main():
    yellow_led.on()
    # waitingForSignals()
    sleep(1)
    yellow_led.off()
    display.lcd_clear()                                # Clear the display of any data
    display.lcd_display_string("Make Great Boxes", 1)   # Refresh the first line of display with a different message
    display.lcd_display_string("Ready to start ", 2)   # Refresh the first line of display with a different message

    while True:
        global machineStarted
        global processedBoxes
        waitingForSignals()
        if safty_button.is_pressed:
            processEmergencyStop()
            print("Safty button pressed")
        if start_button.is_pressed:
            procesBlueButton()
            print("Start button pressed.")
        if (machineStarted):
            sendData()
            motor.value = machineVelocity
            procesVelocityToScrren()
            display.lcd_clear()                                # Clear the display of any data
            display.lcd_display_string(f"Boxes: {processedBoxes}", 1)
            display.lcd_display_string(f"Velocity: {lcdVelocity}", 2)
        checkSaftySensor()

        sleep(1)


if __name__ == "__main__":
    main()