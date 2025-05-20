import wiringpi
import time
import os

TRIG = 1
ECHO = 0

wiringpi.wiringPiSetup()
wiringpi.pinMode(TRIG, 1)  # Output
wiringpi.pinMode(ECHO, 0)  # Input

def send_trigger():
    wiringpi.digitalWrite(TRIG, 0)
    time.sleep(0.000002)
    wiringpi.digitalWrite(TRIG, 1)
    time.sleep(0.00001)
    wiringpi.digitalWrite(TRIG, 0)

def wait_for_echo():
    timeout = time.time() + 1  # 1 second timeout
    while time.time() < timeout:
        if wiringpi.digitalRead(ECHO) == 1:
            return True
    return False

def play_sound():
    os.system("mpg123 ./simple.mp3")

if __name__ == "__main__":
    print("Ready")
    while True:
        send_trigger()
        if wait_for_echo():
            print("Echo detected – playing sound")
            play_sound()
            time.sleep(1)  # Cooldown to prevent spamming
