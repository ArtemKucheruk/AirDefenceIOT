import wiringpi, time
wiringpi.wiringPiSetup()
LASER_PIN = 3  # GPIO3 (WiringPi numbering)

wiringpi.pinMode(LASER_PIN, wiringpi.OUTPUT)
wiringpi.digitalWrite(LASER_PIN, wiringpi.LOW)  # Start with ON

try:
    while True:
        wiringpi.digitalWrite(LASER_PIN, wiringpi.HIGH)  # Laser OFF
        print("⚫ LASER OFF")
        time.sleep(1)
        wiringpi.digitalWrite(LASER_PIN, wiringpi.LOW)   # Laser ON
        print("🔴 LASER ON")
        time.sleep(1)
except KeyboardInterrupt:
    wiringpi.digitalWrite(LASER_PIN, wiringpi.HIGH)  # Force ON on exit
    print("Script stopped. Laser is ON.")


#   [FRONT VIEW (flat side with text)]
#           ________________
#          |       |       |
#        GATE    DRAIN   SOURCE
#          |_______|_______|

# Setup:

# GPIO3 (3.3V) → [220Ω Resistor] → G (Gate)
# Laser (+ (red wire)) → 5V Power
# Laser (− (BLACK wire)) → D (Drain)
# S (Source) → GND
