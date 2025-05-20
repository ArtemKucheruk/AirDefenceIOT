import time
import VL53L1X

# Initialize the sensor
tof = VL53L1X.VL53L1X(i2c_bus=0, i2c_address=0x29)
tof.open()
tof.set_timing_budget(20)  # Minimum budget for faster readings
tof.set_distance_mode(2)   # 4 = Long range
tof.start_ranging()

print("Starting distance readings (press Ctrl+C to stop)...")

try:
    while True:
        distance_mm = tof.get_distance()
        distance_cm = round(distance_mm / 10.0, 1)
        print(f"Distance: {distance_cm} cm")
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Test stopped by user.")

finally:
    tof.stop_ranging()
    tof.close()
