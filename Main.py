from machine import Pin, UART
import time
import math
from machine import Pin, PWM

servo = PWM(Pin(16))
servo.freq(50)

#Sets the angle values on each end
MIN_ANGLE = 0
MAX_ANGLE = 180

#Custom pulse values
MIN_US = 750   #Pulse width for 0 degrees
MAX_US = 2500  #Pulse width for 180 degrees

# Waveshare L76K Hardware Configuration
uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1)) #Configures the pins, based on how the stack assigns them

# Global variables to track state over time
prev_lat = None #Sets the "previous latitude" to be used in the initial position and speed calculation
prev_lon = None #Sets the "previous longitude" to be used in the initial position and speed calculation
last_calc_time = time.ticks_ms() #Sets the last time calculated to the current date

# Global variables to track servo state over time
servo_at_90 = False
timer_start = None

# Speed thresholds
HIGH_SPEED = 15.0 #Speed required to activate servo (MPH)
LOW_SPEED = 11.5 #Speed required to start return timer (MPH)
LOW_SPEED_DURATION = 30000 #30 seconds in milliseconds

def convert_to_degrees(raw_val, direction):
    #Converts NMEA DDMM.MMMMM format to standard Decimal Degrees
    if not raw_val or raw_val == '':
        return None
    try:
        dot_idx = raw_val.find('.') #Idnetifies where in the raw data the "." is
        deg_len = dot_idx - 2 #Identifies the length of the position data as opposed to the time data
        degrees = float(raw_val[:deg_len]) #Sets a variable equal to just the position data
        minutes = float(raw_val[deg_len:]) #Sets a variable equal to just the time data
        decimal_degrees = degrees + (minutes / 60.0) 
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        return decimal_degrees
    except:
        return None

def set_angle_precise(angle):
    #Constrain input to safe boundaries
    if angle < MIN_ANGLE: angle = MIN_ANGLE
    if angle > MAX_ANGLE: angle = MAX_ANGLE
    
    #Maps the minimum and maximum angles
    pulse_us = MIN_US + ((angle - MIN_ANGLE) * (MAX_US - MIN_US) / (MAX_ANGLE - MIN_ANGLE))
    
    #Convert microseconds to 16-bit duty cycle
    duty = int((pulse_us / 20000) * 65535)
    servo.duty_u16(duty)

    
def haversine_distance(lat1, lon1, lat2, lon2):
    #Calculates the distance between two coordinates in meters
    R = 6371000  #Radius of the Earth in meters
    
    # Convert decimal degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c  # Distance in meters


def control_servo(speed_mph):
    #Controls servo position based on current speed
    global servo_at_90, timer_start

    current_time = time.ticks_ms()

    #If speed reaches 4.0 MPH, move servo to 90 degrees
    if speed_mph >= HIGH_SPEED:
        if not servo_at_90:
            set_angle_precise(90)
            servo_at_90 = True
            print("Servo moved to 90 degrees")

        #Reset low-speed timer while speed is high
        timer_start = None

    #If speed is 2.0 MPH or less, begin the 30-second countdown
    elif speed_mph <= LOW_SPEED and servo_at_90:
        if timer_start is None:
            #Speed just dropped <= 2.0, start the 30-second countdown
            timer_start = current_time
            print("Low speed detected. Starting 30-second timer.")

        elif time.ticks_diff(current_time, timer_start) >= LOW_SPEED_DURATION:
            #30 seconds have passed with speed <= 2.0
            set_angle_precise(0)
            servo_at_90 = False
            timer_start = None
            print("30 seconds elapsed. Servo returned to 0 degrees.")

    else:
        #If speed rises above 2.0, cancel countdown but maintain servo position
        timer_start = None


print("Speed Tracker Active. Reading coordinates...")

#Initialize servo at 0 degrees
set_angle_precise(0)

current_line = ""

while True:
    if uart.any():
        raw_char = uart.read(1)
        if raw_char:
            try:
                char = raw_char.decode('utf-8')
                if char == '\n':
                    line_str = current_line.strip()
                    
                    # Parse the RMC sentence
                    if "$GNRMC" in line_str or "$GPRMC" in line_str:
                        parts = line_str.split(',')
                        
                        # Only calculate if we hit our 3-second (3000ms) interval threshold
                        current_time = time.ticks_ms()
                        time_delta_ms = time.ticks_diff(current_time, last_calc_time)
                        
                        if time_delta_ms >= 1000: #Sets the timing between each "check" to 1000 MS, or 1 second
                            # Verify valid GNSS status ('A' = Active Fix)
                            if len(parts) > 6 and parts[2] == 'A':
                                latitude = convert_to_degrees(parts[3], parts[4])
                                longitude = convert_to_degrees(parts[5], parts[6])
                                
                                if latitude is not None and longitude is not None:
                                    # Calculate time passed in seconds
                                    time_seconds = time_delta_ms / 1000.0
                                    
                                    # Default speed is 0 if this is the first reading
                                    speed_kmh = 0.0
                                    speed_mph = 0.0
                                    
                                    if prev_lat is not None and prev_lon is not None:
                                        # Calculate distance traveled in meters
                                        distance_meters = haversine_distance(prev_lat, prev_lon, latitude, longitude)
                                        
                                        # Speed = Distance / Time (meters per second)
                                        speed_mps = distance_meters / time_seconds
                                        
                                        # Convert to standard units
                                        speed_kmh = speed_mps * 3.6
                                        speed_mph = speed_mps * 2.23694

                                    #Control servo based on calculated speed
                                    control_servo(speed_mph)

                                    # Output the metrics
                                    print(f"Latitude:  {latitude:.6f}")
                                    print(f"Longitude: {longitude:.6f}")
                                    print(f"Speed:     {speed_kmh:.1f} km/h  ({speed_mph:.1f} mph)")
                                    print("-" * 30)
                                    
                                    # Save current coordinates and timestamp for the next loop cycle
                                    prev_lat = latitude
                                    prev_lon = longitude
                                    last_calc_time = current_time
                            else:
                                print("Searching for satellites... (Fix lost)")
                                last_calc_time = current_time # Reset timer to prevent flooding
                                
                    current_line = ""
                else:
                    current_line += char
            except Exception as e:
                print("Error:", e)
                
    time.sleep(0.001)

