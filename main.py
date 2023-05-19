"""
Generic Diesel Heater Controller - main.py

Description:    Main file for the Generic Diesel Heater Controller project. This file contains the main loop and
                the serial communication functions. The serial communication functions are responsible for sending and
                receiving data to/from the heater controller. The serial communication functions are called from
                within the main loop. Frames in the Queue are sent every 100ms. After each frame is sent, the response
                is read and the frame is logged. The main loop also checks for user input and updates the frame values
                accordingly.

Copyright (c) 2023 GeneralUltra758

This file is part of Your Project Name.

Your Project Name is licensed under the GNU General Public License (GPL) version 3.
You may obtain a copy of the license at: https://www.gnu.org/licenses/gpl-3.0.en.html
"""

import serial
import time
import sys
import threading
import crcmod
from queue import Queue

exit_flag = False # to exit from within a thread, do not change this!!!

frame_queue = Queue()


# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 25000, timeout=1)

frame = b""
last_byte_time = time.time()

config_values = {
    'heater_control': 0x0,
    'actual_temp': 0,  # Example: 0°C
    'desired_temp': 12,  # Example: 12°C ## that also sets the pump frequency when in fixed power mode, formula: pmin + (((tdes-tmin)/(tmax-tmin))*(pmax-pmin))
    'min_pump_freq': 1.6,  # Example: 1.6Hz
    'max_pump_freq': 5.5,  # Example: 5.5Hz
    'min_fan_speed': 1680,  # Example: 1680 RPM
    'max_fan_speed': 4500,  # Example: 4500 RPM
    'heater_voltage': 120,  # Example: 12.0V
    'fan_sensor_count': 1, # Hardware Specific, DO NOT CHANGE
    'power_mode': 0xCD,  # Example: 0xCD
    'min_temp_setting': 8,  # Example: 8°C
    'max_temp_setting': 35,  # Example: 35°C
    'glow_plug_power': 5,  # Example: 5 (1-6)
    'manual_pump_mode': 0x00, #0x00 (Normal), 0x5A (Prime)
    'altitude': 528
}

def increment_desired_temp():
    global config_values
    max_temp = config_values['max_temp_setting']
    min_temp = config_values['min_temp_setting']
    desired_temp = config_values['desired_temp']

    if desired_temp < max_temp:
        desired_temp += 1

    config_values['desired_temp'] = min(max(min_temp, desired_temp), max_temp)

def decrement_desired_temp():
    global config_values
    max_temp = config_values['max_temp_setting']
    min_temp = config_values['min_temp_setting']
    desired_temp = config_values['desired_temp']

    if desired_temp > min_temp:
        desired_temp -= 1

    config_values['desired_temp'] = min(max(min_temp, desired_temp), max_temp)


#host frame generator
def generate_host_frame(heater_control, actual_temp, desired_temp, min_pump_freq, max_pump_freq, min_fan_speed,
                        max_fan_speed, heater_voltage, fan_sensor_count, power_mode, min_temp_setting,
                        max_temp_setting, glow_plug_power, manual_pump_mode, altitude):
    # Convert values to their respective byte representations
    heater_control_byte = bytes([heater_control])
    actual_temp_byte = bytes([actual_temp])
    desired_temp_byte = bytes([desired_temp])
    min_pump_freq_byte = bytes([int(min_pump_freq * 10)])
    max_pump_freq_byte = bytes([int(max_pump_freq * 10)])
    min_fan_speed_bytes = min_fan_speed.to_bytes(2, 'big')
    max_fan_speed_bytes = max_fan_speed.to_bytes(2, 'big')
    heater_voltage_byte = bytes([heater_voltage])
    fan_sensor_count_byte = bytes([fan_sensor_count])
    power_mode_byte = bytes([power_mode])
    min_temp_setting_byte = bytes([min_temp_setting])
    max_temp_setting_byte = bytes([max_temp_setting])
    glow_plug_power_byte = bytes([glow_plug_power])
    manual_pump_mode_byte = bytes([manual_pump_mode])
    altitude_bytes = bytes([altitude >> 8, altitude & 0xFF])  # Splitting altitude into MSB and LSB

    # Concatenate the bytes to form the frame
    frame = (b'\x76\x16' + heater_control_byte + actual_temp_byte + desired_temp_byte + min_pump_freq_byte +
         max_pump_freq_byte + min_fan_speed_bytes + max_fan_speed_bytes + heater_voltage_byte +
         fan_sensor_count_byte + power_mode_byte + min_temp_setting_byte + max_temp_setting_byte +
         glow_plug_power_byte + manual_pump_mode_byte + b'\xeb\x47' + altitude_bytes)


    # Calculate and append the CRC-16 checksum
    crc_func = crcmod.predefined.mkPredefinedCrcFun('modbus')
    checksum = crc_func(frame)
    frame += checksum.to_bytes(2, 'big')

    return frame



def readSerial():
    time.sleep(0.05)
    frame = ser.read(48)
    ser.reset_input_buffer()
    host_frame = frame[:24]
    host_translation = translate_host_bytes(host_frame)
    print("\rHost:", host_translation, end="")
    print("-----------")
    # Log device frame
    device_frame = frame[24:]
    device_translation = translate_device_bytes(device_frame)
    print("\rDevice:", device_translation, end="")

    # Full frame
    print("\rFull Frame:", frame.hex())
    print("--------------------")



def translate_host_bytes(bytes):
    start_of_frame = bytes[0]
    data_size = bytes[1]
    heater_control = bytes[2]
    actual_temperature = bytes[3]
    desired_temperature = bytes[4]
    min_pump_frequency = bytes[5]
    max_pump_frequency = bytes[6]
    min_fan_speed = int.from_bytes(bytes[7:9], 'big')
    max_fan_speed = int.from_bytes(bytes[9:11], 'big')
    heater_voltage = bytes[11]
    fan_speed_sensor_count = bytes[12]
    power_mode = bytes[13]
    min_temp_setting = bytes[14]
    max_temp_setting = bytes[15]
    glow_plug_power = bytes[16]
    manual_pump_mode = bytes[17]
    altitude = int.from_bytes(bytes[20:22], 'big')

    translation = f"Start of Frame: {hex(start_of_frame)}\n"
    translation += f"Data Size: {hex(data_size)}\n"
    translation += f"Heater Control: {hex(heater_control)}\n"
    translation += f"Actual Temperature: {actual_temperature}°C\n"
    translation += f"Desired Temperature: {desired_temperature}°C\n"
    translation += f"Min Pump Frequency: {min_pump_frequency / 10}Hz\n"
    translation += f"Max Pump Frequency: {max_pump_frequency / 10}Hz\n"
    translation += f"Min Fan Speed: {min_fan_speed} RPM\n"
    translation += f"Max Fan Speed: {max_fan_speed} RPM\n"
    translation += f"Heater Voltage: {heater_voltage * 0.1}V\n"
    translation += f"Fan Speed Sensor Count: {hex(fan_speed_sensor_count)}\n"
    translation += f"Power Mode: {hex(power_mode)}\n"
    translation += f"Min Temp Setting: {hex(min_temp_setting)}\n"
    translation += f"Max Temp Setting: {hex(max_temp_setting)}\n"
    translation += f"Glow Plug Power: {hex(glow_plug_power)}\n"
    translation += f"Manual Pump Mode: {hex(manual_pump_mode)}\n"
    translation += f"Altitude: {altitude}m\n"

    return translation

def translate_device_bytes(bytes):
    start_of_frame = bytes[0]
    data_size = bytes[1]
    run_state = bytes[2]
    error_state = bytes[3]
    measured_voltage = int.from_bytes(bytes[4:6], 'big') / 10
    fan_rpm = int.from_bytes(bytes[6:8], 'big')
    fan_voltage = int.from_bytes(bytes[8:10], 'big') / 10
    heat_exchanger_temp = int.from_bytes(bytes[10:12], 'big')
    glow_plug_voltage = int.from_bytes(bytes[12:14], 'big') / 10
    glow_plug_current = int.from_bytes(bytes[14:16], 'big') * 10
    pump_frequency_actual = bytes[16] / 10
    stored_error_code = bytes[17] - 1
    fixed_mode_pump_frequency = bytes[19] / 10
    run_mode_transition_temp = bytes[20]

    # Readable strings for run state
    run_state_strings = [
        "Off / Standby",
        "Start Acknowledge",
        "Glow plug pre-heat",
        "Failed ignition - pausing for retry",
        "Ignited – heating to full temp phase",
        "Running",
        "Skipped – stop acknowledge",
        "Stopping - Post run glow re-heat",
        "Cooldown"
    ]
    run_state_string = run_state_strings[run_state]

    # Readable strings for error state
    error_state_strings = [
        "No Error",
        "No Error, but started",
        "Voltage too low",
        "Voltage too high",
        "Ignition plug failure",
        "Pump Failure – over current",
        "Too hot",
        "Motor Failure",
        "Serial connection lost",
        "Fire is extinguished",
        "Temperature sensor failure"
    ]
    if error_state == 0:
        error_state_string = "Idle"
    elif error_state == 1:
        error_state_string = "Running normally"
    else:
        error_state_string = error_state_strings[error_state - 2]

    translation = f"Start of Frame: {hex(start_of_frame)}\n"
    translation += f"Data Size: {hex(data_size)}\n"
    translation += f"Run State: {run_state_string} ({hex(run_state)})\n"
    translation += f"Error State: {error_state_string} ({hex(error_state)})\n"
    translation += f"Measured Voltage: {measured_voltage}V\n"
    translation += f"Fan RPM: {fan_rpm}\n"
    translation += f"Fan Voltage: {fan_voltage}V\n"
    translation += f"Heat Exchanger Temperature: {heat_exchanger_temp}°C\n"
    translation += f"Glow Plug Voltage: {glow_plug_voltage}V\n"
    translation += f"Glow Plug Current: {glow_plug_current}mA\n"
    translation += f"Pump Frequency (Actual): {pump_frequency_actual}Hz\n"
    translation += f"Stored Error Code: {stored_error_code}\n"
    translation += f"Fixed Mode Pump Frequency: {fixed_mode_pump_frequency}Hz\n"
    translation += f"Run Mode Transition Temperature: {hex(run_mode_transition_temp)}\n"

    return translation





def get_host_frame1():
    print("Injecting host frame 1: turning on heater")
    # Function to inject a modified host frame
    modified_frame = generate_host_frame(**config_values)

    # modify byte 3 to 0xA0
    modified_frame = modified_frame[:2] + bytes([0xA0]) + modified_frame[3:]

    # Calculate the checksum for the modified frame
    crc_func = crcmod.predefined.mkPredefinedCrcFun('modbus')
    checksum = crc_func(modified_frame[:-2])
    modified_frame = modified_frame[:-2] + checksum.to_bytes(2, 'big')


    # Send the modified frame
    print("modified frame: ", modified_frame.hex())
    return(modified_frame)

def get_host_frame2():

    print("Injecting host frame 2: turning off heater") 
    # Function to inject a modified host frame
    modified_frame = generate_host_frame(**config_values)

    # modify byte 3 to 0x05
    modified_frame = modified_frame[:2] + bytes([0x05]) + modified_frame[3:]

    # Calculate the checksum for the modified frame
    crc_func = crcmod.predefined.mkPredefinedCrcFun('modbus')
    checksum = crc_func(modified_frame[:-2])
    modified_frame = modified_frame[:-2] + checksum.to_bytes(2, 'big')

    # Send the modified frame
    return(modified_frame)


def get_host_frame3():
    
    print("Injecting host frame 3: no command, just read") 
    # Function to inject a modified host frame
    modified_frame = generate_host_frame(**config_values)

    # modify byte 3 to 0x05
    modified_frame = modified_frame[:2] + bytes([0x00]) + modified_frame[3:]

    # Calculate the checksum for the modified frame
    crc_func = crcmod.predefined.mkPredefinedCrcFun('modbus')
    checksum = crc_func(modified_frame[:-2])
    modified_frame = modified_frame[:-2] + checksum.to_bytes(2, 'big')

    # Send the modified frame
    ser.reset_input_buffer()
    return(modified_frame)


# Function to listen for keypress events
def check_keypress():
    while True:
        key = sys.stdin.read(1)
        if key == "1":
            frame_queue.put(get_host_frame1())
        elif key == "2":
            frame_queue.put(get_host_frame2())
        elif key == "3":
            frame_queue.put(get_host_frame3())
        elif key == "q":
            exit(0)
        elif key == "+":
            increment_desired_temp()
            frame_queue.put(get_host_frame3())
        elif key == "-":
            decrement_desired_temp()
            frame_queue.put(get_host_frame3())


# Function to send default frame every 3 seconds
def send_default_frame():
    while True:
        frame_queue.put(generate_host_frame(**config_values))
        time.sleep(3)

# Start the keypress listener in a separate thread
keypress_thread = threading.Thread(target=check_keypress)
keypress_thread.daemon = True
keypress_thread.start()

# Start the default frame sender in a separate thread
default_frame_thread = threading.Thread(target=send_default_frame)
default_frame_thread.daemon = True
default_frame_thread.start()

# Main loop to check frame queue and send frames
while not exit_flag:
    if not frame_queue.empty():
        frame = frame_queue.get()
        ser.write(frame)
        readSerial()

    time.sleep(0.1)  # Delay for 100ms before checking the frame queue again