"""
Generic Diesel Heater Controller - monitor.py

Description:    This file was used for understanding the communication protocol
                between the heater controller and the heater itself. It is not
                used in the final project. However, it is left here for reference
                and may be useful for others if a different heater controller is
                used.
                This Code does no sending. It only receives data from the heater
                and the controller and prints it to the console.

Copyright (c) 2023 GeneralUltra758

This file is part of Your Project Name.

Your Project Name is licensed under the GNU General Public License (GPL) version 3.
You may obtain a copy of the license at: https://www.gnu.org/licenses/gpl-3.0.en.html
"""

import serial
import time

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 25000, timeout=1)


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





frame = b""
last_byte_time = time.time()

while True:
    # Read incoming byte
    data = ser.read(1)

    if data:
        frame += data
        #print("\rdebug:", time.time()-last_byte_time>0.1)
        # Check if the delay between bytes is greater than 100ms
        if time.time() - last_byte_time >= 0.1:
            # this needs some fixes , it somehow captures one byte too much (seems to be the first byte of the next frame?)
            if len(frame) >= 48:
                #print("\rFull Frame:", len(frame))
                # Log host frame
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

            # Reset frame and update last byte time for the next frame
            frame = data
            last_byte_time = time.time()
        else:
            # Update last byte time
            last_byte_time = time.time()