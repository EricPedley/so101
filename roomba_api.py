#!/usr/bin/env python3
"""
iRobot Create 2 Open Interface Python API

This module provides a Python interface for controlling the iRobot Create 2
robot through its Open Interface (OI) specification.

Based on iRobot Create 2 Open Interface (OI) Specification
Last Updated April 13, 2021
"""

import serial
import time
import struct
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum

# Hardcoded serial port - modify this for your system
SERIAL_PORT = "/dev/ttyUSB0"  # Linux/Mac example, use "COM3" for Windows

class Mode(Enum):
    """Robot operating modes"""
    OFF = 0
    PASSIVE = 1
    SAFE = 2
    FULL = 3

class ChargingState(Enum):
    """Battery charging states"""
    NOT_CHARGING = 0
    RECONDITIONING = 1
    FULL_CHARGING = 2
    TRICKLE_CHARGING = 3
    WAITING = 4
    FAULT = 5

class BaudRate(Enum):
    """Available baud rates"""
    BAUD_300 = 0
    BAUD_600 = 1
    BAUD_1200 = 2
    BAUD_2400 = 3
    BAUD_4800 = 4
    BAUD_9600 = 5
    BAUD_14400 = 6
    BAUD_19200 = 7
    BAUD_28800 = 8
    BAUD_38400 = 9
    BAUD_57600 = 10
    BAUD_115200 = 11

class Create2:
    """
    iRobot Create 2 Open Interface API
    
    This class provides methods to control and interact with the iRobot Create 2
    robot through its serial interface.
    """
    
    def __init__(self, port: str = SERIAL_PORT, baud: int = 115200, timeout: float = 1.0):
        """
        Initialize the Create 2 connection
        
        Args:
            port: Serial port path (default: SERIAL_PORT constant)
            baud: Baud rate (default: 115200)
            timeout: Serial timeout in seconds (default: 1.0)
        """
        self.port = port
        self.baud_rate = baud
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.current_mode = Mode.OFF
        
    def connect(self) -> bool:
        """
        Establish serial connection to the robot
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=8,
                parity=serial.PARITY_NONE,
                stopbits=1,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for connection to stabilize
            self.serial_conn.read_all() # Flush inbound connection
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Close the serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.stop()  # Send stop command before disconnecting
            self.serial_conn.close()
            self.serial_conn = None
    
    def _send_command(self, *args) -> bool:
        """
        Send command bytes to the robot
        
        Args:
            *args: Command bytes as integers
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Error: No active serial connection")
            return False
        
        try:
            command = bytes(args)
            self.serial_conn.write(command)
            self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"Command send error: {e}")
            return False
    
    def _read_response(self, num_bytes: int) -> Optional[bytes]:
        """
        Read response bytes from the robot
        
        Args:
            num_bytes: Number of bytes to read
            
        Returns:
            Bytes read or None if error
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            return self.serial_conn.read(num_bytes)
        except Exception as e:
            print(f"Read error: {e}")
            return None
    
    # Getting Started Commands
    
    def start(self) -> bool:
        """
        Start the Open Interface
        
        Returns:
            True if command sent successfully
        """
        success = self._send_command(128)
        if success:
            self.current_mode = Mode.PASSIVE
            time.sleep(0.1)  # Brief pause after start
        return success
    
    def reset(self) -> bool:
        """
        Reset the robot (equivalent to battery removal/insertion)
        
        Returns:
            True if command sent successfully
        """
        success = self._send_command(7)
        if success:
            self.current_mode = Mode.OFF
            time.sleep(2)  # Wait for reset to complete
        return success
    
    def stop(self) -> bool:
        """
        Stop the Open Interface
        
        Returns:
            True if command sent successfully
        """
        success = self._send_command(173)
        if success:
            self.current_mode = Mode.OFF
        return success
    
    def set_baud_rate(self, baud_code: BaudRate) -> bool:
        """
        Set the baud rate
        
        Args:
            baud_code: BaudRate enum value
            
        Returns:
            True if command sent successfully
        """
        success = self._send_command(129, baud_code.value)
        if success:
            time.sleep(0.1)  # Wait 100ms as specified
        return success
    
    # Mode Commands
    
    def safe_mode(self) -> bool:
        """
        Enter Safe mode
        
        Returns:
            True if command sent successfully
        """
        success = self._send_command(131)
        if success:
            self.current_mode = Mode.SAFE
        return success
    
    def full_mode(self) -> bool:
        """
        Enter Full mode (disables safety features)
        
        Returns:
            True if command sent successfully
        """
        success = self._send_command(132)
        if success:
            self.current_mode = Mode.FULL
        return success
    
    # Cleaning Commands
    
    def clean(self) -> bool:
        """Start default cleaning mode"""
        success = self._send_command(135)
        if success:
            self.current_mode = Mode.PASSIVE
        return success
    
    def max_clean(self) -> bool:
        """Start max cleaning mode"""
        success = self._send_command(136)
        if success:
            self.current_mode = Mode.PASSIVE
        return success
    
    def spot_clean(self) -> bool:
        """Start spot cleaning mode"""
        success = self._send_command(134)
        if success:
            self.current_mode = Mode.PASSIVE
        return success
    
    def seek_dock(self) -> bool:
        """Direct robot to seek dock"""
        success = self._send_command(143)
        if success:
            self.current_mode = Mode.PASSIVE
        return success
    
    def power_down(self) -> bool:
        """Power down the robot"""
        success = self._send_command(133)
        if success:
            self.current_mode = Mode.PASSIVE
        return success
    
    def schedule(self, days: int, times: List[Tuple[int, int]]) -> bool:
        """
        Set cleaning schedule
        
        Args:
            days: Bit field for days of week (bit 0=Sunday, bit 6=Saturday)
            times: List of (hour, minute) tuples for each day
            
        Returns:
            True if command sent successfully
        """
        if len(times) != 7:
            print("Error: Must provide 7 time tuples (one for each day)")
            return False
        
        command = [167, days]
        for hour, minute in times:
            command.extend([hour, minute])
        
        return self._send_command(*command)
    
    def set_day_time(self, day: int, hour: int, minute: int) -> bool:
        """
        Set the robot's clock
        
        Args:
            day: Day of week (0=Sunday, 6=Saturday)
            hour: Hour (0-23)
            minute: Minute (0-59)
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(168, day, hour, minute)
    
    # Movement Commands
    
    def drive(self, velocity: int, radius: int) -> bool:
        """
        Control robot movement
        
        Args:
            velocity: Velocity in mm/s (-500 to 500)
            radius: Turning radius in mm (-2000 to 2000)
                   Special values: 32768=straight, -1=clockwise turn, 1=counter-clockwise turn
                   
        Returns:
            True if command sent successfully
        """
        # Convert to signed 16-bit values
        vel_bytes = struct.pack('>h', velocity)
        rad_bytes = struct.pack('>h', radius)
        
        return self._send_command(137, vel_bytes[0], vel_bytes[1], rad_bytes[0], rad_bytes[1])
    
    def drive_direct(self, right_velocity: int, left_velocity: int) -> bool:
        """
        Control wheels independently
        
        Args:
            right_velocity: Right wheel velocity in mm/s (-500 to 500)
            left_velocity: Left wheel velocity in mm/s (-500 to 500)
            
        Returns:
            True if command sent successfully
        """
        right_bytes = struct.pack('>h', right_velocity)
        left_bytes = struct.pack('>h', left_velocity)
        
        return self._send_command(145, right_bytes[0], right_bytes[1], left_bytes[0], left_bytes[1])
    
    def drive_pwm(self, right_pwm: int, left_pwm: int) -> bool:
        """
        Control wheels with PWM values
        
        Args:
            right_pwm: Right wheel PWM (-255 to 255)
            left_pwm: Left wheel PWM (-255 to 255)
            
        Returns:
            True if command sent successfully
        """
        right_bytes = struct.pack('>h', right_pwm)
        left_bytes = struct.pack('>h', left_pwm)
        
        return self._send_command(146, right_bytes[0], right_bytes[1], left_bytes[0], left_bytes[1])
    
    def stop_motors(self) -> bool:
        """Stop all movement"""
        return self.drive(0, 0)
    
    def turn_clockwise(self, velocity: int = 200) -> bool:
        """Turn in place clockwise"""
        return self.drive(velocity, -1)
    
    def turn_counter_clockwise(self, velocity: int = 200) -> bool:
        """Turn in place counter-clockwise"""
        return self.drive(velocity, 1)
    
    def drive_straight(self, velocity: int) -> bool:
        """Drive straight forward/backward"""
        return self.drive(velocity, 32768)  # Special straight value
    
    # Actuator Commands
    
    def set_motors(self, main_brush: bool = False, side_brush: bool = False, 
                   vacuum: bool = False, main_brush_reverse: bool = False, 
                   side_brush_reverse: bool = False) -> bool:
        """
        Control cleaning motors
        
        Args:
            main_brush: Enable main brush
            side_brush: Enable side brush
            vacuum: Enable vacuum
            main_brush_reverse: Reverse main brush direction
            side_brush_reverse: Reverse side brush direction
            
        Returns:
            True if command sent successfully
        """
        motor_bits = 0
        if vacuum:
            motor_bits |= 0x02
        if side_brush:
            motor_bits |= 0x01
        if main_brush:
            motor_bits |= 0x04
        if main_brush_reverse:
            motor_bits |= 0x10
        if side_brush_reverse:
            motor_bits |= 0x08
            
        return self._send_command(138, motor_bits)
    
    def set_motors_pwm(self, main_brush_pwm: int, side_brush_pwm: int, vacuum_pwm: int) -> bool:
        """
        Control motor speeds with PWM
        
        Args:
            main_brush_pwm: Main brush PWM (-127 to 127)
            side_brush_pwm: Side brush PWM (-127 to 127)
            vacuum_pwm: Vacuum PWM (0 to 127)
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(144, main_brush_pwm & 0xFF, side_brush_pwm & 0xFF, vacuum_pwm & 0xFF)
    
    def set_leds(self, home: bool = False, spot: bool = False, check_robot: bool = False,
                 debris: bool = False, power_color: int = 0, power_intensity: int = 0) -> bool:
        """
        Control LEDs
        
        Args:
            home: Home LED on/off
            spot: Spot LED on/off
            check_robot: Check Robot LED on/off
            debris: Debris LED on/off
            power_color: Power LED color (0=green, 255=red)
            power_intensity: Power LED intensity (0=off, 255=full)
            
        Returns:
            True if command sent successfully
        """
        led_bits = 0
        if debris:
            led_bits |= 0x01
        if spot:
            led_bits |= 0x02
        if home:
            led_bits |= 0x04
        if check_robot:
            led_bits |= 0x08
            
        return self._send_command(139, led_bits, power_color, power_intensity)
    
    def press_buttons(self, clean: bool = False, spot: bool = False, dock: bool = False,
                      minute: bool = False, hour: bool = False, day: bool = False,
                      schedule: bool = False, clock: bool = False) -> bool:
        """
        Simulate button presses (buttons auto-release after 1/6 second)
        
        Args:
            clean: Clean button
            spot: Spot button
            dock: Dock button
            minute: Minute button
            hour: Hour button
            day: Day button
            schedule: Schedule button
            clock: Clock button
            
        Returns:
            True if command sent successfully
        """
        button_bits = 0
        if clean:
            button_bits |= 0x01
        if spot:
            button_bits |= 0x02
        if dock:
            button_bits |= 0x04
        if minute:
            button_bits |= 0x08
        if hour:
            button_bits |= 0x10
        if day:
            button_bits |= 0x20
        if schedule:
            button_bits |= 0x40
        if clock:
            button_bits |= 0x80
            
        return self._send_command(165, button_bits)
    
    # Song Commands
    
    def define_song(self, song_number: int, notes: List[Tuple[int, int]]) -> bool:
        """
        Define a song
        
        Args:
            song_number: Song number (0-4)
            notes: List of (note, duration) tuples. Note is MIDI number (31-127),
                   duration is in 1/64th seconds
                   
        Returns:
            True if command sent successfully
        """
        if not (0 <= song_number <= 4):
            print("Error: Song number must be 0-4")
            return False
        
        if not (1 <= len(notes) <= 16):
            print("Error: Song must have 1-16 notes")
            return False
        
        command = [140, song_number, len(notes)]
        for note, duration in notes:
            command.extend([note, duration])
        
        return self._send_command(*command)
    
    def play_song(self, song_number: int) -> bool:
        """
        Play a previously defined song
        
        Args:
            song_number: Song number (0-4)
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(141, song_number)
    
    # Sensor Commands
    
    def get_sensors(self, packet_id: int) -> Optional[bytes]:
        """
        Request sensor data
        
        Args:
            packet_id: Sensor packet ID (0-58 or 100-107)
            
        Returns:
            Raw sensor data bytes or None if error
        """
        if self._send_command(142, packet_id):
            # Determine expected response length based on packet ID
            response_length = self._get_packet_length(packet_id)
            if response_length > 0:
                return self._read_response(response_length)
        return None
    
    def _get_packet_length(self, packet_id: int) -> int:
        """Get expected length for sensor packet"""
        # This is a simplified mapping - full implementation would include all packet lengths
        packet_lengths = {
            7: 1,   # Bumps and wheel drops
            8: 1,   # Wall
            9: 1,   # Cliff left
            10: 1,  # Cliff front left
            11: 1,  # Cliff front right
            12: 1,  # Cliff right
            13: 1,  # Virtual wall
            19: 2,  # Distance
            20: 2,  # Angle
            21: 1,  # Charging state
            22: 2,  # Voltage
            23: 2,  # Current
            24: 1,  # Temperature
            25: 2,  # Battery charge
            26: 2,  # Battery capacity
            # Add more as needed
        }
        return packet_lengths.get(packet_id, 1)
    
    def get_bumper_state(self) -> Optional[Dict[str, bool]]:
        """
        Get bumper and wheel drop states
        
        Returns:
            Dictionary with bumper and wheel drop states or None if error
        """
        data = self.get_sensors(7)
        if data and len(data) >= 1:
            value = data[0]
            return {
                'bump_right': bool(value & 0x01),
                'bump_left': bool(value & 0x02),
                'wheel_drop_right': bool(value & 0x04),
                'wheel_drop_left': bool(value & 0x08),
                'wheel_drop_caster': bool(value & 0x10)
            }
        return None
    
    def get_battery_info(self) -> Optional[Dict[str, Any]]:
        """
        Get battery information
        
        Returns:
            Dictionary with battery data or None if error
        """
        voltage_data = self.get_sensors(22)
        current_data = self.get_sensors(23)
        charge_data = self.get_sensors(25)
        capacity_data = self.get_sensors(26)
        temp_data = self.get_sensors(24)
        charging_state_data = self.get_sensors(21)
        
        if all(data is not None for data in [voltage_data, current_data, charge_data, capacity_data, temp_data, charging_state_data]):
            voltage = struct.unpack('>H', voltage_data)[0]
            current = struct.unpack('>h', current_data)[0]
            charge = struct.unpack('>H', charge_data)[0]
            capacity = struct.unpack('>H', capacity_data)[0]
            temperature = struct.unpack('b', temp_data)[0]
            charging_state = charging_state_data[0]
            
            return {
                'voltage_mv': voltage,
                'current_ma': current,
                'charge_mah': charge,
                'capacity_mah': capacity,
                'temperature_c': temperature,
                'charging_state': ChargingState(charging_state),
                'charge_percentage': (charge / capacity * 100) if capacity > 0 else 0
            }
        return None
    
    def get_distance_angle(self) -> Optional[Tuple[int, int]]:
        """
        Get distance traveled and angle turned since last call
        
        Returns:
            Tuple of (distance_mm, angle_degrees) or None if error
        """
        distance_data = self.get_sensors(19)
        angle_data = self.get_sensors(20)
        
        if distance_data and angle_data:
            distance = struct.unpack('>h', distance_data)[0]
            angle = struct.unpack('>h', angle_data)[0]
            return distance, angle
        return None

# Convenience functions for common operations

def create_simple_song(frequency_duration_pairs: List[Tuple[float, float]]) -> List[Tuple[int, int]]:
    """
    Convert frequency/duration pairs to MIDI note/duration format
    
    Args:
        frequency_duration_pairs: List of (frequency_hz, duration_seconds) tuples
        
    Returns:
        List of (midi_note, duration_64ths) tuples
    """
    import math
    
    notes = []
    for freq, duration in frequency_duration_pairs:
        # Convert frequency to MIDI note number
        if freq <= 0:
            midi_note = 31  # Rest note (below audible range)
        else:
            midi_note = int(round(12 * math.log2(freq / 440.0) + 69))
            midi_note = max(31, min(127, midi_note))  # Clamp to valid range
        
        # Convert duration to 64ths of a second
        duration_64ths = int(round(duration * 64))
        duration_64ths = max(1, min(255, duration_64ths))  # Clamp to valid range
        
        notes.append((midi_note, duration_64ths))
    
    return notes

# Example usage
if __name__ == "__main__":
    # Create robot instance
    robot = Create2()
    
    try:
        # Connect to robot
        if robot.connect():
            print("Connected to Create 2")
            
            # Start Open Interface
            robot.start()
            robot.safe_mode()
            
            # Turn on power LED (green, half intensity)
            robot.set_leds(power_color=0, power_intensity=128)
            
            # Get battery info
            battery = robot.get_battery_info()
            if battery:
                print(f"Battery: {battery['charge_percentage']:.1f}% ({battery['voltage_mv']}mV)")
            
            # Drive forward for 1 second
            #robot.drive_straight(200)  # 200 mm/s forward
            #time.sleep(1)
            #robot.stop_motors()
            
            # Define and play a simple song
            notes = create_simple_song([(440, 0.5), (523, 0.5), (659, 0.5)])  # A, C, E
            robot.define_song(0, notes)
            robot.play_song(0)

            print("Success")
            
        else:
            print("Failed to connect to Create 2")
            
    finally:
        # Always disconnect when done
        robot.disconnect()
        print("Disconnected")
