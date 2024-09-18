import os
import socket
import sys
import threading
import logging
from zeroconf import ServiceBrowser, Zeroconf
from cmd2 import Cmd
import requests
import json
from apscheduler.schedulers.background import BackgroundScheduler
from datetime import datetime
import time

# Setup Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("hydroponics.log"),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Automatically locate certificate and key files in the same directory
script_dir = os.path.dirname(os.path.realpath(__file__))
certs_dir = os.path.join(script_dir, 'main/certs')
os.makedirs(certs_dir, exist_ok=True)

ca_bundle = os.path.join(certs_dir, 'ca_bundle.pem')
client_cert = os.path.join(certs_dir, 'client.crt')
client_key = os.path.join(certs_dir, 'client.key')
schedules_file = os.path.join(script_dir, 'schedules.json')

# Global dictionary to store discovered devices
devices = {}

# Initialize the scheduler
scheduler = BackgroundScheduler()
scheduler.start()

class HydroponicsServiceListener:
    def __init__(self, console):
        self.console = console

    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info:
            address = socket.inet_ntoa(info.addresses[0])
            device_name = info.server.replace('.local.', '')
            devices[device_name] = {
                'address': address,
                'port': info.port,
            }
            print(f"Discovered device: {device_name} at {address}:{info.port}")
            self.console.device_added(device_name)

    def remove_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info:
            device_name = info.server.replace('.local.', '')
            if device_name in devices:
                del devices[device_name]
                print(f"Device removed: {device_name}")
                self.console.device_removed(device_name)

    def update_service(self, zeroconf, type, name):
        # Placeholder for future support
        pass

class HydroponicsConsole(Cmd):
    intro = 'Welcome to the Hydroponics console. Type help or ? to list commands.\n'
    prompt = '(hydroponics) '
    
    def __init__(self):
        super().__init__()
        self.selected_device = None  # Initialize selected_device to None
        self.custom_commands = {
            'list': 'List all discovered devices.',
            'select': 'Select a device to interact with: select <device_number>',
            'status': 'Get sensor data from the selected device.',
            'airpump': 'Set air pump PWM value: airpump <value>',
            'waterpump': 'Set water pump PWM value: waterpump <value>',
            'solenoid': 'Set solenoid valve state: solenoid <on/off>',
            'led': 'Set LED array state: led <on/off>',
            'schedule_actuators': 'Set actuator schedule: schedule_actuators',
            'view_schedules': 'View all actuator schedules: view_schedules',
            'delete_schedule': 'Delete a schedule: delete_schedule <schedule_name>',
            'exit': 'Exit the console.',
        }
        self.session = requests.Session()
        self.session.verify = False  # Disable SSL verification for testing or replace with 'ca_bundle'

        # If mutual TLS is enabled, set client cert and key
        if os.path.exists(client_cert) and os.path.exists(client_key):
            self.session.cert = (client_cert, client_key)
        
        # Dictionary to store schedules
        self.schedules = {}
        
        # Load existing schedules from JSON
        self.load_schedules()

    def device_added(self, device_name):
        print(f"Handling schedules for newly added device: {device_name}")
        # Find all schedules associated with this device
        for name, details in self.schedules.items():
            if details['device_name'] == device_name:
                self.schedule_job(name, details)

    def device_removed(self, device_name):
        print(f"Handling schedules for removed device: {device_name}")
        # Optionally, remove or pause schedules associated with this device
        for name, details in self.schedules.items():
            if details['device_name'] == device_name:
                try:
                    scheduler.remove_job(name)
                    print(f"Removed job '{name}' as device '{device_name}' was removed.")
                except Exception as e:
                    print(f"Error removing job '{name}': {e}")

    def load_schedules(self):
        'Load schedules from the JSON file and add them to the scheduler'
        if not os.path.exists(schedules_file):
            return
        
        try:
            with open(schedules_file, 'r') as f:
                self.schedules = json.load(f)
        except Exception as e:
            print(f"Error loading schedules: {e}")
            self.schedules = {}
            return
        
        for name, details in self.schedules.items():
            device_name = details.get('device_name')
            if device_name in devices:
                self.schedule_job(name, details)
            else:
                print(f"Device '{device_name}' not found for schedule '{name}'. Will schedule when device is available.")

    def schedule_job(self, name, details):
        device_name = details.get('device_name')
        actions = details.get('actions', {})
        duration = details.get('duration_minutes', 0)
        frequency = details.get('frequency', 'daily')
        start_time_str = details.get('start_time', '00:00')
        day_of_week = details.get('day_of_week', None)
        
        try:
            start_time = datetime.strptime(start_time_str, "%H:%M").time()
        except ValueError:
            print(f"Invalid start time format for schedule '{name}'. Skipping.")
            return
        
        if frequency == 'weekly':
            days_map = {
                'monday': 'mon',
                'tuesday': 'tue',
                'wednesday': 'wed',
                'thursday': 'thu',
                'friday': 'fri',
                'saturday': 'sat',
                'sunday': 'sun'
            }
            day_short = days_map.get(day_of_week.lower(), None)
            if not day_short:
                print(f"Invalid day of week for schedule '{name}'. Skipping.")
                return
        else:
            day_short = None
        
        # Define the job function
        def execute_schedule_wrapper():
            self.execute_schedule(name, device_name, actions, duration)
        
        # Schedule the job based on frequency
        try:
            if frequency == 'daily':
                scheduler.add_job(
                    execute_schedule_wrapper,
                    'cron',
                    hour=start_time.hour,
                    minute=start_time.minute,
                    id=name,
                    replace_existing=True
                )
                print(f"Scheduled daily job '{name}' at {start_time_str} for device '{device_name}'.")
            elif frequency == 'weekly':
                scheduler.add_job(
                    execute_schedule_wrapper,
                    'cron',
                    day_of_week=day_short,
                    hour=start_time.hour,
                    minute=start_time.minute,
                    id=name,
                    replace_existing=True
                )
                print(f"Scheduled weekly job '{name}' on {day_short} at {start_time_str} for device '{device_name}'.")
        except Exception as e:
            print(f"Error scheduling job '{name}': {e}")

    def execute_schedule(self, name, device_name, actions, duration):
        if device_name not in devices:
            print(f"Device '{device_name}' not found for schedule '{name}'. Retrying in 10 seconds.")
            threading.Timer(10, self.execute_schedule, args=[name, device_name, actions, duration]).start()
            return

        print(f"\nExecuting schedule '{name}' for device '{device_name}' at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        device_info = devices[device_name]

        # Send commands to actuators
        for actuator, command in actions.items():
            try:
                url = f"https://{device_info['address']}:{device_info['port']}/api/actuators/{actuator}"
                response = self.session.post(url, json=command, timeout=5, verify=False)
                print(f"Actuator '{actuator}' responded with: {response.text}")
            except Exception as e:
                print(f"Error sending command to '{actuator}' for device '{device_name}': {e}")

        # Schedule turning off actuators after the specified duration
        def turn_off_actuators():
            print(f"Turning off actuators for schedule '{name}' on device '{device_name}' at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            for actuator, command in actions.items():
                if actuator in ['airpump', 'waterpump']:
                    # Turning off pumps by setting PWM to 0
                    self._post_actuator_command(actuator, {'value': 0}, device_name)
                elif actuator in ['solenoid', 'led']:
                    # Turning off binary actuators
                    self._post_actuator_command(actuator, {'state': 'off'}, device_name)
        
        # Schedule turning off actuators after 'duration' minutes
        if duration > 0:
            threading.Timer(duration * 60, turn_off_actuators).start()

    def do_list(self, arg):
        'List all discovered devices.'
        if devices:
            for idx, (name, info) in enumerate(devices.items(), start=1):
                selected = '*' if self.selected_device == name else ' '
                print(f"[{selected}] {idx}. {name} at {info['address']}:{info['port']}")
        else:
            print("No devices discovered.")

    def do_select(self, arg):
        'Select a device to interact with: select <device_number>'
        try:
            idx = int(arg.strip()) - 1
            if idx < 0 or idx >= len(devices):
                print("Invalid device number.")
                return
            device_name = list(devices.keys())[idx]
            self.selected_device = device_name
            print(f"Selected device: {device_name}")
        except ValueError:
            print("Please provide a valid device number.")

    def do_status(self, arg):
        'Get sensor data from the selected device.'
        if not self._check_device_selected():
            return
        device_info = devices[self.selected_device]
        url = f"https://{device_info['address']}:{device_info['port']}/api/sensors"
        try:
            response = self.session.get(url, timeout=5)
            response.raise_for_status()
            data = response.json()
            
            # Display Device Name and IP
            device_name = self.selected_device
            device_ip = f"{device_info['address']}:{device_info['port']}"
            print(f"\nDevice Name: {device_name}")
            print(f"Device IP: {device_ip}\n")
            logger.info(f"Fetching status for device '{device_name}' at {device_ip}.")
            
            # Print Sensor Data Table
            self._print_sensor_data_table(data)
            logger.info(f"Fetched sensor data from device '{device_name}'.")
        except requests.exceptions.RequestException as e:
            logger.error(f"Error fetching sensor data: {e}")
            print(f"Error fetching sensor data: {e}")


    def do_airpump(self, arg):
        'Set air pump PWM value: airpump <value>'
        if not self._check_device_selected():
            return
        try:
            value = int(arg.strip())
            if value < 0 or value > 100:
                print("Value must be between 0 and 100.")
                return
            self._post_actuator_command('airpump', {'value': value}, self.selected_device)
        except ValueError:
            print("Please provide a valid integer value.")

    def do_waterpump(self, arg):
        'Set water pump PWM value: waterpump <value>'
        if not self._check_device_selected():
            return
        try:
            value = int(arg.strip())
            if value < 0 or value > 100:
                print("Value must be between 0 and 100.")
                return
            self._post_actuator_command('waterpump', {'value': value}, self.selected_device)
        except ValueError:
            print("Please provide a valid integer value.")

    def do_solenoid(self, arg):
        'Set solenoid valve state: solenoid <on/off>'
        if not self._check_device_selected():
            return
        state = arg.strip().lower()
        if state not in ['on', 'off']:
            print("Please specify 'on' or 'off'.")
            return
        self._post_actuator_command('solenoid', {'state': state}, self.selected_device)

    def do_led(self, arg):
        'Set LED array state: led <on/off>'
        if not self._check_device_selected():
            return
        state = arg.strip().lower()
        if state not in ['on', 'off']:
            print("Please specify 'on' or 'off'.")
            return
        self._post_actuator_command('led', {'state': state}, self.selected_device)

    def do_schedule_actuators(self, arg):
        'Set actuator schedule: schedule_actuators'
        if not self._check_device_selected():
            return
        
        print("\n--- Actuator Schedule Setup ---")
        
        # Prompt for schedule name
        schedule_name = self._prompt_input("Enter schedule name: ")
        if not schedule_name:
            print("Schedule name cannot be empty.")
            return
        if schedule_name in self.schedules:
            print(f"Schedule '{schedule_name}' already exists.")
            return
        
        # Prompt for start time (24-hour format, e.g., 18:00)
        start_time_str = self._prompt_input("Enter start time (HH:MM, 24-hour format): ")
        try:
            start_time = datetime.strptime(start_time_str, "%H:%M").time()
        except ValueError:
            print("Invalid time format.")
            return
        
        # Prompt for duration in hours:minutes
        duration_str = self._prompt_input("Enter duration (HH:MM): ")
        try:
            parts = duration_str.split(":")
            if len(parts) != 2:
                raise ValueError("Incorrect format")
            hours = int(parts[0])
            minutes = int(parts[1])
            if hours < 0 or minutes < 0 or minutes >= 60:
                raise ValueError("Hours must be >= 0 and minutes must be between 0 and 59.")
            duration = hours * 60 + minutes
            if duration <= 0:
                raise ValueError("Duration must be greater than 0.")
        except ValueError as ve:
            print(f"Invalid duration format: {ve}")
            return
        
        # Prompt for frequency (daily, weekly)
        frequency = self._prompt_input("Enter frequency ('daily' or 'weekly'): ").lower()
        if frequency not in ['daily', 'weekly']:
            print("Frequency must be 'daily' or 'weekly'.")
            return
        
        day_of_week = None
        if frequency == 'weekly':
            # Prompt for the day of the week
            day_of_week = self._prompt_input("Enter day of the week (e.g., Monday): ").lower()
            days_map = {
                'monday': 'mon',
                'tuesday': 'tue',
                'wednesday': 'wed',
                'thursday': 'thu',
                'friday': 'fri',
                'saturday': 'sat',
                'sunday': 'sun'
            }
            if day_of_week not in days_map:
                print("Invalid day of the week.")
                return
            day_of_week_short = days_map[day_of_week]
        else:
            day_of_week_short = None
        
        # Prompt for actuator commands
        print("\n--- Define Actuator Commands ---")
        actions = {}
        
        # LED
        led_choice = self._prompt_input("Schedule LED? (y/n): ").lower()
        if led_choice == 'y':
            led_state = self._prompt_input("Enter LED state ('on' or 'off'): ").lower()
            if led_state not in ['on', 'off']:
                print("Invalid LED state. Skipping LED scheduling.")
            else:
                actions['led'] = {'state': led_state}
        
        # Air Pump
        airpump_choice = self._prompt_input("Schedule Air Pump? (y/n): ").lower()
        if airpump_choice == 'y':
            try:
                airpump_value = int(self._prompt_input("Enter Air Pump PWM value (0-100): "))
                if 0 <= airpump_value <= 100:
                    actions['airpump'] = {'value': airpump_value}
                else:
                    print("Invalid Air Pump value. Skipping Air Pump scheduling.")
            except ValueError:
                print("Invalid Air Pump value. Skipping Air Pump scheduling.")
        
        # Water Pump
        waterpump_choice = self._prompt_input("Schedule Water Pump? (y/n): ").lower()
        if waterpump_choice == 'y':
            try:
                waterpump_value = int(self._prompt_input("Enter Water Pump PWM value (0-100): "))
                if 0 <= waterpump_value <= 100:
                    actions['waterpump'] = {'value': waterpump_value}
                else:
                    print("Invalid Water Pump value. Skipping Water Pump scheduling.")
            except ValueError:
                print("Invalid Water Pump value. Skipping Water Pump scheduling.")
        
        # Solenoid Valve
        solenoid_choice = self._prompt_input("Schedule Solenoid Valve? (y/n): ").lower()
        if solenoid_choice == 'y':
            solenoid_state = self._prompt_input("Enter Solenoid Valve state ('on' or 'off'): ").lower()
            if solenoid_state not in ['on', 'off']:
                print("Invalid Solenoid Valve state. Skipping Solenoid Valve scheduling.")
            else:
                actions['solenoid'] = {'state': solenoid_state}
        
        if not actions:
            print("No actuators scheduled. Exiting schedule setup.")
            return
        
        # Get device info
        device_info = devices[self.selected_device]
        device_name = self.selected_device
        device_ip = device_info['address']
        
        # Store the schedule
        self.schedules[schedule_name] = {
            'device_name': device_name,
            'device_ip': device_ip,
            'start_time': start_time_str,
            'duration_minutes': duration,
            'frequency': frequency,
            'day_of_week': day_of_week.capitalize() if frequency == 'weekly' else None,
            'actions': actions
        }
        
        # Schedule the job if device is available
        self.schedule_job(schedule_name, self.schedules[schedule_name])
        
        # Save schedules to JSON
        self.save_schedules()
        
        print(f"\nActuator schedule '{schedule_name}' has been set successfully for device '{device_name}'.\n")

    def do_view_schedules(self, arg):
        'View all actuator schedules: view_schedules'
        if not self.schedules:
            print("No schedules set.")
            return
        
        # Define table headers
        headers = ["Name", "Device Name", "Device IP", "Start Time (H:M)", "Duration (H:M)", "Frequency", "Day of Week", "LED", "Air Pump", "Water Pump", "Solenoid Valve"]
        col_widths = [15, 15, 15, 15, 15, 10, 15, 10, 10, 12, 15]
        
        # Print header
        header_row = " | ".join([headers[i].ljust(col_widths[i]) for i in range(len(headers))])
        separator = "-+-".join(['-' * col_widths[i] for i in range(len(headers))])
        print("\n" + header_row)
        print(separator)
        
        # Print each schedule
        for name, details in self.schedules.items():
            device_name = details['device_name']
            device_ip = details['device_ip']
            start_time = details['start_time']
            duration_minutes = details['duration_minutes']
            frequency = details['frequency']
            day = details['day_of_week'] if details['frequency'] == 'weekly' else '-'
            
            # Convert duration back to HH:MM format for display
            hours = duration_minutes // 60
            minutes = duration_minutes % 60
            duration_display = f"{hours}:{minutes:02d}"
            
            # Get actuator commands
            led = details['actions'].get('led', {}).get('state', '-')
            airpump = str(details['actions'].get('airpump', {}).get('value', '-'))
            waterpump = str(details['actions'].get('waterpump', {}).get('value', '-'))
            solenoid = details['actions'].get('solenoid', {}).get('state', '-')
            
            row = [
                name.ljust(col_widths[0]),
                device_name.ljust(col_widths[1]),
                device_ip.ljust(col_widths[2]),
                start_time.ljust(col_widths[3]),
                duration_display.ljust(col_widths[4]),
                frequency.ljust(col_widths[5]),
                (day.capitalize() if day != '-' else '-').ljust(col_widths[6]),
                led.ljust(col_widths[7]),
                airpump.ljust(col_widths[8]),
                waterpump.ljust(col_widths[9]),
                solenoid.ljust(col_widths[10])
            ]
            print(" | ".join(row))
        print()  # Newline at the end

    def do_delete_schedule(self, arg):
        'Delete a schedule: delete_schedule <schedule_name>'
        schedule_name = arg.strip()
        if not schedule_name:
            print("Please provide the schedule name to delete.")
            return
        if schedule_name not in self.schedules:
            print(f"Schedule '{schedule_name}' does not exist.")
            return
        
        # Remove from scheduler
        try:
            scheduler.remove_job(schedule_name)
            print(f"Removed job '{schedule_name}' from scheduler.")
        except Exception as e:
            print(f"Error removing job from scheduler: {e}")
        
        # Remove from schedules dictionary
        del self.schedules[schedule_name]
        
        # Save updated schedules to JSON
        self.save_schedules()
        
        print(f"Schedule '{schedule_name}' has been deleted successfully.")

    def do_exit(self, arg):
        'Exit the console.'
        print("Exiting Hydroponics console.")
        return True

    def do_EOF(self, arg):
        'Handle EOF (Ctrl+D) to exit.'
        print("Exiting Hydroponics console.")
        return True

    def _post_actuator_command(self, actuator, payload, device_name):
        if device_name not in devices:
            print(f"Device '{device_name}' is not available.")
            return
        device_info = devices[device_name]
        url = f"https://{device_info['address']}:{device_info['port']}/api/actuators/{actuator}"
        try:
            response = self.session.post(url, json=payload, timeout=5, verify=False)
            print(response.text)
        except Exception as e:
            print(f"Error sending command to '{actuator}' on device '{device_name}': {e}")

    def _check_device_selected(self):
        if not self.selected_device:
            print("No device selected. Use 'list' to see devices and 'select <number>' to select one.")
            return False
        if self.selected_device not in devices:
            print("Selected device is no longer available.")
            self.selected_device = None
            return False
        return True

    def _prompt_input(self, prompt_text):
        'Helper function to prompt user input'
        try:
            return input(prompt_text).strip()
        except EOFError:
            return ''

    def _print_sensor_data_table(self, data):
        'Helper function to print sensor data in table format'
        headers = ["Actuator", "Current (mA)", "Voltage (mV)", "Power (mW)", "Flow Rate (L/min)"]
        rows = []
        
        for actuator in data.get("sensors_data", []):
            actuator_name = actuator.get("actuator", "Unknown")
            current = actuator.get("current_mA", 0.0)
            voltage = actuator.get("voltage_mV", 0.0)
            power = actuator.get("power_mW", 0.0)
            flow_rate = actuator.get("flow_rate_L_min", "-")
            row = [
                actuator_name,
                f"{current:.2f}",
                f"{voltage:.2f}",
                f"{power:.2f}",
                f"{flow_rate}" if flow_rate != "-" else "-"
            ]
            rows.append(row)
        
        # Determine column widths
        col_widths = [max(len(str(item)) for item in [header] + [row[idx] for row in rows]) for idx, header in enumerate(headers)]
        
        # Print header
        header_row = " | ".join([headers[i].ljust(col_widths[i]) for i in range(len(headers))])
        separator = "-+-".join(['-' * col_widths[i] for i in range(len(headers))])
        print("\n" + header_row)
        print(separator)
        
        # Print rows
        for row in rows:
            print(" | ".join([str(item).ljust(col_widths[idx]) for idx, item in enumerate(row)]))
        print()  # Newline at the end

    def save_schedules(self):
        'Save schedules to the JSON file'
        try:
            with open(schedules_file, 'w') as f:
                json.dump(self.schedules, f, indent=4)
        except Exception as e:
            print(f"Error saving schedules: {e}")

    def do_help(self, arg):
        'Display help for commands, grouped by category.'
        if arg:
            # If the user types `help <command>`, provide detailed help for that command
            try:
                func = getattr(self, f'do_{arg}')
                doc = func.__doc__ or "No documentation available."
                print(doc)
            except AttributeError:
                print(f"No help available for '{arg}'")
        else:
            self._print_help()

    def _print_help(self):
        'Override the help command to group commands.'
        print("\nCustom Commands:")
        for command, description in self.custom_commands.items():
            print(f"  {command:<20} {description}")

        print("\nDefault Commands (cmd2):")
        # Fetch all commands and filter out the custom ones
        all_commands = self.get_all_commands()
        default_commands = set(all_commands) - set(self.custom_commands.keys())
        for command in sorted(default_commands):
            print(f"  {command:<20} {self.get_command_description(command)}")

    def get_command_description(self, command_name):
        'Helper function to return command descriptions for default commands'
        func = getattr(self, f'do_{command_name}', None)
        if func and func.__doc__:
            return (func.__doc__.splitlines()[0])
        return "No documentation available."

def start_service_discovery(console):
    zeroconf = Zeroconf()
    listener = HydroponicsServiceListener(console)
    browser = ServiceBrowser(zeroconf, "_hydroponics._tcp.local.", listener)
    return zeroconf

def main():
    # Create the console instance first
    console = HydroponicsConsole()

    # Start mDNS service discovery in a separate thread, passing the console reference
    zeroconf = start_service_discovery(console)
    
    try:
        console.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting Hydroponics console.")
    finally:
        zeroconf.close()
        scheduler.shutdown()

if __name__ == '__main__':
    main()
