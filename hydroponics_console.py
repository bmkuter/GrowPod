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
import tkinter as tk

# JUST FOR DEBUG, REMOVE ME DURING PRODUCTION
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

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

def prompt_schedule_24(initial_schedule=None):
    """
    Opens a Tkinter window with 24 horizontal sliders (one for each hour).
    Each slider lets the user set a PWM percentage (0..100).
    Returns a list of 24 integer values if confirmed, or None if cancelled.
    """
    result = None

    def on_confirm():
        nonlocal result
        result = [scale.get() for scale in scales]
        root.destroy()

    def on_cancel():
        nonlocal result
        result = None
        root.destroy()

    root = tk.Tk()
    root.title("24-Hour Schedule Editor")

    scales = []
    # For each hour, create a row with a label and a slider
    for hour in range(24):
        frame = tk.Frame(root)
        frame.pack(fill="x", padx=10, pady=2)

        label = tk.Label(frame, text=f"{hour:02d}:00", width=5)
        label.pack(side="left")

        scale = tk.Scale(frame, from_=0, to=100, orient="horizontal", length=300)
        if initial_schedule and len(initial_schedule) == 24:
            scale.set(initial_schedule[hour])
        else:
            scale.set(0)
        scale.pack(side="left", fill="x", expand=True)
        scales.append(scale)

    # Confirm / Cancel
    button_frame = tk.Frame(root)
    button_frame.pack(pady=10)
    confirm_btn = tk.Button(button_frame, text="Confirm", command=on_confirm, width=10)
    confirm_btn.pack(side="left", padx=5)
    cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, width=10)
    cancel_btn.pack(side="left", padx=5)

    root.mainloop()
    return result

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

        # Updated custom commands:
        self.custom_commands = {
            'list': 'List all discovered devices.',
            'select': 'Select a device to interact with: select <device_number>',
            'routine': 'Send a routine command. Example: routine empty_pod',
            'routine_status': 'Check routine status by ID.',
            'status': 'Get sensor data from the selected device.',
            'airpump': 'Set air pump PWM value: airpump <value>',
            # Water Pump → Source Pump
            'sourcepump': 'Set source pump PWM value: sourcepump <value>',
            'planterpump': 'Set planter pump PWM value: planterpump <value>',
            # Servo → Drain Pump
            'drainpump': 'Set drain pump PWM value: drainpump <value>',
            'led': 'Set LED array state: led <on/off>',
            'schedule_actuators': 'Set actuator schedule: schedule_actuators',
            'view_schedules': 'View all actuator schedules: view_schedules',
            'delete_schedule': 'Delete a schedule: delete_schedule <schedule_name>',
            'start_feeding_cycle': 'Start the feeding cycle.',
            'stop_feeding_cycle': 'Stop the feeding cycle.',
            'start_emptying_water': 'Start emptying water.',
            'stop_emptying_water': 'Stop emptying water.',
            'exit': 'Exit the console.',
            'hostname_suffix': 'Set mDNS suffix: hostname_suffix <suffix>',
            'rescan': 'Re-initialize Zeroconf to discover devices again.',
            'all_schedules' : 'Set schedules for all actuators at once: routine all_schedules',
            'showschedules'  : 'Print current schedules on device console',
        }

        # Initialize flow status tracking
        self.flow_status = {
            'drain': False,
            'source': False,
            'overflow': False
        }
        self.polling_interval = 1  # Poll every 1 second

        # Track previous system status
        self.previous_system_status = None

        # # Start the background polling thread
        # self.polling_thread = threading.Thread(target=self.poll_sensors)
        # self.polling_thread.daemon = True
        # self.polling_thread.start()

        # HTTP session config
        self.session = requests.Session()
        self.session.verify = False  # In production, use a proper CA bundle

        # If mutual TLS is enabled, set client cert and key
        if os.path.exists(client_cert) and os.path.exists(client_key):
            self.session.cert = (client_cert, client_key)
        
        # Dictionary to store schedules
        self.schedules = {}
        
        # Load existing schedules from JSON
        self.load_schedules()

    def device_added(self, device_name):
        print(f"Handling schedules for newly added device: {device_name}")
        # Re-schedule any relevant tasks
        for name, details in self.schedules.items():
            if details['device_name'] == device_name:
                self.schedule_job(name, details)

    def device_removed(self, device_name):
        print(f"Handling schedules for removed device: {device_name}")
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

        # Function to turn off the actuators
        def turn_off_actuators():
            print(f"Turning off actuators for schedule '{name}' on device '{device_name}' at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            for actuator, command in actions.items():
                # Rename waterpump -> sourcepump, servo -> drainpump
                # So if actuator is in these sets, we set them to 0
                if actuator in ['airpump', 'sourcepump', 'planterpump', 'drainpump']:
                    self._post_actuator_command(actuator, {'value': 0}, device_name)
                elif actuator == 'led':
                    self._post_actuator_command(actuator, {'state': 'off'}, device_name)

        # Schedule turning off actuators after 'duration' minutes
        if duration > 0:
            threading.Timer(duration * 60, turn_off_actuators).start()

    # --------------------------------------------------------------------------
    # Basic device listing/selection
    # --------------------------------------------------------------------------
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


    def _prompt_schedule_24(self):
        """
        Replaces the console input prompt with a GUI-based editor.
        Returns a list of 24 integer PWM values or None if cancelled.
        """
        try:
            # Call the GUI editor (blocking until closed)
            schedule = prompt_schedule_24()
            if schedule is None:
                print("Schedule editing cancelled.")
                return None
            # Log the returned schedule
            logger.info("Schedule returned: %s", schedule)
            return schedule
        except Exception as e:
            print(f"Error launching schedule editor: {e}")
            return None
        
    def do_routine(self, arg):
        """
        routine <name>
        - empty_pod
        - fill_pod
        - calibrate_pod
        - light_schedule
        - planter_pod_schedule
        - air_pump_schedule
        If a schedule routine, fetch the current schedule from device, show GUI, post updated.
        Then poll routine status.
        """
        if not self._check_device_selected():
            return
        name = arg.strip().lower()
        if not name:
            print("Usage: routine <name>")
            return

        payload = {}
        # If it's one of the schedule routines, first fetch existing schedule from device
        if name in ["light_schedule","planter_pod_schedule","air_pump_schedule"]:
            # deduce type param
            schedule_type = "light"
            if name == "planter_pod_schedule":
                schedule_type = "planter"
            elif name == "air_pump_schedule":
                schedule_type = "air"

            # GET /api/routines/saved?type=<light|planter|air>
            init_sched = self._fetch_saved_schedule(schedule_type)
            print("DEBUG: Device returned schedule=", init_sched)
            new_sched = prompt_schedule_24(init_sched)
            if new_sched is None:
                print("Schedule editing cancelled.")
                return
            logger.info("Schedule returned: %s", new_sched)
            payload = {"schedule": new_sched}

        # 2) Fetch existing schedules
        if name == "all_schedules":
            light_sched = self._fetch_saved_schedule("light")
            planter_sched = self._fetch_saved_schedule("planter")
            air_sched = self._fetch_saved_schedule("air")

            # 3) Present a multi-schedule GUI or similar
            updated_light, updated_planter, updated_air = self._prompt_schedule_multi(
                light_sched, planter_sched, air_sched
            )
            if updated_light is None or updated_planter is None or updated_air is None:
                print("Schedule editing cancelled.")
                return

            # 4) Post each updated schedule
            self._post_routine("light_schedule", {"schedule": updated_light})
            self._post_routine("planter_pod_schedule", {"schedule": updated_planter})
            self._post_routine("air_pump_schedule", {"schedule": updated_air})
            return

        # Post the routine
        routine_id = self._post_routine(name, payload)
        if routine_id is None:
            print("Failed to start routine.")
            return

        print(f"Routine '{name}' started with ID {routine_id}. Polling for status...")
        self._poll_routine_status(routine_id)

    def _fetch_saved_schedule(self, schedule_type):
        """
        Attempts to GET /api/routines/saved?type=<schedule_type>
        Returns a list of 24 ints or a default [0..0].
        """
        if not self.selected_device:
            return [0]*24
        devinfo = devices[self.selected_device]
        url = f"https://{devinfo['address']}:{devinfo['port']}/api/routines/saved?type={schedule_type}"
        try:
            resp = self.session.get(url, timeout=5)
            if resp.status_code==200:
                data = resp.json()
                sched = data.get("schedule",[0]*24)
                if len(sched)<24:
                    sched = [0]*24
                return sched
            else:
                print(f"Cannot fetch saved schedule: {resp.status_code} {resp.text}")
        except requests.RequestException as e:
            print(f"Error fetching saved schedule: {e}")
        return [0]*24

    def do_routine_status(self, arg):
        """
        routine_status <routine_id>
        """
        if not self._check_selected():
            return
        try:
            rid = int(arg.strip())
        except ValueError:
            print("Usage: routine_status <id>")
            return

        dev = devices[self.selected_device]
        url = f"https://{dev['address']}:{dev['port']}/api/routines/status?id={rid}"
        try:
            r = self.session.get(url, timeout=5)
            if r.status_code == 200:
                data = r.json()
                print(json.dumps(data, indent=2))
            else:
                print(f"Error {r.status_code}: {r.text}")
        except requests.RequestException as e:
            print(f"Error: {e}")

    def _check_selected(self):
        if not self.selected_device:
            print("No device selected. 'list' then 'select <num>'.")
            return False
        if self.selected_device not in devices:
            print("Selected device not available.")
            self.selected_device = None
            return False
        return True

    def _post_routine(self, routine_name, json_body):
        dev = devices[self.selected_device]
        url = f"https://{dev['address']}:{dev['port']}/api/routines/{routine_name}"
        try:
            resp = self.session.post(url, json=json_body, timeout=5)
            if resp.status_code == 200:
                data = resp.json()
                rid = data.get("routine_id")
                msg = data.get("message", "")
                print(f"Routine '{routine_name}' started. ID={rid}. {msg}")
                return rid
            else:
                print(f"Error {resp.status_code}: {resp.text}")
        except requests.RequestException as e:
            print(f"Error sending routine: {e}")
        return None

    def _poll_routine_status(self, routine_id, timeout=60, interval=2):
        dev = devices[self.selected_device]
        url = f"https://{dev['address']}:{dev['port']}/api/routines/status?id={routine_id}"
        start_time = time.time()
        while True:
            try:
                r = self.session.get(url, timeout=5)
                if r.status_code == 200:
                    data = r.json()
                    status = data.get("status", "")
                    print(f"Routine {routine_id} status: {status}")
                    if status != "RUNNING":
                        print("Final routine status:")
                        print(json.dumps(data, indent=2))
                        return
                else:
                    print(f"Error polling: {r.status_code}: {r.text}")
            except requests.RequestException as e:
                print(f"Error polling: {e}")
            if time.time() - start_time > timeout:
                print("Polling timed out.")
                return
            time.sleep(interval)

    # --------------------------------------------------------------------------
    # Sensor Status
    # --------------------------------------------------------------------------
    def do_status(self, arg):
        'Get sensor data (unit metrics) from the selected device.'
        if not self._check_device_selected():
            return
        device_info = devices[self.selected_device]
        url = f"https://{device_info['address']}:{device_info['port']}/api/unit-metrics"
        try:
            response = self.session.get(url, timeout=5)
            response.raise_for_status()
            data = response.json()
            if 'mac_address' in data:
                devices[self.selected_device]['mac'] = data['mac_address']

            print(f"Device Name: {self.selected_device}")
            print(f"Device IP: {device_info['address']}")
            print(f"Device MAC: {device_info.get('mac', 'N/A')}")

            print("\n=== Device Metrics ===")
            print(f"Current: {data['current_mA']} mA")
            print(f"Voltage: {data['voltage_mV']} mV")
            print(f"Power: {data['power_consumption_mW']} mW")
            print(f"Water Level: {data['water_level_mm']} mm")
        except requests.exceptions.RequestException as e:
            print(f"Device Name: {self.selected_device}")
            print(f"Device IP: {device_info['address']}")
            logger.error(f"Error fetching sensor data: {e}")
            print(f"Error fetching sensor data: {e}")

    def do_hostname_suffix(self, arg):
        'Set mDNS suffix: hostname_suffix <suffix>'
        if not self._check_device_selected():
            return
        device_info = devices[self.selected_device]
        suffix = arg.strip()
        if not suffix:
            print("Please provide a valid suffix.")
            return
        url = f"https://{device_info['address']}:{device_info['port']}/api/hostnameSuffix"
        try:
            resp = self.session.post(url, json={"suffix": suffix}, timeout=5)
            if resp.status_code == 200:
                print("Suffix updated successfully.")
                reboot_url = f"https://{device_info['address']}:{device_info['port']}/api/restart"
                try:
                    self.session.post(reboot_url, timeout=5)
                    print("Device is rebooting to apply new hostname.")
                except Exception as e:
                    print(f"Error rebooting device: {e}")
            else:
                print("Error updating suffix:", resp.text)
        except Exception as e:
            print(f"Error updating suffix: {e}")

    # --------------------------------------------------------------------------
    # Actuator Commands
    # --------------------------------------------------------------------------
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

    def do_sourcepump(self, arg):
        'Set source pump PWM value: sourcepump <value>'
        if not self._check_device_selected():
            return
        try:
            value = int(arg.strip())
            if value < 0 or value > 100:
                print("Value must be between 0 and 100.")
                return
            self._post_actuator_command('sourcepump', {'value': value}, self.selected_device)
        except ValueError:
            print("Please provide a valid integer value.")

    def do_planterpump(self, arg):
        'Set planter pump PWM value: planterpump <value>'
        if not self._check_device_selected():
            return
        try:
            value = int(arg.strip())
            if value < 0 or value > 100:
                print("Value must be between 0 and 100.")
                return
            self._post_actuator_command('planterpump', {'value': value}, self.selected_device)
        except ValueError:
            print("Please provide a valid integer value.")

    def do_drainpump(self, arg):
        'Set drain pump PWM value: drainpump <value>'
        if not self._check_device_selected():
            return
        try:
            value = int(arg.strip())
            if value < 0 or value > 100:
                print("Value must be between 0 and 100.")
                return
            self._post_actuator_command('drainpump', {'value': value}, self.selected_device)
        except ValueError:
            print("Please provide a valid integer value.")

    def do_led(self, arg):
        'Set LED array state: led <on/off>'
        if not self._check_device_selected():
            return
        state = arg.strip().lower()
        if state not in ['on', 'off']:
            print("Please specify 'on' or 'off'.")
            return
        self._post_actuator_command('led', {'state': state}, self.selected_device)

    # --------------------------------------------------------------------------
    # Scheduling Commands
    # --------------------------------------------------------------------------
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
        
        # Source Pump
        sourcepump_choice = self._prompt_input("Schedule Source Pump? (y/n): ").lower()
        if sourcepump_choice == 'y':
            try:
                sourcepump_value = int(self._prompt_input("Enter Source Pump PWM value (0-100): "))
                if 0 <= sourcepump_value <= 100:
                    actions['sourcepump'] = {'value': sourcepump_value}
                else:
                    print("Invalid Source Pump value. Skipping Source Pump scheduling.")
            except ValueError:
                print("Invalid Source Pump value. Skipping Source Pump scheduling.")

        # Planter Pump
        planterpump_choice = self._prompt_input("Schedule Planter Pump? (y/n): ").lower()
        if planterpump_choice == 'y':
            try:
                planterpump_value = int(self._prompt_input("Enter Planter Pump PWM value (0-100): "))
                if 0 <= planterpump_value <= 100:
                    actions['planterpump'] = {'value': planterpump_value}
                else:
                    print("Invalid Planter Pump value. Skipping Planter Pump scheduling.")
            except ValueError:
                print("Invalid Planter Pump value. Skipping Planter Pump scheduling.")

        # Drain Pump
        drainpump_choice = self._prompt_input("Schedule Drain Pump? (y/n): ").lower()
        if drainpump_choice == 'y':
            try:
                drainpump_value = int(self._prompt_input("Enter Drain Pump PWM value (0-100): "))
                if 0 <= drainpump_value <= 100:
                    actions['drainpump'] = {'value': drainpump_value}
                else:
                    print("Invalid Drain Pump value. Skipping Drain Pump scheduling.")
            except ValueError:
                print("Invalid Drain Pump value. Skipping Drain Pump scheduling.")
        
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
        
        # Updated table columns to include 'sourcepump' and 'drainpump' instead of 'waterpump'/'servo'
        headers = ["Name", "Device Name", "Device IP", "Start Time (H:M)", "Duration (H:M)",
                   "Frequency", "Day of Week", "LED", "Air Pump", "Source Pump", "Planter Pump", "Drain Pump"]
        col_widths = [15, 15, 15, 15, 15, 10, 15, 10, 10, 12, 12, 11]
        
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
            
            # Retrieve actuator values
            led = details['actions'].get('led', {}).get('state', '-')
            airpump = str(details['actions'].get('airpump', {}).get('value', '-'))
            sourcepump = str(details['actions'].get('sourcepump', {}).get('value', '-'))
            planterpump = str(details['actions'].get('planterpump', {}).get('value', '-'))
            drainpump = str(details['actions'].get('drainpump', {}).get('value', '-'))
            
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
                sourcepump.ljust(col_widths[9]),
                planterpump.ljust(col_widths[10]),
                drainpump.ljust(col_widths[11])
            ]
            print(" | ".join(row))
        print()

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

    # --------------------------------------------------------------------------
    # Exit / Help
    # --------------------------------------------------------------------------
    def do_exit(self, arg):
        'Exit the console.'
        print("Exiting Hydroponics console.")
        return True

    def do_EOF(self, arg):
        'Handle EOF (Ctrl+D) to exit.'
        print("Exiting Hydroponics console.")
        return True

    # --------------------------------------------------------------------------
    # Internal Command Helpers
    # --------------------------------------------------------------------------
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
        
        for actuator_data in data.get("sensors_data", []):
            actuator_name = actuator_data.get("actuator", "Unknown")
            current = actuator_data.get("current_mA", None)
            voltage = actuator_data.get("voltage_mV", None)
            power = actuator_data.get("power_mW", None)
            flow_rate = actuator_data.get("flow_rate_L_min", None)
            
            # Format the values, handling None types
            current_str = f"{current:.2f}" if current is not None else "-"
            voltage_str = f"{voltage:.2f}" if voltage is not None else "-"
            power_str = f"{power:.2f}" if power is not None else "-"
            flow_rate_str = f"{flow_rate:.2f}" if flow_rate is not None else "-"
            
            row = [
                actuator_name,
                current_str,
                voltage_str,
                power_str,
                flow_rate_str
            ]
            rows.append(row)
        
        # Determine column widths
        col_widths = [max(len(str(item)) for item in [header] + [row[idx] for row in rows]) 
                      for idx, header in enumerate(headers)]
        
        # Print header
        header_row = " | ".join([headers[i].ljust(col_widths[i]) for i in range(len(headers))])
        separator = "-+-".join(['-' * col_widths[i] for i in range(len(headers))])
        print("\n" + header_row)
        print(separator)
        
        # Print rows
        for row in rows:
            print(" | ".join([str(item).ljust(col_widths[idx]) for idx, item in enumerate(row)]))
        print()

    def save_schedules(self):
        'Save schedules to the JSON file'
        try:
            with open(schedules_file, 'w') as f:
                json.dump(self.schedules, f, indent=4)
        except Exception as e:
            print(f"Error saving schedules: {e}")

    # --------------------------------------------------------------------------
    # Additional Start/Stop Commands (Feeding, Emptying, etc.)
    # --------------------------------------------------------------------------
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

    # Start/Stop feeding cycle
    def do_start_feeding_cycle(self, arg):
        'Start the feeding cycle: start_feeding_cycle'
        if not self._check_device_selected():
            return
        self._post_control_command('start_feeding_cycle')

    def do_stop_feeding_cycle(self, arg):
        'Stop the feeding cycle: stop_feeding_cycle'
        if not self._check_device_selected():
            return
        self._post_control_command('stop_feeding_cycle')

    # Start/Stop emptying water
    def do_start_emptying_water(self, arg):
        'Start emptying water: start_emptying_water'
        if not self._check_device_selected():
            return
        self._post_control_command('start_emptying_water')

    def do_stop_emptying_water(self, arg):
        'Stop emptying water: stop_emptying_water'
        if not self._check_device_selected():
            return
        self._post_control_command('stop_emptying_water')

    def _post_control_command(self, command):
        if self.selected_device not in devices:
            print(f"Device '{self.selected_device}' is not available.")
            return
        device_info = devices[self.selected_device]
        url = f"https://{device_info['address']}:{device_info['port']}/api/control/{command}"
        try:
            response = self.session.post(url, timeout=5)
            response.raise_for_status()
            print(response.text)
        except Exception as e:
            print(f"Error sending control command '{command}' to device '{self.selected_device}': {e}")

    def do_rescan(self, arg):
        'Re-initialize Zeroconf to discover devices again.'
        global devices
        devices.clear()
        try:
            self.zeroconf.close()
        except:
            pass
        self.zeroconf = Zeroconf()
        self.listener = HydroponicsServiceListener(self)
        self.browser = ServiceBrowser(self.zeroconf, "_hydroponics._tcp.local.", self.listener)
        print("Restarted Zeroconf discovery.")

    # --------------------------------------------------------------------------
    # Background Polling
    # --------------------------------------------------------------------------
    def poll_sensors(self):
        while True:
            time.sleep(self.polling_interval)
            if not self.selected_device:
                continue
            device_info = devices.get(self.selected_device)
            if not device_info:
                continue
            url = f"https://{device_info['address']}:{device_info['port']}/api/sensors"
            try:
                response = self.session.get(url, timeout=5)
                response.raise_for_status()
                data = response.json()

                # Extract and display system status
                system_status = data.get('system_status', 'Unknown')
                if system_status != self.previous_system_status:
                    self.previous_system_status = system_status
                    print(f"\nSystem Status Updated: {system_status}")

                # Process flow rates
                for sensor_data in data.get('sensors_data', []):
                    flowmeter = sensor_data.get('flowmeter')
                    flow_rate = sensor_data.get('flow_rate_L_min', 0.0)
                    previous_status = self.flow_status.get(flowmeter, False)
                    current_status = flow_rate > 0.0

                    if not previous_status and current_status:
                        # Flow started
                        self.flow_status[flowmeter] = True
                        print(f"\nFlow started on {flowmeter} flowmeter.")

                    elif previous_status and not current_status:
                        # Flow stopped
                        self.flow_status[flowmeter] = False
                        print(f"\nFlow stopped on {flowmeter} flowmeter.")

            except requests.exceptions.RequestException as e:
                print(f"Error polling sensor data: {e}")

    def _prompt_schedule_multi(self, light_sched, planter_sched, air_sched):
        '''
        Prompt the user to edit all schedules together, returning updated schedules.
        '''
        try:
            print("Editing Light Schedule")
            updated_light = prompt_schedule_24(light_sched)
            print("Editing Planter Schedule")
            updated_planter = prompt_schedule_24(planter_sched)
            print("Editing Air Pump Schedule")
            updated_air = prompt_schedule_24(air_sched)
            return updated_light, updated_planter, updated_air
        except Exception as e:
            print(f"Error in multi-schedule editing: {e}")
            return None, None, None

    def do_showschedules(self, arg):
        'Print current LED, planter & air schedules on the selected device'
        if not self._check_device_selected():
            return
        info = devices[self.selected_device]
        url = f"https://{info['address']}:{info['port']}/api/schedules"
        try:
            resp = self.session.get(url, timeout=5)
            if resp.status_code == 200:
                print(resp.text)
            else:
                print(f"Error {resp.status_code}: {resp.text}")
        except Exception as e:
            print(f"Error fetching schedules: {e}")

def start_service_discovery(console):
    zeroconf = Zeroconf()
    listener = HydroponicsServiceListener(console)
    browser = ServiceBrowser(zeroconf, "_hydroponics._tcp.local.", listener)
    console.zeroconf = zeroconf
    console.listener = listener
    console.browser = browser
    return zeroconf

def main():
    # Create the console instance
    console = HydroponicsConsole()

    # Start mDNS service discovery in a separate thread
    zeroconf = start_service_discovery(console)
    
    try:
        console.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting Hydroponics console.")
    finally:
        zeroconf.close()
        scheduler.shutdown()

if __name__ == '__main__':
    # schedule = prompt_schedule_24([10] * 24)
    # if schedule is not None:
    #     print("Schedule:", schedule)
    # else:
    #     print("Cancelled")
    main()