import os
import socket
import sys
import threading
from zeroconf import ServiceBrowser, Zeroconf
from cmd2 import Cmd
import requests
import json

# Automatically locate certificate and key files in the same directory
script_dir = os.path.dirname(os.path.realpath(__file__))
ca_bundle = os.path.join(script_dir, 'main/certs', 'ca_bundle.pem')
client_cert = os.path.join(script_dir, 'main/certs', 'client.crt')
client_key = os.path.join(script_dir, 'main/certs', 'client.key')

# Global dictionary to store discovered devices
devices = {}

class HydroponicsServiceListener:
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

    def remove_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info:
            device_name = info.server.replace('.local.', '')
            if device_name in devices:
                del devices[device_name]
                print(f"Device removed: {device_name}")

    def update_service(self, zeroconf, type, name):
        # Placeholder for future support
        pass

class HydroponicsConsole(Cmd):
    intro = 'Welcome to the Hydroponics console. Type help or ? to list commands.\n'
    prompt = '(hydroponics) '
    
    def __init__(self):
        super().__init__()
        self.selected_device = None
        self.session = requests.Session()
        self.session.verify = False #ca_bundle  # Path to the CA bundle

        # If mutual TLS is enabled, set client cert and key
        if os.path.exists(client_cert) and os.path.exists(client_key):
            self.session.cert = (client_cert, client_key)

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
            response = self.session.get(url, timeout=5, verify=False)
            data = response.json()
            print(json.dumps(data, indent=2))
        except Exception as e:
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
            self._post_actuator_command('airpump', {'value': value})
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
            self._post_actuator_command('waterpump', {'value': value})
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
        value = 1 if state == 'on' else 0
        self._post_actuator_command('solenoid', {'state': value})

    def do_led(self, arg):
        'Set LED array state: led <on/off>'
        if not self._check_device_selected():
            return
        state = arg.strip().lower()
        if state not in ['on', 'off']:
            print("Please specify 'on' or 'off'.")
            return
        # value = 1 if state == 'on' else 0
        self._post_actuator_command('led', {'state': state})

    def do_exit(self, arg):
        'Exit the console.'
        print("Exiting Hydroponics console.")
        return True

    def do_EOF(self, arg):
        'Handle EOF (Ctrl+D) to exit.'
        print("Exiting Hydroponics console.")
        return True

    def _post_actuator_command(self, actuator, payload):
        device_info = devices[self.selected_device]
        url = f"https://{device_info['address']}:{device_info['port']}/api/actuators/{actuator}"
        try:
            response = self.session.post(url, json=payload, timeout=5, verify=False)
            print(response.text)
        except Exception as e:
            print(f"Error sending command to {actuator}: {e}")

    def _check_device_selected(self):
        if not self.selected_device:
            print("No device selected. Use 'list' to see devices and 'select <number>' to select one.")
            return False
        if self.selected_device not in devices:
            print("Selected device is no longer available.")
            self.selected_device = None
            return False
        return True

def start_service_discovery():
    zeroconf = Zeroconf()
    listener = HydroponicsServiceListener()
    browser = ServiceBrowser(zeroconf, "_hydroponics._tcp.local.", listener)
    return zeroconf

def main():
    # Start mDNS service discovery in a separate thread
    zeroconf = start_service_discovery()
    try:
        console = HydroponicsConsole()
        console.cmdloop()
    except KeyboardInterrupt:
        print("\nExiting Hydroponics console.")
    finally:
        zeroconf.close()

if __name__ == '__main__':
    main()
