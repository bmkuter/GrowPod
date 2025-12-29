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
from datetime import datetime, timedelta
import time
import tkinter as tk
from tkinter import messagebox, ttk
import csv
from collections import deque
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for embedding in tkinter
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.dates as mdates

# Add SQLite for persistent historical data storage
import sqlite3

# JUST FOR DEBUG, REMOVE ME DURING PRODUCTION
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

"""
=============================================================================
TODO: FOOD PUMP SYSTEM - Complete Integration Checklist
=============================================================================

FIRMWARE TASKS (ESP32):
1. [✓] Create foodpump API endpoint in https_server.c:
    - POST /api/actuators/foodpump
    - Accept JSON: {"value": 0-100} for PWM mode
    - Accept JSON: {"dose": duration_ms, "speed": 0-100} for timed dosing
    
2. [✓] Implement foodpump_post_handler() in https_server.c
    - Parse JSON payload
    - Call set_food_pump_pwm() for PWM mode
    - Call dose_food_pump_ms() for dose mode
    
3. [✓] Register foodpump endpoint in start_webserver()

PYTHON CONSOLE TASKS (hydroponics_console.py):
4. [✓] Uncomment API calls in do_foodpump() command
    - Line ~1428: self._post_actuator_command('foodpump', ...)
    
5. [✓] Uncomment API calls in execute_schedule()
    - Line ~1048: Enable food_dose HTTP POST to device
    
CALIBRATION SYSTEM (Future):
6. [ ] Create calibration routine to measure:
    - Volume dispensed per millisecond at various speeds
    - Create lookup table: ms -> mL conversion
    
7. [ ] Add calibration GUI:
    - Run pump for known duration
    - User measures dispensed volume
    - Calculate and store calibration factor
    
8. [ ] Update food schedule GUI:
    - Display volume (mL) instead of time (ms)
    - Use calibration data for conversion
    
9. [ ] Store calibration data:
    - In device NVS (firmware)
    - In Python config file (console)

NOTES:
- Food pump uses MOTOR_FOOD_PUMP (motor channel 2) on first shield (0x60)
- Firmware functions already exist: set_food_pump_pwm(), dose_food_pump_ms()
- UART command 'foodpump' already works in firmware
- This GUI provides scheduled dosing split into intervals throughout the day
=============================================================================
"""

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

# =============================================================================
# Sensor Data Storage and Graphing
# =============================================================================

# Create data directory for sensor logs
data_dir = os.path.join(script_dir, 'sensor_data')
os.makedirs(data_dir, exist_ok=True)

# In-memory data storage for graphing (last N readings)
MAX_GRAPH_POINTS = 100  # Keep last 100 data points in memory for graphing

class SensorDatabase:
    """Handles persistent sensor data storage using SQLite"""
    
    def __init__(self, device_name):
        self.device_name = device_name
        self.db_path = os.path.join(data_dir, f"{device_name}_history.db")
        self.conn = None
        self._connect()
        self._create_schema()
    
    def _connect(self):
        """Establish database connection"""
        self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row  # Enable column access by name
        
        # Enable WAL mode for better concurrency
        self.conn.execute("PRAGMA journal_mode=WAL")
        self.conn.execute("PRAGMA synchronous=NORMAL")
        
    def _create_schema(self):
        """Create database schema if it doesn't exist"""
        cursor = self.conn.cursor()
        
        # Create main sensor readings table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp INTEGER NOT NULL,
                temperature_c REAL,
                humidity_rh REAL,
                light_lux REAL,
                light_visible INTEGER,
                light_infrared INTEGER,
                power_mw REAL,
                current_ma REAL,
                voltage_mv REAL,
                water_level_mm REAL,
                UNIQUE(timestamp) ON CONFLICT IGNORE
            )
        """)
        
        # Create index on timestamp for fast time-range queries
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_timestamp 
            ON sensor_readings(timestamp)
        """)
        
        # Create metadata table for tracking sync status
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sync_metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            )
        """)
        
        self.conn.commit()
        logging.info(f"Database initialized: {self.db_path}")
    
    def insert_readings(self, readings):
        """
        Insert multiple sensor readings (batch insert)
        
        Args:
            readings: List of dicts with keys matching column names
                     Each dict must have 'timestamp' key
        
        Returns:
            Number of rows inserted (duplicates are ignored)
        """
        if not readings:
            return 0
        
        cursor = self.conn.cursor()
        
        insert_sql = """
            INSERT OR IGNORE INTO sensor_readings 
            (timestamp, temperature_c, humidity_rh, light_lux, light_visible, 
             light_infrared, power_mw, current_ma, voltage_mv, water_level_mm)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        
        rows = []
        for reading in readings:
            rows.append((
                reading['timestamp'],
                reading.get('temperature_c'),
                reading.get('humidity_rh'),
                reading.get('light_lux'),
                reading.get('light_visible'),
                reading.get('light_infrared'),
                reading.get('power_mw'),
                reading.get('current_ma'),
                reading.get('voltage_mv'),
                reading.get('water_level_mm')
            ))
        
        cursor.executemany(insert_sql, rows)
        inserted = cursor.rowcount
        self.conn.commit()
        
        return inserted
    
    def get_latest_timestamp(self):
        """
        Get the most recent timestamp in the database
        
        Returns:
            Integer timestamp or 0 if database is empty
        """
        cursor = self.conn.cursor()
        cursor.execute("SELECT MAX(timestamp) FROM sensor_readings")
        result = cursor.fetchone()
        
        return result[0] if result[0] is not None else 0
    
    def get_readings(self, start_timestamp=None, end_timestamp=None, limit=None):
        """
        Query sensor readings within a time range
        
        Args:
            start_timestamp: Unix timestamp (inclusive), None for no lower bound
            end_timestamp: Unix timestamp (inclusive), None for no upper bound
            limit: Maximum number of rows to return, None for all
        
        Returns:
            List of dicts containing sensor readings
        """
        cursor = self.conn.cursor()
        
        query = "SELECT * FROM sensor_readings WHERE 1=1"
        params = []
        
        if start_timestamp is not None:
            query += " AND timestamp >= ?"
            params.append(start_timestamp)
        
        if end_timestamp is not None:
            query += " AND timestamp <= ?"
            params.append(end_timestamp)
        
        query += " ORDER BY timestamp ASC"
        
        if limit is not None:
            query += " LIMIT ?"
            params.append(limit)
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        
        # Convert to list of dicts
        return [dict(row) for row in rows]
    
    def get_count(self, start_timestamp=None, end_timestamp=None):
        """Get count of readings in database within optional time range"""
        cursor = self.conn.cursor()
        
        query = "SELECT COUNT(*) FROM sensor_readings WHERE 1=1"
        params = []
        
        if start_timestamp is not None:
            query += " AND timestamp >= ?"
            params.append(start_timestamp)
        
        if end_timestamp is not None:
            query += " AND timestamp <= ?"
            params.append(end_timestamp)
        
        cursor.execute(query, params)
        return cursor.fetchone()[0]
    
    def get_stats(self):
        """Get database statistics"""
        cursor = self.conn.cursor()
        
        cursor.execute("""
            SELECT 
                COUNT(*) as total_entries,
                MIN(timestamp) as oldest_timestamp,
                MAX(timestamp) as newest_timestamp
            FROM sensor_readings
        """)
        
        row = cursor.fetchone()
        return dict(row) if row else None
    
    def close(self):
        """Close database connection"""
        if self.conn:
            self.conn.close()
            self.conn = None

class SensorDataLogger:
    """Handles sensor data logging to CSV and in-memory storage for graphing"""
    
    def __init__(self, device_name):
        self.device_name = device_name
        self.csv_file = os.path.join(data_dir, f"{device_name}_sensor_data.csv")
        
        # In-memory storage using deque for efficient append/pop
        self.timestamps = deque(maxlen=MAX_GRAPH_POINTS)
        self.temperature = deque(maxlen=MAX_GRAPH_POINTS)
        self.humidity = deque(maxlen=MAX_GRAPH_POINTS)
        self.light_lux = deque(maxlen=MAX_GRAPH_POINTS)
        self.light_visible = deque(maxlen=MAX_GRAPH_POINTS)
        self.light_infrared = deque(maxlen=MAX_GRAPH_POINTS)
        self.power_mw = deque(maxlen=MAX_GRAPH_POINTS)
        self.current_ma = deque(maxlen=MAX_GRAPH_POINTS)
        self.voltage_mv = deque(maxlen=MAX_GRAPH_POINTS)
        self.water_level_mm = deque(maxlen=MAX_GRAPH_POINTS)
        
        # Create CSV file with headers if it doesn't exist
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'temperature_c', 'humidity_rh', 'light_lux', 
                    'light_visible', 'light_infrared', 'power_mw', 'current_ma', 
                    'voltage_mv', 'water_level_mm'
                ])
    
    def log_data(self, sensor_data):
        """
        Log sensor data to CSV and in-memory storage
        
        Args:
            sensor_data: Dictionary from /api/unit-metrics JSON response
        """
        timestamp = datetime.now()
        
        # Extract data with defaults for unavailable sensors
        temp_c = sensor_data.get('temperature_c', -999)
        humidity = sensor_data.get('humidity_rh', -999)
        lux = sensor_data.get('light_lux', -999)
        visible = sensor_data.get('light_visible', 0)
        infrared = sensor_data.get('light_infrared', 0)
        power = sensor_data.get('power_consumption_mW', 0)
        current = sensor_data.get('current_mA', 0)
        voltage = sensor_data.get('voltage_mV', 0)
        water = sensor_data.get('water_level_mm', -1)
        
        # Store in memory (only valid data for graphing)
        self.timestamps.append(timestamp)
        self.temperature.append(temp_c if temp_c != -999 else None)
        self.humidity.append(humidity if humidity != -999 else None)
        self.light_lux.append(lux if lux != -999 else None)
        self.light_visible.append(visible if visible > 0 else None)
        self.light_infrared.append(infrared if infrared > 0 else None)
        self.power_mw.append(power)
        self.current_ma.append(current)
        self.voltage_mv.append(voltage)
        self.water_level_mm.append(water if water >= 0 else None)
        
        # Append to CSV file
        try:
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    timestamp.strftime('%Y-%m-%d %H:%M:%S'),
                    temp_c, humidity, lux, visible, infrared,
                    power, current, voltage, water
                ])
        except Exception as e:
            logger.error(f"Failed to write sensor data to CSV: {e}")
    
    def get_graph_data(self):
        """Return current in-memory data for graphing"""
        return {
            'timestamps': list(self.timestamps),
            'temperature': list(self.temperature),
            'humidity': list(self.humidity),
            'light_lux': list(self.light_lux),
            'light_visible': list(self.light_visible),
            'light_infrared': list(self.light_infrared),
            'power_mw': list(self.power_mw),
            'current_ma': list(self.current_ma),
            'voltage_mv': list(self.voltage_mv),
            'water_level_mm': list(self.water_level_mm)
        }

# Global dictionary to store data loggers per device
sensor_data_loggers = {}

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

def prompt_schedule_actuators():
    """
    Opens a unified Tkinter window for configuring all schedule parameters.
    Shows all actuator controls in a single grouped panel.
    Returns a dictionary with schedule configuration or None if cancelled.
    """
    result = None

    def on_confirm():
        nonlocal result
        
        # Validate schedule name
        sched_name = schedule_name_entry.get().strip()
        if not sched_name:
            tk.messagebox.showerror("Error", "Schedule name cannot be empty.")
            return
        
        # Validate start time
        start_time_str = start_time_entry.get().strip()
        try:
            datetime.strptime(start_time_str, "%H:%M")
        except ValueError:
            tk.messagebox.showerror("Error", "Invalid start time format. Use HH:MM (e.g., 18:00)")
            return
        
        # Validate duration
        duration_str = duration_entry.get().strip()
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
            tk.messagebox.showerror("Error", f"Invalid duration format: {ve}")
            return
        
        # Get frequency
        frequency = frequency_var.get().lower()
        
        # Get day of week if weekly
        day_of_week = None
        if frequency == 'weekly':
            day_of_week = day_var.get()
            if not day_of_week or day_of_week == "Select Day":
                tk.messagebox.showerror("Error", "Please select a day of the week for weekly schedules.")
                return
        
        # Collect actuator actions
        actions = {}
        
        if led_enabled.get():
            actions['led'] = {'value': led_scale.get()}
        
        if airpump_enabled.get():
            actions['airpump'] = {'value': airpump_scale.get()}
        
        if sourcepump_enabled.get():
            actions['sourcepump'] = {'value': sourcepump_scale.get()}
        
        if planterpump_enabled.get():
            actions['planterpump'] = {'value': planterpump_scale.get()}
        
        if drainpump_enabled.get():
            actions['drainpump'] = {'value': drainpump_scale.get()}
        
        if not actions:
            tk.messagebox.showerror("Error", "No actuators enabled. Please enable at least one actuator.")
            return
        
        # Build result dictionary
        result = {
            'schedule_name': sched_name,
            'start_time': start_time_str,
            'duration_minutes': duration,
            'frequency': frequency,
            'day_of_week': day_of_week,
            'actions': actions
        }
        
        root.destroy()

    def on_cancel():
        nonlocal result
        result = None
        root.destroy()

    def toggle_day_selector(*args):
        """Enable/disable day selector based on frequency"""
        if frequency_var.get() == 'weekly':
            day_menu.config(state='normal')
        else:
            day_menu.config(state='disabled')

    # Create main window
    root = tk.Tk()
    root.title("Actuator Schedule Setup")
    root.geometry("600x700")
    
    # Main container with scrollbar
    main_frame = tk.Frame(root)
    main_frame.pack(fill="both", expand=True, padx=10, pady=10)
    
    # ========== Schedule Information Section ==========
    info_frame = tk.LabelFrame(main_frame, text="Schedule Information", padx=10, pady=10)
    info_frame.pack(fill="x", pady=(0, 10))
    
    # Schedule Name
    tk.Label(info_frame, text="Schedule Name:", anchor="w").grid(row=0, column=0, sticky="w", pady=5)
    schedule_name_entry = tk.Entry(info_frame, width=30)
    schedule_name_entry.grid(row=0, column=1, sticky="ew", pady=5)
    
    # Start Time
    tk.Label(info_frame, text="Start Time (HH:MM):", anchor="w").grid(row=1, column=0, sticky="w", pady=5)
    start_time_entry = tk.Entry(info_frame, width=30)
    start_time_entry.insert(0, "18:00")
    start_time_entry.grid(row=1, column=1, sticky="ew", pady=5)
    
    # Duration
    tk.Label(info_frame, text="Duration (HH:MM):", anchor="w").grid(row=2, column=0, sticky="w", pady=5)
    duration_entry = tk.Entry(info_frame, width=30)
    duration_entry.insert(0, "01:00")
    duration_entry.grid(row=2, column=1, sticky="ew", pady=5)
    
    # Frequency
    tk.Label(info_frame, text="Frequency:", anchor="w").grid(row=3, column=0, sticky="w", pady=5)
    frequency_var = tk.StringVar(value="daily")
    freq_frame = tk.Frame(info_frame)
    freq_frame.grid(row=3, column=1, sticky="w", pady=5)
    tk.Radiobutton(freq_frame, text="Daily", variable=frequency_var, value="daily", command=toggle_day_selector).pack(side="left", padx=5)
    tk.Radiobutton(freq_frame, text="Weekly", variable=frequency_var, value="weekly", command=toggle_day_selector).pack(side="left", padx=5)
    
    # Day of Week (for weekly)
    tk.Label(info_frame, text="Day of Week:", anchor="w").grid(row=4, column=0, sticky="w", pady=5)
    day_var = tk.StringVar(value="Monday")
    days = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
    day_menu = tk.OptionMenu(info_frame, day_var, *days)
    day_menu.grid(row=4, column=1, sticky="w", pady=5)
    day_menu.config(state='disabled')  # Initially disabled for daily
    
    info_frame.columnconfigure(1, weight=1)
    
    # ========== Actuators Section ==========
    actuators_frame = tk.LabelFrame(main_frame, text="Actuators Configuration", padx=10, pady=10)
    actuators_frame.pack(fill="both", expand=True, pady=(0, 10))
    
    # Helper function to create actuator control
    def create_actuator_control(parent, row, name, default_value=0):
        enabled_var = tk.BooleanVar(value=False)
        
        # Checkbox
        cb = tk.Checkbutton(parent, text=name, variable=enabled_var, width=15, anchor="w")
        cb.grid(row=row, column=0, sticky="w", pady=5)
        
        # Scale
        scale = tk.Scale(parent, from_=0, to=100, orient="horizontal", length=300, 
                        state='disabled', label="PWM %")
        scale.set(default_value)
        scale.grid(row=row, column=1, sticky="ew", pady=5, padx=(10, 0))
        
        # Enable/disable scale based on checkbox
        def toggle_scale():
            scale.config(state='normal' if enabled_var.get() else 'disabled')
        
        cb.config(command=toggle_scale)
        
        return enabled_var, scale
    
    # Create actuator controls
    led_enabled, led_scale = create_actuator_control(actuators_frame, 0, "LED Array", 100)
    airpump_enabled, airpump_scale = create_actuator_control(actuators_frame, 1, "Air Pump", 80)
    sourcepump_enabled, sourcepump_scale = create_actuator_control(actuators_frame, 2, "Source Pump", 100)
    planterpump_enabled, planterpump_scale = create_actuator_control(actuators_frame, 3, "Planter Pump", 100)
    drainpump_enabled, drainpump_scale = create_actuator_control(actuators_frame, 4, "Drain Pump", 100)
    
    actuators_frame.columnconfigure(1, weight=1)
    
    # ========== Buttons Section ==========
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=10)
    
    confirm_btn = tk.Button(button_frame, text="Create Schedule", command=on_confirm, width=15, bg="#4CAF50", fg="white")
    confirm_btn.pack(side="left", padx=5)
    
    cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, width=15)
    cancel_btn.pack(side="left", padx=5)
    
    root.mainloop()
    return result

def prompt_schedule_multi(light_sched, planter_sched):
    """
    Opens a unified Tkinter window with tabs for Light and Planter schedules.
    Shows both 24-hour schedules in a single tabbed interface.
    Returns tuple (updated_light, updated_planter) or (None, None) if cancelled.
    """
    result = None

    def on_confirm():
        nonlocal result
        # Collect all schedules from the two tabs
        updated_light = [light_scales[i].get() for i in range(24)]
        updated_planter = [planter_scales[i].get() for i in range(24)]
        
        result = (updated_light, updated_planter)
        root.destroy()

    def on_cancel():
        nonlocal result
        result = (None, None)
        root.destroy()

    def create_schedule_tab(parent, schedule_name, initial_schedule, color="#4CAF50"):
        """Create a tab with 24 hour sliders for a single schedule"""
        # Create a frame with scrollbar for the tab
        tab_frame = tk.Frame(parent)
        
        # Title
        title_frame = tk.Frame(tab_frame, bg=color, height=40)
        title_frame.pack(fill="x", pady=(0, 10))
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(title_frame, text=f"{schedule_name} Schedule", 
                              font=("Arial", 14, "bold"), bg=color, fg="white")
        title_label.pack(expand=True)
        
        # Create canvas with scrollbar for schedules
        canvas = tk.Canvas(tab_frame)
        scrollbar = tk.Scrollbar(tab_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Create 24 hour sliders
        scales = []
        for hour in range(24):
            hour_frame = tk.Frame(scrollable_frame)
            hour_frame.pack(fill="x", padx=10, pady=3)
            
            # Hour label
            hour_label = tk.Label(hour_frame, text=f"{hour:02d}:00", width=6, font=("Arial", 10))
            hour_label.pack(side="left", padx=5)
            
            # PWM value display
            value_var = tk.StringVar(value="0%")
            value_label = tk.Label(hour_frame, textvariable=value_var, width=5, 
                                   font=("Arial", 10, "bold"), fg=color)
            value_label.pack(side="right", padx=5)
            
            # Slider
            scale = tk.Scale(hour_frame, from_=0, to=100, orient="horizontal", 
                           length=400, showvalue=0)
            
            # Set initial value
            if initial_schedule and len(initial_schedule) == 24:
                scale.set(initial_schedule[hour])
            else:
                scale.set(0)
            
            # Update value label when slider changes
            def update_label(val, var=value_var):
                var.set(f"{int(float(val))}%")
            
            scale.config(command=update_label)
            update_label(scale.get(), value_var)  # Set initial label
            
            scale.pack(side="left", fill="x", expand=True, padx=5)
            scales.append(scale)
        
        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        return tab_frame, scales

    # Create main window
    root = tk.Tk()
    root.title("All Schedules Editor")
    root.geometry("700x650")
    
    # Main frame
    main_frame = tk.Frame(root)
    main_frame.pack(fill="both", expand=True, padx=10, pady=10)
    
    # Info label
    info_label = tk.Label(main_frame, 
                         text="Configure 24-hour schedules for all actuators (0-100% PWM for each hour)",
                         font=("Arial", 10), fg="#666")
    info_label.pack(pady=(0, 10))
    
    # Create tabbed notebook
    notebook = ttk.Notebook(main_frame)
    notebook.pack(fill="both", expand=True)
    
    # Create two tabs with different colors
    light_tab, light_scales = create_schedule_tab(notebook, "💡 Light", light_sched, "#FFA500")
    planter_tab, planter_scales = create_schedule_tab(notebook, "🌱 Planter Pump", planter_sched, "#4CAF50")
    
    # Add tabs to notebook
    notebook.add(light_tab, text="💡 Light Schedule")
    notebook.add(planter_tab, text="🌱 Planter Schedule")
    
    # Buttons frame
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=10)
    
    confirm_btn = tk.Button(button_frame, text="Save All Schedules", command=on_confirm, 
                           width=18, bg="#4CAF50", fg="white", font=("Arial", 11, "bold"))
    confirm_btn.pack(side="left", padx=5)
    
    cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, 
                          width=12, font=("Arial", 11))
    cancel_btn.pack(side="left", padx=5)
    
    # Add keyboard shortcuts info
    shortcuts_label = tk.Label(main_frame, 
                              text="💡 Tip: Use tabs to switch between schedules quickly",
                              font=("Arial", 9), fg="#999")
    shortcuts_label.pack(pady=(5, 0))
    
    root.mainloop()
    return result

def prompt_unified_schedule_manager(initial_light_schedule=None, initial_planter_schedule=None, existing_schedules=None):
    """
    Comprehensive schedule management GUI combining:
    1. Hourly schedules (Light curve + Planter intervals)
    2. Food dosing schedule
    3. Routine commands (Fill/Empty/Maintenance schedules)
    4. View existing schedules
    
    Args:
        initial_light_schedule: List of 24 integers (0-100) for light schedule
        initial_planter_schedule: List of 24 integers (0-100) for planter schedule
        existing_schedules: Dictionary of existing schedules to display
    
    Returns a dictionary with both schedule types or None if cancelled.
    """
    result = None
    
    if existing_schedules is None:
        existing_schedules = {}

    def on_save_all():
        nonlocal result
        
        # Collect hourly schedules
        light_schedule = [light_scales[i].get() for i in range(24)]
        planter_schedule = [planter_scales[i].get() for i in range(24)]
        
        # Collect food schedule info
        food_config = None
        if enable_food.get():
            try:
                total_ms = int(food_total_entry.get())
                intervals = int(food_intervals_spinbox.get())
                speed = int(food_speed_scale.get())
                
                if total_ms <= 0:
                    tk.messagebox.showerror("Error", "Total daily amount must be greater than 0.")
                    return
                if intervals <= 0:
                    tk.messagebox.showerror("Error", "Number of intervals must be greater than 0.")
                    return
                
                food_config = {
                    'total_daily_ms': total_ms,
                    'intervals': intervals,
                    'speed': speed,
                    'dose_per_interval': total_ms // intervals
                }
            except ValueError:
                tk.messagebox.showerror("Error", "Invalid food schedule values. Please enter valid numbers.")
                return
        
        # Collect routine schedule info
        routine_config = None
        if enable_routine.get():
            # Validate schedule name
            sched_name = routine_name_entry.get().strip()
            if not sched_name:
                tk.messagebox.showerror("Error", "Routine schedule name cannot be empty.")
                return
            
            # Validate start time
            start_time_str = routine_start_entry.get().strip()
            try:
                datetime.strptime(start_time_str, "%H:%M")
            except ValueError:
                tk.messagebox.showerror("Error", "Invalid start time format. Use HH:MM (e.g., 18:00)")
                return
            
            # Get frequency
            frequency = routine_freq_var.get().lower()
            
            # Get day of week if weekly
            day_of_week = None
            if frequency == 'weekly':
                day_of_week = routine_day_var.get()
                if not day_of_week or day_of_week == "Select Day":
                    tk.messagebox.showerror("Error", "Please select a day of the week for weekly schedules.")
                    return
            
            # Get selected routine command
            routine_command = routine_command_var.get()
            if not routine_command or routine_command == "Select Command":
                tk.messagebox.showerror("Error", "Please select a routine command.")
                return
            
            routine_config = {
                'schedule_name': sched_name,
                'start_time': start_time_str,
                'frequency': frequency,
                'day_of_week': day_of_week,
                'command': routine_command
            }
        
        result = {
            'light_schedule': light_schedule,
            'planter_schedule': planter_schedule,
            'food_config': food_config,
            'routine_config': routine_config
        }
        
        root.destroy()

    def on_cancel():
        nonlocal result
        result = None
        root.destroy()

    def toggle_routine_controls():
        """Enable/disable routine controls based on checkbox"""
        state = 'normal' if enable_routine.get() else 'disabled'
        routine_name_entry.config(state=state)
        routine_start_entry.config(state=state)
        routine_freq_daily.config(state=state)
        routine_freq_weekly.config(state=state)
        routine_command_menu.config(state=state)
        toggle_day_selector()

    def toggle_day_selector():
        """Enable/disable day selector based on frequency"""
        if enable_routine.get() and routine_freq_var.get() == 'weekly':
            routine_day_menu.config(state='normal')
        else:
            routine_day_menu.config(state='disabled')

    def create_hourly_schedule_tab(parent, schedule_name, initial_schedule, color="#4CAF50"):
        """Create a tab with 24 hour sliders for hourly schedules"""
        tab_frame = tk.Frame(parent)
        
        # Title
        title_frame = tk.Frame(tab_frame, bg=color, height=40)
        title_frame.pack(fill="x", pady=(0, 10))
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(title_frame, text=f"{schedule_name} Schedule", 
                              font=("Arial", 14, "bold"), bg=color, fg="white")
        title_label.pack(expand=True)
        
        # Create canvas with scrollbar
        canvas = tk.Canvas(tab_frame)
        scrollbar = tk.Scrollbar(tab_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Create 24 hour sliders
        scales = []
        for hour in range(24):
            hour_frame = tk.Frame(scrollable_frame)
            hour_frame.pack(fill="x", padx=10, pady=3)
            
            # Hour label
            hour_label = tk.Label(hour_frame, text=f"{hour:02d}:00", width=6, font=("Arial", 10))
            hour_label.pack(side="left", padx=5)
            
            # PWM value display
            value_var = tk.StringVar(value="0%")
            value_label = tk.Label(hour_frame, textvariable=value_var, width=5, 
                                   font=("Arial", 10, "bold"), fg=color)
            value_label.pack(side="right", padx=5)
            
            # Slider
            scale = tk.Scale(hour_frame, from_=0, to=100, orient="horizontal", 
                           length=400, showvalue=0)
            
            # Set initial value
            if initial_schedule and len(initial_schedule) == 24:
                scale.set(initial_schedule[hour])
            else:
                scale.set(0)
            
            # Update value label when slider changes
            def update_label(val, var=value_var):
                var.set(f"{int(float(val))}%")
            
            scale.config(command=update_label)
            update_label(scale.get(), value_var)
            
            # Add mouse wheel support for slider adjustment
            def on_slider_mousewheel(event, s=scale):
                current = s.get()
                # Scroll up = increase, scroll down = decrease
                if event.delta > 0:
                    s.set(min(100, current + 1))
                else:
                    s.set(max(0, current - 1))
                return "break"  # Prevent event propagation
            
            scale.bind("<MouseWheel>", on_slider_mousewheel)
            
            scale.pack(side="left", fill="x", expand=True, padx=5)
            scales.append(scale)
        
        # Enable mouse wheel scrolling on canvas area
        def on_canvas_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        # Bind to canvas, scrollable frame, and all labels/frames (but not sliders)
        canvas.bind("<MouseWheel>", on_canvas_mousewheel)
        scrollable_frame.bind("<MouseWheel>", on_canvas_mousewheel)
        
        # Bind to all child widgets except scales
        for child in scrollable_frame.winfo_children():
            child.bind("<MouseWheel>", on_canvas_mousewheel)
            for grandchild in child.winfo_children():
                if not isinstance(grandchild, tk.Scale):
                    grandchild.bind("<MouseWheel>", on_canvas_mousewheel)
        
        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        return tab_frame, scales

    # Create main window
    root = tk.Tk()
    root.title("Unified Schedule Manager")
    root.geometry("800x950")  # Increased height for food section and schedules view
    
    # Bring window to foreground
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    root.focus_force()
    
    # Main frame
    main_frame = tk.Frame(root)
    main_frame.pack(fill="both", expand=True, padx=10, pady=10)
    
    # ========== Top Info Label ==========
    info_label = tk.Label(main_frame, 
                         text="Manage hourly schedules (Light & Planter), food dosing, and routine commands",
                         font=("Arial", 10), fg="#666", wraplength=700)
    info_label.pack(pady=(0, 10))
    
    # ========== Hourly Schedules Section (Tabbed) ==========
    hourly_frame = tk.LabelFrame(main_frame, text="Hourly Schedules", padx=5, pady=5)
    hourly_frame.pack(fill="both", expand=True, pady=(0, 10))
    
    # Create tabbed notebook for hourly schedules
    hourly_notebook = ttk.Notebook(hourly_frame)
    hourly_notebook.pack(fill="both", expand=True)
    
    # Create two tabs - Light and Planter (no Air)
    light_tab, light_scales = create_hourly_schedule_tab(hourly_notebook, "💡 Light", initial_light_schedule, "#FFA500")
    planter_tab, planter_scales = create_hourly_schedule_tab(hourly_notebook, "🌱 Planter", initial_planter_schedule, "#4CAF50")
    
    # Add tabs to notebook
    hourly_notebook.add(light_tab, text="💡 Light Curve")
    hourly_notebook.add(planter_tab, text="🌱 Planter Intervals")
    
    # ========== Food Schedule Section ==========
    food_frame = tk.LabelFrame(main_frame, text="🍽️ Food Dosing Schedule (Daily Distribution)", padx=10, pady=10)
    food_frame.pack(fill="x", pady=(0, 10))
    
    # Enable/Disable food schedule checkbox
    enable_food = tk.BooleanVar(value=False)
    
    def toggle_food_controls():
        """Enable/disable food controls based on checkbox"""
        state = 'normal' if enable_food.get() else 'disabled'
        food_total_entry.config(state=state)
        food_intervals_spinbox.config(state=state)
        food_speed_scale.config(state=state)
        food_calibrate_btn.config(state=state)
    
    enable_food_cb = tk.Checkbutton(food_frame, text="Enable Daily Food Dosing Schedule", 
                                    variable=enable_food, command=toggle_food_controls,
                                    font=("Arial", 10, "bold"))
    enable_food_cb.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))
    
    # Total daily amount (in milliseconds of pump runtime)
    tk.Label(food_frame, text="Total Daily Amount (ms):", anchor="w").grid(row=1, column=0, sticky="w", pady=5)
    food_total_entry = tk.Entry(food_frame, width=15, state='disabled')
    food_total_entry.insert(0, "5000")  # Default 5 seconds total per day
    food_total_entry.grid(row=1, column=1, sticky="w", pady=5, padx=(0, 5))
    tk.Label(food_frame, text="(Total pump runtime per day)", fg="#666", font=("Arial", 9)).grid(row=1, column=2, sticky="w", pady=5)
    
    # Number of intervals
    tk.Label(food_frame, text="Number of Intervals:", anchor="w").grid(row=2, column=0, sticky="w", pady=5)
    food_intervals_spinbox = tk.Spinbox(food_frame, from_=1, to=24, width=13, state='disabled')
    food_intervals_spinbox.delete(0, "end")
    food_intervals_spinbox.insert(0, "4")  # Default 4 feedings per day
    food_intervals_spinbox.grid(row=2, column=1, sticky="w", pady=5, padx=(0, 5))
    tk.Label(food_frame, text="(Evenly spaced throughout the day)", fg="#666", font=("Arial", 9)).grid(row=2, column=2, sticky="w", pady=5)
    
    # Pump speed
    tk.Label(food_frame, text="Pump Speed (%):", anchor="w").grid(row=3, column=0, sticky="w", pady=5)
    food_speed_scale = tk.Scale(food_frame, from_=1, to=100, orient="horizontal", 
                                length=200, state='disabled')
    food_speed_scale.set(100)  # Default 100% speed
    food_speed_scale.grid(row=3, column=1, sticky="w", pady=5, padx=(0, 5))
    tk.Label(food_frame, text="(Speed during dosing)", fg="#666", font=("Arial", 9)).grid(row=3, column=2, sticky="w", pady=5)
    
    # Calculated dose per interval (read-only display)
    tk.Label(food_frame, text="Dose Per Interval:", anchor="w", fg="#0066cc", font=("Arial", 9, "bold")).grid(row=4, column=0, sticky="w", pady=5)
    food_dose_label = tk.Label(food_frame, text="0 ms", fg="#0066cc", font=("Arial", 9, "bold"))
    food_dose_label.grid(row=4, column=1, sticky="w", pady=5)
    
    def update_dose_calculation(*args):
        """Update the calculated dose per interval"""
        try:
            total = int(food_total_entry.get())
            intervals = int(food_intervals_spinbox.get())
            dose_per_interval = total // intervals if intervals > 0 else 0
            food_dose_label.config(text=f"{dose_per_interval} ms per feeding")
        except ValueError:
            food_dose_label.config(text="Invalid input")
    
    # Bind calculation updates
    food_total_entry.bind("<KeyRelease>", update_dose_calculation)
    food_intervals_spinbox.bind("<<Increment>>", update_dose_calculation)
    food_intervals_spinbox.bind("<<Decrement>>", update_dose_calculation)
    food_intervals_spinbox.bind("<KeyRelease>", update_dose_calculation)
    
    # Calibration section (skeleton for future implementation)
    tk.Label(food_frame, text="", anchor="w").grid(row=5, column=0, pady=5)  # Spacer
    food_calibrate_btn = tk.Button(food_frame, text="🔧 Calibrate Food Pump", 
                                   state='disabled',
                                   command=lambda: tk.messagebox.showinfo("Calibration", 
                                       "Food pump calibration feature coming soon!\n\n"
                                       "This will allow you to:\n"
                                       "- Measure actual volume dispensed per millisecond\n"
                                       "- Convert between time and volume units\n"
                                       "- Fine-tune dosing accuracy"))
    food_calibrate_btn.grid(row=5, column=1, sticky="w", pady=5)
    tk.Label(food_frame, text="(Calibration: TODO)", fg="#999", font=("Arial", 9, "italic")).grid(row=5, column=2, sticky="w", pady=5)
    
    food_frame.columnconfigure(2, weight=1)
    
    # ========== Routine Commands Section ==========
    routine_frame = tk.LabelFrame(main_frame, text="Routine Command Schedule (Fill/Empty/Maintenance)", padx=10, pady=10)
    routine_frame.pack(fill="x", pady=(0, 10))
    
    # Enable/Disable routine checkbox
    enable_routine = tk.BooleanVar(value=False)
    enable_cb = tk.Checkbutton(routine_frame, text="Enable Routine Command Schedule", 
                               variable=enable_routine, command=toggle_routine_controls,
                               font=("Arial", 10, "bold"))
    enable_cb.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
    
    # Routine Name
    tk.Label(routine_frame, text="Schedule Name:", anchor="w").grid(row=1, column=0, sticky="w", pady=5)
    routine_name_entry = tk.Entry(routine_frame, width=30, state='disabled')
    routine_name_entry.grid(row=1, column=1, sticky="ew", pady=5)
    
    # Start Time
    tk.Label(routine_frame, text="Start Time (HH:MM):", anchor="w").grid(row=2, column=0, sticky="w", pady=5)
    routine_start_entry = tk.Entry(routine_frame, width=30, state='disabled')
    routine_start_entry.insert(0, "02:00")
    routine_start_entry.grid(row=2, column=1, sticky="ew", pady=5)
    
    # Frequency
    tk.Label(routine_frame, text="Frequency:", anchor="w").grid(row=3, column=0, sticky="w", pady=5)
    routine_freq_var = tk.StringVar(value="daily")
    freq_frame = tk.Frame(routine_frame)
    freq_frame.grid(row=3, column=1, sticky="w", pady=5)
    routine_freq_daily = tk.Radiobutton(freq_frame, text="Daily", variable=routine_freq_var, 
                                        value="daily", command=toggle_day_selector, state='disabled')
    routine_freq_daily.pack(side="left", padx=5)
    routine_freq_weekly = tk.Radiobutton(freq_frame, text="Weekly", variable=routine_freq_var, 
                                         value="weekly", command=toggle_day_selector, state='disabled')
    routine_freq_weekly.pack(side="left", padx=5)
    
    # Day of Week (for weekly)
    tk.Label(routine_frame, text="Day of Week:", anchor="w").grid(row=4, column=0, sticky="w", pady=5)
    routine_day_var = tk.StringVar(value="Monday")
    days = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
    routine_day_menu = tk.OptionMenu(routine_frame, routine_day_var, *days)
    routine_day_menu.grid(row=4, column=1, sticky="w", pady=5)
    routine_day_menu.config(state='disabled')
    
    # Routine Command Selection
    tk.Label(routine_frame, text="Command:", anchor="w").grid(row=5, column=0, sticky="w", pady=5)
    routine_command_var = tk.StringVar(value="Select Command")
    commands = ["empty_pod", "fill_pod", "calibrate_pod"]
    routine_command_menu = tk.OptionMenu(routine_frame, routine_command_var, *commands)
    routine_command_menu.grid(row=5, column=1, sticky="w", pady=5)
    routine_command_menu.config(state='disabled')
    
    routine_frame.columnconfigure(1, weight=1)
    
    # ========== Existing Schedules View Section ==========
    if existing_schedules:
        schedules_frame = tk.LabelFrame(main_frame, text="📅 Existing Schedules", padx=5, pady=5)
        schedules_frame.pack(fill="both", expand=True, pady=(0, 10))
        
        # Create canvas with scrollbar for schedule list
        sched_canvas = tk.Canvas(schedules_frame, height=200)
        sched_scrollbar = tk.Scrollbar(schedules_frame, orient="vertical", command=sched_canvas.yview)
        sched_scrollable = tk.Frame(sched_canvas)
        
        sched_scrollable.bind(
            "<Configure>",
            lambda e: sched_canvas.configure(scrollregion=sched_canvas.bbox("all"))
        )
        
        sched_canvas.create_window((0, 0), window=sched_scrollable, anchor="nw")
        sched_canvas.configure(yscrollcommand=sched_scrollbar.set)
        
        # Header row
        header_frame = tk.Frame(sched_scrollable, relief=tk.RAISED, borderwidth=1)
        header_frame.pack(fill="x", padx=5, pady=(5, 2))
        
        tk.Label(header_frame, text="Name", width=18, font=("Arial", 9, "bold"), anchor="w").pack(side="left", padx=2)
        tk.Label(header_frame, text="Device", width=12, font=("Arial", 9, "bold"), anchor="w").pack(side="left", padx=2)
        tk.Label(header_frame, text="Time", width=8, font=("Arial", 9, "bold"), anchor="w").pack(side="left", padx=2)
        tk.Label(header_frame, text="Frequency", width=10, font=("Arial", 9, "bold"), anchor="w").pack(side="left", padx=2)
        tk.Label(header_frame, text="Actions", width=30, font=("Arial", 9, "bold"), anchor="w").pack(side="left", padx=2)
        
        # Add schedule rows
        for sched_name, sched_details in existing_schedules.items():
            row_frame = tk.Frame(sched_scrollable, relief=tk.GROOVE, borderwidth=1)
            row_frame.pack(fill="x", padx=5, pady=1)
            
            # Name
            name_label = tk.Label(row_frame, text=sched_name, width=18, anchor="w", font=("Arial", 9))
            name_label.pack(side="left", padx=2)
            
            # Device
            device_label = tk.Label(row_frame, text=sched_details.get('device_name', 'N/A'), 
                                   width=12, anchor="w", font=("Arial", 9))
            device_label.pack(side="left", padx=2)
            
            # Time
            time_label = tk.Label(row_frame, text=sched_details.get('start_time', 'N/A'), 
                                 width=8, anchor="w", font=("Arial", 9))
            time_label.pack(side="left", padx=2)
            
            # Frequency with day
            freq = sched_details.get('frequency', 'N/A')
            day = sched_details.get('day_of_week', '')
            freq_text = f"{freq.capitalize()}"
            if day and freq == 'weekly':
                freq_text += f" ({day})"
            freq_label = tk.Label(row_frame, text=freq_text, width=10, anchor="w", font=("Arial", 9))
            freq_label.pack(side="left", padx=2)
            
            # Actions summary
            actions = sched_details.get('actions', {})
            action_parts = []
            
            # Check for routine commands
            if 'routine' in actions:
                action_parts.append(f"Routine: {actions['routine'].get('command', 'N/A')}")
            
            # Check for food dosing
            if 'food_dose' in actions:
                dose_ms = actions['food_dose'].get('duration_ms', 0)
                speed = actions['food_dose'].get('speed', 100)
                action_parts.append(f"Food: {dose_ms}ms@{speed}%")
            
            # Check for actuators
            for actuator in ['led', 'airpump', 'sourcepump', 'planterpump', 'drainpump']:
                if actuator in actions:
                    value = actions[actuator].get('value', 0)
                    if value > 0:
                        action_parts.append(f"{actuator.capitalize()}: {value}%")
            
            action_text = ", ".join(action_parts) if action_parts else "None"
            if len(action_text) > 40:
                action_text = action_text[:37] + "..."
            
            action_label = tk.Label(row_frame, text=action_text, width=30, anchor="w", 
                                   font=("Arial", 9), fg="#0066cc")
            action_label.pack(side="left", padx=2)
        
        # Pack canvas and scrollbar
        sched_canvas.pack(side="left", fill="both", expand=True)
        sched_scrollbar.pack(side="right", fill="y")
        
        # Info label
        info_text = f"Showing {len(existing_schedules)} active schedule(s). Use 'delete_schedule' command to remove."
        tk.Label(schedules_frame, text=info_text, font=("Arial", 8), fg="#666").pack(pady=(5, 0))
    
    # ========== Action Buttons ==========
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=10)
    
    save_btn = tk.Button(button_frame, text="Save All Settings", command=on_save_all, 
                        width=20, bg="#4CAF50", fg="white", font=("Arial", 11, "bold"))
    save_btn.pack(side="left", padx=5)
    
    cancel_btn = tk.Button(button_frame, text="Cancel", command=on_cancel, 
                          width=12, font=("Arial", 11))
    cancel_btn.pack(side="left", padx=5)
    
    # ========== Help Text ==========
    help_label = tk.Label(main_frame, 
                         text="💡 Tip: Hourly schedules control frequent events. Routine commands handle maintenance tasks.",
                         font=("Arial", 9), fg="#999")
    help_label.pack(pady=(5, 0))
    
    root.mainloop()
    return result

# =============================================================================
# Unified GUI Launcher - Main Application Interface
# =============================================================================

def sync_historical_data(console_instance, database):
    """
    Sync historical sensor data from device to local database.
    Only requests data since the last recorded timestamp (gap filling).
    
    Args:
        console_instance: HydroponicsConsole instance with session
        database: SensorDatabase instance
    
    Returns:
        Tuple of (success: bool, new_entries: int, message: str)
    """
    device_name = console_instance.selected_device
    device_info = devices.get(device_name)
    
    if not device_info:
        return (False, 0, "Device not found")
    
    # Get the latest timestamp we have in our database
    last_timestamp = database.get_latest_timestamp()
    
    logging.info(f"Syncing historical data for {device_name}")
    logging.info(f"Latest local timestamp: {last_timestamp}")
    
    try:
        # Build API URL
        base_url = f"https://{device_info['address']}:{device_info['port']}"
        
        if last_timestamp > 0:
            # Request only data since our last timestamp
            url = f"{base_url}/api/sensor-history?start={last_timestamp + 1}"
            logging.info(f"Requesting data since timestamp {last_timestamp + 1}")
        else:
            # No existing data, get everything the device has
            url = f"{base_url}/api/sensor-history"
            logging.info("No existing data, requesting full history")
        
        # Make request with timeout
        response = console_instance.session.get(url, timeout=30, verify=False)
        response.raise_for_status()
        
        data = response.json()
        
        readings = data.get('readings', [])
        stats = data.get('stats', {})
        
        logging.info(f"Received {len(readings)} readings from device")
        logging.info(f"Device stats: {stats}")
        
        if readings:
            # Insert readings into database (duplicates are automatically ignored)
            inserted = database.insert_readings(readings)
            
            # Get updated database stats
            db_stats = database.get_stats()
            
            message = f"Sync complete: {inserted} new entries added"
            if db_stats:
                total = db_stats['total_entries']
                oldest = datetime.fromtimestamp(db_stats['oldest_timestamp']).strftime('%Y-%m-%d %H:%M:%S')
                newest = datetime.fromtimestamp(db_stats['newest_timestamp']).strftime('%Y-%m-%d %H:%M:%S')
                message += f"\nDatabase now has {total} total entries"
                message += f"\nTime range: {oldest} to {newest}"
            
            logging.info(message)
            return (True, inserted, message)
        else:
            message = "No new data to sync (database is up to date)"
            logging.info(message)
            return (True, 0, message)
    
    except requests.exceptions.RequestException as e:
        error_msg = f"Failed to sync historical data: {e}"
        logging.error(error_msg)
        return (False, 0, error_msg)
    except Exception as e:
        error_msg = f"Unexpected error during sync: {e}"
        logging.error(error_msg)
        return (False, 0, error_msg)

def launch_unified_gui(console_instance):
    """
    Main GUI application launcher that combines all features into a tabbed interface.
    
    Tabs:
        - Schedules: Manage hourly light/planter schedules, food dosing, and routines
        - Filesystem: Browse device filesystem, view/edit files
        - Plant Info: Manage plant information and growing data
    
    This replaces the old manage_schedules command as the primary GUI entry point.
    """
    if not console_instance.selected_device:
        tk.messagebox.showerror("No Device", "Please select a device first using 'select <device_name>'")
        return
    
    root = tk.Tk()
    root.title(f"GrowPod Control - {console_instance.selected_device}")
    
    # Get screen dimensions
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    
    # Set window to span full vertical height with reasonable width
    window_width = 1400
    window_height = screen_height - 80  # Leave space for taskbar
    
    # Center horizontally, position at top
    x_position = (screen_width - window_width) // 2
    y_position = 0
    
    root.geometry(f"{window_width}x{window_height}+{x_position}+{y_position}")
    
    # Bring window to foreground
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    root.focus_force()
    
    # Main container
    main_frame = tk.Frame(root, padx=10, pady=10)
    main_frame.pack(fill="both", expand=True)
    
    # Device info header
    device_info = devices[console_instance.selected_device]
    header_frame = tk.Frame(main_frame, bg="#2c3e50", padx=10, pady=5)
    header_frame.pack(fill="x", pady=(0, 10))
    
    device_label = tk.Label(
        header_frame, 
        text=f"🌱 Device: {console_instance.selected_device} ({device_info['address']})",
        font=("Arial", 12, "bold"),
        bg="#2c3e50",
        fg="white"
    )
    device_label.pack(side="left")
    
    # Status indicator
    status_label = tk.Label(
        header_frame,
        text="● Connected",
        font=("Arial", 10),
        bg="#2c3e50",
        fg="#27ae60"
    )
    status_label.pack(side="right")
    
    # Create notebook (tabbed interface)
    notebook = ttk.Notebook(main_frame)
    notebook.pack(fill="both", expand=True)
    
    # Tab 1: Dashboard - Overview and sensor data
    dashboard_tab = tk.Frame(notebook)
    notebook.add(dashboard_tab, text="📊 Dashboard")
    create_dashboard_tab(dashboard_tab, console_instance)
    
    # Tab 2: Light & Planter 24-hour curves
    light_planter_tab = tk.Frame(notebook)
    notebook.add(light_planter_tab, text="💡 Light & Planter")
    create_light_planter_tab(light_planter_tab, console_instance)
    
    # Tab 3: Food Schedule
    food_tab = tk.Frame(notebook)
    notebook.add(food_tab, text="🍽️ Food Schedule")
    create_food_schedule_tab(food_tab, console_instance)
    
    # Tab 4: Routine Calendar
    routine_calendar_tab = tk.Frame(notebook)
    notebook.add(routine_calendar_tab, text="📆 Routine Calendar")
    create_routine_calendar_tab(routine_calendar_tab, console_instance)
    
    # Tab 5: Filesystem Browser
    filesystem_tab = tk.Frame(notebook)
    notebook.add(filesystem_tab, text="📁 Filesystem")
    create_filesystem_tab(filesystem_tab, console_instance)
    
    # Tab 6: Plant Info
    plant_tab = tk.Frame(notebook)
    notebook.add(plant_tab, text="🌱 Plant Info")
    create_plant_info_tab(plant_tab, console_instance)
    
    # Tab 7: Legacy Schedule Manager (moved to end)
    schedules_tab = tk.Frame(notebook)
    notebook.add(schedules_tab, text="⚙️ Legacy Schedule Manager")
    create_schedules_tab(schedules_tab, console_instance)
    
    # Handle tab changes - manage auto-refresh based on active tab
    def on_tab_change(event):
        current_tab = notebook.index(notebook.select())
        # Dashboard is tab 0
        if current_tab == 0:
            # Restart auto-refresh when returning to dashboard
            if hasattr(dashboard_tab, 'start_auto_refresh'):
                dashboard_tab.start_auto_refresh()
        else:
            # Stop auto-refresh when leaving dashboard
            if hasattr(dashboard_tab, 'stop_auto_refresh'):
                dashboard_tab.stop_auto_refresh()
    
    notebook.bind("<<NotebookTabChanged>>", on_tab_change)
    
    # Handle window close - cleanup auto-refresh timer and database
    def on_closing():
        """Clean up resources when window is closed"""
        if hasattr(dashboard_tab, 'stop_auto_refresh'):
            dashboard_tab.stop_auto_refresh()
        if hasattr(dashboard_tab, 'stop_periodic_sync'):
            dashboard_tab.stop_periodic_sync()
        if hasattr(dashboard_tab, 'database'):
            dashboard_tab.database.close()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    root.mainloop()


def create_dashboard_tab(parent_frame, console_instance):
    """
    Create the Dashboard tab.
    Shows device metadata, current sensor readings, and system status on the left.
    Shows real-time graphs on the right.
    """
    # Initialize or get data logger for this device
    device_name = console_instance.selected_device
    if device_name not in sensor_data_loggers:
        sensor_data_loggers[device_name] = SensorDataLogger(device_name)
    data_logger = sensor_data_loggers[device_name]
    
    # Initialize database for persistent historical storage
    database = SensorDatabase(device_name)
    
    # Sync historical data from device to local database
    logging.info(f"Syncing historical data for {device_name}...")
    success, new_entries, sync_message = sync_historical_data(console_instance, database)
    
    if success:
        if new_entries > 0:
            logging.info(f"[OK] Sync successful: {new_entries} new entries")
        else:
            logging.info("[OK] Database already up to date")
    else:
        logging.warning(f"[WARN] Sync failed: {sync_message}")
    
    # Split the frame into left (info) and right (graphs)
    left_frame = tk.Frame(parent_frame)
    left_frame.pack(side="left", fill="both", expand=False, padx=(0, 10))
    
    right_frame = tk.Frame(parent_frame)
    right_frame.pack(side="right", fill="both", expand=True)
    
    # ========== LEFT PANEL: Sensor Info (with scrollbar) ==========
    canvas = tk.Canvas(left_frame, width=600)
    scrollbar = tk.Scrollbar(left_frame, orient="vertical", command=canvas.yview)
    scrollable_frame = tk.Frame(canvas)
    
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")
    
    # Title
    title_label = tk.Label(
        scrollable_frame,
        text="📊 GrowPod Dashboard",
        font=("Arial", 16, "bold")
    )
    title_label.pack(pady=15)
    
    # Device Information Section
    device_frame = tk.LabelFrame(scrollable_frame, text="Device Information", padx=20, pady=15, font=("Arial", 11, "bold"))
    device_frame.pack(fill="x", padx=20, pady=10)
    
    device_info = devices[console_instance.selected_device]
    
    info_items = [
        ("Device Name:", console_instance.selected_device, "🌱"),
        ("IP Address:", device_info.get('address', 'Unknown'), "🔗"),
        ("Port:", str(device_info.get('port', 'Unknown')), "🔌"),
        ("Connection Status:", "Connected", "✅")
    ]
    
    for i, (label, value, icon) in enumerate(info_items):
        row_frame = tk.Frame(device_frame)
        row_frame.pack(fill="x", pady=5)
        
        tk.Label(row_frame, text=f"{icon} {label}", font=("Arial", 10, "bold"), anchor="w", width=20).pack(side="left")
        tk.Label(row_frame, text=value, font=("Arial", 10), fg="#2c3e50", anchor="w").pack(side="left", padx=10)
    
    # Plant Information Section
    plant_frame = tk.LabelFrame(scrollable_frame, text="Plant Information", padx=20, pady=15, font=("Arial", 11, "bold"))
    plant_frame.pack(fill="x", padx=20, pady=10)
    
    plant_info_labels = {}
    
    def refresh_plant_info():
        result = console_instance._get_plant_info()
        
        if result and result.get('exists'):
            plant_info_labels['name'].config(text=result.get('plant_name', 'Unknown'))
            plant_info_labels['date'].config(text=result.get('start_date', 'Unknown'))
            plant_info_labels['days'].config(text=f"{result.get('days_growing', 0)} days")
        else:
            plant_info_labels['name'].config(text="(not set)")
            plant_info_labels['date'].config(text="(not set)")
            plant_info_labels['days'].config(text="(not set)")
    
    plant_items = [
        ("Plant Name:", "name", "🌿"),
        ("Start Date:", "date", "📅"),
        ("Days Growing:", "days", "⏳")
    ]
    
    for label, key, icon in plant_items:
        row_frame = tk.Frame(plant_frame)
        row_frame.pack(fill="x", pady=5)
        
        tk.Label(row_frame, text=f"{icon} {label}", font=("Arial", 10, "bold"), anchor="w", width=20).pack(side="left")
        plant_info_labels[key] = tk.Label(row_frame, text="Loading...", font=("Arial", 10), fg="#2c3e50", anchor="w")
        plant_info_labels[key].pack(side="left", padx=10)
    
    # Current Sensor Readings Section
    sensor_frame = tk.LabelFrame(scrollable_frame, text="Current Sensor Readings", padx=20, pady=15, font=("Arial", 11, "bold"))
    sensor_frame.pack(fill="x", padx=20, pady=10)
    
    sensor_labels = {}
    
    def refresh_sensors():
        try:
            device = devices[console_instance.selected_device]
            url = f"https://{device['address']}:{device['port']}/api/unit-metrics"
            response = console_instance.session.get(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                
                # Log data for historical storage and graphing
                data_logger.log_data(data)
                
                # Update MAC address if available
                if 'mac_address' in data:
                    devices[console_instance.selected_device]['mac'] = data['mac_address']
                
                # Overall device metrics
                sensor_labels['total_current'].config(text=f"{data.get('current_mA', 0):.2f} mA")
                sensor_labels['total_voltage'].config(text=f"{data.get('voltage_mV', 0):.2f} mV")
                sensor_labels['total_power'].config(text=f"{data.get('power_consumption_mW', 0):.2f} mW")
                sensor_labels['water_level'].config(text=f"{data.get('water_level_mm', -1)} mm")
                
                # Environment sensors
                temp_c = data.get('temperature_c', -999)
                humidity = data.get('humidity_rh', -999)
                if temp_c != -999:
                    temp_f = (temp_c * 9/5) + 32
                    sensor_labels['temperature'].config(text=f"{temp_c:.1f}°C ({temp_f:.1f}°F)")
                else:
                    sensor_labels['temperature'].config(text="N/A")
                
                if humidity != -999:
                    sensor_labels['humidity'].config(text=f"{humidity:.1f}%")
                else:
                    sensor_labels['humidity'].config(text="N/A")
                
                # Light sensor
                lux = data.get('light_lux', -999)
                visible = data.get('light_visible', 0)
                infrared = data.get('light_infrared', 0)
                if lux != -999:
                    sensor_labels['light_lux'].config(text=f"{lux:.1f} lux")
                    sensor_labels['light_visible'].config(text=f"{visible}")
                    sensor_labels['light_infrared'].config(text=f"{infrared}")
                else:
                    sensor_labels['light_lux'].config(text="N/A")
                    sensor_labels['light_visible'].config(text="N/A")
                    sensor_labels['light_infrared'].config(text="N/A")
                
                # Update graphs
                update_graphs()
                
                status_label.config(text="✅ Sensor data updated", fg="#27ae60")
            else:
                status_label.config(text="⚠️ Failed to fetch sensor data", fg="#e67e22")
        except Exception as e:
            status_label.config(text=f"❌ Error: {str(e)}", fg="#e74c3c")
    
    # Device metrics display
    metrics_subframe = tk.LabelFrame(sensor_frame, text="Current Device Metrics", padx=15, pady=10)
    metrics_subframe.pack(fill="x", pady=5)
    
    metrics_items = [
        ("Total Current:", "total_current", "⚡", "mA"),
        ("Voltage:", "total_voltage", "🔋", "mV"),
        ("Power Consumption:", "total_power", "💡", "mW"),
        ("Water Level:", "water_level", "💧", "mm")
    ]
    
    for i, (label, key, icon, unit) in enumerate(metrics_items):
        row_frame = tk.Frame(metrics_subframe)
        row_frame.pack(fill="x", pady=5)
        
        tk.Label(row_frame, text=f"{icon} {label}", font=("Arial", 10, "bold"), anchor="w", width=25).pack(side="left")
        sensor_labels[key] = tk.Label(row_frame, text=f"0.00 {unit}", font=("Arial", 10), fg="#2c3e50", anchor="w")
        sensor_labels[key].pack(side="left", padx=10)
    
    # Environment sensors display
    env_subframe = tk.LabelFrame(sensor_frame, text="Environment Sensors", padx=15, pady=10)
    env_subframe.pack(fill="x", pady=5)
    
    env_items = [
        ("Temperature:", "temperature", "🌡️", "°C"),
        ("Humidity:", "humidity", "💧", "%RH")
    ]
    
    for i, (label, key, icon, unit) in enumerate(env_items):
        row_frame = tk.Frame(env_subframe)
        row_frame.pack(fill="x", pady=5)
        
        # Use grid for better alignment control - separate emoji and text
        tk.Label(row_frame, text=icon, font=("Arial", 10, "bold"), anchor="w", width=3).grid(row=0, column=0, sticky="w", padx=(0, 5))
        tk.Label(row_frame, text=label, font=("Arial", 10, "bold"), anchor="w", width=15).grid(row=0, column=1, sticky="w")
        sensor_labels[key] = tk.Label(row_frame, text=f"N/A", font=("Arial", 10), fg="#2c3e50", anchor="w")
        sensor_labels[key].grid(row=0, column=2, sticky="w", padx=(10, 0))
    
    # Light sensor display
    light_subframe = tk.LabelFrame(sensor_frame, text="Light Sensor (TSL2591)", padx=15, pady=10)
    light_subframe.pack(fill="x", pady=5)
    
    light_items = [
        ("Illuminance:", "light_lux", "☀️", "lux"),
        ("Visible Light:", "light_visible", "👁️", "counts"),
        ("Infrared:", "light_infrared", "🔴", "counts")
    ]
    
    for i, (label, key, icon, unit) in enumerate(light_items):
        row_frame = tk.Frame(light_subframe)
        row_frame.pack(fill="x", pady=5)
        
        # Use grid for better alignment control
        tk.Label(row_frame, text=icon, font=("Arial", 10, "bold"), anchor="w", width=3).grid(row=0, column=0, sticky="w", padx=(0, 5))
        tk.Label(row_frame, text=label, font=("Arial", 10, "bold"), anchor="w", width=15).grid(row=0, column=1, sticky="w")
        sensor_labels[key] = tk.Label(row_frame, text=f"N/A", font=("Arial", 10), fg="#2c3e50", anchor="w")
        sensor_labels[key].grid(row=0, column=2, sticky="w", padx=(10, 0))
    
    # Additional sensors placeholder (for future expansion)
    additional_subframe = tk.LabelFrame(sensor_frame, text="Additional Sensors (Future Expansion)", padx=15, pady=10)
    additional_subframe.pack(fill="x", pady=5)
    
    tk.Label(
        additional_subframe,
        text="Per-actuator power metrics, pH, EC, and other sensors will appear here",
        font=("Arial", 9),
        fg="#999"
    ).pack(pady=10)
    
    # Action buttons
    action_frame = tk.Frame(scrollable_frame)
    action_frame.pack(pady=20)
    
    def refresh_all():
        refresh_plant_info()
        refresh_sensors()
    
    tk.Button(
        action_frame,
        text="🔄 Refresh All Data",
        command=refresh_all,
        font=("Arial", 12, "bold"),
        bg="#3498db",
        fg="white",
        padx=20,
        pady=10
    ).pack()
    
    # Status bar
    status_label = tk.Label(scrollable_frame, text="Ready", font=("Arial", 10), fg="#555")
    status_label.pack(side="bottom", fill="x", pady=10)
    
    # ========== RIGHT PANEL: Real-time Graphs ==========
    # Create matplotlib figure with subplots
    fig = Figure(figsize=(10, 12), dpi=100, facecolor='#f0f0f0')
    fig.subplots_adjust(hspace=0.4, left=0.1, right=0.95, top=0.97, bottom=0.08)
    
    # Create 4 subplots stacked vertically
    ax_temp_humid = fig.add_subplot(411)
    ax_light = fig.add_subplot(412)
    ax_power = fig.add_subplot(413)
    ax_water = fig.add_subplot(414)
    
    # Configure each subplot
    ax_temp_humid.set_title("Temperature & Humidity", fontsize=11, fontweight='bold')
    ax_temp_humid.set_ylabel("Temp (°C) / Humidity (%)", fontsize=9)
    ax_temp_humid.grid(True, alpha=0.3)
    ax_temp_humid.tick_params(labelsize=8)
    
    ax_light.set_title("Light Sensor (Illuminance)", fontsize=11, fontweight='bold')
    ax_light.set_ylabel("Illuminance (lux)", fontsize=9)
    ax_light.grid(True, alpha=0.3)
    ax_light.tick_params(labelsize=8)
    
    ax_power.set_title("Power & Current", fontsize=11, fontweight='bold')
    ax_power.set_ylabel("Power (mW) / Current (mA)", fontsize=9)
    ax_power.grid(True, alpha=0.3)
    ax_power.tick_params(labelsize=8)
    
    ax_water.set_title("Water Level", fontsize=11, fontweight='bold')
    ax_water.set_ylabel("Level (mm)", fontsize=9)
    ax_water.set_xlabel("Time", fontsize=9)
    ax_water.grid(True, alpha=0.3)
    ax_water.tick_params(labelsize=8)
    
    # Format x-axis to show time nicely
    time_formatter = mdates.DateFormatter('%H:%M:%S')
    for ax in [ax_temp_humid, ax_light, ax_power, ax_water]:
        ax.xaxis.set_major_formatter(time_formatter)
        ax.tick_params(axis='x', rotation=45)
    
    # Embed the figure in tkinter
    canvas_graph = FigureCanvasTkAgg(fig, master=right_frame)
    canvas_graph.draw()
    canvas_graph.get_tk_widget().pack(fill="both", expand=True)
    
    # Initialize line objects (will be updated with data)
    line_temp, = ax_temp_humid.plot([], [], 'r-', label='Temperature (°C)', linewidth=2)
    line_humid, = ax_temp_humid.plot([], [], 'b-', label='Humidity (%)', linewidth=2)
    ax_temp_humid.legend(loc='upper left', fontsize=8)
    
    line_lux, = ax_light.plot([], [], 'orange', label='Illuminance (lux)', linewidth=2)
    ax_light.legend(loc='upper left', fontsize=8)
    
    line_power, = ax_power.plot([], [], 'g-', label='Power (mW)', linewidth=2)
    line_current, = ax_power.plot([], [], 'purple', label='Current (mA)', linewidth=2)
    ax_power.legend(loc='upper left', fontsize=8)
    
    line_water, = ax_water.plot([], [], 'cyan', label='Water Level (mm)', linewidth=2, marker='o', markersize=3)
    ax_water.legend(loc='upper left', fontsize=8)
    
    def update_graphs():
        """Update all graphs with data from database (last 24 hours by default)"""
        try:
            # Query last 24 hours from database
            current_time = int(time.time())
            start_time = current_time - (24 * 60 * 60)  # 24 hours ago
            
            readings = database.get_readings(start_timestamp=start_time, end_timestamp=current_time)
            
            if not readings:
                return  # No data yet
            
            # Convert database readings to lists for plotting
            timestamps = []
            temperature = []
            humidity = []
            light_lux = []
            power_mw = []
            current_ma = []
            water_level_mm = []
            
            for reading in readings:
                ts = datetime.fromtimestamp(reading['timestamp'])
                timestamps.append(ts)
                
                # Use None for invalid sensor values
                temperature.append(reading['temperature_c'] if reading['temperature_c'] not in [None, -999] else None)
                humidity.append(reading['humidity_rh'] if reading['humidity_rh'] not in [None, -999] else None)
                light_lux.append(reading['light_lux'] if reading['light_lux'] not in [None, -999] else None)
                power_mw.append(reading['power_mw'] if reading['power_mw'] is not None else None)
                current_ma.append(reading['current_ma'] if reading['current_ma'] is not None else None)
                water_level_mm.append(reading['water_level_mm'] if reading['water_level_mm'] not in [None, -1] else None)
            
            # Filter out None values for plotting
            def filter_data(times, values):
                filtered_times = []
                filtered_values = []
                for t, v in zip(times, values):
                    if v is not None:
                        filtered_times.append(t)
                        filtered_values.append(v)
                return filtered_times, filtered_values
            
            # Update temperature & humidity
            temp_times, temp_values = filter_data(timestamps, temperature)
            humid_times, humid_values = filter_data(timestamps, humidity)
            
            line_temp.set_data(temp_times, temp_values)
            line_humid.set_data(humid_times, humid_values)
            
            if temp_times or humid_times:
                all_times = temp_times + humid_times
                all_values = temp_values + humid_values
                # Add small padding if min==max to avoid singular transformation
                time_min, time_max = min(all_times), max(all_times)
                if time_min == time_max:
                    time_min = time_min - timedelta(seconds=30)
                    time_max = time_max + timedelta(seconds=30)
                ax_temp_humid.set_xlim(time_min, time_max)
                ax_temp_humid.set_ylim(min(all_values) - 5, max(all_values) + 5)
            
            # Update light
            lux_times, lux_values = filter_data(timestamps, light_lux)
            line_lux.set_data(lux_times, lux_values)
            
            if lux_times:
                time_min, time_max = min(lux_times), max(lux_times)
                if time_min == time_max:
                    time_min = time_min - timedelta(seconds=30)
                    time_max = time_max + timedelta(seconds=30)
                ax_light.set_xlim(time_min, time_max)
                ax_light.set_ylim(0, max(lux_values) * 1.1 if lux_values else 100)
            
            # Update power & current
            power_times, power_values = filter_data(timestamps, power_mw)
            current_times, current_values = filter_data(timestamps, current_ma)
            
            line_power.set_data(power_times, power_values)
            line_current.set_data(current_times, current_values)
            
            if power_times or current_times:
                all_times = power_times + current_times
                all_values = power_values + current_values
                time_min, time_max = min(all_times), max(all_times)
                if time_min == time_max:
                    time_min = time_min - timedelta(seconds=30)
                    time_max = time_max + timedelta(seconds=30)
                ax_power.set_xlim(time_min, time_max)
                ax_power.set_ylim(0, max(all_values) * 1.1 if all_values else 100)
            
            # Update water level
            water_times, water_values = filter_data(timestamps, water_level_mm)
            line_water.set_data(water_times, water_values)
            
            if water_times:
                time_min, time_max = min(water_times), max(water_times)
                if time_min == time_max:
                    time_min = time_min - timedelta(seconds=30)
                    time_max = time_max + timedelta(seconds=30)
                ax_water.set_xlim(time_min, time_max)
                ax_water.set_ylim(0, max(water_values) * 1.2 if water_values else 100)
            
            # Redraw the canvas
            canvas_graph.draw()
            
        except Exception as e:
            logging.error(f"Failed to update graphs: {e}", exc_info=True)
    
    # Auto-refresh mechanism (every 5 seconds)
    auto_refresh_id = None
    auto_refresh_enabled = True  # Flag to track if auto-refresh should run
    
    def auto_refresh():
        """Automatically refresh sensor data every 5 seconds"""
        nonlocal auto_refresh_id, auto_refresh_enabled
        if auto_refresh_enabled:
            try:
                refresh_all()
                # Schedule next refresh in 5000ms (5 seconds)
                auto_refresh_id = scrollable_frame.after(5000, auto_refresh)
            except tk.TclError:
                # Widget was destroyed, stop the refresh
                auto_refresh_enabled = False
                auto_refresh_id = None
    
    def stop_auto_refresh():
        """Stop the auto-refresh timer"""
        nonlocal auto_refresh_id, auto_refresh_enabled
        auto_refresh_enabled = False
        if auto_refresh_id is not None:
            scrollable_frame.after_cancel(auto_refresh_id)
            auto_refresh_id = None
    
    def start_auto_refresh():
        """Start or restart the auto-refresh timer"""
        nonlocal auto_refresh_id, auto_refresh_enabled
        auto_refresh_enabled = True
        # Cancel any existing timer first
        if auto_refresh_id is not None:
            scrollable_frame.after_cancel(auto_refresh_id)
        # Refresh immediately and schedule next
        refresh_all()
        auto_refresh_id = scrollable_frame.after(5000, auto_refresh)
    
    # Store functions so they can be called when switching tabs
    parent_frame.stop_auto_refresh = stop_auto_refresh
    parent_frame.start_auto_refresh = start_auto_refresh
    
    # Periodic database sync (every 5 minutes to catch new historical data)
    sync_timer_id = None
    sync_enabled = True
    
    def periodic_sync():
        """Sync database with device every 5 minutes"""
        nonlocal sync_timer_id, sync_enabled
        if sync_enabled:
            try:
                logging.info("Performing periodic database sync...")
                success, new_entries, sync_message = sync_historical_data(console_instance, database)
                if success and new_entries > 0:
                    logging.info(f"[OK] Periodic sync: {new_entries} new entries added")
                    # Refresh graphs to show new data
                    update_graphs()
                # Schedule next sync in 5 minutes (300,000 ms)
                sync_timer_id = scrollable_frame.after(300000, periodic_sync)
            except tk.TclError:
                sync_enabled = False
                sync_timer_id = None
    
    def stop_periodic_sync():
        """Stop the periodic sync timer"""
        nonlocal sync_timer_id, sync_enabled
        sync_enabled = False
        if sync_timer_id is not None:
            scrollable_frame.after_cancel(sync_timer_id)
            sync_timer_id = None
    
    # Store sync stop function and database reference
    parent_frame.stop_periodic_sync = stop_periodic_sync
    parent_frame.database = database
    
    # Start periodic sync (first one in 5 minutes)
    sync_timer_id = scrollable_frame.after(300000, periodic_sync)
    
    # Initial load and start auto-refresh
    start_auto_refresh()


def create_schedules_tab(parent_frame, console_instance):
    """
    Create the schedules management tab.
    Embeds the existing unified schedule manager functionality.
    """
    # Instructions
    instructions = tk.Label(
        parent_frame,
        text="Manage hourly schedules, food dosing, and routine commands",
        font=("Arial", 10),
        fg="#555"
    )
    instructions.pack(pady=10)
    
    # Button to open schedule manager
    def open_schedule_manager():
        # Fetch current hourly schedules from device
        light_sched = console_instance._fetch_saved_schedule("light")
        planter_sched = console_instance._fetch_saved_schedule("planter")
        
        # Open the existing unified GUI window
        config = prompt_unified_schedule_manager(light_sched, planter_sched, console_instance.schedules)
        
        if config is None:
            return
        
        # Handle hourly schedules
        if config['light_schedule'] or config['planter_schedule']:
            # Post light schedule
            if config['light_schedule']:
                console_instance._post_routine("light_schedule", {"schedule": config['light_schedule']})
                print("✓ Light schedule updated")
            
            # Post planter schedule  
            if config['planter_schedule']:
                console_instance._post_routine("planter_pod_schedule", {"schedule": config['planter_schedule']})
                print("✓ Planter schedule updated")
        
        # Handle food schedule
        if config['food_config']:
            food = config['food_config']
            
            # Calculate feeding times (evenly spaced throughout 24 hours)
            hours_between = 24.0 / food['intervals']
            
            # Remove any existing food schedules
            food_schedules = [name for name in console_instance.schedules if name.startswith('food_dose_')]
            for name in food_schedules:
                try:
                    scheduler.remove_job(name)
                    del console_instance.schedules[name]
                except Exception:
                    pass
            
            # Create new food dosing schedules
            device_info = devices[console_instance.selected_device]
            device_name = console_instance.selected_device
            
            for i in range(food['intervals']):
                start_hour = int(i * hours_between)
                start_minute = int((i * hours_between - start_hour) * 60)
                schedule_name = f"food_dose_{i+1}"
                
                console_instance.schedules[schedule_name] = {
                    'device_name': device_name,
                    'device_ip': device_info['address'],
                    'start_time': f"{start_hour:02d}:{start_minute:02d}",
                    'duration_minutes': 0,
                    'frequency': 'daily',
                    'day_of_week': None,
                    'actions': {
                        'food_dose': {
                            'duration_ms': food['dose_per_interval'],
                            'speed': food['speed']
                        }
                    }
                }
                
                console_instance.schedule_job(schedule_name, console_instance.schedules[schedule_name])
            
            console_instance.save_schedules()
            tk.messagebox.showinfo("Success", f"Food dosing schedule created with {food['intervals']} daily feedings")
        
        # Handle routine command schedule
        if config['routine_config']:
            routine = config['routine_config']
            schedule_name = routine['schedule_name']
            
            device_info = devices[console_instance.selected_device]
            device_name = console_instance.selected_device
            
            # Create action based on routine command
            actions = {}
            command = routine['command']
            
            if command == 'empty_pod':
                actions['routine'] = {'command': 'empty_pod'}
            elif command == 'fill_pod':
                actions['routine'] = {'command': 'fill_pod'}
            elif command == 'calibrate_pod':
                actions['routine'] = {'command': 'calibrate_pod'}
            
            console_instance.schedules[schedule_name] = {
                'device_name': device_name,
                'device_ip': device_info['address'],
                'start_time': routine['start_time'],
                'duration_minutes': 0,
                'frequency': routine['frequency'],
                'day_of_week': routine['day_of_week'],
                'actions': actions
            }
            
            console_instance.schedule_job(schedule_name, console_instance.schedules[schedule_name])
            console_instance.save_schedules()
            tk.messagebox.showinfo("Success", f"Routine schedule '{schedule_name}' created")
    
    btn_frame = tk.Frame(parent_frame)
    btn_frame.pack(pady=20)
    
    open_btn = tk.Button(
        btn_frame,
        text="Open Schedule Manager",
        command=open_schedule_manager,
        font=("Arial", 12),
        bg="#3498db",
        fg="white",
        padx=20,
        pady=10
    )
    open_btn.pack()
    
    # Display existing schedules
    schedules_list_frame = tk.LabelFrame(parent_frame, text="Active Schedules", padx=10, pady=10)
    schedules_list_frame.pack(fill="both", expand=True, pady=10)
    
    # Scrollable text widget for schedules
    schedules_text = tk.Text(schedules_list_frame, height=15, wrap="none", font=("Courier", 9))
    schedules_text.pack(side="left", fill="both", expand=True)
    
    scrollbar = tk.Scrollbar(schedules_list_frame, command=schedules_text.yview)
    scrollbar.pack(side="right", fill="y")
    schedules_text.config(yscrollcommand=scrollbar.set)
    
    # Populate schedules
    def refresh_schedules():
        schedules_text.delete(1.0, tk.END)
        if not console_instance.schedules:
            schedules_text.insert(tk.END, "No active schedules.\n")
        else:
            for name, details in console_instance.schedules.items():
                schedules_text.insert(tk.END, f"📌 {name}\n")
                schedules_text.insert(tk.END, f"   Device: {details['device_name']}\n")
                schedules_text.insert(tk.END, f"   Time: {details['start_time']}\n")
                schedules_text.insert(tk.END, f"   Frequency: {details['frequency']}\n")
                schedules_text.insert(tk.END, f"   Actions: {', '.join(details['actions'].keys())}\n")
                schedules_text.insert(tk.END, "\n")
    
    refresh_schedules()
    
    refresh_btn = tk.Button(
        parent_frame,
        text="🔄 Refresh",
        command=refresh_schedules,
        font=("Arial", 10)
    )
    refresh_btn.pack(pady=5)


def create_light_planter_tab(parent_frame, console_instance):
    """
    Create the Light & Planter 24-hour schedule tab.
    Provides a more compact visualization than 24 horizontal sliders.
    Shows LED and Planter schedules side-by-side with scrolling support.
    """
    # Main container with scrollbar
    canvas = tk.Canvas(parent_frame)
    scrollbar = tk.Scrollbar(parent_frame, orient="vertical", command=canvas.yview)
    scrollable_frame = tk.Frame(canvas)
    
    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
    )
    
    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)
    
    # Enable mouse wheel scrolling for canvas only (not bind_all to avoid conflicts)
    def _on_mousewheel(event):
        canvas.yview_scroll(int(-1*(event.delta/120)), "units")
    
    # Bind to canvas and scrollable_frame, but NOT bind_all
    canvas.bind("<MouseWheel>", _on_mousewheel)
    scrollable_frame.bind("<MouseWheel>", _on_mousewheel)
    
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")
    
    # Title
    title_label = tk.Label(
        scrollable_frame,
        text="24-Hour Light & Planter Schedules",
        font=("Arial", 14, "bold")
    )
    title_label.pack(pady=10)
    
    instructions = tk.Label(
        scrollable_frame,
        text="Set hourly PWM values for LED brightness and planter pump operation (scroll to adjust sliders)",
        font=("Arial", 10),
        fg="#555"
    )
    instructions.pack(pady=5)
    
    # Current schedules from device
    current_light = [0] * 24
    current_planter = [0] * 24
    
    def load_current_schedules():
        nonlocal current_light, current_planter
        light_sched = console_instance._fetch_saved_schedule("light")
        planter_sched = console_instance._fetch_saved_schedule("planter")
        
        if light_sched:
            current_light = light_sched
        if planter_sched:
            current_planter = planter_sched
        
        update_displays()
    
    # Side-by-side container for LED and Planter
    schedules_container = tk.Frame(scrollable_frame)
    schedules_container.pack(fill="both", expand=True, padx=20, pady=10)
    
    # LED Schedule Section (LEFT SIDE)
    led_frame = tk.LabelFrame(schedules_container, text="💡 LED Light Schedule", padx=15, pady=15)
    led_frame.pack(side="left", fill="both", expand=True, padx=(0, 10))
    
    led_values_frame = tk.Frame(led_frame)
    led_values_frame.pack(fill="both", expand=True)
    
    led_scales = []
    led_value_labels = []
    
    # Create compact grid of sliders (4 rows of 6)
    for row in range(4):
        row_frame = tk.Frame(led_values_frame)
        row_frame.pack(fill="x", pady=2)
        
        for col in range(6):
            hour = row * 6 + col
            
            hour_container = tk.Frame(row_frame)
            hour_container.pack(side="left", padx=5)
            
            hour_label = tk.Label(hour_container, text=f"{hour:02d}h", font=("Arial", 8, "bold"))
            hour_label.pack()
            
            scale = tk.Scale(
                hour_container,
                from_=100,
                to=0,
                orient=tk.VERTICAL,
                length=100,
                width=18,
                showvalue=0
            )
            scale.set(current_light[hour])
            scale.pack()
            led_scales.append(scale)
            
            value_label = tk.Label(hour_container, text=f"{current_light[hour]}%", font=("Arial", 8))
            value_label.pack()
            led_value_labels.append(value_label)
            
            # Update label when slider moves
            scale.config(command=lambda val, h=hour: led_value_labels[h].config(text=f"{int(float(val))}%"))
            
            # Add mouse wheel scrolling for this slider
            def make_scroll_handler(slider, hour_idx):
                def on_scroll(event):
                    current_val = slider.get()
                    # Scroll up = increase value, scroll down = decrease value
                    delta = 5 if event.delta > 0 else -5
                    new_val = max(0, min(100, current_val + delta))
                    slider.set(new_val)
                    led_value_labels[hour_idx].config(text=f"{int(new_val)}%")
                    return "break"  # Stop event propagation to prevent canvas scrolling
                return on_scroll
            
            scale.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            # Also bind to the container for easier targeting
            hour_container.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            # Bind to label too
            hour_label.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            value_label.bind("<MouseWheel>", make_scroll_handler(scale, hour))
    
    # Planter Schedule Section (RIGHT SIDE)
    planter_frame = tk.LabelFrame(schedules_container, text="💧 Planter Pump Schedule", padx=15, pady=15)
    planter_frame.pack(side="left", fill="both", expand=True, padx=(10, 0))
    
    planter_values_frame = tk.Frame(planter_frame)
    planter_values_frame.pack(fill="both", expand=True)
    
    planter_scales = []
    planter_value_labels = []
    
    # Create compact grid of sliders (4 rows of 6)
    for row in range(4):
        row_frame = tk.Frame(planter_values_frame)
        row_frame.pack(fill="x", pady=2)
        
        for col in range(6):
            hour = row * 6 + col
            
            hour_container = tk.Frame(row_frame)
            hour_container.pack(side="left", padx=5)
            
            hour_label = tk.Label(hour_container, text=f"{hour:02d}h", font=("Arial", 8, "bold"))
            hour_label.pack()
            
            scale = tk.Scale(
                hour_container,
                from_=100,
                to=0,
                orient=tk.VERTICAL,
                length=100,
                width=18,
                showvalue=0
            )
            scale.set(current_planter[hour])
            scale.pack()
            planter_scales.append(scale)
            
            value_label = tk.Label(hour_container, text=f"{current_planter[hour]}%", font=("Arial", 8))
            value_label.pack()
            planter_value_labels.append(value_label)
            
            scale.config(command=lambda val, h=hour: planter_value_labels[h].config(text=f"{int(float(val))}%"))
            
            # Add mouse wheel scrolling for this slider
            def make_scroll_handler(slider, hour_idx):
                def on_scroll(event):
                    current_val = slider.get()
                    # Scroll up = increase value, scroll down = decrease value
                    delta = 5 if event.delta > 0 else -5
                    new_val = max(0, min(100, current_val + delta))
                    slider.set(new_val)
                    planter_value_labels[hour_idx].config(text=f"{int(new_val)}%")
                    return "break"  # Stop event propagation to prevent canvas scrolling
                return on_scroll
            
            scale.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            # Also bind to the container for easier targeting
            hour_container.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            # Bind to label too
            hour_label.bind("<MouseWheel>", make_scroll_handler(scale, hour))
            value_label.bind("<MouseWheel>", make_scroll_handler(scale, hour))
    
    def update_displays():
        for i in range(24):
            led_scales[i].set(current_light[i])
            led_value_labels[i].config(text=f"{current_light[i]}%")
            planter_scales[i].set(current_planter[i])
            planter_value_labels[i].config(text=f"{current_planter[i]}%")
    
    # Preset buttons
    preset_frame = tk.Frame(scrollable_frame)
    preset_frame.pack(pady=15)
    
    def apply_preset_light(preset_type):
        if preset_type == "off":
            for i in range(24):
                led_scales[i].set(0)
        elif preset_type == "full":
            for i in range(24):
                led_scales[i].set(100)
        elif preset_type == "day_night":
            # Sunrise 6am, sunset 8pm
            for i in range(24):
                if 6 <= i < 20:
                    led_scales[i].set(100)
                else:
                    led_scales[i].set(0)
        elif preset_type == "gradual":
            # Gradual sunrise/sunset
            schedule = [0, 0, 0, 0, 0, 10, 30, 60, 80, 100, 100, 100,
                       100, 100, 100, 100, 100, 100, 80, 60, 30, 10, 0, 0]
            for i in range(24):
                led_scales[i].set(schedule[i])
    
    tk.Label(preset_frame, text="LED Presets:", font=("Arial", 10, "bold")).pack(side="left", padx=5)
    tk.Button(preset_frame, text="All Off", command=lambda: apply_preset_light("off"), padx=10, pady=5).pack(side="left", padx=2)
    tk.Button(preset_frame, text="All On", command=lambda: apply_preset_light("full"), padx=10, pady=5).pack(side="left", padx=2)
    tk.Button(preset_frame, text="Day/Night", command=lambda: apply_preset_light("day_night"), padx=10, pady=5).pack(side="left", padx=2)
    tk.Button(preset_frame, text="Gradual", command=lambda: apply_preset_light("gradual"), padx=10, pady=5).pack(side="left", padx=2)
    
    # Action buttons
    action_frame = tk.Frame(scrollable_frame)
    action_frame.pack(pady=20)
    
    def load_schedules():
        load_current_schedules()
        tk.messagebox.showinfo("Success", "Schedules loaded from device")
    
    def save_schedules():
        light_schedule = [led_scales[i].get() for i in range(24)]
        planter_schedule = [planter_scales[i].get() for i in range(24)]
        
        # Post to device
        console_instance._post_routine("light_schedule", {"schedule": light_schedule})
        console_instance._post_routine("planter_pod_schedule", {"schedule": planter_schedule})
        
        tk.messagebox.showinfo("Success", "Schedules saved to device!")
    
    tk.Button(
        action_frame,
        text="🔄 Load from Device",
        command=load_schedules,
        font=("Arial", 11),
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    tk.Button(
        action_frame,
        text="💾 Save to Device",
        command=save_schedules,
        font=("Arial", 11, "bold"),
        bg="#27ae60",
        fg="white",
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    # Initial load
    load_current_schedules()


def create_food_schedule_tab(parent_frame, console_instance):
    """
    Create the Food Schedule tab.
    Configure automated feeding times and doses.
    """
    # Main container
    container = tk.Frame(parent_frame, padx=30, pady=30)
    container.pack(fill="both", expand=True)
    
    # Title
    title_label = tk.Label(
        container,
        text="🍽️ Automated Food Dosing Schedule",
        font=("Arial", 14, "bold")
    )
    title_label.pack(pady=10)
    
    instructions = tk.Label(
        container,
        text="Configure automated nutrient dosing throughout the day",
        font=("Arial", 10),
        fg="#555"
    )
    instructions.pack(pady=5)
    
    # Configuration frame
    config_frame = tk.LabelFrame(container, text="Dosing Configuration", padx=20, pady=20)
    config_frame.pack(fill="x", pady=20)
    
    # Total daily amount
    tk.Label(config_frame, text="Total Daily Amount (ms):", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=5, pady=10)
    total_entry = tk.Entry(config_frame, font=("Arial", 11), width=15)
    total_entry.insert(0, "5000")
    total_entry.grid(row=0, column=1, sticky="w", padx=5, pady=10)
    tk.Label(config_frame, text="Total pump run time per day", font=("Arial", 9), fg="#666").grid(row=0, column=2, sticky="w", padx=5)
    
    # Number of intervals
    tk.Label(config_frame, text="Number of Feedings:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", padx=5, pady=10)
    intervals_spinbox = tk.Spinbox(config_frame, from_=1, to=24, font=("Arial", 11), width=13)
    intervals_spinbox.delete(0, tk.END)
    intervals_spinbox.insert(0, "4")
    intervals_spinbox.grid(row=1, column=1, sticky="w", padx=5, pady=10)
    tk.Label(config_frame, text="Evenly distributed throughout day", font=("Arial", 9), fg="#666").grid(row=1, column=2, sticky="w", padx=5)
    
    # Pump speed
    tk.Label(config_frame, text="Pump Speed (%):", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", padx=5, pady=10)
    speed_scale = tk.Scale(config_frame, from_=0, to=100, orient=tk.HORIZONTAL, length=200)
    speed_scale.set(100)
    speed_scale.grid(row=2, column=1, columnspan=2, sticky="w", padx=5, pady=10)
    
    # Preview frame
    preview_frame = tk.LabelFrame(container, text="Schedule Preview", padx=20, pady=20)
    preview_frame.pack(fill="both", expand=True, pady=20)
    
    preview_text = tk.Text(preview_frame, height=12, font=("Courier", 10), wrap="none")
    preview_text.pack(side="left", fill="both", expand=True)
    
    preview_scrollbar = tk.Scrollbar(preview_frame, command=preview_text.yview)
    preview_scrollbar.pack(side="right", fill="y")
    preview_text.config(yscrollcommand=preview_scrollbar.set)
    
    def update_preview():
        try:
            total_ms = int(total_entry.get())
            intervals = int(intervals_spinbox.get())
            speed = speed_scale.get()
            
            dose_per_interval = total_ms // intervals
            hours_between = 24.0 / intervals
            
            preview_text.delete(1.0, tk.END)
            preview_text.insert(tk.END, f"Configuration Summary\n")
            preview_text.insert(tk.END, f"=" * 60 + "\n\n")
            preview_text.insert(tk.END, f"Total Daily Dose:     {total_ms} ms\n")
            preview_text.insert(tk.END, f"Number of Feedings:   {intervals}\n")
            preview_text.insert(tk.END, f"Dose per Feeding:     {dose_per_interval} ms\n")
            preview_text.insert(tk.END, f"Pump Speed:           {speed}%\n\n")
            preview_text.insert(tk.END, f"Feeding Times\n")
            preview_text.insert(tk.END, f"-" * 60 + "\n\n")
            
            for i in range(intervals):
                start_hour = int(i * hours_between)
                start_minute = int((i * hours_between - start_hour) * 60)
                preview_text.insert(tk.END, f"Feeding {i+1:2d}:  {start_hour:02d}:{start_minute:02d}  ({dose_per_interval} ms @ {speed}%)\n")
        
        except ValueError:
            preview_text.delete(1.0, tk.END)
            preview_text.insert(tk.END, "⚠️ Invalid input. Please enter valid numbers.")
    
    # Update preview when values change
    total_entry.bind("<KeyRelease>", lambda e: update_preview())
    intervals_spinbox.bind("<ButtonRelease-1>", lambda e: update_preview())
    intervals_spinbox.bind("<KeyRelease>", lambda e: update_preview())
    speed_scale.config(command=lambda v: update_preview())
    
    # Action buttons
    action_frame = tk.Frame(container)
    action_frame.pack(pady=20)
    
    def apply_schedule():
        try:
            total_ms = int(total_entry.get())
            intervals = int(intervals_spinbox.get())
            speed = speed_scale.get()
            
            if total_ms <= 0 or intervals <= 0:
                tk.messagebox.showerror("Error", "Values must be greater than 0")
                return
            
            # Remove existing food schedules
            food_schedules = [name for name in console_instance.schedules if name.startswith('food_dose_')]
            for name in food_schedules:
                try:
                    scheduler.remove_job(name)
                    del console_instance.schedules[name]
                except Exception:
                    pass
            
            # Create new schedules
            dose_per_interval = total_ms // intervals
            hours_between = 24.0 / intervals
            device_info = devices[console_instance.selected_device]
            
            for i in range(intervals):
                start_hour = int(i * hours_between)
                start_minute = int((i * hours_between - start_hour) * 60)
                schedule_name = f"food_dose_{i+1}"
                
                console_instance.schedules[schedule_name] = {
                    'device_name': console_instance.selected_device,
                    'device_ip': device_info['address'],
                    'start_time': f"{start_hour:02d}:{start_minute:02d}",
                    'duration_minutes': 0,
                    'frequency': 'daily',
                    'day_of_week': None,
                    'actions': {
                        'food_dose': {
                            'duration_ms': dose_per_interval,
                            'speed': speed
                        }
                    }
                }
                
                console_instance.schedule_job(schedule_name, console_instance.schedules[schedule_name])
            
            console_instance.save_schedules()
            tk.messagebox.showinfo("Success", f"Food schedule created with {intervals} daily feedings!")
            
        except ValueError:
            tk.messagebox.showerror("Error", "Please enter valid numbers")
    
    tk.Button(
        action_frame,
        text="🔄 Update Preview",
        command=update_preview,
        font=("Arial", 11),
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    tk.Button(
        action_frame,
        text="✅ Apply Schedule",
        command=apply_schedule,
        font=("Arial", 11, "bold"),
        bg="#27ae60",
        fg="white",
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    # Initial preview
    update_preview()


def create_routine_calendar_tab(parent_frame, console_instance):
    """
    Create the Routine Calendar tab.
    Google Calendar-style view for scheduling fill/empty/maintenance tasks.
    """
    # Main container
    container = tk.Frame(parent_frame, padx=20, pady=20)
    container.pack(fill="both", expand=True)
    
    # Title
    title_label = tk.Label(
        container,
        text="📆 Routine Command Scheduler",
        font=("Arial", 14, "bold")
    )
    title_label.pack(pady=10)
    
    instructions = tk.Label(
        container,
        text="Schedule maintenance routines like filling, emptying, and calibration",
        font=("Arial", 10),
        fg="#555"
    )
    instructions.pack(pady=5)
    
    # Split view: left = calendar/schedule list, right = create/edit form
    paned = tk.PanedWindow(container, orient=tk.HORIZONTAL, sashrelief=tk.RAISED)
    paned.pack(fill="both", expand=True, pady=10)
    
    # Left panel - Schedule list
    left_panel = tk.Frame(paned)
    paned.add(left_panel, minsize=400)
    
    list_label = tk.Label(left_panel, text="Scheduled Routines", font=("Arial", 11, "bold"))
    list_label.pack(pady=5)
    
    # Filter buttons
    filter_frame = tk.Frame(left_panel)
    filter_frame.pack(fill="x", pady=5)
    
    filter_var = tk.StringVar(value="all")
    
    tk.Radiobutton(filter_frame, text="All", variable=filter_var, value="all").pack(side="left", padx=5)
    tk.Radiobutton(filter_frame, text="Daily", variable=filter_var, value="daily").pack(side="left", padx=5)
    tk.Radiobutton(filter_frame, text="Weekly", variable=filter_var, value="weekly").pack(side="left", padx=5)
    
    # Schedule list with scrollbar
    list_frame = tk.Frame(left_panel)
    list_frame.pack(fill="both", expand=True)
    
    schedule_listbox = tk.Listbox(list_frame, font=("Courier", 9), selectmode=tk.SINGLE)
    schedule_listbox.pack(side="left", fill="both", expand=True)
    
    list_scrollbar = tk.Scrollbar(list_frame, command=schedule_listbox.yview)
    list_scrollbar.pack(side="right", fill="y")
    schedule_listbox.config(yscrollcommand=list_scrollbar.set)
    
    # Right panel - Create/Edit form
    right_panel = tk.Frame(paned)
    paned.add(right_panel, minsize=400)
    
    form_label = tk.Label(right_panel, text="Create New Routine", font=("Arial", 11, "bold"))
    form_label.pack(pady=5)
    
    form_frame = tk.LabelFrame(right_panel, text="Routine Details", padx=15, pady=15)
    form_frame.pack(fill="x", pady=10)
    
    # Schedule name
    tk.Label(form_frame, text="Schedule Name:", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", pady=5)
    name_entry = tk.Entry(form_frame, font=("Arial", 10), width=25)
    name_entry.grid(row=0, column=1, sticky="w", pady=5, padx=5)
    
    # Routine command
    tk.Label(form_frame, text="Routine Command:", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", pady=5)
    command_var = tk.StringVar(value="fill_pod")
    command_frame = tk.Frame(form_frame)
    command_frame.grid(row=1, column=1, sticky="w", pady=5, padx=5)
    
    commands = [
        ("Fill Pod", "fill_pod", "#3498db"),
        ("Empty Pod", "empty_pod", "#e67e22"),
        ("Calibrate", "calibrate_pod", "#9b59b6")
    ]
    
    for i, (label, value, color) in enumerate(commands):
        rb = tk.Radiobutton(
            command_frame,
            text=label,
            variable=command_var,
            value=value,
            font=("Arial", 9)
        )
        rb.pack(anchor="w")
    
    # Start time
    tk.Label(form_frame, text="Start Time (HH:MM):", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", pady=5)
    time_entry = tk.Entry(form_frame, font=("Arial", 10), width=10)
    time_entry.insert(0, "09:00")
    time_entry.grid(row=2, column=1, sticky="w", pady=5, padx=5)
    
    # Frequency
    tk.Label(form_frame, text="Frequency:", font=("Arial", 10, "bold")).grid(row=3, column=0, sticky="w", pady=5)
    freq_var = tk.StringVar(value="daily")
    freq_frame = tk.Frame(form_frame)
    freq_frame.grid(row=3, column=1, sticky="w", pady=5, padx=5)
    
    tk.Radiobutton(freq_frame, text="Daily", variable=freq_var, value="daily", command=lambda: day_dropdown.config(state="disabled")).pack(anchor="w")
    tk.Radiobutton(freq_frame, text="Weekly", variable=freq_var, value="weekly", command=lambda: day_dropdown.config(state="normal")).pack(anchor="w")
    
    # Day of week (for weekly)
    tk.Label(form_frame, text="Day of Week:", font=("Arial", 10, "bold")).grid(row=4, column=0, sticky="w", pady=5)
    day_var = tk.StringVar(value="Monday")
    day_dropdown = ttk.Combobox(
        form_frame,
        textvariable=day_var,
        values=["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"],
        state="disabled",
        width=15
    )
    day_dropdown.grid(row=4, column=1, sticky="w", pady=5, padx=5)
    
    # Visual preview
    preview_frame = tk.LabelFrame(right_panel, text="Schedule Preview", padx=15, pady=15)
    preview_frame.pack(fill="x", pady=10)
    
    preview_label = tk.Label(preview_frame, text="", font=("Arial", 10), justify="left", fg="#2c3e50")
    preview_label.pack()
    
    def update_preview(*args):
        name = name_entry.get() or "(unnamed)"
        cmd = command_var.get()
        time = time_entry.get()
        freq = freq_var.get()
        day = day_var.get() if freq == "weekly" else "N/A"
        
        cmd_display = {"fill_pod": "Fill Pod 💧", "empty_pod": "Empty Pod 🚰", "calibrate_pod": "Calibrate ⚙️"}
        
        preview_text = f"📋 {name}\n"
        preview_text += f"🔧 Command: {cmd_display.get(cmd, cmd)}\n"
        preview_text += f"⏰ Time: {time}\n"
        preview_text += f"🔄 Frequency: {freq.title()}\n"
        if freq == "weekly":
            preview_text += f"📅 Day: {day}\n"
        
        preview_label.config(text=preview_text)
    
    # Bind updates
    name_entry.bind("<KeyRelease>", update_preview)
    command_var.trace("w", update_preview)
    time_entry.bind("<KeyRelease>", update_preview)
    freq_var.trace("w", update_preview)
    day_var.trace("w", update_preview)
    
    # Action buttons
    action_frame = tk.Frame(right_panel)
    action_frame.pack(pady=15)
    
    def create_schedule():
        name = name_entry.get().strip()
        if not name:
            tk.messagebox.showerror("Error", "Please enter a schedule name")
            return
        
        time_str = time_entry.get().strip()
        try:
            datetime.strptime(time_str, "%H:%M")
        except ValueError:
            tk.messagebox.showerror("Error", "Invalid time format. Use HH:MM")
            return
        
        freq = freq_var.get()
        day = day_var.get() if freq == "weekly" else None
        cmd = command_var.get()
        
        # Create schedule
        device_info = devices[console_instance.selected_device]
        actions = {'routine': {'command': cmd}}
        
        console_instance.schedules[name] = {
            'device_name': console_instance.selected_device,
            'device_ip': device_info['address'],
            'start_time': time_str,
            'duration_minutes': 0,
            'frequency': freq,
            'day_of_week': day,
            'actions': actions
        }
        
        console_instance.schedule_job(name, console_instance.schedules[name])
        console_instance.save_schedules()
        
        tk.messagebox.showinfo("Success", f"Routine '{name}' scheduled!")
        refresh_list()
        
        # Clear form
        name_entry.delete(0, tk.END)
        time_entry.delete(0, tk.END)
        time_entry.insert(0, "09:00")
    
    def delete_selected():
        selection = schedule_listbox.curselection()
        if not selection:
            tk.messagebox.showwarning("No Selection", "Please select a routine to delete")
            return
        
        # Extract schedule name from listbox item
        item = schedule_listbox.get(selection[0])
        # Parse name from format "📋 name | ..."
        name = item.split("|")[0].strip().replace("📋 ", "")
        
        if name in console_instance.schedules:
            try:
                scheduler.remove_job(name)
                del console_instance.schedules[name]
                console_instance.save_schedules()
                tk.messagebox.showinfo("Success", f"Routine '{name}' deleted")
                refresh_list()
            except Exception as e:
                tk.messagebox.showerror("Error", f"Failed to delete: {e}")
    
    def refresh_list():
        schedule_listbox.delete(0, tk.END)
        filter_val = filter_var.get()
        
        routine_schedules = {
            name: details for name, details in console_instance.schedules.items()
            if 'routine' in details.get('actions', {})
        }
        
        if not routine_schedules:
            schedule_listbox.insert(tk.END, "No routine schedules found")
            return
        
        for name, details in routine_schedules.items():
            freq = details.get('frequency', 'unknown')
            
            # Apply filter
            if filter_val != "all" and freq != filter_val:
                continue
            
            time = details.get('start_time', '??:??')
            cmd = details.get('actions', {}).get('routine', {}).get('command', 'unknown')
            day = details.get('day_of_week', '')
            
            cmd_icon = {"fill_pod": "💧", "empty_pod": "🚰", "calibrate_pod": "⚙️"}.get(cmd, "❓")
            freq_icon = {"daily": "🔄", "weekly": "📅"}.get(freq, "❓")
            
            item = f"📋 {name} | {cmd_icon} {cmd} | {freq_icon} {freq.title()}"
            if day:
                item += f" ({day})"
            item += f" | ⏰ {time}"
            
            schedule_listbox.insert(tk.END, item)
    
    tk.Button(
        action_frame,
        text="✅ Create Schedule",
        command=create_schedule,
        font=("Arial", 11, "bold"),
        bg="#27ae60",
        fg="white",
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    tk.Button(
        action_frame,
        text="🗑️ Delete Selected",
        command=delete_selected,
        font=("Arial", 11),
        bg="#e74c3c",
        fg="white",
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    tk.Button(
        action_frame,
        text="🔄 Refresh",
        command=refresh_list,
        font=("Arial", 11),
        padx=15,
        pady=8
    ).pack(side="left", padx=5)
    
    # Initial load
    filter_var.trace("w", lambda *args: refresh_list())
    refresh_list()
    update_preview()


def create_filesystem_tab(parent_frame, console_instance):
    """
    Create the filesystem browser tab.
    Allows browsing device filesystem, viewing files, and navigation.
    """
    # Split pane: left = directory/file list, right = content viewer
    paned = tk.PanedWindow(parent_frame, orient=tk.HORIZONTAL, sashrelief=tk.RAISED)
    paned.pack(fill="both", expand=True)
    
    # Left panel - Directory browser
    left_frame = tk.Frame(paned)
    paned.add(left_frame, minsize=300)
    
    # Control buttons
    btn_frame = tk.Frame(left_frame)
    btn_frame.pack(fill="x", pady=5)
    
    current_path = tk.StringVar(value="/lfs")
    
    def refresh_listing():
        result = console_instance._get_filesystem_listing(current_path.get())
        if not result:
            tk.messagebox.showerror("Error", f"Failed to list directory: {current_path.get()}")
            return
        
        # Clear list
        file_list.delete(0, tk.END)
        
        # Add parent directory option if not at root
        if current_path.get() != "/lfs":
            file_list.insert(tk.END, "📁 ..")
        
        # Add directories
        for dir_name in result.get('directories', []):
            file_list.insert(tk.END, f"📁 {dir_name}")
        
        # Add files
        for file_info in result.get('files', []):
            file_list.insert(tk.END, f"📄 {file_info['name']} ({file_info['size']} bytes)")
        
        path_label.config(text=f"Path: {current_path.get()}")
        status_label.config(text=f"✓ Listed {result['total_dirs']} directories, {result['total_files']} files")
    
    def on_item_double_click(event):
        selection = file_list.curselection()
        if not selection:
            return
        
        item = file_list.get(selection[0])
        
        # Handle parent directory
        if item == "📁 ..":
            # Go up one level
            parts = current_path.get().rstrip('/').split('/')
            if len(parts) > 2:  # Don't go above /lfs
                current_path.set('/'.join(parts[:-1]))
            else:
                current_path.set("/lfs")
            refresh_listing()
            return
        
        # Handle directory
        if item.startswith("📁"):
            dir_name = item.split(" ", 1)[1]
            new_path = f"{current_path.get()}/{dir_name}".replace("//", "/")
            current_path.set(new_path)
            refresh_listing()
            return
        
        # Handle file
        if item.startswith("📄"):
            file_name = item.split(" ", 1)[1].split(" (")[0]
            file_path = f"{current_path.get()}/{file_name}".replace("//", "/")
            
            # Read file content
            result = console_instance._read_file_content(file_path)
            if not result:
                tk.messagebox.showerror("Error", f"Failed to read file: {file_path}")
                return
            
            # Display in right panel
            content_text.delete(1.0, tk.END)
            content = result.get('content', '')
            
            # Try to pretty-print JSON
            try:
                parsed = json.loads(content)
                content = json.dumps(parsed, indent=2)
            except:
                pass
            
            content_text.insert(tk.END, content)
            content_path_label.config(text=f"File: {file_path} ({result['size']} bytes)")
            status_label.config(text=f"✓ Loaded file: {file_name}")
    
    refresh_btn = tk.Button(btn_frame, text="🔄 Refresh", command=refresh_listing, width=10)
    refresh_btn.pack(side="left", padx=2)
    
    root_btn = tk.Button(btn_frame, text="🏠 Root", command=lambda: [current_path.set("/lfs"), refresh_listing()], width=10)
    root_btn.pack(side="left", padx=2)
    
    config_btn = tk.Button(btn_frame, text="⚙️ Config", command=lambda: [current_path.set("/lfs/config"), refresh_listing()], width=10)
    config_btn.pack(side="left", padx=2)
    
    # Path label
    path_label = tk.Label(left_frame, text=f"Path: {current_path.get()}", font=("Arial", 10, "bold"))
    path_label.pack(fill="x", pady=5)
    
    # File list
    list_frame = tk.Frame(left_frame)
    list_frame.pack(fill="both", expand=True)
    
    file_list = tk.Listbox(list_frame, font=("Courier", 10))
    file_list.pack(side="left", fill="both", expand=True)
    file_list.bind("<Double-Button-1>", on_item_double_click)
    
    list_scrollbar = tk.Scrollbar(list_frame, command=file_list.yview)
    list_scrollbar.pack(side="right", fill="y")
    file_list.config(yscrollcommand=list_scrollbar.set)
    
    # Right panel - Content viewer
    right_frame = tk.Frame(paned)
    paned.add(right_frame, minsize=400)
    
    content_path_label = tk.Label(right_frame, text="File: (select a file)", font=("Arial", 10, "bold"))
    content_path_label.pack(fill="x", pady=5)
    
    content_frame = tk.Frame(right_frame)
    content_frame.pack(fill="both", expand=True)
    
    content_text = tk.Text(content_frame, wrap="none", font=("Courier", 9))
    content_text.pack(side="left", fill="both", expand=True)
    
    content_scrollbar_y = tk.Scrollbar(content_frame, command=content_text.yview)
    content_scrollbar_y.pack(side="right", fill="y")
    content_text.config(yscrollcommand=content_scrollbar_y.set)
    
    content_scrollbar_x = tk.Scrollbar(right_frame, orient=tk.HORIZONTAL, command=content_text.xview)
    content_scrollbar_x.pack(fill="x")
    content_text.config(xscrollcommand=content_scrollbar_x.set)
    
    # Status bar
    status_label = tk.Label(parent_frame, text="Ready", relief=tk.SUNKEN, anchor="w")
    status_label.pack(side="bottom", fill="x")
    
    # Initial load
    refresh_listing()


def create_plant_info_tab(parent_frame, console_instance):
    """
    Create the plant info management tab.
    Allows viewing and editing plant information.
    """
    # Main container
    container = tk.Frame(parent_frame, padx=20, pady=20)
    container.pack(fill="both", expand=True)
    
    # Display section
    display_frame = tk.LabelFrame(container, text="Current Plant Information", padx=15, pady=15)
    display_frame.pack(fill="x", pady=(0, 20))
    
    info_labels = {}
    
    def refresh_plant_info():
        result = console_instance._get_plant_info()
        
        if not result or not result.get('exists'):
            for key in info_labels:
                info_labels[key].config(text="(not set)")
            status_label.config(text="ℹ️ No plant information found", fg="#e67e22")
            return
        
        info_labels['name'].config(text=result.get('plant_name', 'Unknown'))
        info_labels['date'].config(text=result.get('start_date', 'Unknown'))
        info_labels['timestamp'].config(text=str(result.get('start_timestamp', 'Unknown')))
        info_labels['days'].config(text=str(result.get('days_growing', 0)))
        
        status_label.config(text="✅ Plant information loaded", fg="#27ae60")
    
    # Info display grid
    fields = [
        ("Plant Name:", "name"),
        ("Start Date:", "date"),
        ("Start Timestamp:", "timestamp"),
        ("Days Growing:", "days")
    ]
    
    for i, (label_text, key) in enumerate(fields):
        tk.Label(display_frame, text=label_text, font=("Arial", 10, "bold"), anchor="w").grid(row=i, column=0, sticky="w", padx=5, pady=5)
        info_labels[key] = tk.Label(display_frame, text="(not set)", font=("Arial", 10), anchor="w", fg="#555")
        info_labels[key].grid(row=i, column=1, sticky="w", padx=5, pady=5)
    
    # Edit section
    edit_frame = tk.LabelFrame(container, text="Update Plant Information", padx=15, pady=15)
    edit_frame.pack(fill="x", pady=(0, 20))
    
    tk.Label(edit_frame, text="Plant Name:", font=("Arial", 10)).grid(row=0, column=0, sticky="w", padx=5, pady=5)
    name_entry = tk.Entry(edit_frame, font=("Arial", 10), width=30)
    name_entry.grid(row=0, column=1, sticky="w", padx=5, pady=5)
    
    tk.Label(edit_frame, text="Start Date (YYYY-MM-DD):", font=("Arial", 10)).grid(row=1, column=0, sticky="w", padx=5, pady=5)
    date_entry = tk.Entry(edit_frame, font=("Arial", 10), width=30)
    date_entry.grid(row=1, column=1, sticky="w", padx=5, pady=5)
    
    # Date preset buttons
    preset_frame = tk.Frame(edit_frame)
    preset_frame.grid(row=1, column=2, padx=5)
    
    def set_today():
        date_entry.delete(0, tk.END)
        date_entry.insert(0, datetime.now().strftime("%Y-%m-%d"))
    
    def set_week_ago():
        date_entry.delete(0, tk.END)
        week_ago = datetime.now() - timedelta(days=7)
        date_entry.insert(0, week_ago.strftime("%Y-%m-%d"))
    
    tk.Button(preset_frame, text="Today", command=set_today, width=8).pack(side="left", padx=2)
    tk.Button(preset_frame, text="1 Week Ago", command=set_week_ago, width=12).pack(side="left", padx=2)
    
    # Save button
    def save_plant_info():
        name = name_entry.get().strip()
        date = date_entry.get().strip()
        
        if not name:
            tk.messagebox.showerror("Error", "Plant name cannot be empty")
            return
        
        if not date:
            tk.messagebox.showerror("Error", "Start date cannot be empty")
            return
        
        # Validate date format
        try:
            datetime.strptime(date, "%Y-%m-%d")
        except ValueError:
            tk.messagebox.showerror("Error", "Invalid date format. Use YYYY-MM-DD")
            return
        
        # Save to device
        result = console_instance._set_plant_info(name, date)
        
        if result:
            status_label.config(text="✅ Plant information saved successfully!", fg="#27ae60")
            refresh_plant_info()
            tk.messagebox.showinfo("Success", "Plant information saved!")
        else:
            status_label.config(text="❌ Failed to save plant information", fg="#e74c3c")
            tk.messagebox.showerror("Error", "Failed to save plant information")
    
    save_btn = tk.Button(
        edit_frame,
        text="💾 Save",
        command=save_plant_info,
        font=("Arial", 11, "bold"),
        bg="#27ae60",
        fg="white",
        padx=20,
        pady=5
    )
    save_btn.grid(row=2, column=1, sticky="w", padx=5, pady=15)
    
    # Action buttons
    action_frame = tk.Frame(container)
    action_frame.pack(fill="x")
    
    refresh_btn = tk.Button(
        action_frame,
        text="🔄 Refresh",
        command=refresh_plant_info,
        font=("Arial", 10),
        padx=15,
        pady=5
    )
    refresh_btn.pack(side="left", padx=5)
    
    # Status bar
    status_label = tk.Label(container, text="Ready", font=("Arial", 10), fg="#555")
    status_label.pack(side="bottom", fill="x", pady=(10, 0))
    
    # Initial load
    refresh_plant_info()


# =============================================================================
# Phase 1 GUI Test Windows - Filesystem Browser and Plant Info
# =============================================================================

def test_filesystem_browser_gui(console_instance):
    """
    Standalone GUI test window for filesystem browser.
    Shows directory tree, file list, and file content viewer.
    Can be integrated as a tab in the main GUI later.
    """
    root = tk.Tk()
    root.title("Filesystem Browser - Test Window")
    root.geometry("900x600")
    
    # Main container
    main_frame = tk.Frame(root, padx=10, pady=10)
    main_frame.pack(fill="both", expand=True)
    
    # Title
    title_label = tk.Label(main_frame, text="📁 Device Filesystem Browser", 
                          font=("Arial", 16, "bold"))
    title_label.pack(pady=(0, 10))
    
    # Current path display
    path_frame = tk.Frame(main_frame)
    path_frame.pack(fill="x", pady=(0, 10))
    
    tk.Label(path_frame, text="Current Path:", font=("Arial", 10, "bold")).pack(side="left", padx=(0, 5))
    current_path = tk.StringVar(value="/lfs")
    path_label = tk.Label(path_frame, textvariable=current_path, font=("Arial", 10), fg="#0066cc")
    path_label.pack(side="left")
    
    # Split view: left = directory tree, right = file content
    split_frame = tk.Frame(main_frame)
    split_frame.pack(fill="both", expand=True)
    
    # Left panel - Directory and file listing
    left_panel = tk.Frame(split_frame, width=400)
    left_panel.pack(side="left", fill="both", expand=True, padx=(0, 5))
    
    tk.Label(left_panel, text="Directories & Files", font=("Arial", 11, "bold")).pack(anchor="w")
    
    # Listbox for directories and files
    list_frame = tk.Frame(left_panel)
    list_frame.pack(fill="both", expand=True, pady=(5, 0))
    
    scrollbar = tk.Scrollbar(list_frame)
    scrollbar.pack(side="right", fill="y")
    
    items_listbox = tk.Listbox(list_frame, yscrollcommand=scrollbar.set, font=("Courier", 10))
    items_listbox.pack(side="left", fill="both", expand=True)
    scrollbar.config(command=items_listbox.yview)
    
    # Right panel - File content viewer
    right_panel = tk.Frame(split_frame, width=400)
    right_panel.pack(side="right", fill="both", expand=True, padx=(5, 0))
    
    tk.Label(right_panel, text="File Content", font=("Arial", 11, "bold")).pack(anchor="w")
    
    # Text widget for file content
    content_frame = tk.Frame(right_panel)
    content_frame.pack(fill="both", expand=True, pady=(5, 0))
    
    content_scrollbar = tk.Scrollbar(content_frame)
    content_scrollbar.pack(side="right", fill="y")
    
    content_text = tk.Text(content_frame, yscrollcommand=content_scrollbar.set, 
                          font=("Courier", 9), wrap="word")
    content_text.pack(side="left", fill="both", expand=True)
    content_scrollbar.config(command=content_text.yview)
    
    # Status label
    status_var = tk.StringVar(value="Ready")
    status_label = tk.Label(main_frame, textvariable=status_var, fg="#666", anchor="w")
    status_label.pack(fill="x", pady=(10, 0))
    
    # Button frame
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=(10, 0))
    
    def load_directory(path):
        """Load and display directory contents"""
        current_path.set(path)
        status_var.set(f"Loading {path}...")
        items_listbox.delete(0, tk.END)
        content_text.delete(1.0, tk.END)
        
        # Get listing from device
        data = console_instance._get_filesystem_listing(path)
        
        if not data:
            status_var.set("Error: Failed to load directory")
            items_listbox.insert(tk.END, "✗ Failed to load directory")
            return
        
        # Add parent directory option if not at root
        if path != "/lfs":
            items_listbox.insert(tk.END, "📁 .. (parent)")
        
        # Add directories
        dirs = sorted(data.get('directories', []))
        for dir_name in dirs:
            items_listbox.insert(tk.END, f"📁 {dir_name}/")
        
        # Add files
        files = sorted(data.get('files', []), key=lambda x: x['name'])
        for file in files:
            size_kb = file['size'] / 1024.0
            size_str = f"{size_kb:.1f}KB" if size_kb >= 1 else f"{file['size']}B"
            items_listbox.insert(tk.END, f"📄 {file['name']} ({size_str})")
        
        total_items = len(dirs) + len(files)
        status_var.set(f"Loaded {total_items} items from {path}")
    
    def on_item_double_click(event):
        """Handle double-click on listbox item"""
        selection = items_listbox.curselection()
        if not selection:
            return
        
        item = items_listbox.get(selection[0])
        
        # Handle parent directory
        if item.startswith("📁 .. (parent)"):
            parent_path = "/".join(current_path.get().rstrip('/').split('/')[:-1])
            if not parent_path:
                parent_path = "/lfs"
            load_directory(parent_path)
            return
        
        # Handle directory navigation
        if item.startswith("📁"):
            dir_name = item.split(" ", 1)[1].rstrip('/')
            new_path = f"{current_path.get().rstrip('/')}/{dir_name}"
            load_directory(new_path)
            return
        
        # Handle file viewing
        if item.startswith("📄"):
            file_name = item.split(" ", 1)[1].split(" (")[0]
            file_path = f"{current_path.get().rstrip('/')}/{file_name}"
            
            status_var.set(f"Loading {file_name}...")
            content_text.delete(1.0, tk.END)
            
            # Read file content
            data = console_instance._read_file_content(file_path)
            
            if not data:
                status_var.set(f"Error: Failed to read {file_name}")
                content_text.insert(1.0, f"✗ Failed to read file: {file_path}")
                return
            
            content = data.get('content', '')
            
            # Try to pretty-print JSON
            try:
                json_content = json.loads(content)
                formatted_content = json.dumps(json_content, indent=2)
                content_text.insert(1.0, formatted_content)
            except:
                # Not JSON, display as-is
                content_text.insert(1.0, content)
            
            status_var.set(f"Viewing: {file_name} ({data.get('size', 0)} bytes)")
    
    def refresh():
        """Refresh current directory"""
        load_directory(current_path.get())
    
    def go_to_root():
        """Navigate to root directory"""
        load_directory("/lfs")
    
    def go_to_config():
        """Navigate to config directory"""
        load_directory("/lfs/config")
    
    # Bind double-click event
    items_listbox.bind("<Double-Button-1>", on_item_double_click)
    
    # Buttons
    tk.Button(button_frame, text="🔄 Refresh", command=refresh, width=12).pack(side="left", padx=5)
    tk.Button(button_frame, text="🏠 Root", command=go_to_root, width=12).pack(side="left", padx=5)
    tk.Button(button_frame, text="⚙️ Config", command=go_to_config, width=12).pack(side="left", padx=5)
    tk.Button(button_frame, text="Close", command=root.destroy, width=12).pack(side="left", padx=5)
    
    # Load initial directory
    load_directory("/lfs")
    
    root.mainloop()


def test_plant_info_gui(console_instance):
    """
    Standalone GUI test window for plant information.
    Shows current plant info and allows editing.
    Can be integrated as a tab in the main GUI later.
    """
    root = tk.Tk()
    root.title("Plant Information - Test Window")
    root.geometry("600x500")
    
    # Main container
    main_frame = tk.Frame(root, padx=20, pady=20)
    main_frame.pack(fill="both", expand=True)
    
    # Title
    title_label = tk.Label(main_frame, text="🌱 Plant Information", 
                          font=("Arial", 16, "bold"))
    title_label.pack(pady=(0, 20))
    
    # Current plant info display frame
    display_frame = tk.LabelFrame(main_frame, text="Current Plant Information", 
                                  font=("Arial", 11, "bold"), padx=15, pady=15)
    display_frame.pack(fill="x", pady=(0, 20))
    
    # Plant info labels
    plant_name_var = tk.StringVar(value="Not configured")
    start_date_var = tk.StringVar(value="N/A")
    days_growing_var = tk.StringVar(value="N/A")
    
    info_labels_frame = tk.Frame(display_frame)
    info_labels_frame.pack(fill="x")
    
    tk.Label(info_labels_frame, text="Plant Name:", font=("Arial", 10, "bold"), 
             anchor="w", width=15).grid(row=0, column=0, sticky="w", pady=5)
    tk.Label(info_labels_frame, textvariable=plant_name_var, font=("Arial", 10), 
             anchor="w").grid(row=0, column=1, sticky="w", pady=5)
    
    tk.Label(info_labels_frame, text="Start Date:", font=("Arial", 10, "bold"), 
             anchor="w", width=15).grid(row=1, column=0, sticky="w", pady=5)
    tk.Label(info_labels_frame, textvariable=start_date_var, font=("Arial", 10), 
             anchor="w").grid(row=1, column=1, sticky="w", pady=5)
    
    tk.Label(info_labels_frame, text="Days Growing:", font=("Arial", 10, "bold"), 
             anchor="w", width=15).grid(row=2, column=0, sticky="w", pady=5)
    tk.Label(info_labels_frame, textvariable=days_growing_var, font=("Arial", 10), 
             anchor="w").grid(row=2, column=1, sticky="w", pady=5)
    
    # Edit form frame
    edit_frame = tk.LabelFrame(main_frame, text="Update Plant Information", 
                              font=("Arial", 11, "bold"), padx=15, pady=15)
    edit_frame.pack(fill="x", pady=(0, 20))
    
    # Plant name entry
    tk.Label(edit_frame, text="Plant Name:", anchor="w").grid(row=0, column=0, sticky="w", pady=5)
    plant_name_entry = tk.Entry(edit_frame, width=30, font=("Arial", 10))
    plant_name_entry.grid(row=0, column=1, sticky="w", pady=5, padx=(10, 0))
    
    # Start date entry with example
    tk.Label(edit_frame, text="Start Date:", anchor="w").grid(row=1, column=0, sticky="w", pady=5)
    date_frame = tk.Frame(edit_frame)
    date_frame.grid(row=1, column=1, sticky="w", pady=5, padx=(10, 0))
    
    start_date_entry = tk.Entry(date_frame, width=15, font=("Arial", 10))
    start_date_entry.pack(side="left")
    tk.Label(date_frame, text="(YYYY-MM-DD)", fg="#666", font=("Arial", 9)).pack(side="left", padx=(5, 0))
    
    # Preset date buttons
    preset_frame = tk.Frame(edit_frame)
    preset_frame.grid(row=2, column=1, sticky="w", pady=5, padx=(10, 0))
    
    def set_today():
        today = datetime.now().strftime('%Y-%m-%d')
        start_date_entry.delete(0, tk.END)
        start_date_entry.insert(0, today)
    
    def set_week_ago():
        week_ago = (datetime.now() - timedelta(days=7)).strftime('%Y-%m-%d')
        start_date_entry.delete(0, tk.END)
        start_date_entry.insert(0, week_ago)
    
    tk.Button(preset_frame, text="Today", command=set_today, width=10).pack(side="left", padx=(0, 5))
    tk.Button(preset_frame, text="1 Week Ago", command=set_week_ago, width=12).pack(side="left")
    
    # Status message
    status_var = tk.StringVar(value="")
    status_label = tk.Label(main_frame, textvariable=status_var, fg="#666", 
                           font=("Arial", 9), wraplength=500, justify="left")
    status_label.pack(pady=(0, 10))
    
    def load_plant_info():
        """Load current plant info from device"""
        status_var.set("Loading plant information...")
        
        data = console_instance._get_plant_info()
        
        if not data:
            status_var.set("❌ Error: Failed to load plant information")
            plant_name_var.set("Error loading data")
            start_date_var.set("N/A")
            days_growing_var.set("N/A")
            return
        
        if data.get('exists'):
            plant_name_var.set(data.get('plant_name', 'N/A'))
            start_date_var.set(data.get('start_date', 'N/A'))
            days_growing_var.set(f"{data.get('days_growing', 'N/A')} days")
            status_var.set("✓ Plant information loaded successfully")
            
            # Pre-fill edit fields
            plant_name_entry.delete(0, tk.END)
            plant_name_entry.insert(0, data.get('plant_name', ''))
            start_date_entry.delete(0, tk.END)
            start_date_entry.insert(0, data.get('start_date', ''))
        else:
            plant_name_var.set("Not configured")
            start_date_var.set("N/A")
            days_growing_var.set("N/A")
            status_var.set("ℹ️ No plant information configured yet")
    
    def save_plant_info():
        """Save plant info to device"""
        plant_name = plant_name_entry.get().strip()
        start_date = start_date_entry.get().strip()
        
        if not plant_name:
            status_var.set("❌ Error: Plant name is required")
            return
        
        if not start_date:
            status_var.set("❌ Error: Start date is required")
            return
        
        # Validate date format
        try:
            datetime.strptime(start_date, '%Y-%m-%d')
        except ValueError:
            status_var.set("❌ Error: Invalid date format. Use YYYY-MM-DD")
            return
        
        status_var.set("Saving plant information...")
        
        success = console_instance._set_plant_info(plant_name, start_date)
        
        if success:
            status_var.set("✅ Plant information saved successfully!")
            load_plant_info()  # Reload to show updated info
        else:
            status_var.set("❌ Error: Failed to save plant information")
    
    # Button frame
    button_frame = tk.Frame(main_frame)
    button_frame.pack(pady=(10, 0))
    
    tk.Button(button_frame, text="🔄 Refresh", command=load_plant_info, 
             width=15, bg="#4CAF50", fg="white", font=("Arial", 10, "bold")).pack(side="left", padx=5)
    tk.Button(button_frame, text="💾 Save", command=save_plant_info, 
             width=15, bg="#2196F3", fg="white", font=("Arial", 10, "bold")).pack(side="left", padx=5)
    tk.Button(button_frame, text="Close", command=root.destroy, 
             width=15, font=("Arial", 10)).pack(side="left", padx=5)
    
    # Load initial data
    load_plant_info()
    
    root.mainloop()


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
            # GUI
            'launch_gui': 'Launch unified GrowPod GUI (main interface): launch_gui',
            'manage_schedules': 'Unified schedule manager (hourly + routines): manage_schedules',
            # Routine commands
            'routine': 'Send a routine command. Example: routine empty_pod',
            'routine_status': 'Check routine status by ID.',
            'status': 'Get sensor data from the selected device.',
            # Actuator control
            'airpump': 'Set air pump PWM value: airpump <value>',
            # Water Pump → Source Pump
            'sourcepump': 'Set source pump PWM value: sourcepump <value>',
            'planterpump': 'Set planter pump PWM value: planterpump <value>',
            # Servo → Drain Pump
            'drainpump': 'Set drain pump PWM value: drainpump <value>',
            'foodpump': 'Control food pump: foodpump <value> | dose <duration_ms> [speed]',
            'led': 'Set LED brightness: led <value> [channel]',
            # Scheduling
            'schedule_actuators': 'Set actuator schedule: schedule_actuators',
            'view_schedules': 'View all actuator schedules: view_schedules',
            'delete_schedule': 'Delete a schedule: delete_schedule <schedule_name>',
            'start_feeding_cycle': 'Start the feeding cycle.',
            'stop_feeding_cycle': 'Stop the feeding cycle.',
            'start_emptying_water': 'Start emptying water.',
            'stop_emptying_water': 'Stop emptying water.',
            'all_schedules' : 'Set schedules for all actuators at once: routine all_schedules',
            'showschedules'  : 'Print current schedules on device console',
            # Filesystem commands
            'fs_browse': 'Browse device filesystem: fs_browse [path]',
            'fs_cat': 'Display file contents: fs_cat <path>',
            # Plant info commands
            'plant_info': 'Display current plant information: plant_info',
            'plant_set': 'Set plant information: plant_set <name> <date>',
            # GUI test windows
            'test_gui_filesystem': 'Launch filesystem browser GUI test window',
            'test_gui_plant': 'Launch plant info GUI test window',
            # System
            'exit': 'Exit the console.',
            'hostname_suffix': 'Set mDNS suffix: hostname_suffix <suffix>',
            'rescan': 'Re-initialize Zeroconf to discover devices again.',
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
                # Handle routine commands specially
                if actuator == 'routine' and 'command' in command:
                    # Post to /api/routines/<command_name>
                    routine_cmd = command['command']
                    url = f"https://{device_info['address']}:{device_info['port']}/api/routines/{routine_cmd}"
                    response = self.session.post(url, json={}, timeout=10, verify=False)
                    print(f"Routine '{routine_cmd}' started: {response.text}")
                elif actuator == 'food_dose':
                    # Handle food dosing (timed pump operation)
                    duration_ms = command.get('duration_ms', 1000)
                    speed = command.get('speed', 100)
                    url = f"https://{device_info['address']}:{device_info['port']}/api/actuators/foodpump"
                    payload = {'dose': duration_ms, 'speed': speed}
                    response = self.session.post(url, json=payload, timeout=10, verify=False)
                    print(f"Food pump dosed: {response.text}")
                else:
                    # Regular actuator command
                    url = f"https://{device_info['address']}:{device_info['port']}/api/actuators/{actuator}"
                    response = self.session.post(url, json=command, timeout=5, verify=False)
                    print(f"Actuator '{actuator}' responded with: {response.text}")
            except Exception as e:
                print(f"Error sending command to '{actuator}' for device '{device_name}': {e}")

        # Function to turn off the actuators
        def turn_off_actuators():
            print(f"Turning off actuators for schedule '{name}' on device '{device_name}' at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            for actuator, command in actions.items():
                # Set all actuators to 0
                if actuator in ['airpump', 'sourcepump', 'planterpump', 'drainpump', 'led']:
                    self._post_actuator_command(actuator, {'value': 0}, device_name)

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

            # 3) Present a multi-schedule GUI
            updated_light, updated_planter = self._prompt_schedule_multi(
                light_sched, planter_sched
            )
            if updated_light is None or updated_planter is None:
                print("Schedule editing cancelled.")
                return

            # 4) Post each updated schedule
            self._post_routine("light_schedule", {"schedule": updated_light})
            self._post_routine("planter_pod_schedule", {"schedule": updated_planter})
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
            resp = self.session.post(url, json=json_body, timeout=50)
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
            
            print("\n=== Environment Sensors ===")
            temp_c = data.get('temperature_c', -999)
            humidity = data.get('humidity_rh', -999)
            if temp_c != -999:
                temp_f = (temp_c * 9/5) + 32
                print(f"Temperature: {temp_c:.1f}°C ({temp_f:.1f}°F)")
            else:
                print("Temperature: N/A")
            
            if humidity != -999:
                print(f"Humidity: {humidity:.1f}%")
            else:
                print("Humidity: N/A")
            
            print("\n=== Light Sensor (TSL2591) ===")
            lux = data.get('light_lux', -999)
            visible = data.get('light_visible', 0)
            infrared = data.get('light_infrared', 0)
            if lux != -999:
                print(f"Illuminance: {lux:.1f} lux")
                print(f"Visible Light: {visible} counts")
                print(f"Infrared: {infrared} counts")
            else:
                print("Light sensor: N/A")
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

    def do_foodpump(self, arg):
        '''Control food pump: foodpump <value> | dose <duration_ms> [speed]
        Examples:
            foodpump 50           - Run at 50% speed continuously
            foodpump 0            - Stop food pump
            foodpump dose 1000    - Dose for 1 second at 100% speed
            foodpump dose 500 80  - Dose for 500ms at 80% speed
        '''
        if not self._check_device_selected():
            return
        
        args = arg.strip().split()
        if len(args) == 0:
            print("Usage: foodpump <value> | dose <duration_ms> [speed]")
            print("  foodpump <value>                  - Set continuous PWM (0-100%)")
            print("  foodpump dose <duration_ms>       - Dose for specified milliseconds at 100% speed")
            print("  foodpump dose <duration_ms> <speed> - Dose for specified milliseconds at custom speed")
            return
        
        # Check if it's a dose command
        if args[0].lower() == 'dose':
            if len(args) < 2:
                print("Usage: foodpump dose <duration_ms> [speed]")
                return
            
            try:
                duration_ms = int(args[1])
                speed = int(args[2]) if len(args) > 2 else 100
                
                if duration_ms <= 0:
                    print("Duration must be greater than 0")
                    return
                if speed < 0 or speed > 100:
                    print("Speed must be between 0 and 100")
                    return
                
                self._post_actuator_command('foodpump', {'dose': duration_ms, 'speed': speed}, self.selected_device)
            except ValueError:
                print("Invalid numeric value. Usage: foodpump dose <duration_ms> [speed]")
        else:
            # Continuous PWM mode
            try:
                value = int(args[0])
                if value < 0 or value > 100:
                    print("Value must be between 0 and 100")
                    return
                
                self._post_actuator_command('foodpump', {'value': value}, self.selected_device)
            except ValueError:
                print("Invalid numeric value. Usage: foodpump <value>")

    # --------------------------------------------------------------------------
    # Phase 1 Test Commands - Filesystem and Plant Info API Testing
    # --------------------------------------------------------------------------
    
    def do_test_fs_list(self, arg):
        '''Test filesystem list API: test_fs_list [path]
        Examples:
            test_fs_list           - List files in /lfs
            test_fs_list /lfs/config - List files in /lfs/config
        '''
        if not self._check_device_selected():
            return
        
        path = arg.strip() if arg.strip() else '/lfs'
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/filesystem/list"
            params = {'path': path}
            
            print(f"\n--- Testing Filesystem List API ---")
            print(f"Path: {path}")
            print(f"URL: {url}?path={path}")
            
            response = self.session.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            
            print(f"\n✓ Response received:")
            print(f"  Path: {data.get('path', 'N/A')}")
            print(f"  Total Directories: {data.get('total_dirs', 0)}")
            print(f"  Total Files: {data.get('total_files', 0)}")
            
            if data.get('directories'):
                print(f"\n  Directories:")
                for dir_name in data['directories']:
                    print(f"    [DIR]  {dir_name}")
            
            if data.get('files'):
                print(f"\n  Files:")
                for file in data['files']:
                    print(f"    [FILE] {file['name']} ({file['size']} bytes)")
            
            print(f"\n✓ Test passed!")
            
        except requests.exceptions.RequestException as e:
            print(f"✗ Request failed: {e}")
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON response: {e}")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def do_test_fs_read(self, arg):
        '''Test filesystem read API: test_fs_read <path>
        Examples:
            test_fs_read /lfs/config/plant.json
            test_fs_read /lfs/test.txt
        '''
        if not self._check_device_selected():
            return
        
        if not arg.strip():
            print("Usage: test_fs_read <path>")
            print("Example: test_fs_read /lfs/config/plant.json")
            return
        
        path = arg.strip()
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/filesystem/read"
            params = {'path': path}
            
            print(f"\n--- Testing Filesystem Read API ---")
            print(f"Path: {path}")
            print(f"URL: {url}?path={path}")
            
            response = self.session.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            
            print(f"\n✓ Response received:")
            print(f"  Path: {data.get('path', 'N/A')}")
            print(f"  Size: {data.get('size', 0)} bytes")
            print(f"\n  Content:")
            print("  " + "-" * 60)
            content = data.get('content', '')
            # Pretty print JSON if possible
            try:
                json_content = json.loads(content)
                print(json.dumps(json_content, indent=4))
            except:
                print(content)
            print("  " + "-" * 60)
            
            print(f"\n✓ Test passed!")
            
        except requests.exceptions.RequestException as e:
            print(f"✗ Request failed: {e}")
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON response: {e}")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def do_test_plant_get(self, arg):
        '''Test plant info GET API: test_plant_get
        Retrieves current plant information from the device.
        '''
        if not self._check_device_selected():
            return
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/plant/info"
            
            print(f"\n--- Testing Plant Info GET API ---")
            print(f"URL: {url}")
            
            response = self.session.get(url, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            
            print(f"\n✓ Response received:")
            
            if data.get('exists'):
                print(f"  Plant Name: {data.get('plant_name', 'N/A')}")
                print(f"  Start Date: {data.get('start_date', 'N/A')}")
                print(f"  Start Timestamp: {data.get('start_timestamp', 'N/A')}")
                print(f"  Days Growing: {data.get('days_growing', 'N/A')}")
            else:
                print(f"  Status: No plant information configured")
                print(f"  Message: {data.get('message', 'N/A')}")
            
            print(f"\n✓ Test passed!")
            
        except requests.exceptions.RequestException as e:
            print(f"✗ Request failed: {e}")
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON response: {e}")
        except Exception as e:
            print(f"✗ Error: {e}")
    
    def do_test_plant_set(self, arg):
        '''Test plant info POST API: test_plant_set <plant_name> <start_date>
        Examples:
            test_plant_set "Micro Tom" 2025-12-16
            test_plant_set Basil 2025-12-01
        '''
        if not self._check_device_selected():
            return
        
        args = arg.strip().split(maxsplit=1)
        if len(args) < 2:
            print("Usage: test_plant_set <plant_name> <start_date>")
            print("  plant_name: Name of the plant (use quotes if it contains spaces)")
            print("  start_date: Start date in YYYY-MM-DD format")
            print("\nExamples:")
            print('  test_plant_set "Micro Tom" 2025-12-16')
            print('  test_plant_set Basil 2025-12-01')
            return
        
        # Parse plant_name and start_date
        plant_name = args[0].strip('"').strip("'")
        start_date = args[1].strip() if len(args) > 1 else args[0].split()[1] if len(args[0].split()) > 1 else None
        
        # Handle case where name has spaces and is quoted
        if not start_date or len(start_date) != 10:
            parts = arg.strip().split()
            if len(parts) >= 2:
                start_date = parts[-1]
                plant_name = ' '.join(parts[:-1]).strip('"').strip("'")
        
        # Validate date format
        try:
            datetime.strptime(start_date, '%Y-%m-%d')
        except ValueError:
            print(f"✗ Invalid date format: {start_date}")
            print("  Date must be in YYYY-MM-DD format")
            return
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/plant/info"
            
            payload = {
                'plant_name': plant_name,
                'start_date': start_date
            }
            
            print(f"\n--- Testing Plant Info POST API ---")
            print(f"URL: {url}")
            print(f"Payload:")
            print(f"  Plant Name: {plant_name}")
            print(f"  Start Date: {start_date}")
            
            response = self.session.post(url, json=payload, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            
            print(f"\n✓ Response received:")
            print(f"  Success: {data.get('success', False)}")
            print(f"  Message: {data.get('message', 'N/A')}")
            
            print(f"\n✓ Test passed!")
            print(f"\nTip: Run 'test_plant_get' to verify the data was saved correctly.")
            
        except requests.exceptions.RequestException as e:
            print(f"✗ Request failed: {e}")
        except json.JSONDecodeError as e:
            print(f"✗ Invalid JSON response: {e}")
        except Exception as e:
            print(f"✗ Error: {e}")

    # --------------------------------------------------------------------------
    # Phase 1 Helper Functions - Reusable API Access
    # --------------------------------------------------------------------------
    
    def _get_filesystem_listing(self, path='/lfs'):
        """
        Helper function to get filesystem listing from device.
        Returns dict with 'directories', 'files', 'total_dirs', 'total_files', 'path'
        Returns None on error.
        """
        if not self._check_device_selected():
            return None
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/filesystem/list"
            params = {'path': path}
            
            response = self.session.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            return response.json()
        except Exception as e:
            print(f"Error fetching filesystem listing: {e}")
            return None
    
    def _read_file_content(self, path):
        """
        Helper function to read file content from device.
        Returns dict with 'path', 'size', 'content'
        Returns None on error.
        """
        if not self._check_device_selected():
            return None
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/filesystem/read"
            params = {'path': path}
            
            response = self.session.get(url, params=params, timeout=10)
            response.raise_for_status()
            
            return response.json()
        except Exception as e:
            print(f"Error reading file: {e}")
            return None
    
    def _get_plant_info(self):
        """
        Helper function to get plant information from device.
        Returns dict with plant info if exists, or {'exists': False} if not configured.
        Returns None on error.
        """
        if not self._check_device_selected():
            return None
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/plant/info"
            
            response = self.session.get(url, timeout=10)
            response.raise_for_status()
            
            return response.json()
        except Exception as e:
            print(f"Error fetching plant info: {e}")
            return None
    
    def _set_plant_info(self, plant_name, start_date):
        """
        Helper function to set plant information on device.
        Returns True on success, False on error.
        """
        if not self._check_device_selected():
            return False
        
        # Validate date format
        try:
            datetime.strptime(start_date, '%Y-%m-%d')
        except ValueError:
            print(f"Invalid date format: {start_date}. Use YYYY-MM-DD")
            return False
        
        try:
            device = devices[self.selected_device]
            url = f"https://{device['address']}/api/plant/info"
            
            payload = {
                'plant_name': plant_name,
                'start_date': start_date
            }
            
            response = self.session.post(url, json=payload, timeout=10)
            response.raise_for_status()
            
            data = response.json()
            return data.get('success', False)
        except Exception as e:
            print(f"Error setting plant info: {e}")
            return False

    # --------------------------------------------------------------------------
    # Phase 1 Production Commands - User-Facing Filesystem and Plant Info
    # --------------------------------------------------------------------------
    
    def do_fs_browse(self, arg):
        '''Browse filesystem on device: fs_browse [path]
        Examples:
            fs_browse           - Browse /lfs root directory
            fs_browse /lfs/config - Browse /lfs/config directory
        '''
        if not self._check_device_selected():
            return
        
        path = arg.strip() if arg.strip() else '/lfs'
        
        print(f"\n{'='*70}")
        print(f"  Filesystem Browser: {path}")
        print(f"{'='*70}")
        
        data = self._get_filesystem_listing(path)
        if not data:
            print("Failed to retrieve filesystem listing.")
            return
        
        # Display directories
        if data.get('directories'):
            print(f"\n📁 Directories ({data.get('total_dirs', 0)}):")
            for dir_name in sorted(data['directories']):
                print(f"   {dir_name}/")
        
        # Display files
        if data.get('files'):
            print(f"\n📄 Files ({data.get('total_files', 0)}):")
            for file in sorted(data['files'], key=lambda x: x['name']):
                size_kb = file['size'] / 1024.0
                if size_kb < 1:
                    size_str = f"{file['size']} B"
                else:
                    size_str = f"{size_kb:.2f} KB"
                print(f"   {file['name']:<40} {size_str:>12}")
        
        if not data.get('directories') and not data.get('files'):
            print("\n  (empty directory)")
        
        print(f"\n{'='*70}")
        print(f"Tip: Use 'fs_cat <path>' to view file contents")
        print(f"{'='*70}\n")
    
    def do_fs_cat(self, arg):
        '''Display file contents: fs_cat <path>
        Examples:
            fs_cat /lfs/config/plant.json
            fs_cat /lfs/config/system.json
        '''
        if not self._check_device_selected():
            return
        
        if not arg.strip():
            print("Usage: fs_cat <path>")
            print("Example: fs_cat /lfs/config/plant.json")
            return
        
        path = arg.strip()
        
        data = self._read_file_content(path)
        if not data:
            print(f"Failed to read file: {path}")
            return
        
        print(f"\n{'='*70}")
        print(f"  File: {data.get('path', path)}")
        print(f"  Size: {data.get('size', 0)} bytes")
        print(f"{'='*70}")
        
        content = data.get('content', '')
        
        # Try to pretty-print JSON
        try:
            json_content = json.loads(content)
            print(json.dumps(json_content, indent=2))
        except:
            # Not JSON, print as-is
            print(content)
        
        print(f"{'='*70}\n")
    
    def do_plant_info(self, arg):
        '''Display current plant information: plant_info'''
        if not self._check_device_selected():
            return
        
        data = self._get_plant_info()
        if not data:
            print("Failed to retrieve plant information.")
            return
        
        print(f"\n{'='*70}")
        print(f"  🌱 Plant Information")
        print(f"{'='*70}")
        
        if data.get('exists'):
            print(f"\n  Plant Name:     {data.get('plant_name', 'N/A')}")
            print(f"  Start Date:     {data.get('start_date', 'N/A')}")
            print(f"  Days Growing:   {data.get('days_growing', 'N/A')} days")
            print(f"  Start Timestamp: {data.get('start_timestamp', 'N/A')}")
            
            # Calculate some useful dates
            start_date_str = data.get('start_date')
            if start_date_str:
                try:
                    start_date = datetime.strptime(start_date_str, '%Y-%m-%d')
                    today = datetime.now()
                    days_elapsed = (today - start_date).days
                    print(f"  Days Elapsed:   {days_elapsed} days (calculated from PC time)")
                except:
                    pass
        else:
            print(f"\n  Status: No plant information configured")
            print(f"  Message: {data.get('message', 'N/A')}")
            print(f"\n  💡 Tip: Use 'plant_set <name> <date>' to configure plant info")
        
        print(f"{'='*70}\n")
    
    def do_plant_set(self, arg):
        '''Set plant information: plant_set <plant_name> <start_date>
        Examples:
            plant_set "Micro Tom" 2025-12-16
            plant_set Basil 2025-12-01
        '''
        if not self._check_device_selected():
            return
        
        args = arg.strip().split(maxsplit=1)
        if len(args) < 2:
            print("Usage: plant_set <plant_name> <start_date>")
            print("  plant_name: Name of the plant (use quotes if it contains spaces)")
            print("  start_date: Start date in YYYY-MM-DD format")
            print("\nExamples:")
            print('  plant_set "Micro Tom" 2025-12-16')
            print('  plant_set Basil 2025-12-01')
            return
        
        # Parse plant_name and start_date
        plant_name = args[0].strip('"').strip("'")
        start_date = args[1].strip() if len(args) > 1 else args[0].split()[1] if len(args[0].split()) > 1 else None
        
        # Handle case where name has spaces and is quoted
        if not start_date or len(start_date) != 10:
            parts = arg.strip().split()
            if len(parts) >= 2:
                start_date = parts[-1]
                plant_name = ' '.join(parts[:-1]).strip('"').strip("'")
        
        print(f"\n{'='*70}")
        print(f"  Setting Plant Information")
        print(f"{'='*70}")
        print(f"  Plant Name: {plant_name}")
        print(f"  Start Date: {start_date}")
        
        success = self._set_plant_info(plant_name, start_date)
        
        if success:
            print(f"\n  ✓ Plant information saved successfully!")
            print(f"\n{'='*70}")
            print(f"  💡 Tip: Use 'plant_info' to view the saved information")
            print(f"{'='*70}\n")
        else:
            print(f"\n  ✗ Failed to save plant information.")
            print(f"{'='*70}\n")

    # --------------------------------------------------------------------------
    # Main GUI Launcher
    # --------------------------------------------------------------------------
    
    def do_launch_gui(self, arg):
        '''Launch unified GrowPod GUI: launch_gui
        Opens the main GUI application with all features:
        - Schedules: Manage hourly schedules, food dosing, and routines
        - Filesystem: Browse device filesystem and view files
        - Plant Info: Manage plant information and growing data
        '''
        if not self._check_device_selected():
            return
        
        print("\n🚀 Launching GrowPod Control GUI...")
        print(f"📱 Device: {self.selected_device}")
        print(f"🔗 Address: {devices[self.selected_device]['address']}\n")
        
        launch_unified_gui(self)
    
    # --------------------------------------------------------------------------
    # Phase 1 GUI Test Commands - Launch Standalone GUI Test Windows
    # --------------------------------------------------------------------------
    
    def do_test_gui_filesystem(self, arg):
        '''Launch filesystem browser GUI test window: test_gui_filesystem
        Opens a standalone window to browse device filesystem visually.
        '''
        if not self._check_device_selected():
            return
        
        print("\n🖥️  Launching Filesystem Browser GUI...")
        print("💡 This is a test window. Close it when done.\n")
        
        test_filesystem_browser_gui(self)
    
    def do_test_gui_plant(self, arg):
        '''Launch plant info GUI test window: test_gui_plant
        Opens a standalone window to view and edit plant information.
        '''
        if not self._check_device_selected():
            return
        
        print("\n🖥️  Launching Plant Information GUI...")
        print("💡 This is a test window. Close it when done.\n")
        
        test_plant_info_gui(self)

    def do_led(self, arg):
        '''Set LED brightness: led <value> [channel]
        Examples:
            led 100       - Set all LED channels to 100%
            led 50        - Set all LED channels to 50%
            led 75 2      - Set LED channel 2 to 75%
            led 0         - Turn off all LEDs
        '''
        if not self._check_device_selected():
            return
        
        args = arg.strip().split()
        if len(args) == 0:
            print("Usage: led <value> [channel]")
            print("  value: 0-100 (PWM percentage)")
            print("  channel: 1-4 (optional, sets specific channel)")
            print("Examples:")
            print("  led 100       - Set all LEDs to 100%")
            print("  led 50 2      - Set channel 2 to 50%")
            return
        
        try:
            value = int(args[0])
            if value < 0 or value > 100:
                print("Value must be between 0 and 100")
                return
            
            if len(args) == 1:
                # Set all channels
                self._post_actuator_command('led', {'value': value}, self.selected_device)
            elif len(args) == 2:
                # Set specific channel
                channel = int(args[1])
                if channel < 1 or channel > 4:
                    print("Channel must be between 1 and 4")
                    return
                self._post_actuator_command('led', {'channel': channel, 'value': value}, self.selected_device)
            else:
                print("Too many arguments. Usage: led <value> [channel]")
        except ValueError:
            print("Invalid numeric value. Usage: led <value> [channel]")

    # --------------------------------------------------------------------------
    # Scheduling Commands
    # --------------------------------------------------------------------------
    def do_schedule_actuators(self, arg):
        'Set actuator schedule: schedule_actuators'
        if not self._check_device_selected():
            return
        
        print("\n--- Opening Actuator Schedule Setup Window ---")
        
        # Open the unified GUI window
        config = prompt_schedule_actuators()
        
        if config is None:
            print("Schedule setup cancelled.")
            return
        
        schedule_name = config['schedule_name']
        
        # Check if schedule already exists
        if schedule_name in self.schedules:
            print(f"Schedule '{schedule_name}' already exists.")
            return
        
        # Get device info
        device_info = devices[self.selected_device]
        device_name = self.selected_device
        device_ip = device_info['address']
        
        # Store the schedule
        self.schedules[schedule_name] = {
            'device_name': device_name,
            'device_ip': device_ip,
            'start_time': config['start_time'],
            'duration_minutes': config['duration_minutes'],
            'frequency': config['frequency'],
            'day_of_week': config['day_of_week'],
            'actions': config['actions']
        }
        
        # Schedule the job if device is available
        self.schedule_job(schedule_name, self.schedules[schedule_name])
        
        # Save schedules to JSON
        self.save_schedules()
        
        print(f"\nActuator schedule '{schedule_name}' has been set successfully for device '{device_name}'.")
        print(f"  Start Time: {config['start_time']}")
        print(f"  Duration: {config['duration_minutes']} minutes")
        print(f"  Frequency: {config['frequency']}")
        if config['day_of_week']:
            print(f"  Day: {config['day_of_week']}")
        print(f"  Actuators configured: {', '.join(config['actions'].keys())}\n")

    def do_manage_schedules(self, arg):
        '''Unified schedule manager: manage_schedules
        Opens comprehensive GUI for managing both:
        - Hourly schedules (Light curve + Planter intervals)  
        - Routine commands (Fill/Empty/Maintenance tasks)
        '''
        if not self._check_device_selected():
            return
        
        print("\n--- Opening Unified Schedule Manager ---")
        
        # Fetch current hourly schedules from device
        light_sched = self._fetch_saved_schedule("light")
        planter_sched = self._fetch_saved_schedule("planter")
        
        # Open the unified GUI window (load with current schedules from device and existing schedules)
        config = prompt_unified_schedule_manager(light_sched, planter_sched, self.schedules)
        
        if config is None:
            print("Schedule management cancelled.")
            return
        
        # Handle hourly schedules
        if config['light_schedule'] or config['planter_schedule']:
            print("\n--- Updating Hourly Schedules ---")
            
            # Post light schedule
            if config['light_schedule']:
                self._post_routine("light_schedule", {"schedule": config['light_schedule']})
                print("✓ Light schedule updated")
            
            # Post planter schedule  
            if config['planter_schedule']:
                self._post_routine("planter_pod_schedule", {"schedule": config['planter_schedule']})
                print("✓ Planter schedule updated")
        
        # Handle food schedule
        if config['food_config']:
            food = config['food_config']
            
            print("\n--- Setting Up Food Dosing Schedule ---")
            print(f"  Total daily amount: {food['total_daily_ms']} ms")
            print(f"  Number of intervals: {food['intervals']}")
            print(f"  Dose per interval: {food['dose_per_interval']} ms")
            print(f"  Pump speed: {food['speed']}%")
            
            # Calculate feeding times (evenly spaced throughout 24 hours)
            hours_between = 24.0 / food['intervals']
            
            # Remove any existing food schedules
            food_schedules = [name for name in self.schedules if name.startswith('food_dose_')]
            for name in food_schedules:
                try:
                    scheduler.remove_job(name)
                    del self.schedules[name]
                    print(f"  Removed old schedule: {name}")
                except Exception as e:
                    print(f"  Warning: Could not remove old schedule {name}: {e}")
            
            # Create new food dosing schedules
            device_info = devices[self.selected_device]
            device_name = self.selected_device
            
            for i in range(food['intervals']):
                # Calculate start hour for this interval
                start_hour = int(i * hours_between)
                start_minute = int((i * hours_between - start_hour) * 60)
                schedule_name = f"food_dose_{i+1}"
                
                # Store the food dosing schedule
                self.schedules[schedule_name] = {
                    'device_name': device_name,
                    'device_ip': device_info['address'],
                    'start_time': f"{start_hour:02d}:{start_minute:02d}",
                    'duration_minutes': 0,  # Instant dose
                    'frequency': 'daily',
                    'day_of_week': None,
                    'actions': {
                        'food_dose': {
                            'duration_ms': food['dose_per_interval'],
                            'speed': food['speed']
                        }
                    }
                }
                
                # Schedule the job
                self.schedule_job(schedule_name, self.schedules[schedule_name])
                print(f"  ✓ Scheduled feeding {i+1} at {start_hour:02d}:{start_minute:02d} ({food['dose_per_interval']} ms)")
            
            # Save schedules to JSON
            self.save_schedules()
            print(f"\n✓ Food dosing schedule created with {food['intervals']} daily feedings")
        
        # Handle routine command schedule
        if config['routine_config']:
            routine = config['routine_config']
            schedule_name = routine['schedule_name']
            
            # Check if schedule already exists
            if schedule_name in self.schedules:
                print(f"Warning: Schedule '{schedule_name}' already exists. Overwriting...")
            
            # Get device info
            device_info = devices[self.selected_device]
            device_name = self.selected_device
            device_ip = device_info['address']
            
            # Create action based on routine command
            actions = {}
            command = routine['command']
            
            # Map routine commands to actions
            # These will be handled as API calls to the routine endpoint
            if command == 'empty_pod':
                actions['routine'] = {'command': 'empty_pod'}
            elif command == 'fill_pod':
                actions['routine'] = {'command': 'fill_pod'}
            elif command == 'calibrate_pod':
                actions['routine'] = {'command': 'calibrate_pod'}
            
            # Store the routine schedule
            self.schedules[schedule_name] = {
                'device_name': device_name,
                'device_ip': device_ip,
                'start_time': routine['start_time'],
                'duration_minutes': 0,  # Routines don't have duration
                'frequency': routine['frequency'],
                'day_of_week': routine['day_of_week'],
                'actions': actions
            }
            
            # Schedule the job
            self.schedule_job(schedule_name, self.schedules[schedule_name])
            
            # Save schedules to JSON
            self.save_schedules()
            
            print(f"\n✓ Routine schedule '{schedule_name}' created:")
            print(f"  Command: {command}")
            print(f"  Start Time: {routine['start_time']}")
            print(f"  Frequency: {routine['frequency']}")
            if routine['day_of_week']:
                print(f"  Day: {routine['day_of_week']}")
        
        print("\n✓ All schedule updates completed successfully!\n")

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
            led = str(details['actions'].get('led', {}).get('value', '-'))
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

    def _prompt_schedule_multi(self, light_sched, planter_sched):
        '''
        Prompt the user to edit Light and Planter schedules using a unified tabbed interface.
        Returns tuple (updated_light, updated_planter) or (None, None) if cancelled.
        '''
        try:
            print("Opening unified schedule editor...")
            return prompt_schedule_multi(light_sched, planter_sched)
        except Exception as e:
            print(f"Error in multi-schedule editing: {e}")
            return None, None

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

# =============================================================================
# Device Selector GUI
# =============================================================================

def create_device_selector_gui():
    """
    Opens a device selector GUI that scans for devices and allows user to select one.
    Returns the selected device name, or None if cancelled.
    """
    selected_device = None
    
    def on_select():
        nonlocal selected_device
        selection = device_listbox.curselection()
        if selection:
            index = selection[0]
            selected_device = device_names[index]
            root.destroy()
        else:
            tk.messagebox.showwarning("No Selection", "Please select a device from the list")
    
    def on_cancel():
        nonlocal selected_device
        selected_device = None
        root.destroy()
    
    def refresh_devices():
        """Refresh the device list"""
        device_listbox.delete(0, tk.END)
        device_names.clear()
        
        if not devices:
            device_listbox.insert(tk.END, "No devices found. Waiting for discovery...")
            status_label.config(text="🔍 Scanning for devices...", fg="#e67e22")
        else:
            for name, info in devices.items():
                device_names.append(name)
                display_text = f"🌱 {name} ({info['address']}:{info['port']})"
                device_listbox.insert(tk.END, display_text)
            status_label.config(text=f"✅ Found {len(devices)} device(s)", fg="#27ae60")
    
    def auto_refresh():
        """Automatically refresh device list every 2 seconds"""
        refresh_devices()
        root.after(2000, auto_refresh)
    
    # Create main window
    root = tk.Tk()
    root.title("GrowPod Device Selector")
    root.geometry("600x520")
    root.minsize(550, 500)
    
    # Bring to front
    root.lift()
    root.attributes('-topmost', True)
    root.after_idle(root.attributes, '-topmost', False)
    root.focus_force()
    
    # Header
    header_frame = tk.Frame(root, bg="#2c3e50", padx=15, pady=15)
    header_frame.pack(fill="x")
    
    title_label = tk.Label(
        header_frame,
        text="🌱 GrowPod Device Selector",
        font=("Arial", 16, "bold"),
        bg="#2c3e50",
        fg="white"
    )
    title_label.pack()
    
    subtitle_label = tk.Label(
        header_frame,
        text="Select a device to manage",
        font=("Arial", 10),
        bg="#2c3e50",
        fg="#bdc3c7"
    )
    subtitle_label.pack()
    
    # Main content frame
    content_frame = tk.Frame(root, padx=20, pady=20)
    content_frame.pack(fill="both", expand=True)
    
    # Instructions
    instructions = tk.Label(
        content_frame,
        text="Scanning for GrowPod devices on the local network...",
        font=("Arial", 10),
        fg="#555"
    )
    instructions.pack(pady=(0, 10))
    
    # Device list frame
    list_frame = tk.LabelFrame(content_frame, text="Available Devices", padx=10, pady=10)
    list_frame.pack(fill="both", expand=True)
    
    # Listbox with scrollbar
    scrollbar = tk.Scrollbar(list_frame)
    scrollbar.pack(side="right", fill="y")
    
    device_listbox = tk.Listbox(
        list_frame,
        font=("Arial", 11),
        selectmode=tk.SINGLE,
        yscrollcommand=scrollbar.set,
        height=10
    )
    device_listbox.pack(side="left", fill="both", expand=True)
    scrollbar.config(command=device_listbox.yview)
    
    # Double-click to select
    device_listbox.bind('<Double-Button-1>', lambda e: on_select())
    
    device_names = []
    
    # Status label
    status_label = tk.Label(content_frame, text="🔍 Scanning...", font=("Arial", 10))
    status_label.pack(pady=10)
    
    # Buttons frame
    button_frame = tk.Frame(content_frame)
    button_frame.pack(pady=15)
    
    refresh_btn = tk.Button(
        button_frame,
        text="🔄 Refresh",
        command=refresh_devices,
        font=("Arial", 11),
        padx=15,
        pady=8
    )
    refresh_btn.pack(side="left", padx=5)
    
    select_btn = tk.Button(
        button_frame,
        text="✅ Select Device",
        command=on_select,
        font=("Arial", 11, "bold"),
        bg="#27ae60",
        fg="white",
        padx=20,
        pady=8
    )
    select_btn.pack(side="left", padx=5)
    
    cancel_btn = tk.Button(
        button_frame,
        text="❌ Cancel",
        command=on_cancel,
        font=("Arial", 11),
        padx=15,
        pady=8
    )
    cancel_btn.pack(side="left", padx=5)
    
    # Help text
    help_text = tk.Label(
        content_frame,
        text="💡 Tip: Double-click a device to select it quickly",
        font=("Arial", 9),
        fg="#999"
    )
    help_text.pack()
    
    # Initial refresh and start auto-refresh
    refresh_devices()
    auto_refresh()
    
    root.mainloop()
    
    return selected_device

def start_service_discovery(console):
    zeroconf = Zeroconf()
    listener = HydroponicsServiceListener(console)
    browser = ServiceBrowser(zeroconf, "_hydroponics._tcp.local.", listener)
    console.zeroconf = zeroconf
    console.listener = listener
    console.browser = browser
    return zeroconf

def main():
    """
    Main entry point - launches GUI mode by default.
    Use --console flag to start in console mode.
    """
    import sys
    
    # Check if console mode is requested
    console_mode = '--console' in sys.argv or '-c' in sys.argv
    
    # Create the console instance
    console = HydroponicsConsole()

    # Start mDNS service discovery
    zeroconf = start_service_discovery(console)
    
    if console_mode:
        # Traditional console mode
        print("Starting in console mode...")
        try:
            console.cmdloop()
        except KeyboardInterrupt:
            print("\nExiting Hydroponics console.")
        finally:
            zeroconf.close()
            scheduler.shutdown()
    else:
        # GUI mode (default)
        print("🚀 Starting GrowPod Control GUI...")
        print("🔍 Scanning for devices...")
        
        # Wait a moment for device discovery to start
        time.sleep(2)
        
        # Show device selector
        selected_device = create_device_selector_gui()
        
        if selected_device:
            # User selected a device, set it in console and launch main GUI
            console.selected_device = selected_device
            print(f"\n📱 Selected device: {selected_device}")
            print(f"🔗 Address: {devices[selected_device]['address']}\n")
            
            # Launch main control GUI
            launch_unified_gui(console)
            
            print("\n✅ GUI closed")
        else:
            print("\n❌ No device selected. Exiting.")
        
        # Cleanup
        zeroconf.close()
        scheduler.shutdown()

if __name__ == '__main__':
    main()