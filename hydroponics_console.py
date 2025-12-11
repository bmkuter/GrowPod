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
from tkinter import messagebox, ttk

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

def prompt_unified_schedule_manager(initial_light_schedule=None, initial_planter_schedule=None):
    """
    Comprehensive schedule management GUI combining:
    1. Hourly schedules (Light curve + Planter intervals)
    2. Routine commands (Fill/Empty/Maintenance schedules)
    
    Args:
        initial_light_schedule: List of 24 integers (0-100) for light schedule
        initial_planter_schedule: List of 24 integers (0-100) for planter schedule
    
    Returns a dictionary with both schedule types or None if cancelled.
    """
    result = None

    def on_save_all():
        nonlocal result
        
        # Collect hourly schedules
        light_schedule = [light_scales[i].get() for i in range(24)]
        planter_schedule = [planter_scales[i].get() for i in range(24)]
        
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
    root.geometry("750x750")
    
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
                         text="Manage hourly schedules (Light & Planter) and routine commands (Fill/Empty/Maintenance)",
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
            'led': 'Set LED brightness: led <value> [channel]',
            'schedule_actuators': 'Set actuator schedule: schedule_actuators',
            'manage_schedules': 'Unified schedule manager (hourly + routines): manage_schedules',
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
                # Handle routine commands specially
                if actuator == 'routine' and 'command' in command:
                    # Post to /api/routines/<command_name>
                    routine_cmd = command['command']
                    url = f"https://{device_info['address']}:{device_info['port']}/api/routines/{routine_cmd}"
                    response = self.session.post(url, json={}, timeout=10, verify=False)
                    print(f"Routine '{routine_cmd}' started: {response.text}")
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
        
        # Open the unified GUI window (load with current schedules from device)
        config = prompt_unified_schedule_manager(light_sched, planter_sched)
        
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