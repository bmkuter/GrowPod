"""
GrowPod Calendar Scheduler

Manages calendar-based events and schedules for GrowPod devices.
Events are stored in SQLite and executed by APScheduler when triggered.
"""

import sqlite3
import json
from datetime import datetime, timedelta
from typing import List, Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class CalendarEvent:
    """Represents a calendar event for a GrowPod device"""
    
    def __init__(self, event_id: Optional[int] = None, device_name: str = "", 
                 event_type: str = "", title: str = "", description: str = "",
                 scheduled_time: datetime = None, duration_minutes: int = 0,
                 command_type: str = "", command_params: dict = None,
                 recurrence: str = "none", recurrence_end: datetime = None,
                 enabled: bool = True, color: str = "#3498db"):
        self.event_id = event_id
        self.device_name = device_name
        self.event_type = event_type  # 'dosing', 'water_change', 'maintenance', 'milestone', 'custom'
        self.title = title
        self.description = description
        self.scheduled_time = scheduled_time or datetime.now()
        self.duration_minutes = duration_minutes
        self.command_type = command_type  # 'dose_food', 'drain_fill', 'custom_api', etc.
        self.command_params = command_params or {}
        self.recurrence = recurrence  # 'none', 'daily', 'weekly', 'biweekly', 'monthly'
        self.recurrence_end = recurrence_end
        self.enabled = enabled
        self.color = color
        
    def to_dict(self) -> dict:
        """Convert to dictionary for storage"""
        return {
            'event_id': self.event_id,
            'device_name': self.device_name,
            'event_type': self.event_type,
            'title': self.title,
            'description': self.description,
            'scheduled_time': self.scheduled_time.isoformat(),
            'duration_minutes': self.duration_minutes,
            'command_type': self.command_type,
            'command_params': json.dumps(self.command_params),
            'recurrence': self.recurrence,
            'recurrence_end': self.recurrence_end.isoformat() if self.recurrence_end else None,
            'enabled': self.enabled,
            'color': self.color
        }
    
    @classmethod
    def from_dict(cls, data: dict) -> 'CalendarEvent':
        """Create event from dictionary"""
        event = cls()
        event.event_id = data.get('event_id')
        event.device_name = data.get('device_name', '')
        event.event_type = data.get('event_type', '')
        event.title = data.get('title', '')
        event.description = data.get('description', '')
        event.scheduled_time = datetime.fromisoformat(data['scheduled_time'])
        event.duration_minutes = data.get('duration_minutes', 0)
        event.command_type = data.get('command_type', '')
        event.command_params = json.loads(data.get('command_params', '{}'))
        event.recurrence = data.get('recurrence', 'none')
        recurrence_end_str = data.get('recurrence_end')
        event.recurrence_end = datetime.fromisoformat(recurrence_end_str) if recurrence_end_str else None
        event.enabled = bool(data.get('enabled', True))
        event.color = data.get('color', '#3498db')
        return event


class CalendarScheduler:
    """Manages calendar events and scheduling for GrowPod devices"""
    
    def __init__(self, db_path: str = 'growpod_calendar.db'):
        self.db_path = db_path
        self._init_database()
        
    def _init_database(self):
        """Initialize the calendar database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS calendar_events (
                event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                device_name TEXT NOT NULL,
                event_type TEXT NOT NULL,
                title TEXT NOT NULL,
                description TEXT,
                scheduled_time TEXT NOT NULL,
                duration_minutes INTEGER DEFAULT 0,
                command_type TEXT,
                command_params TEXT,
                recurrence TEXT DEFAULT 'none',
                recurrence_end TEXT,
                enabled INTEGER DEFAULT 1,
                color TEXT DEFAULT '#3498db',
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                updated_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS event_execution_log (
                log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_id INTEGER,
                execution_time TEXT NOT NULL,
                success INTEGER,
                error_message TEXT,
                response_data TEXT,
                FOREIGN KEY (event_id) REFERENCES calendar_events(event_id)
            )
        ''')
        
        conn.commit()
        conn.close()
        logger.info(f"Calendar database initialized at {self.db_path}")
    
    def add_event(self, event: CalendarEvent) -> int:
        """Add a new calendar event, returns event_id"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        event_data = event.to_dict()
        del event_data['event_id']  # Auto-increment
        
        columns = ', '.join(event_data.keys())
        placeholders = ', '.join(['?' for _ in event_data])
        
        cursor.execute(
            f'INSERT INTO calendar_events ({columns}) VALUES ({placeholders})',
            list(event_data.values())
        )
        
        event_id = cursor.lastrowid
        conn.commit()
        conn.close()
        
        logger.info(f"Added calendar event: {event.title} (ID: {event_id})")
        return event_id
    
    def update_event(self, event: CalendarEvent):
        """Update an existing calendar event"""
        if not event.event_id:
            raise ValueError("Event ID required for update")
        
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        event_data = event.to_dict()
        event_id = event_data.pop('event_id')
        
        set_clause = ', '.join([f'{k} = ?' for k in event_data.keys()])
        set_clause += ', updated_at = CURRENT_TIMESTAMP'
        
        cursor.execute(
            f'UPDATE calendar_events SET {set_clause} WHERE event_id = ?',
            list(event_data.values()) + [event_id]
        )
        
        conn.commit()
        conn.close()
        logger.info(f"Updated calendar event ID: {event_id}")
    
    def delete_event(self, event_id: int):
        """Delete a calendar event"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('DELETE FROM calendar_events WHERE event_id = ?', (event_id,))
        conn.commit()
        conn.close()
        logger.info(f"Deleted calendar event ID: {event_id}")
    
    def delete_all_events(self, device_name: Optional[str] = None) -> int:
        """
        Delete all calendar events, optionally filtered by device.
        Returns the number of events deleted.
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        if device_name:
            cursor.execute('SELECT COUNT(*) FROM calendar_events WHERE device_name = ?', (device_name,))
            count = cursor.fetchone()[0]
            cursor.execute('DELETE FROM calendar_events WHERE device_name = ?', (device_name,))
            logger.info(f"Deleted {count} calendar events for device: {device_name}")
        else:
            cursor.execute('SELECT COUNT(*) FROM calendar_events')
            count = cursor.fetchone()[0]
            cursor.execute('DELETE FROM calendar_events')
            logger.info(f"Deleted all {count} calendar events")
        
        conn.commit()
        conn.close()
        return count
    
    def get_event(self, event_id: int) -> Optional[CalendarEvent]:
        """Get a specific event by ID"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        cursor.execute('SELECT * FROM calendar_events WHERE event_id = ?', (event_id,))
        row = cursor.fetchone()
        conn.close()
        
        if row:
            return CalendarEvent.from_dict(dict(row))
        return None
    
    def get_events_for_device(self, device_name: str, 
                              start_date: Optional[datetime] = None,
                              end_date: Optional[datetime] = None) -> List[CalendarEvent]:
        """Get all events for a device within date range"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        query = 'SELECT * FROM calendar_events WHERE device_name = ?'
        params = [device_name]
        
        if start_date:
            query += ' AND scheduled_time >= ?'
            params.append(start_date.isoformat())
        
        if end_date:
            query += ' AND scheduled_time <= ?'
            params.append(end_date.isoformat())
        
        query += ' ORDER BY scheduled_time'
        
        cursor.execute(query, params)
        rows = cursor.fetchall()
        conn.close()
        
        return [CalendarEvent.from_dict(dict(row)) for row in rows]
    
    def get_events_for_date(self, device_name: str, date: datetime) -> List[CalendarEvent]:
        """Get all events for a specific date"""
        start = datetime(date.year, date.month, date.day, 0, 0, 0)
        end = start + timedelta(days=1)
        return self.get_events_for_device(device_name, start, end)
    
    def get_upcoming_events(self, device_name: str, hours: int = 24) -> List[CalendarEvent]:
        """Get upcoming events within specified hours"""
        now = datetime.now()
        end_time = now + timedelta(hours=hours)
        
        events = self.get_events_for_device(device_name, now, end_time)
        return [e for e in events if e.enabled]
    
    def log_execution(self, event_id: int, success: bool, 
                     error_message: str = None, response_data: str = None):
        """Log event execution result"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO event_execution_log 
            (event_id, execution_time, success, error_message, response_data)
            VALUES (?, ?, ?, ?, ?)
        ''', (event_id, datetime.now().isoformat(), int(success), error_message, response_data))
        
        conn.commit()
        conn.close()
    
    def get_execution_log(self, event_id: int, limit: int = 10) -> List[Dict]:
        """Get execution log for an event"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT * FROM event_execution_log 
            WHERE event_id = ? 
            ORDER BY execution_time DESC 
            LIMIT ?
        ''', (event_id, limit))
        
        rows = cursor.fetchall()
        conn.close()
        
        return [dict(row) for row in rows]
    
    def create_dosing_events_from_profile(self, device_name: str, profile_data: dict, 
                                         start_date: datetime) -> List[int]:
        """
        Create recurring dosing events from plant profile feeding schedule.
        Returns list of created event IDs.
        """
        event_ids = []
        
        feeding_schedule = profile_data.get('nutrition', {}).get('feeding_schedule', [])
        plant_name = profile_data.get('plant_info', {}).get('name', 'Unknown Plant')
        
        for stage in feeding_schedule:
            stage_name = stage.get('stage', 'unknown')
            day_start = stage.get('day_start', 0)
            day_end = stage.get('day_end', 0)
            concentration = stage.get('concentration_ml_per_liter', 0)
            frequency_per_week = stage.get('frequency_per_week', 0)
            notes = stage.get('notes', '')
            
            if frequency_per_week == 0:
                continue
            
            # Calculate days between feedings
            days_between = 7 / frequency_per_week
            
            # Create events for this stage
            # Use < instead of <= to avoid overlap at stage boundaries
            current_day = day_start
            while current_day < day_end:
                event_date = start_date + timedelta(days=current_day)
                
                # Create dosing event
                event = CalendarEvent(
                    device_name=device_name,
                    event_type='dosing',
                    title=f'{plant_name} - {stage_name.capitalize()} Feeding',
                    description=f'{concentration}ml/L - {notes}',
                    scheduled_time=datetime(event_date.year, event_date.month, event_date.day, 9, 0),  # 9 AM default
                    duration_minutes=5,
                    command_type='dose_food',
                    command_params={
                        'dose': 750,  # Default duration, user can adjust
                        'speed': 100,
                        'concentration': concentration,
                        'stage': stage_name
                    },
                    recurrence='none',
                    enabled=True,
                    color='#9b59b6'  # Purple for feeding
                )
                
                event_id = self.add_event(event)
                event_ids.append(event_id)
                
                current_day += days_between
        
        logger.info(f"Created {len(event_ids)} dosing events from plant profile")
        return event_ids
    
    def create_water_change_events_from_profile(self, device_name: str, profile_data: dict,
                                                start_date: datetime) -> List[int]:
        """
        Create recurring water change events from plant profile water_change_schedule.
        Returns list of created event IDs.
        """
        event_ids = []
        
        water_change_schedule = profile_data.get('water_change_schedule', {}).get('schedule', [])
        plant_name = profile_data.get('plant_info', {}).get('name', 'Unknown Plant')
        procedure = profile_data.get('water_change_schedule', {}).get('procedure', {})
        
        drain_target = procedure.get('drain_target_mm', 75)
        refill_target = procedure.get('refill_target_mm', 57)
        
        for stage in water_change_schedule:
            stage_name = stage.get('stage', 'unknown')
            day_start = stage.get('day_start', 0)
            day_end = stage.get('day_end', 0)
            interval_days = stage.get('interval_days', 7)
            notes = stage.get('notes', '')
            
            # Create events for this stage
            # Use < instead of <= to avoid overlap at stage boundaries
            current_day = day_start
            while current_day < day_end:
                event_date = start_date + timedelta(days=current_day)
                
                # Create water change event
                event = CalendarEvent(
                    device_name=device_name,
                    event_type='water_change',
                    title=f'{plant_name} - {stage_name.capitalize()} Water Change',
                    description=f'Drain & refill reservoir - {notes}',
                    scheduled_time=datetime(event_date.year, event_date.month, event_date.day, 10, 0),  # 10 AM default
                    duration_minutes=15,
                    command_type='drain_fill',
                    command_params={
                        'drain_target_mm': drain_target,
                        'refill_target_mm': refill_target,
                        'stage': stage_name,
                        'settling_minutes': procedure.get('post_change_settling_minutes', 5)
                    },
                    recurrence='none',
                    enabled=True,
                    color='#3498db'  # Blue for water changes
                )
                
                event_id = self.add_event(event)
                event_ids.append(event_id)
                
                current_day += interval_days
        
        logger.info(f"Created {len(event_ids)} water change events from plant profile")
        return event_ids
    
    def create_all_events_from_profile(self, device_name: str, profile_data: dict,
                                      start_date: datetime) -> Dict[str, List[int]]:
        """
        Create all calendar events from plant profile (dosing + water changes).
        Returns dictionary with event type keys and lists of created event IDs.
        """
        result = {
            'dosing_events': [],
            'water_change_events': []
        }
        
        # Create dosing events
        result['dosing_events'] = self.create_dosing_events_from_profile(
            device_name, profile_data, start_date
        )
        
        # Create water change events
        result['water_change_events'] = self.create_water_change_events_from_profile(
            device_name, profile_data, start_date
        )
        
        total_events = len(result['dosing_events']) + len(result['water_change_events'])
        logger.info(f"Created {total_events} total events from plant profile "
                   f"({len(result['dosing_events'])} dosing, {len(result['water_change_events'])} water changes)")
        
        return result
