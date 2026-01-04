# Plant Profile System

This directory contains plant profiles that define growth parameters and care instructions for different plant varieties.

## Profile Structure

Each plant profile is a JSON file with the following structure:

```json
{
  "profile_version": "1.0",
  "plant_info": {
    "name": "Plant Name",
    "variety": "Specific Variety",
    "botanical_name": "Scientific name",
    "seed_source": "URL or vendor",
    "description": "Brief description"
  },
  "growth_parameters": {
    "germination_days": 7,
    "harvest_days": 85,
    "growth_stages": [...]
  },
  "environmental": {
    "temperature_range_c": [18, 28],
    "humidity_range_percent": [50, 70],
    "light_hours_per_day": 16
  },
  "nutrition": {
    "nutrient_solution": "FloraGro",
    "concentration_ml_per_liter": 2.5,
    "feeding_schedule": [...]
  },
  "watering": {
    "reservoir_capacity_ml": 400,
    "target_water_level_mm": 57,
    "empty_level_mm": 75,
    "refill_interval_hours": 168
  }
}
```

## Usage

1. Place profile JSON files in this directory
2. The GrowPod Controller GUI will scan and display available profiles
3. Select a profile to view details and apply to your device
4. The GUI will push appropriate schedules to the ESP32 device

## Current Profiles

- `micro_tom_tomato.json` - Micro Tom Tomato (compact variety for hydroponic systems)

## Adding New Profiles

1. Copy an existing profile as a template
2. Update all fields with plant-specific information
3. Test the profile on a development device before production use
4. Document any specific requirements or notes
