# scheduler.py
import sys
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.jobstores.sqlalchemy import SQLAlchemyJobStore
import logging

# Setup Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler("hydroponics.log"),
        logging.StreamHandler(sys.stdout)  # Now sys is defined
    ]
)
logger = logging.getLogger(__name__)

# Initialize the scheduler with a persistent job store
jobstores = {
    'default': SQLAlchemyJobStore(url='sqlite:///hydroponics_scheduler.sqlite')
}
scheduler = BackgroundScheduler(jobstores=jobstores, timezone="UTC")
scheduler.start()
logger.info("APScheduler started with persistent job store.")
