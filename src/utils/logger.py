import logging
import sys
from datetime import datetime, timezone

#Default logger settings to hide known annoying messages 
logging.getLogger("matplotlib").setLevel(logging.WARNING)
logging.getLogger("PIL").setLevel(logging.WARNING)

class AsciiFormatter(logging.Formatter):
    def format(self, record):
        date_str = datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S.%f')
        return f"{date_str} - {record.levelname} - {record.name} - {record.getMessage()}"


def get_logger(name=__name__, level=logging.DEBUG):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = True  

    if not logger.handlers:
        formatter = AsciiFormatter()
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    return logger
