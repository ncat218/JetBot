from jetbot_hri.platform import get_wifi_status
from dataclasses import asdict
import json
print(json.dumps(asdict(get_wifi_status())))

