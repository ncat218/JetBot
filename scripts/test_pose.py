from jetbot_hri.configuration import load_config
from jetbot_hri.perception import TRTPOSEBackend
backend = TRTPOSEBackend(load_config("config/calibration.yaml")["pose"])
print("Pose adapter loaded; inspect live output with --mode perception")

