from jetbot_hri.configuration import load_config
from jetbot_hri.perception import CollisionModelBackend
backend = CollisionModelBackend(load_config("config/calibration.yaml")["collision"])
print("Collision adapter loaded; inspect live output with --mode perception")

