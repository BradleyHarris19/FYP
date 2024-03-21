import paho.mqtt.subscribe as subscribe 
import argparse

parser = argparse.ArgumentParser(description="Listen to the Drive mqtt messages from the jetbot")
parser.add_argument("--bot", type=str, default="jetbot1", help="jetbot to listen to")
args = parser.parse_args()

while True:
    fwd = subscribe.simple(f"{args.bot}/drive/fwd", hostname="localhost")
    rot = subscribe.simple(f"{args.bot}/drive/rot", hostname="localhost")
    left = subscribe.simple(f"{args.bot}/drive/L", hostname="localhost")
    right = subscribe.simple(f"{args.bot}/drive/R", hostname="localhost")
    speed = subscribe.simple(f"{args.bot}/drive/S", hostname="localhost")  
    print(f"Forward:{float(fwd.payload):.3f} Rotation:{float(rot.payload):.3f}, Left:{float(left.payload):.3f}, Right:{float(right.payload):.3f}, Speed:{float(speed.payload):.3f}")
