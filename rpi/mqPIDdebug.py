import paho.mqtt.subscribe as subscribe 
import argparse

parser = argparse.ArgumentParser(description="Listen to the PID mqtt messages from the jetbot")
parser.add_argument("--bot", type=str, default="jetbot1", help="jetbot to listen to")
parser.add_argument("--pid", type=str, default="steering", help="listen to steering or forward pid messages")
args = parser.parse_args()

while True:
    inpt = subscribe.simple(f"{args.bot}/{args.pid}/pid/in", hostname="localhost")
    err = subscribe.simple(f"{args.bot}/{args.pid}/pid/error", hostname="localhost")
    p = subscribe.simple(f"{args.bot}/{args.pid}/pid/P_term", hostname="localhost")
    i = subscribe.simple(f"{args.bot}/{args.pid}/pid/I_term", hostname="localhost")
    d = subscribe.simple(f"{args.bot}/{args.pid}/pid/D_term", hostname="localhost")  
    out = subscribe.simple(f"{args.bot}/{args.pid}/pid/out", hostname="localhost")
    print(f"in:{float(inpt.payload):.3f} err:{float(err.payload):.3f}, P:{float(p.payload):.3f}, i:{float(i.payload):.3f}, d:{float(d.payload):.3f}, out:{float(out.payload):.3f}")
