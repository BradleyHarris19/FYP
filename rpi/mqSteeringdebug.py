#!/bin/python3
import paho.mqtt.subscribe as subscribe 
import argparse

# This script listens to the Steering mqtt messages from the jetbot

parser = argparse.ArgumentParser(description="Listen to the mqtt messages from the jetbot when line following")
parser.add_argument("--bot", type=str, default="jetbot1", help="jetbot to listen to")
args = parser.parse_args()

while True:
    err = subscribe.simple(f"{args.bot}./steering/pid/error", hostname="localhost")
    p = subscribe.simple(f"{args.bot}/steering/pid/P_term", hostname="localhost")
    i = subscribe.simple(f"{args.bot}/steering/pid/I_term", hostname="localhost")
    d = subscribe.simple(f"{args.bot}/steering/pid/D_term", hostname="localhost")  
    out = subscribe.simple(f"{args.bot}/steering/pid/out", hostname="localhost")
    print(f"angle:{float(err.payload):.3f}, P:{float(p.payload):.3f}, I:{float(i.payload):.3f}, d:{float(d.payload):.3f}, out:{float(out.payload):.3f}")
