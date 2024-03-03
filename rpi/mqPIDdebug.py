import paho.mqtt.subscribe as subscribe 


while True:
    inpt = subscribe.simple("jetbot1/steering/pid/in", hostname="localhost")
    err = subscribe.simple("jetbot1/steering/pid/error", hostname="localhost")
    p = subscribe.simple("jetbot1/steering/pid/P_term", hostname="localhost")
    i = subscribe.simple("jetbot1/steering/pid/I_term", hostname="localhost")
    d = subscribe.simple("jetbot1/steering/pid/D_term", hostname="localhost")  
    out = subscribe.simple("jetbot1/steering/pid/out", hostname="localhost")
    print(f"in:{float(inpt.payload):.3f} err:{float(err.payload):.3f}, P:{float(p.payload):.3f}, i:{float(i.payload):.3f}, d:{float(d.payload):.3f}, out:{float(out.payload):.3f}")
