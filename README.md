# Leader follower movement control
This project aims to emulate the leader-follower paradigm through developing and merging
a variety of vision and movement control algorithms implemented on a group of mobile robots.
This is in effort to effectively implement leader-follower movement, with an emphasis on
tracking performance, algorithmic simplicity and cost. The Leader leads the Follower either
by line following on a track, using a pre-programmed course or following another subject. The
following robot then uses its camera to track the position, velocity and course of the leader in
an attempt to emulate its path accurately.

## Hardware and archetecture
Utilising a Rapberry Pi as a gateway, hosting a wireless network and running an MQTT broker. This project uses two Waveshare [jetbots](https://www.waveshare.com/wiki/JetBot_AI_Kit) as the agents.
![](assets/FPY%20architecture.png)
