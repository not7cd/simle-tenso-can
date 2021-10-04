# simle-tenso-can

Before a rocket can liftoff, we need to tank it. In order to know how much oxidizer has been pumped into the tank we want to weight it.

This is where tenso-can comes in. It has a tensometer, which weighs rocket mounter on a launch rail. 
It is placed outside and is connected with 4 wire cable, two providing power to charge the battery, and two other for CAN.
This cable is then ripped after the ignition.

Data is sent with UAVCAN frames. While some metrics from inside the rocket are displed on LCD screen, 
giving the ground crew the status of a rocket during tanking process.
