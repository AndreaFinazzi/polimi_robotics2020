## Finazzi Andrea
## Personal ids 10530588 - 868454

lla2odom:
valori di Fixed Point reali (approx floating point):
lat: 45.6216545
lon: 9.28155231
alt: 224.616623

Service: 
stateless, as suggested in official best practices here https://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X
input params: OdomA, OdomB
output: 3D distance

Launch file:
launches two instances of the node.
params' type is implicitly solved as floating point (http://wiki.ros.org/roslaunch/XML/param).
node.1: subscribes to front/gps and publishes what?
node.2: subscribes to obs/gps and publishes what?

Client node: 
permanent connection with simple connection recovery