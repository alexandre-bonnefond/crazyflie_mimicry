# crazyflie_mimicry

## Project Overview

This project aims at controlling multiple crazyflies (https://www.bitcraze.io/) using the motion of the body. The idea is to develop different versions of the software adapted to different appplications but also to the variety of users. 
For example, one of the versions will be adapted to disabled people where only the motion of the eyes will be sufficient to move the quadcopters. Another version will be suited for artistic applications where the 
crazyflies will move "accordingly" to some dancer's movements. 

For this project, the motion of the crazyflies is a combination of 3 controllers: 
  1. The first one is a low-level controller used to stabilize each crazyflie independently. For this, we use the built-in PID controller combined to a state estimator (https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/).
  2. The second one is a decentralized controller based on the flocking behavior. This controller will handle the interactions between the different crazyflies in order to create a bio-inspired, collision-free and cohesive motion.
     This controller will also act as a fail-safe whenever unfeasible movements will be fed to the high-level controller.
  3. The last controller is the high-level controller. It is the one that will handle the conversion between body movements and crazyflie commands. For example the motion of the hand can be seen as a wind blow for the swarm. Also, some movement combinations
     or signs can correspond to distinct maneuvers or patterns formation.

Most of the work lies into the design and implementation of the second and third controllers. The first controller can be modified if needed, as well as the state estimator but I am confident about their efficiency given the number of amazing projects 
that already exist out there. Also, I already tried some basic swarm controllers with my crazyflies and it works like a charm.



## Set-up

This project is intended for indoor applications as it requires a positioning system compatible with the crazyflies and also because we don't want external disturbances to hinder the motion. 
My current set-up is composed of:
  * 3x Crazyflie
  * 2x Lighthouse V2 base station
  * 3x Lighthouse positioning deck

Any positioning system will work as only the position (and the derived states) of the crazyflies is required. This also means that we could use this software for outdoors applications but with bigger quadcopters (not crazyflie) equipped with GPS sensors
for example. But for now we will only focus on the indoor set-up. 

The number of quadcopters is also not a requirement, this project aims at being scalable to any number of crazyflies with equivalent performances.

## Installation



