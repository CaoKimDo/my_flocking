# Execution loop explanation
1. Init
- The controller connects to robot hardware:
    + Differential steering actuator (wheels)
    + Light sensor (detects goal light)
    + Omni-camera sensor (detects neighbors’ LEDs)
    + LEDs actuator (sets beacon color)
- Reads parameters from the XML config:
    + Wheel turning threshold
    + Flocking interaction parameters
- Resets robot state:
    + Enables omni-camera
    + Sets beacon LED to red

2. ControlStep (runs every simulation tick)
- Calculate light vector:
    + calcVectorToLight() produces a vector pointing toward the brightest light source.
    + Always normalized and scaled to 25% max speed
- Calculate flocking vector:
    + calcFlockingInteractionVector() produces a vector based on neighbors’ positions (using generalized Lennard-Jones potential).
    + Averaged across neighbors, clamped to max speed
- Combine vectors:
    + Add light vector + flocking vector -> final desired movement vector
- Convert to wheel speeds
    + setWheelSpeedsFromVector() interprets the vector’s angle and length.
    + Uses state machine logic (NO_TURN, SOFT_TURN, HARD_TURN)
    + Sets left/right wheel speeds accordingly.

# State transition logic
- If heading angle is small, then NO_TURN (straight).
- If heading angle is moderate, then SOFT_TURN (curve).
- If heading angle is large, then HARD_TURN (spin).
. Smooth transitions prevent jitter

# Reset
- Can be called to re-enable sensors and reset LED
- Ensures robot starts fresh if simulation is restarted

# TO RUN:
mkdir build
cd build
cmake ..
make
cd ..
argos3 -c experiments/my_flocking.argos
