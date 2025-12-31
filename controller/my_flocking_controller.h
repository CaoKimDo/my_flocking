#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/configuration/argos_configuration.h>  // For XML parsing
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

using namespace argos;

class MyFlocking : public CCI_Controller {
public:
   struct WheelTurningParams {
      enum TurningMechanism {
         NO_TURN = 0, // Go straight
         SOFT_TURN,  // Both wheels are turning forwards, but at different speeds.
         HARD_TURN  // Wheels are turning with opposite speeds.
      } turningMechanism;

      // Angle thresholds (in radians)
      CRadians hardTurnOnAngleThreshold;
      CRadians softTurnOnAngleThreshold;
      CRadians noTurnAngleThreshold;

      Real maxSpeed;  // Maximum wheel speed

      void init(TConfigurationNode& node);
   };

   struct FlockingInteractionParams {
      Real targetDistance;  // Target robot-robot distance in cm (d0)
      Real gain;  // Gain of the Lennard-Jones potential (g)
      /* This scales the magnitude of the force.
      Larger gain -> faster response
      Smaller gain -> gentle motion */
      Real exponent;  // Exponent of the Lennard-Jones potential (n)
      /* This controls how sharp the interaction is.
      Large n -> very stiff repulsion, narrow attraction
      Small n -> smoother, long-range interaction */

      void init(TConfigurationNode& node);

      Real generalizedLennardJones(Real distance);
      /* This function computes the force magnitude based on the distance and is tuned for behavioral interaction.
      With r = distance between two robots, we have:
         F(r) = -g / r * (A - B)
      where A = (d0 / r)^2n -> repulsion, B = (d0 / r)^n -> attraction */
   };

   MyFlocking();  // Class constructor

   virtual ~MyFlocking() {}  // Class destructor

   // Overridden virtual functions from the CCI_Controller class
   virtual void Init(TConfigurationNode& node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

protected:
   virtual CVector2 calcVectorToLight();  // Calculates the vector to the light
   virtual CVector2 calcFlockingInteractionVector();  // Calculates the flocking interaction vector

   // Gets a direction vector as input and transforms it into wheel actuations
   void setWheelSpeedsFromVector(const CVector2& directionVector);

private:
   CCI_DifferentialSteeringActuator* diffSteeringActuator;  // Pointer to the differential steering actuator (controls the left & right wheels)
   CCI_FootBotLightSensor* footBotLightSensor;  // Pointer to the foot-bot light sensor (detects the light source)
   CCI_LEDsActuator* ledsActuator;  // Pointer to the LEDs actuator (controls the robot's LEDs)
   CCI_ColoredBlobOmnidirectionalCameraSensor* omniCameraSensor;  // Pointer to the omnidirectional camera sensor (detects colored blobs (other robots))
   
   WheelTurningParams wheelTurningParams;  // The wheel turning parameters (control how the robot turns) 
   FlockingInteractionParams flockingInteractionParams;  // The flocking interaction parameters (control how the robot interacts with its neighbors)
};
