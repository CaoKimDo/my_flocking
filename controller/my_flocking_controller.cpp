#include "my_flocking_controller.h"

void MyFlocking::WheelTurningParams::init(TConfigurationNode& node) {
   try {
      turningMechanism = NO_TURN;
      CDegrees angle;  // Angle value in degrees

      GetNodeAttribute(node, "hard_turn_angle_threshold", angle);
      hardTurnOnAngleThreshold = ToRadians(angle);
      GetNodeAttribute(node, "soft_turn_angle_threshold", angle);
      softTurnOnAngleThreshold = ToRadians(angle);
      GetNodeAttribute(node, "no_turn_angle_threshold", angle);
      noTurnAngleThreshold = ToRadians(angle);
      GetNodeAttribute(node, "max_speed", maxSpeed);
   } catch(CARGoSException& e) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", e);
   }
}

void MyFlocking::FlockingInteractionParams::init(TConfigurationNode& node) {
   try {
      GetNodeAttribute(node, "target_distance", targetDistance);
      GetNodeAttribute(node, "gain", gain);
      GetNodeAttribute(node, "exponent", exponent);
   } catch(CARGoSException& e) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking interaction parameters.", e);
   }
}

// Generalization of the Lennard-Jones potential
Real MyFlocking::FlockingInteractionParams::generalizedLennardJones(Real distance) {
   Real normDistExp = ::pow(targetDistance / distance, exponent);
   return -gain / distance * (normDistExp * normDistExp - normDistExp);
}

// Initializes hardware pointers with NULL using initializer list (to be set in init())
MyFlocking::MyFlocking() :
   diffSteeringActuator(NULL),
   footBotLightSensor(NULL),
   ledsActuator(NULL),
   omniCameraSensor(NULL) {}

void MyFlocking::Init(TConfigurationNode& node) {
   diffSteeringActuator = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   footBotLightSensor = GetSensor<CCI_FootBotLightSensor>("footbot_light");
   ledsActuator = GetActuator<CCI_LEDsActuator>("leds");
   omniCameraSensor = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");

   // Parse the config file
   try {
      wheelTurningParams.init(GetNode(node, "wheel_turning"));  // Wheel turning
      flockingInteractionParams.init(GetNode(node, "flocking"));  // Flocking-related
   } catch(CARGoSException& e) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", e);
   }

   Reset();
}

void MyFlocking::ControlStep() {
   setWheelSpeedsFromVector(calcVectorToLight() + calcFlockingInteractionVector());
}

void MyFlocking::Reset() {
   omniCameraSensor->Enable();  // Enable camera filtering
   ledsActuator->SetSingleColor(12, CColor::RED);  // Set beacon color to all red to be visible for other robots
}

CVector2 MyFlocking::calcVectorToLight() {
   const CCI_FootBotLightSensor::TReadings& tReadings = footBotLightSensor->GetReadings();  // Get light readings

   // Calculate a normalized vector that points to the closest light
   CVector2 accumulator;

   for(size_t i = 0; i < tReadings.size(); ++i)
      accumulator += CVector2(tReadings[i].Value, tReadings[i].Angle);

   // If light was detected (vector length > 0)
   if(accumulator.Length() > 0.0f) {
      accumulator.Normalize();
      accumulator *= 0.25f * wheelTurningParams.maxSpeed;  // Make the vector long as 1/4 of the max speed
   }

   return accumulator;
}

CVector2 MyFlocking::calcFlockingInteractionVector() {
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = omniCameraSensor->GetReadings();  // Get the camera readings

   // Go through the camera readings to calculate the flocking interaction vector
   if(!sReadings.BlobList.empty()) {
      CVector2 accumulator;
      Real lj;
      size_t seenRobots = 0;  // A counter to store the number of seen robot(s)

      for(size_t i = 0; i < sReadings.BlobList.size(); ++i)
         // Consider only the closest neighbors, to avoid attraction to the farthest ones. Taking 180% of d0 is a good rule of thumb.
         if(sReadings.BlobList[i]->Color == CColor::RED && sReadings.BlobList[i]->Distance < flockingInteractionParams.targetDistance * 1.80f) {
            lj = flockingInteractionParams.generalizedLennardJones(sReadings.BlobList[i]->Distance);
            accumulator += CVector2(lj, sReadings.BlobList[i]->Angle);  // Sum to accumulator

            ++seenRobots;  // Increment the counter
         }
         
      if(seenRobots > 0) {
         accumulator /= seenRobots;  // Divide the accumulator by the counter to get an average flocking force vector

         // Clamp the length of the vector to the max speed
         if(accumulator.Length() > wheelTurningParams.maxSpeed) {
            accumulator.Normalize();
            accumulator *= wheelTurningParams.maxSpeed;
         }

         return accumulator;
      } else
         return CVector2();
   } else
      return CVector2();
}

void MyFlocking::setWheelSpeedsFromVector(const CVector2& directionVector) {
   CRadians headingAngle = directionVector.Angle().SignedNormalize();  // Get the heading angle
   Real headingLength = directionVector.Length();  // Get the length of the heading vector
   // Clamp the speed so that it's not greater than maxSpeed
   Real baseAngularWheelSpeed = Min<Real>(headingLength, wheelTurningParams.maxSpeed);

   // State transition logic
   // If the robot was spinning hard, but the angle becomes small enough, downgrade to a soft turn.
   if(wheelTurningParams.turningMechanism == WheelTurningParams::HARD_TURN && Abs(headingAngle) <= wheelTurningParams.softTurnOnAngleThreshold)
      wheelTurningParams.turningMechanism = WheelTurningParams::SOFT_TURN;
   if(wheelTurningParams.turningMechanism == WheelTurningParams::SOFT_TURN) {
      if(Abs(headingAngle) > wheelTurningParams.hardTurnOnAngleThreshold)  // If the angle grows too large, escalate to hard turn.
         wheelTurningParams.turningMechanism = WheelTurningParams::HARD_TURN;
      else if(Abs(headingAngle) <= wheelTurningParams.noTurnAngleThreshold)  // If the angle shrinks very small, downgrade to no turn (straight).
         wheelTurningParams.turningMechanism = WheelTurningParams::NO_TURN;
   }
   if(wheelTurningParams.turningMechanism == WheelTurningParams::NO_TURN) {
      if(Abs(headingAngle) > wheelTurningParams.hardTurnOnAngleThreshold)  // If the angle is very large, jump straight to hard turn.
         wheelTurningParams.turningMechanism = WheelTurningParams::HARD_TURN;
      else if(Abs(headingAngle) > wheelTurningParams.noTurnAngleThreshold)  // If the angle is moderate, switch to soft turn.
         wheelTurningParams.turningMechanism = WheelTurningParams::SOFT_TURN;
   }

   // Wheel speeds based on current turning state
   Real speed1, speed2;
   switch(wheelTurningParams.turningMechanism) {
      case WheelTurningParams::NO_TURN: {
         // Just go straight
         speed1 = baseAngularWheelSpeed;
         speed2 = baseAngularWheelSpeed;

         break;
      }
      case WheelTurningParams::SOFT_TURN: {
         // Both wheels go straight, but one is faster than the other
         Real speedFactor = (wheelTurningParams.hardTurnOnAngleThreshold - Abs(headingAngle)) / wheelTurningParams.hardTurnOnAngleThreshold;
         speed1 = baseAngularWheelSpeed - baseAngularWheelSpeed * (1.0 - speedFactor);
         speed2 = baseAngularWheelSpeed + baseAngularWheelSpeed * (1.0 - speedFactor);

         break;
      }
      case WheelTurningParams::HARD_TURN: {
         // Opposite wheel speeds
         speed1 = -wheelTurningParams.maxSpeed;
         speed2 =  wheelTurningParams.maxSpeed;

         break;
      }
   }

   // Apply the calculated speeds to the appropriate wheels
   Real leftWheelSpeed, rightWheelSpeed;
   if(headingAngle > CRadians::ZERO) {
      // Turn Left
      leftWheelSpeed  = speed1;
      rightWheelSpeed = speed2;
   } else {
      // Turn Right
      leftWheelSpeed  = speed2;
      rightWheelSpeed = speed1;
   }

   diffSteeringActuator->SetLinearVelocity(leftWheelSpeed, rightWheelSpeed);  // Finally, set the wheel speeds
}

REGISTER_CONTROLLER(MyFlocking, "my_flocking_controller")
