//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 20012 by Christian Gloor
// Modified by Ronja Gueldenring
//

#include "ped_agent.h"
#include "ped_obstacle.h"
#include "ped_scene.h"
#include "ped_waypoint.h"
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

using namespace std;

int Ped::Tagent::staticid = 0;
default_random_engine generator;

namespace
{
// File-local controller state.
std::unordered_map<std::string, double> smoothedLateralCorrectionByAgent;

struct ExternalRobotAvoidanceState
{
  int passingSide = 0;
};

std::unordered_map<std::string, ExternalRobotAvoidanceState>
    externalRobotAvoidanceStateByAgent;

double clampValue(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(value, maximum));
}
}

/// Default Constructor
Ped::Tagent::Tagent()
{
  id = std::to_string(staticid++);
  p.x = 0;
  p.y = 0;
  p.z = 0;
  v.x = 0;
  v.y = 0;
  v.z = 0;
  type = ADULT;
  scene = nullptr;
  teleop = false;

  // assign random maximal speed in m/s
  normal_distribution<double> distribution(0.6, 0.2);
  vmax = distribution(generator);
  vmaxDefault = vmax;
  forceFactorDesired = 1.0;
  forceFactorSocial = 2.1;
  forceFactorObstacle = 10.0;

  // Robot interactions remain disabled while the obstacle model is isolated.
  forceFactorRobot = 0.0;

  forceSigmaObstacle = 0.8;
  forceSigmaRobot = 0.3 * vmax / 0.4;  // retained for header/API compatibility

  agentRadius = 0.35;
  relaxationTime = 0.5;
  robotPosDiffScalingFactor = 5;
  obstacleForceRange = 2.0;

  keepDistanceForceDistanceDefault = 0.8;
  keepDistanceForceDistance = keepDistanceForceDistanceDefault;
  keepDistanceTo = Tvector(0.0, 0.0);

  desiredforce = Ped::Tvector(0.0, 0.0);
  socialforce = Ped::Tvector(0.0, 0.0);
  obstacleforce = Ped::Tvector(0.0, 0.0);
  robotforce = Ped::Tvector(0.0, 0.0);
  keepdistanceforce = Ped::Tvector(0.0, 0.0);
  myforce = Ped::Tvector(0.0, 0.0);

  ROS_DEBUG("created agent with id: %s", id.c_str());
  // ROS_INFO("created agent with id: %s", "100");
}

/// Destructor
Ped::Tagent::~Tagent()
{
  smoothedLateralCorrectionByAgent.erase(id);
  externalRobotAvoidanceStateByAgent.erase(id);
}

/// Assigns a Tscene to the agent. Tagent uses this to iterate over all
/// obstacles and other agents in a scene.
/// The scene will invoke this function when Tscene::addAgent() is called.
/// \warning Bad things will happen if the agent is not assigned to a scene. But
/// usually, Tscene takes care of that.
/// \param   *s A valid Tscene initialized earlier.
void Ped::Tagent::assignScene(Ped::Tscene *sceneIn) { scene = sceneIn; }

void Ped::Tagent::removeAgentFromNeighbors(const Ped::Tagent *agentIn)
{
  // search agent in neighbors, and remove him
  set<const Ped::Tagent *>::iterator foundNeighbor = neighbors.find(agentIn);
  if (foundNeighbor != neighbors.end())
    neighbors.erase(foundNeighbor);
}

/// Sets the maximum velocity of an agent (vmax). Even if pushed by other
/// agents, it will not move faster than this.
/// \param   pvmax The maximum velocity. In scene units per timestep, multiplied
/// by the simulation's precision h.
void Ped::Tagent::setVmax(double pvmax) { vmax = pvmax; }

/// Defines how much the position difference between this agent
/// and a robot is scaled: the bigger the number is, the smaller
/// the position based force contribution will be.
/// \param   scalingFactor should be positive.
void Ped::Tagent::setRobotPosDiffScalingFactor(double scalingFactor)
{
  if (scalingFactor > 0)
  {
    robotPosDiffScalingFactor = scalingFactor;
  }
}

/// Sets the agent's position. This, and other getters returning coordinates,
/// will eventually changed to returning a
/// Tvector.
/// \param   px Position x
/// \param   py Position y
/// \param   pz Position z
void Ped::Tagent::setPosition(double px, double py, double pz)
{
  p.x = px;
  p.y = py;
  p.z = pz;
}

/// Sets the factor by which the desired force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorDesired(double f) { forceFactorDesired = f; }

/// Sets the factor by which the social force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorSocial(double f) { forceFactorSocial = f; }

/// Sets the factor by which the obstacle force is multiplied. Values between 0
/// and about 10 do make sense.
/// \param   f The factor
void Ped::Tagent::setForceFactorObstacle(double f) { forceFactorObstacle = f; }

double Ped::Tagent::keepDistanceForceFunction(double distance)
{
  return -10 * (distance - keepDistanceForceDistance);
}

// force to keep a specific distance
Ped::Tvector Ped::Tagent::keepDistanceForce()
{
  Tvector diff = p - keepDistanceTo;
  if (diff.lengthSquared() <= 1e-12)
  {
    return Tvector(0.0, 0.0);
  }

  Tvector direction = diff.normalized();
  double magnitude = keepDistanceForceFunction(diff.length());
  return direction * magnitude;
}

/// Calculates the force between this agent and the next assigned waypoint.
/// If the waypoint has been reached, the next waypoint in the list will be
/// selected.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::desiredForce()
{
  // get destination
  Twaypoint *waypoint = getCurrentWaypoint();

  // if there is no destination, don't move
  if (waypoint == nullptr)
  {
    desiredDirection = Ped::Tvector();
    Tvector antiMove = -v / relaxationTime;
    return antiMove;
  }

  // compute force
  Tvector force = waypoint->getForce(*this, &desiredDirection);

  return force;
}

/// Calculates the social force between this agent and all the other agents
/// belonging to the same scene.
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::socialForce() const
{
  // Moussaid-Helbing 2009 parameters.
  const double lambdaImportance = 2.0;
  const double gamma = 0.35;
  const double n = 2.0;
  const double nPrime = 3.0;
  const double eps = 1e-12;

  Tvector force(0.0, 0.0);

  for (const Ped::Tagent *other : neighbors)
  {
    // Ignore yourself and ignore robots while robot interaction is disabled.
    if (other->id == id || other->getType() == ROBOT)
    {
      continue;
    }

    Tvector diff = other->p - p;

    // Skip a degenerate pair rather than disabling social forces from every
    // remaining pedestrian for the entire update.
    if (diff.lengthSquared() <= 0.001)
    {
      continue;
    }

    Tvector diffDirection = diff.normalized();
    int quadrantBefore = diff.getQuadrant();

    // Measure approximate edge-to-edge separation.
    diff -= diffDirection * agentRadius;
    diff -= diffDirection * other->agentRadius;

    if (quadrantBefore != diff.getQuadrant())
    {
      // The agents overlap. Keep a small signed separation to avoid unstable
      // behavior in the exponential model.
      diff = diffDirection * 0.01;
    }

    Tvector velocityDifference = v - other->v;
    Tvector interactionVector =
        lambdaImportance * velocityDifference + diffDirection;
    double interactionLength = interactionVector.length();

    if (interactionLength <= eps)
    {
      continue;
    }

    Tvector interactionDirection = interactionVector / interactionLength;
    Ped::Tangle theta = interactionDirection.angleTo(diffDirection);
    double B = gamma * interactionLength;

    if (B <= eps)
    {
      continue;
    }

    double thetaRad = theta.toRadian();
    double velocityAmount =
        -std::exp(-diff.length() / B -
                  (nPrime * B * thetaRad) * (nPrime * B * thetaRad));
    double angleAmount =
        -theta.sign() *
        std::exp(-diff.length() / B -
                 (n * B * thetaRad) * (n * B * thetaRad));

    Tvector velocityForce = velocityAmount * interactionDirection;
    Tvector angleForce =
        angleAmount * interactionDirection.leftNormalVector();

    force += velocityForce + angleForce;
  }

  return force;
}

// Retained only because robotForce() is declared in the class interface.
// Robot force remains disabled while the obstacle response is isolated.
Ped::Tvector Ped::Tagent::robotForce()
{
  return Tvector(0.0, 0.0);
}

/// Calculates the force between this agent and the nearest obstacle in this
/// scene.
/// Iterates over all obstacles == O(N).
/// \return  Tvector: the calculated force
Ped::Tvector Ped::Tagent::obstacleForce()
{
  if (scene == nullptr || scene->obstacles.empty())
  {
    return Tvector(0.0, 0.0);
  }

  //should only respond to nearby obstacles
  const double eps = 1e-9;
  const double influenceRange = obstacleForceRange;  // edge-to-edge distance
  const double minimumClearance = 0.10;
  const double maximumContribution = 3.0;

  Tvector totalForce(0.0, 0.0);

  for (const Tobstacle *obstacle : scene->obstacles)
  {
    Tvector closestPoint = obstacle->closestPoint(p);
    Tvector awayFromObstacle = p - closestPoint;
    double rawDistance = awayFromObstacle.length();

    // closestPoint() can collapse onto the pedestrian position when geometry
    // overlaps. A zero-length vector has no usable steering direction.
    if (rawDistance <= eps)
    {
      continue;
    }

    double clearance = rawDistance - agentRadius;

    // Ignore distant walls and other far-away scene geometry completely.
    if (clearance >= influenceRange)
    {
      continue;
    }

    double safeClearance = std::max(clearance, minimumClearance);

    // Smoothly reaches zero at influenceRange and remains bounded near walls.
    double contribution =
        (1.0 / safeClearance) - (1.0 / influenceRange);
    contribution = std::max(0.0, contribution);
    contribution = std::min(contribution, maximumContribution);

    totalForce += contribution * (awayFromObstacle / rawDistance);
  }

  return totalForce;
}


Ped::Tvector Ped::Tagent::myForce(Ped::Tvector e)
{
  return Ped::Tvector(0.0, 0.0);
}

void Ped::Tagent::computeForces()
{
  const double neighborhoodRange = 10.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);

  desiredforce = desiredForce();
  socialforce = (forceFactorSocial > 0.0)
                    ? socialForce()
                    : Tvector(0.0, 0.0);
  obstacleforce = (forceFactorObstacle > 0.0)
                      ? obstacleForce()
                      : Tvector(0.0, 0.0);

  
  robotforce = Tvector(0.0, 0.0);

  keepdistanceforce = keepDistanceForce();
  myforce = myForce(desiredDirection);
}

const Ped::Tvector Ped::Tagent::getForce() const
{
  return forceFactorDesired * desiredforce;
}

/// Does the agent dynamics stuff. Calls the methods to calculate the individual
/// forces, adds them
/// to get the total force affecting the agent. This will then be translated
/// into a velocity difference,
/// which is applied to the agents velocity, and then to its position.
/// \param   stepSizeIn This tells the simulation how far the agent should
/// proceed


Ped::Tvector Ped::Tagent::getFreshForce()
{
  const double neighborhoodRange = 10.0;
  neighbors = scene->getNeighbors(p.x, p.y, neighborhoodRange);

  desiredforce = Tvector(0.0, 0.0);
  socialforce = Tvector(0.0, 0.0);
  obstacleforce = Tvector(0.0, 0.0);
  robotforce = Tvector(0.0, 0.0);
  myforce = Tvector(0.0, 0.0);

  desiredforce = desiredForce();

  if (forceFactorSocial > 0.0)
  {
    socialforce = socialForce();
  }

  if (forceFactorObstacle > 0.0)
  {
    obstacleforce = obstacleForce();
  }

  myforce = myForce(desiredDirection);

  // No robot force is calculated or applied in this isolated obstacle build.
  return forceFactorDesired * desiredforce +
         forceFactorSocial * socialforce +
         forceFactorObstacle * obstacleforce +
         myforce;
}

bool Ped::Tagent::applyExternalRobotAvoidance(
    const Ped::Tvector& robotPosition,
    double stepSizeIn)
{
  const double eps = 1e-6;
  const double detectionRange = 4.0;
  const double releaseRange = 5.0;
  const double behindReleaseDistance = 1.0;
  const double sideDecisionThreshold = 0.15;
  const double maxTurnRate = 1.40;  
  const double lateralSmoothing = 0.22;

  if (scene == nullptr || getTeleop())
  {
    externalRobotAvoidanceStateByAgent.erase(id);
    return false;
  }

  Tvector toRobot = robotPosition - p;
  double robotDistance = toRobot.length();

  ExternalRobotAvoidanceState& bypassState =
      externalRobotAvoidanceStateByAgent[id];

  if (robotDistance > releaseRange)
  {
    externalRobotAvoidanceStateByAgent.erase(id);
    return false;
  }

  // Match the previously successful simulator-level implementation:
  // refresh forces and waypoint direction before deciding the bypass heading.
  getFreshForce();

  Tvector goalDirection(0.0, 0.0);

  if (desiredDirection.length() > eps)
  {
    goalDirection = desiredDirection.normalized();
  }
  else if (v.length() > eps)
  {
    goalDirection = v.normalized();
  }

  if (goalDirection.length() <= eps)
  {
    externalRobotAvoidanceStateByAgent.erase(id);
    return false;
  }

  double forwardDistance =
      toRobot.x * goalDirection.x +
      toRobot.y * goalDirection.y;

  if (forwardDistance < behindReleaseDistance)
  {
    externalRobotAvoidanceStateByAgent.erase(id);
    return false;
  }

  if (robotDistance > detectionRange && bypassState.passingSide == 0)
  {
    return false;
  }

  Tvector leftOfGoal = goalDirection.leftNormalVector();

  double lateralOffset =
      toRobot.x * leftOfGoal.x +
      toRobot.y * leftOfGoal.y;

  if (bypassState.passingSide == 0)
  {
    if (std::abs(lateralOffset) > sideDecisionThreshold)
    {
      // Robot on the left: pass on the right. Robot on the right: pass left.
      bypassState.passingSide = (lateralOffset > 0.0) ? -1 : 1;
    }
    else
    {
      // Stable tie-breaker for a perfectly head-on encounter.
      bypassState.passingSide = 1;
    }
  }

  double proximity =
      clampValue(
          (detectionRange - robotDistance) / detectionRange,
          0.0,
          1.0);

  // Keep normal social and wall steering active, but use only its lateral
  // contribution so it cannot reverse the pedestrian.
  Tvector standardAvoidance =
      forceFactorSocial * socialforce +
      forceFactorObstacle * obstacleforce +
      myforce;

  double standardLateral =
      standardAvoidance.x * leftOfGoal.x +
      standardAvoidance.y * leftOfGoal.y;

  standardLateral =
      clampValue(
          lateralSmoothing * standardLateral,
          -0.45,
          0.45);

  double forceFactorPassedParameter = 0.0;
  ros::param::get("/force_factor", forceFactorPassedParameter);

  double robotStrength =
      (forceFactorRobot > 0.0) ? forceFactorRobot : forceFactorPassedParameter;

  double robotLateral =
      robotStrength *
      static_cast<double>(bypassState.passingSide) *
      (0.65 + 1.20 * proximity);

  Tvector targetDirection =
      goalDirection +
      leftOfGoal * (robotLateral + standardLateral);

  if (targetDirection.length() > eps)
  {
    targetDirection = targetDirection.normalized();
  }
  else
  {
    targetDirection = goalDirection;
  }

  double speedScale = 1.0 - 0.25 * proximity;
  double cruiseSpeed = 0.90 * vmaxDefault * speedScale;

  if (v.length() > eps)
  {
    double currentAngle = std::atan2(v.y, v.x);
    double targetAngle =
        std::atan2(targetDirection.y, targetDirection.x);

    double angleDiff = targetAngle - currentAngle;

    while (angleDiff > M_PI)
    {
      angleDiff -= 2.0 * M_PI;
    }

    while (angleDiff < -M_PI)
    {
      angleDiff += 2.0 * M_PI;
    }

    double maxAngleStep = maxTurnRate * stepSizeIn;
    angleDiff =
        clampValue(angleDiff, -maxAngleStep, maxAngleStep);

    double newAngle = currentAngle + angleDiff;

    v = Tvector(std::cos(newAngle), std::sin(newAngle)) * cruiseSpeed;
  }
  else
  {
    v = targetDirection * cruiseSpeed;
  }

  a = Tvector(0.0, 0.0);
  p += stepSizeIn * v;
  scene->moveAgent(this);

  return true;
}

void Ped::Tagent::move(double stepSizeIn)
{
  still_time += stepSizeIn;

  if (isForceOverridden)
  {
    a = forceOverride;
  }
  else
  {
    // Computes all forces and updates desiredDirection.
    a = getFreshForce();
  }

  if (getTeleop() == false)
  {
    const double eps = 1e-6;

    // Fixed walking speed. This avoids acceleration buildup.
    const double cruiseSpeed = 0.90 * vmaxDefault;

    // Turning control.
    // Higher = tighter turns. Lower = smoother turns.
    const double maxTurnRate = 1.2;  // rad/s

    // Avoidance steering control.
    // Keep this small. This should bend heading, not dominate it.
    const double avoidanceScale = 0.18;
    const double maxAvoidanceWeight = 0.60;

    Ped::Tvector targetDir(0.0, 0.0);

    if (desiredDirection.length() > eps)
    {
      Ped::Tvector goalDir = desiredDirection.normalized();

      // Use ONLY non-desired forces for avoidance steering.
      // Do not use desiredforce here, because that causes oscillation.
      Ped::Tvector avoidance =
        forceFactorSocial * socialforce +
        forceFactorObstacle * obstacleforce +
        myforce;

      // This movement model keeps a fixed walking speed. A forward or backward
      // avoidance component cannot slow the pedestrian.  Keep
      // only the component perpendicular to the waypoint direction.
      Ped::Tvector leftOfGoal = goalDir.leftNormalVector();
      double lateralAvoidance =
          avoidance.x * leftOfGoal.x + avoidance.y * leftOfGoal.y;

      const double lateralDeadband = 0.03;
      if (std::abs(lateralAvoidance) < lateralDeadband)
      {
        lateralAvoidance = 0.0;
      }

      double requestedCorrection = clampValue(
          avoidanceScale * lateralAvoidance,
          -maxAvoidanceWeight,
          maxAvoidanceWeight);

      // Smooth the correction so the pedestrian recenters gradually after clearing obstacle
      const double steeringSmoothingTime = 0.22;  // seconds
      double smoothingAlpha =
          1.0 - std::exp(-stepSizeIn / steeringSmoothingTime);

      double &smoothedCorrection = smoothedLateralCorrectionByAgent[id];
      smoothedCorrection +=
          smoothingAlpha * (requestedCorrection - smoothedCorrection);

      if (std::abs(smoothedCorrection) < 1e-4)
      {
        smoothedCorrection = 0.0;
      }

      Ped::Tvector steeringCorrection = leftOfGoal * smoothedCorrection;
      targetDir = goalDir + steeringCorrection;

      if (targetDir.length() > eps)
      {
        targetDir = targetDir.normalized();
      }
      else
      {
        targetDir = goalDir;
      }
    }
    else if (v.length() > eps)
    {
      targetDir = v.normalized();
    }
    else if (a.length() > eps)
    {
      targetDir = a.normalized();
    }

    if (targetDir.length() > eps)
    {
      if (v.length() > eps)
      {
        double currentAngle = std::atan2(v.y,               v.x);
        double targetAngle = std::atan2(targetDir.y, targetDir.x);

        double angleDiff = targetAngle - currentAngle;

        while (angleDiff > M_PI)
        {
          angleDiff -= 2.0 * M_PI;
        }

        while (angleDiff < -M_PI)
        {
          angleDiff += 2.0 * M_PI;
        }

        double maxAngleStep = maxTurnRate * stepSizeIn;

        if (angleDiff > maxAngleStep)
        {
          angleDiff = maxAngleStep;
        }
        else if (angleDiff < -maxAngleStep)
        {
          angleDiff = -maxAngleStep;
        }

        double newAngle = currentAngle + angleDiff;

        v = Ped::Tvector(std::cos(newAngle), std::sin(newAngle)) * cruiseSpeed;
      }
      else
      {
        v = targetDir * cruiseSpeed;
      }
    }
    else if (v.length() > eps)
    {
      v = v.normalized() * cruiseSpeed;
    }

    // Since this movement model uses force for steering, not acceleration,
    // prevent acceleration spikes from being published.
    a = Ped::Tvector(0.0, 0.0);
  }

  p += stepSizeIn * v;

  scene->moveAgent(this);
}
void Ped::Tagent::overrideForce(){
  isForceOverridden = false;
}

void Ped::Tagent::overrideForce(Ped::Tvector force){
  forceOverride = force;
  isForceOverridden = true;
}

void Ped::Tagent::overrideVmax(double factor_){
  factorVmax = factor_;
}