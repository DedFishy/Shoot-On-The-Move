using System;
using System.Diagnostics;
using System.Numerics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;

public class ShootOnTheMove : MonoBehaviour
{

    public ShooterController shooterController;
    public TurretController turretController;
    public HoodController hoodController;
    public GameObject target;

    public DrivetrainController driveTrainController;
    public LERP lerpTable;

    public int numberOfAlgorithmIterations;

    public float radialVelocityTestX;
    public float radialVelocityReleaseTime;

    public float radialVelocity;

    public GameObject offsetPoseCube;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }



    // Update is called once per frame
    void Update()
    {
    }

    void FixedUpdate()
    {
        ElijahSpecial();
    }

    public float getDistanceToTarget() {return (getTargetPosition() - convertToVector2(turretController.getTranslation())).magnitude;}

    UnityEngine.Vector2 convertToVector2(UnityEngine.Vector3 vec3)
    {
        return new UnityEngine.Vector2(vec3.x, vec3.z);
    }

    UnityEngine.Vector2 getTranslation()
    {
        return convertToVector2(transform.position);
    }
    float getRotation()
    {
        return transform.rotation.eulerAngles.y;
    }
    UnityEngine.Vector2 getTargetPosition()
    {
        return convertToVector2(target.transform.position);
    }

    UnityEngine.Vector2 getLinearVelocity()
    {
        return convertToVector2(GetComponent<Rigidbody>().linearVelocity);
    }
    float getAngularVelocity()
    {
        return GetComponent<Rigidbody>().angularVelocity.y;
    }
    

    void CheezyPoofs()
    {
        float turretAngle = turretController.getRotation();
        var targetPosition = getTargetPosition();
        var turretPosition = convertToVector2(turretController.getTranslation());
        var targetDifference = turretPosition - targetPosition;
        
        float turretError = Mathf.Rad2Deg * Mathf.Atan2(targetDifference.y, targetDifference.x) - turretAngle;
        
        float uncompensatedYaw = turretAngle + turretError;
        float uncompensatedRange = targetDifference.magnitude;


        UnityEngine.Vector2 linearVelocity = getLinearVelocity();
        UnityEngine.Vector2 translationalVelocity = linearVelocity;
        float angleBetweenGoalAndRobot = -Mathf.Atan2(targetPosition.y-turretPosition.y, targetPosition.x-turretPosition.x);


        //radialVelocity = translationalVelocity.x * Mathf.Cos(angleBetweenGoalAndRobot) - translationalVelocity.y * Mathf.Sin(angleBetweenGoalAndRobot);
        float tangentialComponent = translationalVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot) + translationalVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot);

        //float shotSpeed = uncompensatedRange / shotTime - radialVelocity;
        //if (shotSpeed < 0.0) shotSpeed = 0f;
        //float turretAdjustment = Mathf.Atan2(-(tangentialComponent), shotSpeed) * Mathf.Rad2Deg; // Angle formed by velocities on the ball

        //effectiveRange = shotTime * Mathf.Sqrt(tangentialComponent * tangentialComponent + shotSpeed * shotSpeed);

        float effectiveYaw = uncompensatedYaw;// + turretAdjustment;
        //float effectiveYawFeedForward = -(getAngularVelocity().y + Mathf.Rad2Deg * tangentialComponent/uncompensatedRange) * Time.deltaTime;

        turretController.setTargetRotation(effectiveYaw);

        //turretController.setTargetRotationFF(effectiveYawFeedForward); // A velocity to be at by the time we reach the target rotation

        //shooterController.setRangeFF(-radialComponent);
    }

    void MechanicalAdvantage()
    {
        Vector2 robotPose = getTranslation();
        float robotRotation = getRotation();
        // No compensation is done for phase delay (time it takes to shoot) because it's Unity so it's instant

        Vector2 linearRobotVelocity = getLinearVelocity();
        float angularRobotVelocity = getAngularVelocity();

        Vector2 turretPosition = convertToVector2(turretController.getTranslation()); // Usually would be robot-relative transform
        Vector2 robotToTurret = turretPosition - robotPose;

        robotRotation *= Mathf.Deg2Rad;

        

        // "Tangential" Velocity?
        float turretVelocityX = 
            linearRobotVelocity.x
                + angularRobotVelocity
                    * (robotToTurret.y * Mathf.Cos(robotRotation)
                        - robotToTurret.x * Mathf.Sin(robotRotation)); // Looks just like what Cheezy Poofs were doing to get radial and tangential velocity...

        // "Radial" Velocity?
        float turretVelocityY =
            linearRobotVelocity.y
                + angularRobotVelocity
                    * (robotToTurret.x * Mathf.Cos(robotRotation)
                        - robotToTurret.y * Mathf.Sin(robotRotation));

        float timeOfFlight = (float)lerpTable.getTimeOfFlight();
        
        Vector2 lookaheadPose = turretPosition;
        float lookaheadTurretToTargetDistance = (getTargetPosition() - turretPosition).magnitude;

        for (int i = 0; i < numberOfAlgorithmIterations; i++)
        {
            timeOfFlight = (float)lerpTable.getTimeOfFlight(lookaheadTurretToTargetDistance);
            float offsetX = turretVelocityX * timeOfFlight;
            float offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Vector2(
                turretPosition.x + offsetX,
                turretPosition.y + offsetY
            );
            lookaheadTurretToTargetDistance = (lookaheadPose - getTargetPosition()).magnitude;
        }



        lerpTable.putNewTimeOfFlight(timeOfFlight);
        

        float distance = (getTargetPosition() - turretPosition).magnitude;

        
        

        Vector2 differenceVector = (getTargetPosition() - turretPosition);

        float turretAngle = Mathf.Atan2(differenceVector.y, -differenceVector.x) * Mathf.Rad2Deg;

        float hoodAngle = (float)lerpTable.getAngle(distance);
        float shooterVelocity = (float)lerpTable.getVelocity(distance);

        shooterController.setVelocity(shooterVelocity);
        hoodController.setAngles(hoodAngle, turretAngle);
    }

    void ElijahSpecial()
    {
        var targetPosition = getTargetPosition();
        var turretPosition = convertToVector2(turretController.getTranslation());
        Vector2 robotPose = getTranslation();
        UnityEngine.Vector2 linearVelocity = getLinearVelocity();
        float angularRobotVelocity = getAngularVelocity();
        var targetDifference = targetPosition - turretPosition;

        var pureTargetDifference = targetPosition - turretPosition;

        Vector2 robotToTurret = turretPosition - robotPose;

        float angleBetweenGoalAndRobot = -Mathf.Atan2(targetDifference.y, targetDifference.x);
        radialVelocity = linearVelocity.x * Mathf.Cos(angleBetweenGoalAndRobot) - linearVelocity.y * Mathf.Sin(angleBetweenGoalAndRobot);
        //print("Radial velocity: " + radialVelocity);
        float tangentialVelocity = linearVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot) + linearVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot);
        //print("Tangential velocity: " + tangentialVelocity);


        float robotRotation = getRotation() * Mathf.Deg2Rad;

        // "Tangential" Velocity?
        float turretVelocityX = 
            linearVelocity.x
                + angularRobotVelocity
                    * (robotToTurret.y * Mathf.Cos(robotRotation)
                        - robotToTurret.x * Mathf.Sin(robotRotation)); // Looks just like what Cheezy Poofs were doing to get radial and tangential velocity...

        // "Radial" Velocity?
        float turretVelocityY =
            linearVelocity.y
                + angularRobotVelocity
                    * (robotToTurret.x * Mathf.Cos(robotRotation)
                        - robotToTurret.y * Mathf.Sin(robotRotation));

        float timeOfFlight = (float)lerpTable.getTimeOfFlight();
        
        Vector2 turretToTargetVector = (getTargetPosition() - turretPosition);
        //Vector2 offset = Vector2.zero;
        float offsetDistance = pureTargetDifference.magnitude;

        float angleCompensationFactor = 0.5f * (tangentialVelocity == 0 ? 0 : Math.Sign(pureTargetDifference.y));

        

        Vector2 distanceVector = new Vector2();

        Vector2 V_required_xy = new Vector2();

        float turretAngleDeg = 0;


        for (int i = 0; i < numberOfAlgorithmIterations; i++)
        {

            //offsetDistance = pureTargetDifference.magnitude / Mathf.Cos(turretAngleDeg * Mathf.Deg2Rad - angleBetweenGoalAndRobot);

            //timeOfFlight = (float)lerpTable.getTimeOfFlight(offsetDistance - radialVelocity * timeOfFlight);

            //offset = new Vector2(linearVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot), linearVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot)) * timeOfFlight;

                    // Inputs (FIELD FRAME)
            Vector2 goalPos_xy = targetPosition;          // (x, y)
            Vector2 shooterExitPos_xy = turretPosition;   // (x, y)
            Vector2 robotVel_xy = linearVelocity;         // (vx, vy) field-relative
            float robotYaw = robotRotation;              // radians, field-relative
            float t = timeOfFlight;                     // ball flight time (seconds)

            // 1) Displacement to goal (field frame)
            Vector2 D_xy = goalPos_xy - shooterExitPos_xy;

            // 2) Required FIELD-relative ball horizontal velocity
            V_required_xy = D_xy / t;

            // 3) Shooter-relative horizontal velocity
            Vector2 V_shooter_xy = V_required_xy - robotVel_xy;

            // 4) Rotate into ROBOT frame (turret frame if turret is robot-relative)
            float cosYaw = Mathf.Cos(-robotYaw);
            float sinYaw = Mathf.Sin(-robotYaw);

            Vector2 V_turret = new Vector2(
                V_shooter_xy.x * cosYaw - V_shooter_xy.y * sinYaw,
                V_shooter_xy.x * sinYaw + V_shooter_xy.y * cosYaw
            );

            // 5) Turret angle command
            turretAngleDeg = Mathf.Atan2(V_turret.y, -V_turret.x) * Mathf.Rad2Deg;

            print(turretAngleDeg);

            distanceVector = V_shooter_xy * timeOfFlight;

            timeOfFlight = (float)lerpTable.getTimeOfFlight(distanceVector.magnitude);

        }



        // Account for tangential movement relative to the goal
        
        

        lerpTable.putNewTimeOfFlight(timeOfFlight);

        //print("Time of Flight: " + timeOfFlight);



        //offset *= Mathf.Sign(targetDifference.y);


        //targetDifference -= offset * 1f;
        

        offsetPoseCube.transform.position = convertVec2To3(targetDifference + robotPose);

        float offsetTurretAngle = Mathf.Atan2(targetDifference.y, -targetDifference.x) * Mathf.Rad2Deg;

        float distance = distanceVector.magnitude;//offsetDistance;//targetDifference.magnitude;
        


        float notOffsetTurretAngle = Mathf.Rad2Deg * Mathf.Atan2(pureTargetDifference.y, -pureTargetDifference.x);

        float magnitudeOfTurretAngleOffset = offsetTurretAngle - notOffsetTurretAngle;
        
        magnitudeOfTurretAngleOffset *= 0f;

        shooterController.setVelocity((float)lerpTable.getVelocity(distance));

        //print(offsetTurretAngle + " vs " + notOffsetTurretAngle);
        //print(offsetTurretAngle - notOffsetTurretAngle);

        hoodController.setAngles( (float)lerpTable.getAngle(distance),
            turretAngleDeg);
            //(Mathf.Atan2(tangentialVelocity * timeOfFlight, distance) - angleCompensationFactor) * Mathf.Rad2Deg + notOffsetTurretAngle);

        //turretController.lookTowards(offsetPoseCube);
    }

    void BoyneSpecial()
    {
        
        Vector2 actualTargetPosition = convertToVector2(target.transform.position);
        Vector2 actualTurretPosition = convertToVector2(turretController.getTranslation());

        Vector2 linearVelocity = getLinearVelocity();
        float angularVelocity = getAngularVelocity();
        Vector2 actualVectorBetweenTurretAndGoal = actualTargetPosition - actualTurretPosition;
        float angleBetweenGoalAndRobot = -Mathf.Atan2(actualVectorBetweenTurretAndGoal.y, actualVectorBetweenTurretAndGoal.x);

        float actualDistanceBetweetTurretAndGoal = actualVectorBetweenTurretAndGoal.magnitude;

        float radialVelocity = linearVelocity.x * Mathf.Cos(angleBetweenGoalAndRobot) - linearVelocity.y * Mathf.Sin(angleBetweenGoalAndRobot);
        float tangentialVelocity = linearVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot) + linearVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot);

        float baseTurretAngle = Mathf.Rad2Deg * Mathf.Atan2(actualVectorBetweenTurretAndGoal.y, -actualVectorBetweenTurretAndGoal.x);

        float timeOfFlight = (float)lerpTable.getTimeOfFlight(actualDistanceBetweetTurretAndGoal);

        float tangentialVelocityDistanceOffset = 0;
        float radialVelocityDistanceOffset = 0;

        float offsetDistanceBetweenTurretAndGoal = actualDistanceBetweetTurretAndGoal;

        for (int i = 0; i < numberOfAlgorithmIterations; i++)
        {
            offsetDistanceBetweenTurretAndGoal = actualDistanceBetweetTurretAndGoal + tangentialVelocityDistanceOffset + radialVelocityDistanceOffset;

            // Tangential velocity calculations
            tangentialVelocityDistanceOffset = 
                actualDistanceBetweetTurretAndGoal - 
                (actualDistanceBetweetTurretAndGoal / Mathf.Cos(Mathf.Atan2(tangentialVelocity * timeOfFlight, actualDistanceBetweetTurretAndGoal))); // Updated distance value
            
            // Radial velocity calculations
            radialVelocityDistanceOffset = -radialVelocity * timeOfFlight;

            timeOfFlight = (float)lerpTable.getTimeOfFlight(offsetDistanceBetweenTurretAndGoal);

        }

        lerpTable.putNewTimeOfFlight(timeOfFlight);

        shooterController.setVelocity((float)lerpTable.getVelocity(offsetDistanceBetweenTurretAndGoal));

        hoodController.setAngles( (float)lerpTable.getAngle(offsetDistanceBetweenTurretAndGoal),
            Mathf.Atan2(tangentialVelocity * timeOfFlight, actualDistanceBetweetTurretAndGoal) * Mathf.Rad2Deg + baseTurretAngle); // TODO: make this a saved variable


    }
    
    void ElijahSpecial2()
    {
        
    }

    public UnityEngine.Vector3 convertVec2To3(Vector2 vec)
    {
        return new UnityEngine.Vector3(vec.x, 2, vec.y);
    }

    public void runTest(float radialVelocity)
    {
        driveTrainController.teleportAndDrive(radialVelocityTestX, radialVelocity, radialVelocityReleaseTime);
    }
}
