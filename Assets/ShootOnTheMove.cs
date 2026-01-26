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

        for (int i = 0; i < numberOfAlgorithmIterations; i++)
        {

            offsetDistance = pureTargetDifference.magnitude / Mathf.Cos(Mathf.Atan2(tangentialVelocity * timeOfFlight, offsetDistance));

            timeOfFlight = (float)lerpTable.getTimeOfFlight(offsetDistance - radialVelocity * timeOfFlight);

            //offset = new Vector2(linearVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot), linearVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot)) * timeOfFlight;



        }



        // Account for tangential movement relative to the goal
        
        

        lerpTable.putNewTimeOfFlight(timeOfFlight);

        //print("Time of Flight: " + timeOfFlight);



        //offset *= Mathf.Sign(targetDifference.y);


        //targetDifference -= offset * 1f;
        

        offsetPoseCube.transform.position = convertVec2To3(targetDifference + robotPose);

        float offsetTurretAngle = Mathf.Atan2(targetDifference.y, -targetDifference.x) * Mathf.Rad2Deg;

        float distance = offsetDistance;//targetDifference.magnitude;
        


        float notOffsetTurretAngle = Mathf.Rad2Deg * Mathf.Atan2(pureTargetDifference.y, -pureTargetDifference.x);

        float magnitudeOfTurretAngleOffset = offsetTurretAngle - notOffsetTurretAngle;
        
        magnitudeOfTurretAngleOffset *= 0f;

        shooterController.setVelocity((float)lerpTable.getVelocity(distance));

        //print(offsetTurretAngle + " vs " + notOffsetTurretAngle);
        //print(offsetTurretAngle - notOffsetTurretAngle);

        hoodController.setAngles( (float)lerpTable.getAngle(distance),
            Mathf.Atan2(tangentialVelocity * timeOfFlight, distance) * Mathf.Rad2Deg + notOffsetTurretAngle);

        //turretController.lookTowards(offsetPoseCube);
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
