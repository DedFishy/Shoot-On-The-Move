using System;
using System.Diagnostics;
using System.Numerics;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

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

    public float compensationFactor;
    public float distanceCompensationFactor;


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

    public float tangentialVelocityDragOffset;

    public float timeToGetToNewPositionTurretHood;

    /*
    Find a velocity such that:
    - The average velocity of the ball with drag is equal to requiredVelocity
    To do this:
    - Make an initial guess, which is requiredVelocity (it will be wrong)
    - Find end velocity based on drag & shot time
    - Use this value to calculate a new inital guess
    - Repeat this until the average velocity with initial guess is equal to required velocity
    */

    Vector2 GetInitialVelocityAccountingForDrag(Vector2 requiredHorizontalDisplacement, Vector2 initialHorizontalVelocity, float initialVerticalVelocity, float dragCoeff, float timeOfFlight, float timeInterval, int numIterations2)
    {
        Vector3 velocityWithDrag = new Vector3(initialHorizontalVelocity.x, initialVerticalVelocity, initialHorizontalVelocity.y);

        Vector2 horizontalDisplacementWithDrag = Vector2.zero;

        Vector3 velocityAtPreviousFrame = initialHorizontalVelocity;

        //Vector2 averageVelocity = Vector2.zero;
        int numIterations = 0;
        for (float i = 0; i < timeOfFlight + timeInterval; i += timeInterval)
        {
            if (i >= timeOfFlight)
            {
                timeInterval = timeOfFlight - (i - timeInterval);
            }
            velocityWithDrag -= timeInterval * (dragCoeff * (velocityWithDrag.magnitude*velocityWithDrag) / 0.01f); // 0.01=mass of ball
            Vector3 velocityFrameAvg = (velocityAtPreviousFrame + velocityWithDrag) / 2;
            horizontalDisplacementWithDrag += new Vector2(velocityFrameAvg.x, velocityFrameAvg.z) * timeInterval;
            //averageVelocity += velocityWithDrag;
            velocityAtPreviousFrame = velocityWithDrag;
            numIterations++;
        }
        //averageVelocity /= numIterations;
        //Vector2 delta = (requiredVelocity - averageVelocity);
        //initialVelocity = (requiredVelocity * 2) - velocityWithDrag;
        Vector2 delta = requiredHorizontalDisplacement - horizontalDisplacementWithDrag;

        if (Mathf.Abs(delta.magnitude) < 0.1 || numIterations2 > 99) {
            print("--- DRAG ALGORITHM ---");
            print("Iterations: " + numIterations2);
            print("Delta: " + delta);
            print("Displacement after drag: " + horizontalDisplacementWithDrag + " versus required: " + requiredHorizontalDisplacement);
            print("Initial horizontal velocity: " + initialHorizontalVelocity);
            print("---                ---");
            /*print(averageVelocity );
            print(requiredVelocity);*/

            return initialHorizontalVelocity;
        }

        else return GetInitialVelocityAccountingForDrag(requiredHorizontalDisplacement, initialHorizontalVelocity + delta / timeOfFlight, initialVerticalVelocity, dragCoeff, timeOfFlight, timeInterval, numIterations2+1);

        
        /*Vector2 currentAverageVelocity = (velocityWithDrag + intialVelocity) / 2;
        Vector2 delta = requiredVelocity - currentAverageVelocity;

        while(delta.magnitude > 0.1) // arbitrary tolerance
        {
            intialVelocity += delta;
            Vector2 intialVelocityCopy = intialVelocity;
            for (float i = 0; i < timeOfFlight; i += timeInterval)
            {
                intialVelocityCopy -= dragCoeff * (intialVelocityCopy.magnitude*intialVelocityCopy) / 0.01f; // 0.01=mass of ball
            }
            currentAverageVelocity = (intialVelocityCopy + intialVelocity) / 2;
            delta = requiredVelocity - currentAverageVelocity;
        }*/

    }

    private Vector2[] velocityMeasurementFrames = new Vector2[6];
    private Vector2[] accelerationMeasurementFrames = new Vector2[6];
    private Vector2[] jerkMeasurementFrames = new Vector2[6];

    public Vector2 vRequired;

    void ElijahSpecial()
    {
        var targetPosition = getTargetPosition();
        var turretPosition = convertToVector2(turretController.getTranslation());
        Vector2 robotPose = getTranslation();

        /*for (int i = 1; i < velocityMeasurementFrames.Length; i++)
        {
            velocityMeasurementFrames[i-1] = velocityMeasurementFrames[i];
        }
        velocityMeasurementFrames[velocityMeasurementFrames.Length - 1] = robotPose;

        Vector2 linearVelocity = Vector2.zero;
        if (velocityMeasurementFrames[0] != null)
        {
            linearVelocity = ((velocityMeasurementFrames[3] * 0.1f + velocityMeasurementFrames[4] * 0.3f + velocityMeasurementFrames[5] * 0.6f) - (velocityMeasurementFrames[0] * 0.6f + velocityMeasurementFrames[1] * 0.30f + velocityMeasurementFrames[2] * 0.1f)) / ((6f) * 0.02f);
        }

        

        for (int i = 1; i < accelerationMeasurementFrames.Length; i++)
        {
            accelerationMeasurementFrames[i-1] = accelerationMeasurementFrames[i];
        }
        accelerationMeasurementFrames[accelerationMeasurementFrames.Length - 1] = linearVelocity;

        Vector2 acceleration = Vector2.zero;
        if (accelerationMeasurementFrames[0] != null)
        {
            acceleration = (accelerationMeasurementFrames[accelerationMeasurementFrames.Length - 1] - accelerationMeasurementFrames[0]) / (accelerationMeasurementFrames.Length * 0.02f);
        }

        for (int i = 1; i < jerkMeasurementFrames.Length; i++)
        {
            jerkMeasurementFrames[i-1] = jerkMeasurementFrames[i];
        }
        jerkMeasurementFrames[jerkMeasurementFrames.Length - 1] = acceleration;

        Vector2 jerk = Vector2.zero;
        if (jerkMeasurementFrames[0] != null)
        {
            jerk = (jerkMeasurementFrames[jerkMeasurementFrames.Length - 1] - jerkMeasurementFrames[0]) / (jerkMeasurementFrames.Length * 0.02f);
        }

        acceleration += jerk * timeToGetToNewPositionTurretHood;

        linearVelocity += acceleration * timeToGetToNewPositionTurretHood;

        print("Velocity:        " + linearVelocity + " (" + linearVelocity.magnitude + ")");
        print("Actual Velocity: " + getLinearVelocity() + " (" + getLinearVelocity().magnitude + ")");
        print("Jerk: " + jerk + " (" + jerk.magnitude + ")");
        print("Acceleration: " + acceleration + " (" + acceleration.magnitude + ")");

        robotPose += linearVelocity * timeToGetToNewPositionTurretHood;*/
        Vector2 linearVelocity = getLinearVelocity();


        float angularRobotVelocity = getAngularVelocity();
        var targetDifference = targetPosition - turretPosition;

        Vector2 pureTargetDifference = targetPosition - turretPosition;

        Vector2 robotToTurret = turretPosition - robotPose;

        float angleBetweenGoalAndRobot = -Mathf.Atan2(targetDifference.y, targetDifference.x);
        //radialVelocity = linearVelocity.x * Mathf.Cos(angleBetweenGoalAndRobot) - linearVelocity.y * Mathf.Sin(angleBetweenGoalAndRobot);
        //print("Radial velocity: " + radialVelocity);
        //float tangentialVelocity = linearVelocity.x * Mathf.Sin(angleBetweenGoalAndRobot) + linearVelocity.y * Mathf.Cos(angleBetweenGoalAndRobot);
        //print("Tangential velocity: " + tangentialVelocity);

        //float projectionScalar = (linearVelocity.x * targetDifference.x + linearVelocity.y * targetDifference.y) / (targetDifference.x * targetDifference.x + targetDifference.y + targetDifference.y);

        //float tangentialVelocity = (projectionScalar * targetDifference).magnitude;
        //print("Tangential velocity: " + tangentialVelocity);

        float lengthOfProjectionOverMagnitude = ((linearVelocity.x * pureTargetDifference.x + linearVelocity.y * pureTargetDifference.y) / Mathf.Pow(pureTargetDifference.magnitude, 2));
        Vector2 radialVelocity = pureTargetDifference * lengthOfProjectionOverMagnitude;
        Vector2 tangentialVelocity = linearVelocity - radialVelocity;

        print("Radial velocity: " + radialVelocity);
        print("Tangential velocity: " + tangentialVelocity);


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

        //float angleCompensationFactor = 0.5f * (tangentialVelocity == 0 ? 0 : Math.Sign(pureTargetDifference.y));

        

        

        Vector2 distanceVector = new Vector2();

        Vector2 V_required_xy = new Vector2();

        float turretAngleDeg = 0;

        float hoodAngle = (float)lerpTable.getAngle(offsetDistance);
        float flywheelSpeed = (float)lerpTable.getVelocity(offsetDistance);


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

            //Vector2 dragDirection = -V_shooter_xy;
            
            //Vector2 velocityVithDragComp = GetInitialVelocityAccountingForDrag(D_xy, V_shooter_xy, flywheelSpeed * Mathf.Sin(Mathf.Deg2Rad * hoodAngle), 0.0000508198920953f, timeOfFlight, 0.05f, 0);



            //V_shooter_xy = velocityPostDrag;


            // 4) Rotate into ROBOT frame (turret frame if turret is robot-relative)
            float cosYaw = Mathf.Cos(-robotYaw);
            float sinYaw = Mathf.Sin(-robotYaw);

            Vector2 V_turret = new Vector2(
                V_shooter_xy.x * cosYaw - V_shooter_xy.y * sinYaw,
                V_shooter_xy.x * sinYaw + V_shooter_xy.y * cosYaw
            );

            // 5) Turret angle command
            turretAngleDeg = Mathf.Atan2(V_turret.y, -V_turret.x) * Mathf.Rad2Deg;

            distanceVector = V_shooter_xy * timeOfFlight;


            float dragTurretOffset = /*(tangentialVelocity.magnitude * tangentialVelocity.magnitude) * -Mathf.Sign(linearVelocity.y) * distanceVector.magnitude * compensationFactor*/ + 
            -Mathf.Sign(linearVelocity.y) * Mathf.Sin((float)lerpTable.getAngle(distanceVector.magnitude) * Mathf.Deg2Rad) * (float)lerpTable.getVelocity(distanceVector.magnitude) * distanceCompensationFactor * (tangentialVelocity.magnitude);


            /*float originalDistanceDirection = Mathf.Atan2(distanceVector.y, distanceVector.x);

            float angleB = Mathf.PI - (Mathf.PI/2 - originalDistanceDirection);

            float distanceWithDrag = (distanceVector.magnitude / Mathf.Sin(Mathf.PI - Mathf.Abs(turretAngleDeg - originalDistanceDirection - angleB))) * Mathf.Sin(angleB);*/

            float newDistance = (distanceVector.magnitude / Mathf.Sin(Mathf.PI - Mathf.Deg2Rad*(turretAngleDeg+90) - dragTurretOffset*Mathf.Deg2Rad)) * Mathf.Sin(Mathf.Deg2Rad*(turretAngleDeg+90));

            turretAngleDeg += dragTurretOffset;



            timeOfFlight = (float)lerpTable.getTimeOfFlight(distanceVector.magnitude);

            hoodAngle = (float)lerpTable.getAngle(newDistance);
            flywheelSpeed = (float)lerpTable.getVelocity(newDistance);

        }

        vRequired = (V_required_xy);


        // Account for tangential movement relative to the goal
    

        lerpTable.putNewTimeOfFlight(timeOfFlight);

        //turretAngleDeg += tangentialVelocityDragOffset * tangentialVelocity;

        //print(tangentialVelocity);

        //print("Time of Flight: " + timeOfFlight);



        //offset *= Mathf.Sign(targetDifference.y);


        //targetDifference -= offset * 1f;
        

        offsetPoseCube.transform.position = convertVec2To3(targetDifference + robotPose);

        float offsetTurretAngle = Mathf.Atan2(targetDifference.y, -targetDifference.x) * Mathf.Rad2Deg;

        float distance = distanceVector.magnitude;//offsetDistance;//targetDifference.magnitude;
        


        float notOffsetTurretAngle = Mathf.Rad2Deg * Mathf.Atan2(pureTargetDifference.y, -pureTargetDifference.x);

        float magnitudeOfTurretAngleOffset = offsetTurretAngle - notOffsetTurretAngle;
        
        magnitudeOfTurretAngleOffset *= 0f;

        shooterController.setVelocity(flywheelSpeed);

        //print(offsetTurretAngle + " vs " + notOffsetTurretAngle);
        //print(offsetTurretAngle - notOffsetTurretAngle);

        hoodController.setAngles( hoodAngle,
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
