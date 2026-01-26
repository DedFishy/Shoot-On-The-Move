using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;

public class DrivetrainController : MonoBehaviour
{

    private new Rigidbody rigidbody;
    public float translationalAcceleration;
    public float rotationalAcceleration;
    public float maxTranslationalVelocity;
    public float maxRotationalVelocity;

    public GameObject target;

    public ShooterController shooterController;


    private Vector2 drivingVector;
    private float rotationAxis;

    private float driveTestVelocity;

    private float endDriveTestTimestamp;
    private float radialVelocityTestXOffset;
    private bool doRadialVelocityTestNextFrame = false;

    public LERP lerpTable;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
    }

    
    private float maxSpeedAddition = 0;


    // Update is called once per frame
    void Update()
    {
        if (doRadialVelocityTestNextFrame)
        {
            doRadialVelocityTestNextFrame = false;
            Vector3 pos = resetY(target.transform.position + convertToVector3(new Vector2(radialVelocityTestXOffset, 0)));
            transform.position = pos;
            rigidbody.position = pos;
        }
        /*if (endDriveTestTimestamp > Time.fixedTime && (Math.Abs(lerpTable.getDistance()) < 5.5))
        {
            rigidbody.linearVelocity = new Vector2(driveTestVelocity, 0);

        } else {
            if (rigidbody.linearVelocity.magnitude < maxTranslationalVelocity * drivingVector.magnitude) {
                rigidbody.AddForce(convertDriveVectorToForceVector(drivingVector * translationalAcceleration * rigidbody.linearDamping), ForceMode.Acceleration);
            }
            */
            
            if (drivingVector.magnitude != 0)
            {
                maxSpeedAddition += 2f * Time.deltaTime;
                maxSpeedAddition = Mathf.Min(maxSpeedAddition, 1f);//3.5f);
                rigidbody.linearVelocity = convertDriveVectorToForceVector(drivingVector * (maxTranslationalVelocity + maxSpeedAddition));
            } else {
                maxSpeedAddition = 0;
            }
            
            /*if (rigidbody.angularVelocity.magnitude <= maxRotationalVelocity) {
                rigidbody.AddTorque(new Vector3(0, rotationAxis * rotationalAcceleration, 0), ForceMode.Force);
            }
            
        }*/
        

    }

    UnityEngine.Vector3 convertDriveVectorToForceVector(UnityEngine.Vector2 vec2)

    {
        return new UnityEngine.Vector3(vec2.y, 0, -vec2.x);
    }

    public void drive(InputAction.CallbackContext value)
    {
        
        if (value.phase == InputActionPhase.Started || value.phase == InputActionPhase.Performed) {
            drivingVector = value.ReadValue<Vector2>();
        } else if (value.phase == InputActionPhase.Canceled)
        {
            drivingVector = Vector2.zero;
        } else
        {
            print(value.phase);
        }
    }

    public void rotate(InputAction.CallbackContext value)
    {
        if (value.phase == InputActionPhase.Started || value.phase == InputActionPhase.Performed) {
            rotationAxis = value.ReadValue<float>();
        } else if (value.phase == InputActionPhase.Canceled)
        {
            rotationAxis = 0;
        } else
        {
            print(value.phase);
        }
    }

    public void teleportAndDrive(float radialVelocityTestX, float radialVelocity, float radialVelocityReleaseTime)
    {
        endDriveTestTimestamp = Time.fixedTime + radialVelocityReleaseTime;
        driveTestVelocity = radialVelocity;
        radialVelocityTestXOffset = radialVelocityTestX;
        doRadialVelocityTestNextFrame = true;
    }

    public Vector3 convertToVector3(Vector2 vec)
    {
        return new Vector3(vec.x, 0, vec.y);
    }
    public Vector3 resetY(Vector3 vec)
    {
        return new Vector3(vec.x, transform.position.y, vec.z);
    }
}
