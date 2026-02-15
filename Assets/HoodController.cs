using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class HoodController : MonoBehaviour
{

    public Slider angleSlider;
    public bool useSlider;

    public LERP lerp;

    public float currentFrameWeightTurret;
    public float currentFrameWeightHood;

    private float hood = 45;
    private float turret = 45;

    public PID turretPID;

    public PID hoodPID;

    public bool lockTo180;

    public bool useOffsetSlider;
    public Slider offsetSlider;

    [System.Serializable]
    public class PID {
        public float pFactor, iFactor, dFactor;

        float integral;
        float lastError;


        public PID(float pFactor, float iFactor, float dFactor) {
            this.pFactor = pFactor;
            this.iFactor = iFactor;
            this.dFactor = dFactor;
        }


        public float Update(float setpoint, float actual, float timeFrame) {
            float error = (setpoint - actual + 540) % 360 - 180;
            integral += error * timeFrame;
            float deriv = (error - lastError) / timeFrame;
            lastError = error;
            return error * pFactor + integral * iFactor + deriv * dFactor;
        }
    }

    public float turretDelta;
    public float hoodDelta;

    

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    

    public void setRotation(float targetRotation, float turretRotation)
    {

        turretDelta = (-turretRotation - transform.rotation.eulerAngles.y + 540) % 360 - 180;
        hoodDelta = (-targetRotation - transform.rotation.eulerAngles.z + 540) % 360 - 180;

        float newTurretPosition = transform.rotation.eulerAngles.y + 0.02f * turretPID.Update(-turretRotation, transform.rotation.eulerAngles.y, 0.02f);

        float newHoodPosition = transform.rotation.eulerAngles.z + 0.02f * hoodPID.Update(-targetRotation, transform.rotation.eulerAngles.z, 0.02f);

        Quaternion newRot = Quaternion.Euler(new Vector3(
            transform.rotation.eulerAngles.x,
            lockTo180 ? 180 : newTurretPosition,
            newHoodPosition
        ));

        transform.rotation = newRot;//Quaternion.SlerpUnclamped(transform.rotation, newRot, 1);


    }

    public float getRotation()
    {
        return transform.rotation.eulerAngles.y;
    }

    public void setAngles(float hood, float turret)
    {

        this.hood = hood;
        this.turret = turret;

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        
        if (useSlider)
        {
            setRotation(90-angleSlider.value, turret);
        } else
        {
            setRotation((float)(90 -hood - (useOffsetSlider ? offsetSlider.value * lerp.getOffsetMultiplier() : 0)), turret);
        }
    }
}
