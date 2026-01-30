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

    private float weightedTurretAngle = 0;
    private float weightedHoodAngle = 0;

    public bool lockTo180;

    public bool useOffsetSlider;
    public Slider offsetSlider;

    

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    public void setRotation(float targetRotation, float turretRotation)
    {
        weightedTurretAngle = currentFrameWeightTurret * weightedTurretAngle + turretRotation * (1-currentFrameWeightTurret);

        Quaternion newRot = Quaternion.Euler(new Vector3(
            transform.rotation.eulerAngles.x,
            lockTo180 ? 180 : weightedTurretAngle,
            targetRotation
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
    void Update()
    {
        if (useSlider)
        {
            setRotation(90-angleSlider.value, turret);
        } else
        {
            weightedHoodAngle = currentFrameWeightHood * weightedHoodAngle + hood * (1-currentFrameWeightHood);
            setRotation((float)(90 -weightedHoodAngle - (useOffsetSlider ? offsetSlider.value * lerp.getOffsetMultiplier() : 0)), turret);
        }
    }
}
