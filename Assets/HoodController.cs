using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class HoodController : MonoBehaviour
{

    public Slider angleSlider;
    public bool useSlider;

    public LERP lerp;

    public float currentFrameWeight;

    private float hood = 45;
    private float turret = 45;

    private float weightedTurretAngle = 0;

    public bool lockTo180;

    public bool useOffsetSlider;
    public Slider offsetSlider;

    

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    public void setRotation(float targetRotation, float turretRotation)
    {

        weightedTurretAngle = currentFrameWeight * weightedTurretAngle + turretRotation * (1-currentFrameWeight);

        transform.rotation = Quaternion.Euler(new Vector3(
            transform.rotation.eulerAngles.x,
            lockTo180 ? 180 : weightedTurretAngle,
            targetRotation
        ));


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
            setRotation((float)(90 -hood - (useOffsetSlider ? offsetSlider.value * lerp.getOffsetMultiplier() : 0)), turret);
        }
    }
}
