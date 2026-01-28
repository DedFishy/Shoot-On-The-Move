using System;
using System.Diagnostics;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public class ShooterController : MonoBehaviour
{

    public GameObject ball;

    public float shooterPower;

    public Slider velocitySlider;
    public bool useSlider;
        public LERP lerp;

    public float shotStartTime;

    public AccuracyCounter counter;

    private float velocity;

    public bool useOffsetSlider;
    public Slider offsetSlider;

    public ShootOnTheMove shootOnTheMove;

    private float framesSinceLastShoot;

    public float phaseTime;

    private float timestampOfNextShot = 0;
    private bool doNextShot = false;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }


    public void setVelocity(float velocity)
    {
        this.velocity = velocity;
    }

    // Update is called once per frame
    void Update()
    {
        if (Time.time >= timestampOfNextShot && doNextShot)
        {
            doNextShot = false;
            shootNoDelay();
        }
    }

    void FixedUpdate()
    {
        if (activelyShooting) {
            framesSinceLastShoot++;
            if (framesSinceLastShoot >= 40) {
                shootNoInput();
                framesSinceLastShoot = 0;
            }
        }
    }

    private bool activelyShooting = false;

    public void shoot(InputAction.CallbackContext value)
    {
        activelyShooting = (value.phase == InputActionPhase.Started || value.phase == InputActionPhase.Performed);
        //if (!(value.phase == InputActionPhase.Started)) return;
        //shootNoInput();
        

    }
    public void shootNoInput()
    {
        timestampOfNextShot = Time.time + phaseTime;
        doNextShot = true;
    }
    public void shootNoDelay()
    {
        if (shootOnTheMove.getDistanceToTarget() > 7.5) return;

        counter.countShotTaken();
        shotStartTime = Time.time;

        print("Calc'd time of flight: " + lerp.getOffsetTimeOfFlight());
    
        GameObject newBall = Instantiate(ball, transform.position + new UnityEngine.Vector3(-0.1f, 0.1f, 0), ball.transform.rotation, transform.parent.parent);
        newBall.SetActive(true);
        Rigidbody rigidbody = newBall.GetComponent<Rigidbody>();
        rigidbody.linearVelocity = transform.parent.GetComponent<Rigidbody>().linearVelocity;
        UnityEngine.Vector3 shootDirection = gameObject.transform.up;
        //shootDirection.y *= -1;
        rigidbody.AddForce(shootDirection * (float)(useSlider ? velocitySlider.value : velocity + (useOffsetSlider ? offsetSlider.value * lerp.getOffsetMultiplier() : 0)), ForceMode.VelocityChange);

    }
}
