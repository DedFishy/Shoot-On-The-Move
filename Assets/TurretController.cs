using System;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class TurretController : MonoBehaviour
{


    private float rotation = 0;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    public void setTargetRotation(float targetRotation)
    
    {
        //transform.Rotate(0,(-targetRotation) - transform.rotation.eulerAngles.y,0);

    }

    public float getRotation()
    {
        return rotation;
    }

    public UnityEngine.Vector3 getTranslation()
    {
        return transform.position;
    }

    public void lookTowards(GameObject obj)
    {
        transform.LookAt(obj.transform, Vector3.up);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void FixedUpdate()
    {
    }
}
