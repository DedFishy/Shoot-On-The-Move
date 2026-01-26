using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class OffsetOutputProcessor : MonoBehaviour
{

    public Slider radialVelocitySlider;
    public Slider velocityOffsetSlider;
    public Slider angleOffsetSlider;

    public TMP_Text text;
    public GameObject target;
    public GameObject shooter;
    public ShootOnTheMove shootOnTheMove;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        text.text = "new double[]{" + radialVelocitySlider.value + ", " + angleOffsetSlider.value + ", " + velocityOffsetSlider.value + "}";
    }

    public void copy()
    {
        GUIUtility.systemCopyBuffer = text.text;
    }

    Vector2 convertToVector2(Vector3 vec3)
    {
        return new Vector2(vec3.x,vec3.z);
    }

    public void runTest()
    {
        shootOnTheMove.runTest(radialVelocitySlider.value);
    }
}
