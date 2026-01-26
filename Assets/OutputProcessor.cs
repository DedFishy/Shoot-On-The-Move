using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class OutputProcessor : MonoBehaviour
{

    public Slider velocitySlider;
    public Slider angleSlider;

    public TMP_Text text;
    public GameObject target;
    public GameObject shooter;

    private float delta;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        text.text = "new double[]{" + Math.Round(convertToVector2(target.transform.position - shooter.transform.position).magnitude, 3) + ", " + angleSlider.value + ", " + velocitySlider.value + ", " + delta + "}";
    }

    public void acceptDelta(float delta)
    {
        this.delta = delta;
    }

    public void copy()
    {
        GUIUtility.systemCopyBuffer = text.text;
    }

    Vector2 convertToVector2(Vector3 vec3)
    {
        return new Vector2(vec3.x,vec3.z);
    }
}
