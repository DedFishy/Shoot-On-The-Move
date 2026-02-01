using UnityEngine;

public class FuelSelfDestruct : MonoBehaviour
{

    float dragCoeffAndEverythingElseExceptForVelociyOfTheBallSquared = 0.0000508198920953f;

    Rigidbody rb;
    
    public Vector2 initialVelocity = new Vector2(0,0);

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {

        if (initialVelocity.magnitude < Mathf.Epsilon && rb.linearVelocity.magnitude > 0.1) {
            initialVelocity = new Vector2(rb.linearVelocity.x, rb.linearVelocity.z);
        }
        if (transform.position.y < -50) Destroy(gameObject);

        
        
        //print("totalr horizontal velocity (real): " + GetComponent<Rigidbody>().linearVelocity.x);
        //print("totalr vertical velocity (real): " + GetComponent<Rigidbody>().linearVelocity.y);
        //print("totalr sideways velocity (real): " + GetComponent<Rigidbody>().linearVelocity.z);
    }

    void FixedUpdate()
    {
        float drag = dragCoeffAndEverythingElseExceptForVelociyOfTheBallSquared * (rb.linearVelocity.magnitude*rb.linearVelocity.magnitude);
        
        Vector3 dragVector = -rb.linearVelocity.normalized * drag;

        rb.linearVelocity += dragVector/50f/0.01f; // 0.01=mass, 50=(1/time)
    }
}
