using UnityEngine;

public class FuelSelfDestruct : MonoBehaviour
{
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (transform.position.y < -50) Destroy(gameObject);
        //print("totalr horizontal velocity (real): " + GetComponent<Rigidbody>().linearVelocity.x);
        //print("totalr vertical velocity (real): " + GetComponent<Rigidbody>().linearVelocity.y);
        //print("totalr sideways velocity (real): " + GetComponent<Rigidbody>().linearVelocity.z);
    }
}
