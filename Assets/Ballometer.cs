using UnityEngine;

public class Ballometer : MonoBehaviour
{
    public ShooterController shooterController;
    public OutputProcessor outputProcessor;

    public AccuracyCounter counter;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
    {
        GameObject obj = other.gameObject;
        print("Average Velocity: " + ((getAsVec2(obj.GetComponent<Rigidbody>().linearVelocity) + obj.GetComponent<FuelSelfDestruct>().initialVelocity) / 2));
        Destroy(other.gameObject);
        float timeDelta = Time.time - shooterController.shotStartTime;
        outputProcessor.acceptDelta(timeDelta);
        counter.countShotMade();


    }

    Vector2 getAsVec2(Vector3 vec)
    {
        return new Vector2(vec.x, vec.z);
    }
}
