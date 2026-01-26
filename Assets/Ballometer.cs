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
        print("Ball collided");
        Destroy(other.gameObject);
        float timeDelta = Time.time - shooterController.shotStartTime;
        outputProcessor.acceptDelta(timeDelta);
        counter.countShotMade();
    }
}
