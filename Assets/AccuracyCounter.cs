using TMPro;
using UnityEngine;

public class AccuracyCounter : MonoBehaviour
{

    private int shotsTaken = 0;
    private int shotsMade = 0;
    public TMP_Text text;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        float percent;
        if (shotsMade > shotsTaken) shotsMade = shotsTaken;
        if (shotsMade == 0 && shotsTaken == 0) percent = 0;
        else percent = Mathf.Round((float)shotsMade/shotsTaken*100);

        text.text = "Accuracy: " + shotsMade + "/" + shotsTaken + " (" + percent + "%)";
    }

    public void countShotTaken()
    {
        shotsTaken++;
    }
    public void countShotMade()
    {
        shotsMade++;
    }

    public void resetAccuracy()
    {
        shotsMade = 0;
        shotsTaken = 0;
    }
    
}
