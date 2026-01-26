using System;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class LERP : MonoBehaviour
{

    public GameObject target;
    public GameObject shooter;

    public ShootOnTheMove shootOnTheMove;

    public OffsetLERP offsetLERP;


    // Distance value, angle value, velocity value
    private double[][] lerp = new double[][]
    {
        new double[]{0.943, 83, 6.521739, 1.123051},
        new double[]{2.001, 75, 6.917391, 1.180779},
        new double[]{3.127, 71, 7.482608, 1.268265},
        new double[]{4.113, 62, 7.652174, 1.204544},
        new double[]{5.17, 58, 8.16087, 1.241837},
        new double[]{6.297, 57, 8.895652, 1.426422},
        new double[]{7.425, 53, 9.291305, 1.349884},
        new double[]{8.481, 50, 9.8, 1.373215},
        new double[]{9.818, 47, 10.42174, 1.405548},
        new double[]{11.297, 45, 11.15652, 1.46701},
        new double[]{14.042, 45, 12.28696, 1.642212},
        new double[]{15.591, 45, 12.85217, 1.723083},
        new double[]{19.111, 45, 14.15217, 1.928711}

    };


    public double getAngle(double value) {
        
        int[] indices = findClosestIndices(value);
        return findLerpValue(indices[0], indices[1], value, 1) + offsetLERP.getAngleOffset(shootOnTheMove.radialVelocity) * getOffsetMultiplier();
    }
    public double getAngle()
    {
        return getAngle(getDistance());
    }
    public double getVelocity(double value) {
        
        int[] indices = findClosestIndices(value);
        return findLerpValue(indices[0], indices[1], value, 2) + offsetLERP.getVelocityOffset(shootOnTheMove.radialVelocity) * getOffsetMultiplier();
    }
    public double getVelocity()
    {
        return getVelocity(getDistance());
    }

    public double getOffsetMultiplier()
    {
        return getOffsetTimeOfFlight();// * (getDistance() / 50 + 1);
    }

    public double getTimeOfFlight(double value) {
        
        int[] indices = findClosestIndices(value);
        return findLerpValue(indices[0], indices[1], value, 3) - 0.03;
    }
    public double getTimeOfFlight()
    {   
        return getTimeOfFlight(getDistance());
    }

    public double getOffsetTimeOfFlight() {return timeOfFlight;}


    public double getDistance()
    {
        return shootOnTheMove.getDistanceToTarget();
    }

    private float timeOfFlight;

    public void putNewTimeOfFlight(float timeOfFlight)
    {
        this.timeOfFlight = timeOfFlight;
    }

    Vector2 convertToVector2(Vector3 vec3)
    {
        return new Vector2(vec3.x,vec3.z);
    }

    /* Returns {Smaller, Larger} */
    // Algorithm is binary search from GeeksForGeeks
    private int[] findClosestIndices(double x){
        int subArrayStart = 0;
        int subArrayEnd = lerp.Length - 1;
        if (x < lerp[0][0]) return new int[] {0,0};
        if (x > lerp[lerp.Length-1][0]) return new int[] {lerp.Length-1, lerp.Length-1};
        
        while (subArrayStart <= subArrayEnd){
            
            int targetIndex = (subArrayStart + subArrayEnd) / 2;

            // Exact match!
            if (lerp[targetIndex][0] == x) {
                return new int[] {targetIndex, targetIndex};

            }
            // Target lies in smaller portion
            else if (lerp[targetIndex][0] > x) {
                subArrayEnd = targetIndex - 1;
            }
            // Target lies in larger portion
            else {
                subArrayStart = targetIndex + 1;
            }
        }

        // Search finished with no exact match
        return new int[] {subArrayEnd, subArrayStart};
    }

    private double findLerpValue(int lowerIndex, int higherIndex, double value, int targetIndice) {
        if (lowerIndex == higherIndex) return lerp[lowerIndex][targetIndice];
        double x1 = lerp[lowerIndex][0];
        double y1 = lerp[lowerIndex][targetIndice];
        double x2 = lerp[higherIndex][0];
        double y2 = lerp[higherIndex][targetIndice];
        return (y2-y1) / (x2-x1) * (value-x1) + y1; // Two-point form of a line
    }
    
}
