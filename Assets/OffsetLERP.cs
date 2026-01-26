using System;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;

public class OffsetLERP : MonoBehaviour
{

    public GameObject target;
    public GameObject shooter;

    public ShootOnTheMove shootOnTheMove;


    // Distance value, angle value, velocity value
    private double[][] lerp = new double[][]
    {

        new double[]{-4.4, -16.73913, 1.956522},
        new double[]{-3.022609, -10.65217, 1.347826},
        new double[]{-2.525218, -10.86957, 1},
        new double[]{-2.027826, -10.21739, 0.7391304},
        new double[]{-1.568696, -8.913043, 0.6086957},
        new double[]{-1.033044, -5.434783, 0.3913045},
        new double[]{-0.4973913, -5.434783, 0.08695662},
        new double[]{0, 0, 0},
        
    };

    public double getAngleOffset(double value)
    {
        int sign = Math.Sign(value);
        value = -Math.Abs(value);
        int[] indices = findClosestIndices(value);
        return -sign * findLerpValue(indices[0], indices[1], value, 1);
    }
    public double getVelocityOffset(double value)
    {
        int sign = Math.Sign(value);
        value = -Math.Abs(value);
        int[] indices = findClosestIndices(value);
        return -sign * findLerpValue(indices[0], indices[1], value, 2);
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
