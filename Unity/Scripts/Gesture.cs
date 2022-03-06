using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gesture : MonoBehaviour
{
    public OVRCustomSkeleton lskeleton;
    public OVRCustomSkeleton rskeleton;
    private List<OVRBone> lfingerBones;
    private List<OVRBone> rfingerBones;
    // Start is called before the first frame update
    void Start()
    {
        lfingerBones = new List<OVRBone>(lskeleton.Bones);
        rfingerBones = new List<OVRBone>(rskeleton.Bones);
        Debug.Log("here");
    }

    // Update is called once per frame
    void Update()
    {
        lfingerBones = new List<OVRBone>(lskeleton.Bones);
        Debug.Log(lfingerBones.Count);
    }
}
