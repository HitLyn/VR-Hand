using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using System;

using RosMessageTypes.Geometry;
using RosMessageTypes.VrHand;



//[System.Serializable]
//public struct HandTips
//{
//    public string name;
//    public List
//}
public class HandPosePublisher : MonoBehaviour
{
    public static readonly Dictionary<string, string> RobotFingerTipNames = new Dictionary<string, string>
    {
        {"thumb", "world/arm_base_link/arm_base_link_inertia/arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0/lh_forearm/lh_wrist/lh_palm/lh_thbase/lh_thproximal/lh_thhub/lh_thmiddle/lh_thdistal/lh_thtip"},
        {"index", "world/arm_base_link/arm_base_link_inertia/arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0/lh_forearm/lh_wrist/lh_palm/lh_ffknuckle/lh_ffproximal/lh_ffmiddle/lh_ffdistal/lh_fftip"},
        {"middle", "world/arm_base_link/arm_base_link_inertia/arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0/lh_forearm/lh_wrist/lh_palm/lh_mfknuckle/lh_mfproximal/lh_mfmiddle/lh_mfdistal/lh_mftip"},
        {"ring", "world/arm_base_link/arm_base_link_inertia/arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0/lh_forearm/lh_wrist/lh_palm/lh_rfknuckle/lh_rfproximal/lh_rfmiddle/lh_rfdistal/lh_rftip" },
        {"pinky", "world/arm_base_link/arm_base_link_inertia/arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0/lh_forearm/lh_wrist/lh_palm/lh_lfmetacarpal/lh_lfknuckle/lh_lfproximal/lh_lfmiddle/lh_lfdistal/lh_lftip" },
    };

    //private List<OVRBone> FingerBones;
    public List<OVRBone> FingerBones;


    const int NumFingerTips = 5;
    [SerializeField]
    string TopicName = "/finger_tip_pose";
    [SerializeField]
    GameObject tams_f329_with_left_motorhand;
    //public OVRSkeleton Skeleton;
    [SerializeField]
    OVRSkeleton RightHand;
    [SerializeField]
    OVRSkeleton LeftHand;
    // Start is called before the first frame update
    // ROS Connector
    ROSConnection Ros;
    OVRBone Bone1;
    void Start()
    {
        // Get ROS connection static instance
        Ros = ROSConnection.GetOrCreateInstance();
        Ros.RegisterPublisher<HandTipPoseMsg>(TopicName);
    }

    // Update is called once per frame
    public void Publish()
    {
        var HandTipPoseMessage = new HandTipPoseMsg();
        //Vector3 data = FingerBones[12].Transform.position;
        foreach (var bone in LeftHand.Bones)
        {
            if (bone.Id == OVRSkeleton.BoneId.Hand_ThumbTip)
            {
                HandTipPoseMessage.thumb_tip = new PoseMsg
                {
                    position = bone.Transform.position.To<FLU>(),
                    orientation = bone.Transform.rotation.To<FLU>()
                };
            }

            if (bone.Id == OVRSkeleton.BoneId.Hand_IndexTip)
            {
                HandTipPoseMessage.index_tip = new PoseMsg
                {
                    position = bone.Transform.position.To<FLU>(),
                    orientation = bone.Transform.rotation.To<FLU>()
                };
            }

            if (bone.Id == OVRSkeleton.BoneId.Hand_MiddleTip)
            {
                HandTipPoseMessage.middle_tip = new PoseMsg
                {
                    position = bone.Transform.position.To<FLU>(),
                    orientation = bone.Transform.rotation.To<FLU>()
                };
            }

            if (bone.Id == OVRSkeleton.BoneId.Hand_RingTip)
            {
                HandTipPoseMessage.ring_tip = new PoseMsg
                {
                    position = bone.Transform.position.To<FLU>(),
                    orientation = bone.Transform.rotation.To<FLU>()
                };
            }

            if (bone.Id == OVRSkeleton.BoneId.Hand_PinkyTip)
            {
                HandTipPoseMessage.pinky_tip = new PoseMsg
                {
                    position = bone.Transform.position.To<FLU>(),
                    orientation = bone.Transform.rotation.To<FLU>()
                };
            }

        }

        Ros.Publish(TopicName, HandTipPoseMessage);
    }

    private void Update()
    {
        FingerBones = new List<OVRBone>(LeftHand.Bones);
        if (FingerBones.Count > 0)
        {
            Publish();
        }
    }

}
