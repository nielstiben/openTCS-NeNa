package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import geometry_msgs.msg.PoseStamped;

public class MessageLib {
    public static PoseStamped generatePoseStampedByCoordinate(double x, double y){
        geometry_msgs.msg.Point position = new geometry_msgs.msg.Point();
        position.setX(x);
        position.setY(y);

        geometry_msgs.msg.Pose pose = new geometry_msgs.msg.Pose();
        pose.setPosition(position);

        geometry_msgs.msg.PoseStamped poseStamped = new geometry_msgs.msg.PoseStamped();
        poseStamped.setPose(pose);

        return poseStamped;
    }
}
