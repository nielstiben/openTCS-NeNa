package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovariance;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;

public class MessageLib {
    public static PoseWithCovarianceStamped generateInitialPoseMessageByCoordinate(double x, double y) {
        geometry_msgs.msg.Point position = new geometry_msgs.msg.Point();
        position.setX(x);
        position.setY(y);

        geometry_msgs.msg.Pose pose = new geometry_msgs.msg.Pose();
        pose.setPosition(position);

        PoseWithCovariance poseWithCovariance = new PoseWithCovariance();
        poseWithCovariance.setPose(pose);

        PoseWithCovarianceStamped poseWithCovarianceStamped = new PoseWithCovarianceStamped();
        poseWithCovarianceStamped.setPose(poseWithCovariance);

        return poseWithCovarianceStamped;
    }

    public static PoseStamped generateNavigationMessageByPoint(Point point){
        Triple triple = point.getPosition();
        double[] xyz = UnitConverterLib.convertTripleToCoordinatesInMeter(triple);

        geometry_msgs.msg.Point position = new geometry_msgs.msg.Point();
        position.setX(xyz[0]);
        position.setY(xyz[1]);
        position.setZ(xyz[2]);

        geometry_msgs.msg.Pose pose = new geometry_msgs.msg.Pose();
        pose.setPosition(position);

        geometry_msgs.msg.PoseStamped poseStamped = new geometry_msgs.msg.PoseStamped();
        poseStamped.setPose(pose);

        return poseStamped;
    }
}
