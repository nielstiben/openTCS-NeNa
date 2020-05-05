package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import builtin_interfaces.msg.Time;
import geometry_msgs.msg.*;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import std_msgs.msg.Header;

import javax.annotation.Nonnull;
import java.time.Instant;

/**
 * Library class for parsing outgoing rcljava messages
 *
 * @author Niels Tiben
 */
public abstract class OutgoingMessageLib {
    public static PoseWithCovarianceStamped generateInitialPoseMessageByPoint(@Nonnull Point point) {
        Pose pose = generatePoseMessageByPoint(point);
        Quaternion quaternion = new Quaternion();
        quaternion.setW(1.0); // 1.0 = default value for ROS2; Mandatory field
        pose.setOrientation(quaternion);

        PoseWithCovariance poseWithCovariance = new PoseWithCovariance();
        poseWithCovariance.setPose(pose);

        PoseWithCovarianceStamped poseWithCovarianceStamped = new PoseWithCovarianceStamped();
        poseWithCovarianceStamped.setPose(poseWithCovariance);

        Header header = new Header();
        header.setFrameId("map"); // Mandatory field

        Time time = new Time();
        time.setSec((int) Instant.now().getEpochSecond());
        header.setStamp(time);

        poseWithCovarianceStamped.setHeader(header);

        return poseWithCovarianceStamped;
    }

    public static PoseStamped generateScaledNavigationMessageByPoint(@Nonnull Point point) {
        Pose pose = generatePoseMessageByPoint(point);

        geometry_msgs.msg.PoseStamped poseStamped = new geometry_msgs.msg.PoseStamped();
        poseStamped.setPose(pose);

        return poseStamped;
    }

    private static Pose generatePoseMessageByPoint(@Nonnull Point point) {
        Triple triple = point.getPosition();
        double[] xyzUnscaled = UnitConverterLib.convertTripleToCoordinatesInMeter(triple);
        double[] xyzScaled = ScaleCorrector.getInstance().scaleCoordinatesForVehicle(xyzUnscaled);

        geometry_msgs.msg.Point position = new geometry_msgs.msg.Point();
        position.setX(xyzScaled[0]);
        position.setY(xyzScaled[1]);
        position.setZ(xyzScaled[2]);

        geometry_msgs.msg.Pose pose = new geometry_msgs.msg.Pose();
        pose.setPosition(position);

        return pose;
    }
}
