package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import builtin_interfaces.msg.dds.Time;
import geometry_msgs.msg.dds.PoseStamped;
import geometry_msgs.msg.dds.PoseWithCovariance;
import geometry_msgs.msg.dds.PoseWithCovarianceStamped;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;
import std_msgs.msg.dds.Header;
import us.ihmc.euclid.geometry.Pose3D;

import javax.annotation.Nonnull;
import java.time.Instant;

/**
 * Library class for parsing outgoing ROS2 messages
 *
 * @author Niels Tiben
 */
public abstract class OutgoingMessageLib {
    public static PoseWithCovarianceStamped generateInitialPoseMessageByPoint(@Nonnull Point point) {
        // Pose
        Pose3D pose = generatePoseMessageByPoint(point);
        PoseWithCovariance poseWithCovariance = new PoseWithCovariance();
        poseWithCovariance.pose_ = pose;

        // Pose Stamped
        PoseWithCovarianceStamped poseWithCovarianceStamped = new PoseWithCovarianceStamped();
        poseWithCovarianceStamped.pose_ = poseWithCovariance;
        poseWithCovarianceStamped.header_= generateMapHeader();

        return poseWithCovarianceStamped;
    }

    public static PoseStamped generateScaledNavigationMessageByPoint(@Nonnull Point point) {
        // Pose
        Pose3D pose = generatePoseMessageByPoint(point);

        // Pose Stamped
        PoseStamped poseStamped = new PoseStamped();
        poseStamped.pose_ = pose;
        poseStamped.header_ = generateMapHeader();

        return poseStamped;
    }

    private static Pose3D generatePoseMessageByPoint(@Nonnull Point point) {
        Triple triple = point.getPosition();
        double[] xyzUnscaled = UnitConverterLib.convertTripleToCoordinatesInMeter(triple);
        double[] xyzScaled = ScaleCorrector.getInstance().scaleCoordinatesForVehicle(xyzUnscaled);

        return new Pose3D(
                xyzScaled[0],
                xyzScaled[1],
                xyzScaled[2],
                0,
                0,
                0
        );
    }

    private static Header generateMapHeader(){
        Header header = new Header();
        Time time = new Time();
        time.setSec((int) Instant.now().getEpochSecond());
        header.frame_id_ = new StringBuilder("map");
        header.stamp_ = time;

        return header;
    }
}
