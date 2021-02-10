/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import geometry_msgs.msg.dds.PoseWithCovarianceStamped;
import org.opentcs.data.model.Triple;
import us.ihmc.euclid.tuple3D.Point3D;

import javax.annotation.Nonnull;

/**
 * Library class for parsing incoming ROS2 messages
 *
 * @author Niels Tiben
 */
public class IncomingMessageLib {
    private IncomingMessageLib() {
        // Empty Constructor
    }

    public static Triple generateTripleByAmclPose(@Nonnull PoseWithCovarianceStamped amclPose){
        Point3D amclPosePoint = amclPose.getPose().getPose().getPosition();

        Triple estimatePositionUnscaled = UnitConverterLib.convertCoordinatesInMeterToTriple(
                amclPosePoint.getX(),
                amclPosePoint.getY(),
                amclPosePoint.getZ()
        );

        // Scale triple
        return ScaleCorrector.getInstance().scaleTripleForFleetManager(estimatePositionUnscaled);
    }
}
