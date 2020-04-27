package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

import geometry_msgs.msg.Point;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import org.opentcs.data.model.Triple;

import javax.annotation.Nonnull;

/**
 * @author Niels Tiben
 */
public class IncomingMessageLib {
    public static Triple generateTripleByAmclPose(@Nonnull PoseWithCovarianceStamped amclPose){
        geometry_msgs.msg.Point amclPosePoint = amclPose.getPose().getPose().getPosition();

        Triple estimatePositionUnscaled = UnitConverterLib.convertCoordinatesInMeterToTriple(
                amclPosePoint.getX(),
                amclPosePoint.getY(),
                amclPosePoint.getZ()
        );
        // Scale triple
        Triple estimatedPostion = ScaleCorrector.getInstance().scaleTripleForFleetManager(estimatePositionUnscaled);

        return estimatedPostion;
    }
}
