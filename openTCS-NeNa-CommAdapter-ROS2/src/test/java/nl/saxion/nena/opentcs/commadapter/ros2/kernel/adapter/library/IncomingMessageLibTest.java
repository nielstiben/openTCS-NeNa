package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library;

import geometry_msgs.msg.PoseWithCovarianceStamped;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.PointTestLib;
import org.junit.Test;
import org.opentcs.data.model.Point;
import org.opentcs.data.model.Triple;

/**
 * Unit test to cover {@link IncomingMessageLib}.
 *
 * @author Niels Tiben
 */
public class IncomingMessageLibTest {
    @Test
    public void testGenerateTripleByAmclPose(){
        ScaleCorrector.getInstance().setScale(1);

        Point pointForAmclPose = PointTestLib.generatePointByNameAndCoordinate("TestPoint1", new Triple(1500, -1500, 0));

        // The message type for 'outgoing' initial pose message, is exactly the same as the 'incoming'
        // amcl pose message, So we use the OutGoingMessageLib for creating a dummy incoming amcl message.
        PoseWithCovarianceStamped amclPose = OutgoingMessageLib.generateInitialPoseMessageByPoint(pointForAmclPose);

        geometry_msgs.msg.Point amclPosePosition = amclPose.getPose().getPose().getPosition();
        System.out.println(amclPosePosition.getX());
        System.out.println(amclPosePosition.getY());
        assert amclPosePosition.getX() == 1.5;
        assert amclPosePosition.getY() == -1.5;
    }
}
