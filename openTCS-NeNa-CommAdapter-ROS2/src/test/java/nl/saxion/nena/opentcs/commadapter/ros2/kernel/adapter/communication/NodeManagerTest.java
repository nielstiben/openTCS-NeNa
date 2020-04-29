package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;


import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.constants.NodeRunningStatus;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.point.CoordinatePoint;
import org.junit.Before;
import org.junit.Test;
import org.opentcs.data.model.Triple;

import java.util.Calendar;

/**
 * Unit test to cover {@link NodeManager}.
 *
 * @author Niels Tiben
 */
public class NodeManagerTest {
    private NodeRunningStatusListener nodeRunningStatusListener;
    private NodeMessageListener nodeMessageListener;

    //================================================================================
    // Callback variables.
    //================================================================================

    private NodeRunningStatus lastKnownNodeRunningStatus;

    //================================================================================
    // Pre operations
    //================================================================================

    @Before
    public void setUpListeners() {
        this.nodeRunningStatusListener = nodeRunningStatus -> this.lastKnownNodeRunningStatus = nodeRunningStatus;

        // Ignore message callbacks
        this.nodeMessageListener = new NodeMessageListener() {
            @Override
            public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) { }
            @Override
            public void onNewAmclPose(PoseWithCovarianceStamped amclPose) { }
            @Override
            public void onOperationLoadCargoFeedback(String feedback) { }
            @Override
            public void onOperationUnloadCargoFeedback(String feedback) { }
        };
    }

    //================================================================================
    // Tests
    //================================================================================

    @Test
    public void testLifecycle() {
        // 1: Create the node
        NodeManager nodeManager = new NodeManager();
        this.lastKnownNodeRunningStatus = nodeManager.getNodeRunningStatus();
        assert this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.NOT_ACTIVE);

        // 2: Start the node
        nodeManager.start(this.nodeRunningStatusListener, this.nodeMessageListener, "test");
        assert this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.INITIATING);

        // 3: Node (should) have been active.
        assertNodeRunningStatusFromInitiatingToActive();

        // 4: Stop the node
        nodeManager.stop();
        assert this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.NOT_ACTIVE);
    }

    @Test(expected = NullPointerException.class)
    public void testNodeUsageFailsOnNotActive(){
        // 1: Create the node
        NodeManager nodeManager = new NodeManager();

        // 2: Try publishing a message on a inactive node.
        CoordinatePoint testPoint = new CoordinatePoint(new Triple(0,0,0));
        PoseStamped testMessage = OutgoingMessageLib.generateScaledNavigationMessageByPoint(testPoint);

        assert nodeManager.getNodeRunningStatus().equals(NodeRunningStatus.NOT_ACTIVE);
        nodeManager.getNode().getGoalPublisher().publish(testMessage);
    }

    //================================================================================
    // Helper methods
    //================================================================================

    @SneakyThrows
    private synchronized void assertNodeRunningStatusFromInitiatingToActive() {
        Calendar timeLimitStamp = Calendar.getInstance();
        timeLimitStamp.add(Calendar.MILLISECOND, 1000); // It may take 1 second.

        assert this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.INITIATING);

        while (this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.INITIATING)) {
            // they are still the same, keep waiting.
            assert !timeLimitReached(timeLimitStamp);
        }

        assert this.lastKnownNodeRunningStatus.equals(NodeRunningStatus.ACTIVE);

    }

    private boolean timeLimitReached(Calendar timeLimitStamp) {
        Calendar currentTimestamp = Calendar.getInstance();
        return currentTimestamp.getTime().getTime() > timeLimitStamp.getTime().getTime();
    }
}
