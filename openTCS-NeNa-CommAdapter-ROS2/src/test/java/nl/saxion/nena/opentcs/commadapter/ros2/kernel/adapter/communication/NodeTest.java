package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import lombok.Getter;
import lombok.SneakyThrows;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.library.OutgoingMessageLib;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.point.CoordinatePoint;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.opentcs.data.model.Triple;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.SingleThreadedExecutor;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.test_library.Ros2CommAdapterTestLib.TIME_NEEDED_FOR_NODE_INITIALISATION;

/**
 * Unit test to cover {@link Node}.
 * It is required to have a ROS2 instance up and running, which is ready to pick up navigation tasks.
 * Otherwise {#testPublishMessageReceivedByVehicle()} will never get a message.
 *
 * @author Niels Tiben
 */
public class NodeTest {
    private NodeMessageListener nodeMessageListener;

    //================================================================================
    // Callback variables.
    //================================================================================
    private GoalStatusArray lastReceivedGoalStatusArray;

    //================================================================================
    // Pre operation
    //================================================================================
    @Before
    public void setUpListeners() {
        this.nodeMessageListener = new NodeMessageListener() {
            @Override
            public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) {
                lastReceivedGoalStatusArray = goalStatusArray;
            }

            @Override
            public void onNewAmclPose(PoseWithCovarianceStamped amclPose) {
            }

            @Override
            public void onOperationLoadCargoFeedback(String feedback) {
            }

            @Override
            public void onOperationUnloadCargoFeedback(String feedback) {
            }
        };
    }

    //================================================================================
    // Tests
    //================================================================================

    @Test
    public void testCreateNode() {
        RCLJava.rclJavaInit();
        assert RCLJava.ok();

        Node node = new Node(this.nodeMessageListener, "test");
        RCLJava.spinSome(node);
    }

    @Test
    @SneakyThrows
    public void testPublishNavigationMessage() {
        NodeStarterForTesting nodeStarter = new NodeStarterForTesting(this.nodeMessageListener);
        new Thread(nodeStarter).start();
        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION); // wait until the node get's available.

        // Build message
        CoordinatePoint testPoint = new CoordinatePoint(new Triple(100, -150, 0));

        PoseStamped testMessage = OutgoingMessageLib.generateScaledNavigationMessageByPoint(testPoint);
        nodeStarter.getNode().getGoalPublisher().publish(testMessage);
    }

    @Test
    @SneakyThrows
    public void testPublishMessageReceivedByVehicle() {
        // Before starting a subscriber, it should NOT have been possible to know the vehicle's pose.
        assert this.lastReceivedGoalStatusArray == null;
        testPublishNavigationMessage();

        Thread.sleep(TIME_NEEDED_FOR_NODE_INITIALISATION); // Give the vehicle 600 milliseconds to send its navigation stack.
        assert lastReceivedGoalStatusArray != null;
    }

    //================================================================================
    // Post operations
    //================================================================================

    @After
    public void shutdownRCLJava() {
        RCLJava.shutdown();
    }

    //================================================================================
    // Helper methods
    //================================================================================

    /**
     * Simple inner class for starting a node in a new thread.
     */
    public static class NodeStarterForTesting implements Runnable {
        private final NodeMessageListener nodeMessageListener;

        @Getter
        private Node node;

        protected NodeStarterForTesting(NodeMessageListener nodeMessageListener) {
            this.nodeMessageListener = nodeMessageListener;
        }

        @Override
        public void run() {
            RCLJava.rclJavaInit();
            SingleThreadedExecutor executor = new SingleThreadedExecutor();
            this.node = new Node(this.nodeMessageListener, "");
            executor.addNode(node);
            executor.spin();
        }
    }
}
