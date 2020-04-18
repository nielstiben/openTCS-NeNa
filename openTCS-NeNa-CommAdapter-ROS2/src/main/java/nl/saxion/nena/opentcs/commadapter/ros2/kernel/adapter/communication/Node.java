package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.Point;
import geometry_msgs.msg.Pose;
import geometry_msgs.msg.PoseStamped;
import geometry_msgs.msg.PoseWithCovarianceStamped;
import lombok.Getter;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.publisher.Publisher;

import javax.annotation.Nonnull;

/**
 * Class that holds an an instances of a node and of all its publishers and subscriptions.
 */
@Getter
public class Node extends BaseComposableNode {
    private Publisher<PoseWithCovarianceStamped> initialPosePublisher;
    private Publisher<PoseStamped> goalPublisher;

    public Node(@Nonnull NodeListener nodeListener) {
        super("opentcs"); // Node Name
        /* --------------- Publishers ---------------*/
        // Publisher for setting the initial pose
        this.initialPosePublisher = node.createPublisher(PoseWithCovarianceStamped.class, "/initialpose");
        // Publisher for sending a navigation goal
        this.goalPublisher = node.createPublisher(PoseStamped.class, "/move_base_simple/goal");

        /* --------------- Subscriptions ---------------*/
        // Subscriber for navigation status
        node.createSubscription(GoalStatusArray.class, "/NavigateToPose/_action/status",
                nodeListener::onNewGoalStatusArray);

        // Subscription for current location
        // TODO, see: topic /amclpose

        // Subscription for battery data (TurtleBot3 specific)
        // TODO, see: http://docs.ros.org/api/turtlebot3_msgs/html/msg/SensorState.html
    }

    /**
     * Stop the node.
     */
    public void stop() {
        RCLJava.shutdown();
    }
}