package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication;

import action_msgs.msg.dds.GoalStatusArray;
import geometry_msgs.msg.dds.PoseStamped;
import geometry_msgs.msg.dds.PoseWithCovarianceStamped;
import lombok.Getter;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import javax.annotation.Nonnull;
import java.io.IOException;

@Getter
public class Node {
    private final ROS2Node node;
    private final ROS2Publisher<PoseWithCovarianceStamped> initialPosePublisher;
    private final ROS2Publisher<PoseStamped> goalPublisher;

    public Node(@Nonnull NodeMessageListener nodeMessageListener, int domainId, @Nonnull String namespace) throws IOException {
        this.node = new ROS2Node(
                DomainFactory.PubSubImplementation.FAST_RTPS,
                "opentcs",
                namespace,
                domainId
        );

        /* --------------- Publishers ---------------*/
        // Publisher for setting the initial pose
        this.initialPosePublisher = node.createPublisher(PoseWithCovarianceStamped.getPubSubType().get(), "/initialpose");

        // Publisher for sending a navigation goal
        this.goalPublisher = node.createPublisher(PoseStamped.getPubSubType().get(), "/goal_pose");

        /* --------------- Subscriptions ---------------*/
        // Subscriber for navigation status
        this.node.createSubscription(
                GoalStatusArray.getPubSubType().get(),
                subscriber -> {
                    GoalStatusArray message = new GoalStatusArray();
                    if (subscriber.takeNextData(message, null)) {
                        nodeMessageListener.onNewGoalStatusArray(message);
                    }
                },
                "/navigate_to_pose/_action/status"
        );

        // Subscription for current location
        this.node.createSubscription(
                PoseWithCovarianceStamped.getPubSubType().get(),
                subscriber -> {
                    PoseWithCovarianceStamped message = new PoseWithCovarianceStamped();
                    if (subscriber.takeNextData(message, null)) {
                        nodeMessageListener.onNewAmclPose(message);
                    }
                },
                "/amcl_pose"
        );

        // TODO: Implement Subscription for battery data (TurtleBot3 specific)

        // TODO: Implement publisher and subscriber for LOAD CARGO operation.

        // TODO: Implement publisher and subscriber for UNLOAD CARGO operation.
    }
}
