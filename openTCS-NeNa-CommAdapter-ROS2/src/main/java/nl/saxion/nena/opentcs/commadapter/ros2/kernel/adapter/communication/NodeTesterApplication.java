package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.Point;
import geometry_msgs.msg.Pose;
import geometry_msgs.msg.PoseStamped;

public class NodeTesterApplication implements NodeStatusChangeListener, NodeListener {
    NodeManager nodeManager;
    int cnt = 0;
    public static void main(String[] args) {
        new NodeTesterApplication().run();
    }

    private void run() {
        this.nodeManager = new NodeManager();
        this.nodeManager.start(this, this, 0);
    }

    @Override
    public void onNodeStatusChange(NodeStatus nodeStatus) {
        if (nodeStatus == NodeStatus.ACTIVE){
//            sendGoal(this.nodeManager.getNode(), 3, 1);
            nodeManager.stop();
            System.out.println("Stopped");

            if(cnt != 2){
                nodeManager.start(this, this, 0);
                System.out.println("Started");
                cnt++;
            }
        }
    }

    /**
     * Send navigation goal to ROS2 using /move_base_simple topic.
     *
     * @param x X coordinate
     * @param y Y coordinate
     */
    public void sendGoal(Node node, double x, double y) {
        // 1. Set Position
        Point position = new Point()
                .setX(x)
                .setY(y);

        // 2. Set Pose
        Pose pose = new Pose()
                .setPosition(position);

        // 3. Set Message
        PoseStamped message = new geometry_msgs.msg.PoseStamped()
                .setPose(pose);

        // 4. Publish
        node.getGoalPublisher().publish(message);
        System.out.println("sent!");
    }

    @Override
    public void onNewGoalStatusArray(GoalStatusArray goalStatusArray) {
        System.out.println("NEW Message:");
        System.out.println(goalStatusArray.toString());
    }
}
