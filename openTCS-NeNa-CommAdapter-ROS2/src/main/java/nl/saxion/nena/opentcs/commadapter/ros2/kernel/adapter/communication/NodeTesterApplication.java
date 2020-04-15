package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import action_msgs.msg.GoalStatusArray;
import geometry_msgs.msg.Point;
import geometry_msgs.msg.Pose;
import geometry_msgs.msg.PoseStamped;

public class NodeTesterApplication implements NodeListener {
    Node node;

    public static void main(String[] args) throws InterruptedException {
        new NodeTesterApplication().run();
    }

    private void run() throws InterruptedException {
        NodeStarter nodeStarter = new NodeStarter(this);
        this.node = nodeStarter.start();
        sleep();

        sendGoal(3,1);
        System.out.println("hooray!");
    }

    /**
     * Send navigation goal to ROS2 using /move_base_simple topic.
     *
     * @param x X coordinate
     * @param y Y coordinate
     */
    public void sendGoal(double x, double y) {
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

    private void sleep() throws InterruptedException {
        System.out.println("Start counting ...");
        Thread.sleep(5000);
        System.out.println("... stop counting!");
    }

    public void stop() {
        node.stop();
        node = null;
    }

    private boolean isNodeStarted(){
        return node != null;
    }
}
