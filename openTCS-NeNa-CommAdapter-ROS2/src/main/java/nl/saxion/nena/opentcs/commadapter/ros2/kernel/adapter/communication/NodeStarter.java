package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.AllArgsConstructor;
import org.ros2.rcljava.RCLJava;

/**
 * Helper class to start a new ROS2 node in a separate thread.
 */
@AllArgsConstructor
public class NodeStarter {
    NodeListener nodeListener;

    public Node start() {
        NodeRunnable nodeRunnable = new NodeRunnable(nodeListener);
        new Thread(nodeRunnable).start();

        return nodeRunnable.node;
    }

    private static class NodeRunnable implements Runnable {
        private NodeListener nodeListener;
        private Node node;

        public NodeRunnable(NodeListener nodeListener) {
            this.nodeListener = nodeListener;
        }

        @Override
        public void run() {
            RCLJava.rclJavaInit();
            this.node = new Node(nodeListener);
            RCLJava.spin(node);
        }
    }
}
