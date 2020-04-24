package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.SneakyThrows;
import org.ros2.rcljava.RCLJava;

import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.NodeRunningStatus.*;

/**
 * Helper class to start a new ROS2 node in a separate thread.
 * Starting a new ROS2 node takes time so a non-blocking callback
 * is used to notify other instances when a ROS2 node has been initiated.
 *
 * @author Niels Tiben <nielstiben@outlook.com>
 */
@NoArgsConstructor
public class NodeManager implements NodeStarterListener {
    private NodeRunningStatusListener nodeRunningStatusListener;
    private NodeRunnable nodeRunnable;

    @Getter
    private OpentcsNode opentcsNode;

    @Getter
    private NodeRunningStatus nodeRunningStatus = NOT_ACTIVE;

    public void start(NodeRunningStatusListener nodeRunningStatusListener, NodeMessageListener nodeMessageListener, String namespace) {
        assert this.nodeRunningStatus == NodeRunningStatus.NOT_ACTIVE;
        this.nodeRunningStatusListener = nodeRunningStatusListener;

        changeNodeStatus(INITIATING);

        NodeRunnable nodeRunnable = new NodeRunnable(nodeMessageListener, this);

//        NodeWatcher nodeWatcher = new NodeWatcher(nodeRunnable, this);

        new Thread(nodeRunnable).start();
    }

    public void stop() {
        assert this.nodeRunningStatus == ACTIVE; // Only stopping active nodes can be stopped.
        changeNodeStatus(NOT_ACTIVE);

        this.nodeRunnable.stop();
        this.nodeRunnable = null;
        this.opentcsNode = null;
    }

    @Override
    public void onNodeInitiated(NodeRunnable initialisedNodeRunnable) {
        this.nodeRunnable = initialisedNodeRunnable;
        this.opentcsNode = initialisedNodeRunnable.getOpentcsNode();
        changeNodeStatus(ACTIVE);
    }

//    /**
//     * Runnable that watches the NodeRunnable and gives a callback when the node has been initialised.
//     */
//    @AllArgsConstructor
//    private static class NodeWatcher implements Runnable {
//        private NodeRunnable nodeRunnable;
//        private NodeStarterListener nodeStarterListener;
//
//        @SneakyThrows
//        @Override
//        public void run() {
//            new Thread(nodeRunnable).start();
//
////            while (nodeRunnable.getOpentcsNode() == null) {
////                // Waiting for node to initiate...
//////                Thread.sleep(500);
////            }
//            nodeStarterListener.onNodeInitiated(nodeRunnable);
//        }
//    }

    /**
     * Runnable for the node instance.
     */
    protected static class NodeRunnable implements Runnable {
        @Getter
        private OpentcsNode opentcsNode;
        private final NodeMessageListener nodeMessageListener;
        private NodeStarterListener nodeStarterListener;

        private boolean isActive;

        public NodeRunnable(NodeMessageListener nodeMessageListener, NodeStarterListener nodeStarterListener) {
            this.nodeMessageListener = nodeMessageListener;
            this.nodeStarterListener = nodeStarterListener;
        }

        @SneakyThrows
        @Override
        public void run() {
            this.isActive = true;
            RCLJava.rclJavaInit();
            this.opentcsNode = new OpentcsNode(nodeMessageListener);
            nodeStarterListener.onNodeInitiated(this);

            while (RCLJava.ok() && this.isActive) {
                RCLJava.spinSome(this.opentcsNode.getNode());
            }
        }

        public void stop() {
            this.isActive = false;
            this.opentcsNode.shutdown();
            this.opentcsNode = null;
        }
    }

    private void changeNodeStatus(NodeRunningStatus newNodeRunningStatus) {
        this.nodeRunningStatus = newNodeRunningStatus;
        nodeRunningStatusListener.onNodeStatusChange(newNodeRunningStatus);
    }
}
