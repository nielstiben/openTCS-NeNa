package nl.saxion.nena.opentcs.commadapter.ros2.comm;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionController;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionListener;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.communication.ConnectionStatus;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.assertTrue;

public class CommunicationTest {
    @Before
    public void setUp() {
        System.out.println("hi");
    }

    @Test
    public void testOne(){
        assertTrue(true);
    }

        @Test
    public void setupConnection() {
            ConnectionListener listener = new ConnectionListener() {
                @Override
                public void onConnectionStatusChange(ConnectionStatus connectionStatus) {
                    // Do nothing
                }
            };
            ConnectionController connectionController = new ConnectionController(listener);
            connectionController.connect(1);
        }
}
