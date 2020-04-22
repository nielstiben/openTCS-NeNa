/**
 * Copyright (c) Fraunhofer IML
 */
package nl.saxion.nena.opentcs.commadapter.ros2;

import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2AdapterComponentsFactory;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.factory.Ros2CommAdapterFactory;
import org.junit.Before;
import org.junit.Test;
import org.opentcs.data.model.Vehicle;

import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;

/**
 * @author Stefan Walter (Fraunhofer IML)
 */
public class ExampleCommAdapterFactoryTest {

    private Ros2CommAdapterFactory ros2AdapterFactory;

    @Before
    public void setUp() {
        ros2AdapterFactory = new Ros2CommAdapterFactory(mock(Ros2AdapterComponentsFactory.class));
    }

    @Test
    public void provideAdapterForVehicleWithoutProperties() {
        assertTrue(ros2AdapterFactory.providesAdapterFor(new Vehicle("Some vehicle")));
    }

//    @Test
//    public void setupConnection() {
//        ConnectionListener listener = new ConnectionListener() {
//            @Override
//            public void onConnectionStatusChange(ConnectionStatus connectionStatus) {
//                // Do nothing
//            }
//        };
//        ConnectionController connectionController = new ConnectionController(listener);
//        connectionController.connect(1);
//
//    }

}
