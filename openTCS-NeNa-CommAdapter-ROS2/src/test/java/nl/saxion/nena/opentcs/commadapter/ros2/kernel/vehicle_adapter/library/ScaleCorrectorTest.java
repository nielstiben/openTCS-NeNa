/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import org.junit.Test;
import org.opentcs.data.model.Triple;

/**
 * Unit test to cover {@link ScaleCorrector}.
 *
 * @author Niels Tiben
 */
public class ScaleCorrectorTest {
    @Test
    public void testScaleCoordinatesForVehicle() {
        ScaleCorrector.getInstance().setScale(2);
        double[] unscaledCoordinate = new double[]{2, 2, 2};
        double[] scaledCoordinate = ScaleCorrector.getInstance().scaleCoordinatesForVehicle(unscaledCoordinate);

        assert scaledCoordinate[0] == 4;
        assert scaledCoordinate[1] == 4;
        assert scaledCoordinate[2] == 4;
    }

    @Test
    public void testScaleCoordinatesForFleetManager() {
        ScaleCorrector.getInstance().setScale(2);
        Triple unscaledCoordinate = new Triple(4,4,4);
        Triple scaledCoordinate = ScaleCorrector.getInstance().scaleTripleForFleetManager(unscaledCoordinate);

        assert scaledCoordinate.getX() == 2;
        assert scaledCoordinate.getY() == 2;
        assert scaledCoordinate.getZ() == 2;
    }

    @Test(expected = AssertionError.class)
    public void testInvalidScale() {
        ScaleCorrector.getInstance().setScale(0);
        Triple unscaledCoordinate = new Triple(4,4,4);
        ScaleCorrector.getInstance().scaleTripleForFleetManager(unscaledCoordinate);
    }

}
