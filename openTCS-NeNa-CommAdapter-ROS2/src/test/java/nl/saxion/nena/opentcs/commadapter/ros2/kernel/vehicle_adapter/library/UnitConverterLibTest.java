/**
 * Copyright (c) Niels Tiben (nielstiben@outlook.com)
 */
package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.library;

import org.junit.Test;
import org.opentcs.data.model.Triple;
import us.ihmc.euclid.tuple4D.Quaternion;

import java.util.Arrays;

/**
 * @author Niels Tiben
 */
public class UnitConverterLibTest {
    @Test
    public void testConvertMillimetersToMeters() {
        long distanceInMillimeters = 1555;
        double distanceInMeters = UnitConverterLib.convertMillimetersToMeters(distanceInMillimeters);

        assert distanceInMeters == 1.555;
    }

    @Test
    public void testConvertTripleToCoordinatesInMeter() {
        Triple tripleInMillimeter = new Triple(1010, 2020, 3030);
        double[] coordinatesInMeter = UnitConverterLib.convertTripleToCoordinatesInMeter(tripleInMillimeter);

        boolean arraysAreTheSame = Arrays.equals(coordinatesInMeter, new double[]{1.01, 2.02, 3.03});
        assert arraysAreTheSame;
    }

    @Test
    public void testConvertCoordinatesInMeterToTriple() {
        double[] coordinatesInMeter = new double[]{1.01, 2.02, 3.03};
        Triple tripleInMillimeter = UnitConverterLib.convertCoordinatesInMeterToTriple(
                coordinatesInMeter[0],
                coordinatesInMeter[1],
                coordinatesInMeter[2]
        );

        assert tripleInMillimeter.equals(new Triple(1010, 2020, 3030));
    }

    @Test
    public void testQuaternionToAngleDegreeNorth() {
        double x = 0;
        double y = 0;
        double z = 0.71; // approximate north
        double w = 0.71; // approximate north
        Quaternion quaternionTowardsNorth = new Quaternion(x,y,z,w);

        double angleTowardsNorth = UnitConverterLib.quaternionToAngleDegree(quaternionTowardsNorth);
        assert angleTowardsNorth > 89 && angleTowardsNorth < 91; // approximate north
    }

    @Test
    public void testQuaternionToAngleDegreeWest() {
        double x = 0;
        double y = 0;
        double z = 1; // exact west
        double w = 0; // exact west

        Quaternion quaternionTowardsNorth = new Quaternion(x,y,z,w);
        double angleTowardsWest = UnitConverterLib.quaternionToAngleDegree(quaternionTowardsNorth);
        assert angleTowardsWest == 180; // exact west
    }
}
