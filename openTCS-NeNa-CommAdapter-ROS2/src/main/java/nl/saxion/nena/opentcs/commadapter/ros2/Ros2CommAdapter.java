package nl.saxion.nena.opentcs.commadapter.ros2;

import org.opentcs.drivers.vehicle.BasicVehicleCommAdapter;
import org.opentcs.drivers.vehicle.MovementCommand;
import org.opentcs.drivers.vehicle.VehicleProcessModel;
import org.opentcs.util.ExplainedBoolean;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;
import java.util.List;

public class Ros2CommAdapter extends BasicVehicleCommAdapter {

    /**
     * Creates a new instance.
     *
     * @param vehicleModel         An observable model of the vehicle's and its comm adapter's attributes.
     * @param commandQueueCapacity The number of commands this comm adapter's command queue accepts.
     *                             Must be at least 1.
     * @param sentQueueCapacity    The maximum number of orders to be sent to a vehicle.
     * @param rechargeOperation    The string to recognize as a recharge operation.
     */
    public Ros2CommAdapter(VehicleProcessModel vehicleModel, int commandQueueCapacity, int sentQueueCapacity, String rechargeOperation) {
        super(vehicleModel, commandQueueCapacity, sentQueueCapacity, rechargeOperation);
    }

    @Override
    public void sendCommand(MovementCommand cmd) throws IllegalArgumentException {

    }

    @Override
    protected void connectVehicle() {

    }

    @Override
    protected void disconnectVehicle() {

    }

    @Override
    protected boolean isVehicleConnected() {
        return false;
    }

    @Nonnull
    @Override
    public ExplainedBoolean canProcess(@Nonnull List<String> operations) {
        return null;
    }

    @Override
    public void processMessage(@Nullable Object message) {

    }
}