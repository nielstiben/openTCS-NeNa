package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter;

import org.opentcs.configuration.ConfigurationEntry;
import org.opentcs.configuration.ConfigurationPrefix;

/**
 * Configuration parameter holder containing a name, description and type of
 * each parameter that is used in the ROS2 vehicle adapter.
 *
 * @author Niels Tiben
 */
@ConfigurationPrefix(Ros2CommAdapterConfiguration.PREFIX)
public interface Ros2CommAdapterConfiguration {
    String PREFIX = "ros2.adapter";

    @ConfigurationEntry(
            type = "Boolean",
            description = "Whether to register/enable the ROS2 communication adapter.",
            orderKey = "0_enable")
    boolean enable();

    @ConfigurationEntry(
            type = "Double",
            description = "The scale of the plant model. 1:x where x is the input. Provide a small scale for little plants.",
            orderKey = "plantModelScale")
    double plantModelScale();

    @ConfigurationEntry(
            type = "Integer",
            description = "The adapter's command queue capacity.",
            orderKey = "1_attributes_1")
    int commandQueueCapacity();

    @ConfigurationEntry(
            type = "String",
            description = "The string to be treated as a recharge operation.",
            orderKey = "1_attributes_2")
    String rechargeOperation();
}
