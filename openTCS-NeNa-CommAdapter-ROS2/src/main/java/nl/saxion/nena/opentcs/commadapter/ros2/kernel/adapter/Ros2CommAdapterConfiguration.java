package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter;

import org.opentcs.configuration.ConfigurationEntry;
import org.opentcs.configuration.ConfigurationPrefix;

@ConfigurationPrefix(Ros2CommAdapterConfiguration.PREFIX)
public interface Ros2CommAdapterConfiguration {
    String PREFIX = "ros2.adapter";
//    String PREFIX = "example.commadapter";

    @ConfigurationEntry(
            type = "Boolean",
            description = "Whether to register/enable the ROS2 communication adapter.",
            orderKey = "0_enable")
    boolean enable();
    @ConfigurationEntry(
            type = "Double",
            description = "The scale of the plant model. Provide a small number for little plants. 1:x where x is the input.",
            orderKey = "plantModelScale")
    double plantModelScale();


    // TODO: Define the need for the config params below:
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

    @ConfigurationEntry(
            type = "Double",
            description = {"The simulation time factor.",
                    "1.0 is real time, greater values speed up simulation."},
            orderKey = "1_behaviour_3")
    double simulationTimeFactor();
}
