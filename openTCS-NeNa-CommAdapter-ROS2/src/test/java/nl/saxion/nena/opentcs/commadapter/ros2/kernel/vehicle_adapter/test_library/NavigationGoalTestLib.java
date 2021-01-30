package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.test_library;

import action_msgs.msg.dds.GoalInfo;
import action_msgs.msg.dds.GoalStatus;
import action_msgs.msg.dds.GoalStatusArray;
import builtin_interfaces.msg.dds.Time;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.Node;
import unique_identifier_msgs.msg.dds.UUID;
import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.TopicDataType;


import java.util.ArrayList;
import java.util.Collections;

/**
 * Library class for creating GoalStatus dummies.
 *
 * @author Niels Tiben
 */
public class NavigationGoalTestLib {
    /**
     * Mocks a {@link GoalStatusArray} that could come from the {@link Node} amcl_pose subscriber.
     *
     * @return A GoalStatusArray holding one goal status with the given status code.
     */
    public static GoalStatusArray createDummyGoalStatusArraySingleEntry(byte statusCode) {
        final byte LAST_BYTE_UUID = 1;

        GoalStatusArray goalStatusArray = new GoalStatusArray();
        IDLSequence.Object<GoalStatus> status_list = new IDLSequence.Object<>(20, GoalStatus.getPubSubType().get());
        status_list.add(createDummyGoalStatus(LAST_BYTE_UUID, statusCode));
        goalStatusArray.status_list_ = status_list;

        return goalStatusArray;
    }

    /**
     * Mocks a single {@link GoalStatus} that could be part of a {@link GoalStatusArray}.
     *
     * @param uuidLastByte The last byte of the object's UUID
     * @param status       The status byte code
     * @return A dummy GoalStatus
     */
    public static GoalStatus createDummyGoalStatus(byte uuidLastByte, byte status) {
        GoalStatus dummyGoalStatus = new GoalStatus();

        // GoalInfo
        GoalInfo goalInfo = new GoalInfo();
        goalInfo.goal_id_ = createDummyUUID(uuidLastByte);
        Time stamp = new Time();
        stamp.nanosec_ = 0;
        stamp.sec_ = 1;
        goalInfo.stamp_ = stamp;

        dummyGoalStatus.goal_info_ = goalInfo;
        dummyGoalStatus.status_ = status;

        return dummyGoalStatus;
    }

    /**
     * Creates a 'simple' UUID of 16 bytes, of which the last byte is customisable.
     *
     * @param lastByte The last byte of the UUID.
     * @return The full UUID
     */
    private static UUID createDummyUUID(byte lastByte) {
        byte[] uuidArray = new byte[16];

        for (int i = 0; i < 15; i++) {
            uuidArray[i] = ((byte) 1);
        }
        uuidArray[15] = lastByte;

        UUID uuid = new UUID();
        uuid.uuid_ = uuidArray;

        return uuid;
    }
}
