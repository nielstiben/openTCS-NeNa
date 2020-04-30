package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.test_library;

import action_msgs.msg.GoalInfo;
import action_msgs.msg.GoalStatus;
import action_msgs.msg.GoalStatusArray;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.Node;
import org.ros2.rcljava.Time;
import unique_identifier_msgs.msg.UUID;

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
        goalStatusArray.setStatusList(Collections.singletonList(
                createDummyGoalStatus(LAST_BYTE_UUID, statusCode)
        ));

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
        goalInfo.setGoalId(createDummyUUID(uuidLastByte));
        goalInfo.setStamp(Time.now());

        dummyGoalStatus.setGoalInfo(goalInfo);
        dummyGoalStatus.setStatus(status);

        return dummyGoalStatus;
    }

    /**
     * Creates a 'simple' UUID of 16 bytes, of which the last byte is customisable.
     *
     * @param lastByte The last byte of the UUID.
     * @return The full UUID
     */
    private static UUID createDummyUUID(byte lastByte) {
        ArrayList<Byte> uuidList = new ArrayList<>(16);

        for (int i = 0; i < 15; i++) {
            uuidList.add((byte) 1);
        }
        uuidList.add(lastByte);

        UUID uuid = new UUID();
        uuid.setUuid(uuidList);

        return uuid;
    }
}
