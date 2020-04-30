package nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal;

import action_msgs.msg.GoalStatus;
import action_msgs.msg.GoalStatusArray;
import lombok.Setter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.Ros2ProcessModel;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.communication.Node;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants.NavigationGoalStatus;
import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;
import java.time.format.DateTimeFormatter;
import java.util.*;

import static nl.saxion.nena.opentcs.commadapter.ros2.I18nROS2CommAdapter.BUNDLE_PATH;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.vehicle_adapter.navigation_goal.constants.NavigationGoalStatus.ACTIVE;

/**
 * The Navigation Goal Tracker is used for parsing, storing
 * and handling a list of navigation goals that are submitted by the
 * {@link Node} via the {@link Ros2ProcessModel}.
 * TODO | Niels Tiben : write activity diagram.
 *
 * @author Niels Tiben
 */
public class NavigationGoalTracker {
    private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);
    private final NavigationGoalListener navigationGoalListener;
    private final ExternalNavigationGoalListener externalNavigationGoalListener;

    //================================================================================
    // Class variables.
    //================================================================================
    private HashMap<List<Byte>, NavigationGoal> navigationGoalMap = new HashMap<>(); // key-val pair: UUID - Navigation Goal
    @Setter
    private NavigationGoalListener commandExecutorListener = null;
    @Setter
    private Point destinationPointIncomingGoal = null;

    //================================================================================
    // Constructor
    //================================================================================
    public NavigationGoalTracker(
            NavigationGoalListener navigationGoalListener,
            ExternalNavigationGoalListener externalNavigationGoalListener
    ) {
        this.navigationGoalListener = navigationGoalListener;
        this.externalNavigationGoalListener = externalNavigationGoalListener;
    }

    //================================================================================
    // Methods for updating (and notifying about) the navigation goal map.
    //================================================================================

    /**
     * Update the Navigation Goal Tracker by a (new) incoming GoalStatusArray from rcljava.
     *
     * @param goalStatusArray The ros2 navigation stack..
     */
    public void updateByGoalStatusArray(@Nonnull GoalStatusArray goalStatusArray) {
        for (GoalStatus goalStatus : goalStatusArray.getStatusList()) {
            processNavigationGoal(goalStatus);
        }
        this.navigationGoalMap = sortByNewestDate(this.navigationGoalMap);

        // Take the 'newest' navigation goal, and notify other instances if needed.
        Optional<List<Byte>> firstNavigationGoalKey = this.navigationGoalMap.keySet().stream().findFirst();
        firstNavigationGoalKey.ifPresent(key -> notifyListenersIfNeeded(this.navigationGoalMap.get((key))));
    }

    private void processNavigationGoal(@Nonnull GoalStatus goalStatusToProcess) {
        List<Byte> goalStatusToProcessUuid = goalStatusToProcess.getGoalInfo().getGoalId().getUuid();

        if (this.navigationGoalMap.containsKey(goalStatusToProcessUuid)) {
            // Existing goal status => update it.
            updateExistingNavigationGoal(goalStatusToProcess);
        } else {
            // New goal status => add it.
            addNewNavigationGoal(goalStatusToProcess);
        }
    }

    private static HashMap<List<Byte>, NavigationGoal> sortByNewestDate(
            @Nonnull HashMap<List<Byte>, NavigationGoal> unsortedNavigationGoalMap
    ) {
        // Create a list from elements of HashMap
        List<Map.Entry<List<Byte>, NavigationGoal>> list = new LinkedList<>(unsortedNavigationGoalMap.entrySet());

        // Sort the list
        list.sort(Map.Entry.comparingByValue());
        Collections.reverse(list);

        // Put data from the sorted list back into hashmap
        HashMap<List<Byte>, NavigationGoal> temp = new LinkedHashMap<>();
        for (Map.Entry<List<Byte>, NavigationGoal> aa : list) {
            temp.put(aa.getKey(), aa.getValue());
        }
        return temp;
    }

    private void notifyListenersIfNeeded(@Nonnull NavigationGoal navigationGoal) {
        if (navigationGoal.getNavigationGoalStatus().equals(NavigationGoalStatus.SUCCEEDED)) {
            Point destinationPoint = navigationGoal.getDestinationPoint();
            if (destinationPoint != null) {
                this.navigationGoalListener.onNavigationGoalSucceeded(destinationPoint);
                this.commandExecutorListener.onNavigationGoalSucceeded(destinationPoint);
            } else {
                this.externalNavigationGoalListener.onExternalNavigationGoalSucceeded();
            }
        }
    }

    //================================================================================
    // Method for updating a single existing navigation goal.
    //================================================================================

    private void updateExistingNavigationGoal(@Nonnull GoalStatus goalStatusToProcess) {
        List<Byte> goalStatusToProcessUuid = goalStatusToProcess.getGoalInfo().getGoalId().getUuid();

        // Get navigation goal
        NavigationGoal navigationGoalToUpdate = this.navigationGoalMap.get(goalStatusToProcessUuid);

        // Update navigation goal
        navigationGoalToUpdate.setNavigationGoalStatusByGoalStatus(goalStatusToProcess);
        navigationGoalToUpdate.setLastUpdatedByGoalStatus(goalStatusToProcess);
    }

    //================================================================================
    // Methods for adding a new navigation goal.
    //================================================================================

    private void addNewNavigationGoal(@Nonnull GoalStatus goalStatusToProcess) {
        if (isNavigationGoalInitiatedByOpenTCS(goalStatusToProcess)) {

            // Navigation goal initiated by OpenTCS => include the known destination.
            NavigationGoal navigationGoal = new NavigationGoal(goalStatusToProcess, this.destinationPointIncomingGoal);
            this.navigationGoalMap.put(navigationGoal.getUuid(), navigationGoal);

            // Notify listeners
            this.navigationGoalListener.onNavigationGoalActive(this.destinationPointIncomingGoal);
            this.commandExecutorListener.onNavigationGoalActive(this.destinationPointIncomingGoal);

            // Reset destination point for new incoming navigation goals.
            resetDestinationPoint();

        } else {
            // Navigation goal not initiated by OpenTCS => destination is not known.
            NavigationGoal navigationGoal = new NavigationGoal(goalStatusToProcess, null);
            this.navigationGoalMap.put(navigationGoal.getUuid(), navigationGoal);
            this.externalNavigationGoalListener.onExternalNavigationGoalActive();
        }
    }

    private boolean isNavigationGoalInitiatedByOpenTCS(@Nonnull GoalStatus goalStatus) {
        NavigationGoalStatus navigationStatus = NavigationGoalStatus.getByStatusCodeNumber(goalStatus.getStatus());

        return navigationStatus.equals(ACTIVE)
                && this.destinationPointIncomingGoal != null
                && isTimeStampMatching();
    }

    private boolean isTimeStampMatching() {
        // TODO: implement timestamp comparator for extra validation.
        return true;
    }

    private void resetDestinationPoint() {
        this.destinationPointIncomingGoal = null;
    }

    //================================================================================
    // Parser / stringify methods
    //================================================================================

    /**
     * Converts a map of strings that can be used in a JTable.
     *
     * @return a String matrix of navigationGoalMap (dimension = n by 4)
     */
    public String[][] toStringTable() {
        if (navigationGoalMap.isEmpty()) {
            // No navigation goals, return nothing.
            return null;
        }

        ArrayList<String[]> stringMatrixList = new ArrayList<>();

        for (NavigationGoal navigationGoal : this.navigationGoalMap.values()) {
            String[] navigationGoalRule = {
                    Arrays.toString(navigationGoal.getUuid().toArray(new Byte[16])),
                    DateTimeFormatter.ofPattern("HH:mm:ss").format(navigationGoal.getLastUpdated()),
                    parsePointNameByNavigationGoal(navigationGoal),
                    navigationGoal.getNavigationGoalStatus().name(),
            };
            stringMatrixList.add(navigationGoalRule);
        }

        return stringMatrixList.toArray(new String[stringMatrixList.size()][4]); // four columns
    }

    private String parsePointNameByNavigationGoal(@Nonnull NavigationGoal navigationGoal) {
        if (navigationGoal.getDestinationPoint() != null) {
            return navigationGoal.getDestinationPoint().getName();
        } else {
            return bundle.getString("ros2CommAdapterPanel.navigation_goal_unknown_destination.text");
        }
    }

    //================================================================================
    // Reset method
    //================================================================================

    public void reset() {
        this.navigationGoalMap = new HashMap<>();
        this.destinationPointIncomingGoal = null;
    }
}
