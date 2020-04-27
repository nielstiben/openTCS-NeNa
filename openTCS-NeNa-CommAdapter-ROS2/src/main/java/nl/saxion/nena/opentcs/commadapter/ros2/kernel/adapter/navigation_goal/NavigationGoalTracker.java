package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import action_msgs.msg.GoalStatus;
import action_msgs.msg.GoalStatusArray;
import lombok.Setter;
import nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.constants.NavigationGoalStatus;
import org.opentcs.data.model.Point;

import javax.annotation.Nonnull;
import java.time.format.DateTimeFormatter;
import java.util.*;

import static nl.saxion.nena.opentcs.commadapter.ros2.I18nROS2CommAdapter.BUNDLE_PATH;
import static nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal.constants.NavigationGoalStatus.ACTIVE;

public class NavigationGoalTracker {
    private static final ResourceBundle bundle = ResourceBundle.getBundle(BUNDLE_PATH);
    private HashMap<List<Byte>, NavigationGoal> navigationGoalMap = new HashMap<>(); // Pair UUID - Navigation Goal
    private final NavigationGoalListener navigationGoalListener;
    private final ExternalNavigationGoalListener externalNavigationGoalListener;

    @Setter
    private NavigationGoalListener commandExecutorListener = null;

    @Setter
    private Point destinationPointIncomingGoal = null;

    public NavigationGoalTracker(
            NavigationGoalListener navigationGoalListener,
            ExternalNavigationGoalListener externalNavigationGoalListener
    ) {
        this.navigationGoalListener = navigationGoalListener;
        this.externalNavigationGoalListener = externalNavigationGoalListener;
    }

    public void updateByGoalStatusArray(@Nonnull GoalStatusArray goalStatusArray) {
        for (GoalStatus goalStatus : goalStatusArray.getStatusList()) {
            processNavigationGoal(goalStatus);
        }

        this.navigationGoalMap = sortByNewestDate(this.navigationGoalMap);

        // Notify other instances if needed.
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

    private void addNewNavigationGoal(GoalStatus goalStatusToProcess) {
        if (isNavigationGoalInitiatedByOpenTCS(goalStatusToProcess)) {

            // Navigation goal initiated by OpenTCS => include the known destination.
            NavigationGoal navigationGoal = new NavigationGoal(goalStatusToProcess, destinationPointIncomingGoal);
            this.navigationGoalMap.put(navigationGoal.getUuid(), navigationGoal);

            // Notify listeners
            this.navigationGoalListener.onNavigationGoalActive(destinationPointIncomingGoal);
            this.commandExecutorListener.onNavigationGoalActive(destinationPointIncomingGoal);

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
        NavigationGoalStatus navigationStatus = NavigationGoalStatus.getByStatusCode(goalStatus.getStatus());
        if (navigationStatus.equals(ACTIVE) && this.destinationPointIncomingGoal != null && isTimeStampMatching()) {
            // Initiated by OpenTCS
            return true;
        } else {
            // Not initiated by OpenTCS
            return false;
        }
    }

    private boolean isTimeStampMatching() {
        // TODO: implement timestamp comparitor for extra validation.
        return true;
    }

    private void resetDestinationPoint() {
        this.destinationPointIncomingGoal = null;
    }


    private void updateExistingNavigationGoal(@Nonnull GoalStatus goalStatusToProcess) {
        List<Byte> goalStatusToProcessUuid = goalStatusToProcess.getGoalInfo().getGoalId().getUuid();

        // Get navigation goal
        NavigationGoal navigationGoalToUpdate = this.navigationGoalMap.get(goalStatusToProcessUuid);

        // Update navigation goal
        navigationGoalToUpdate.setNavigationGoalStatusByGoalStatus(goalStatusToProcess);
        navigationGoalToUpdate.setLastUpdatedByGoalStatus(goalStatusToProcess);
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

    @Nonnull
    public static HashMap<List<Byte>, NavigationGoal> sortByNewestDate(@Nonnull HashMap<List<Byte>, NavigationGoal> hm) {
        // Create a list from elements of HashMap
        List<Map.Entry<List<Byte>, NavigationGoal>> list = new LinkedList<>(hm.entrySet());

        // Sort the list
        list.sort(Map.Entry.comparingByValue());
        Collections.reverse(list);

        // Put data from sorted list to hashmap
        HashMap<List<Byte>, NavigationGoal> temp = new LinkedHashMap<>();
        for (Map.Entry<List<Byte>, NavigationGoal> aa : list) {
            temp.put(aa.getKey(), aa.getValue());
        }
        return temp;
    }

    /**
     * Converts a map of strings that can be used in a JTable.
     *
     * @return a String matrix of navigationGoalMap (dimension = n by 4)
     */
    public String[][] toStringTable() {
        if (navigationGoalMap.isEmpty()) {
            // No navigation goals, return nothing.
            return null; //TODO better
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

    public void reset(){
        this.navigationGoalMap = new HashMap<>();
        this.destinationPointIncomingGoal = null;
    }
}
