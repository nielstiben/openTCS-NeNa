package nl.saxion.nena.opentcs.commadapter.ros2.kernel.adapter.navigation_goal;

import action_msgs.msg.GoalStatus;
import action_msgs.msg.GoalStatusArray;
import lombok.NoArgsConstructor;

import java.util.*;

@NoArgsConstructor
public class NavigationGoalTracker {
    private HashMap<List<Byte>, NavigationGoal> navigationGoalMap = new HashMap<>(); // Pair UUID - Navigation Goal

    public void updateByGoalStatusArray(GoalStatusArray goalStatusArray) {
        for (GoalStatus goalStatus : goalStatusArray.getStatusList()) {
            NavigationGoal navigationGoal = new NavigationGoal(goalStatus);
            navigationGoalMap.put(navigationGoal.getUuid(), navigationGoal);
        }

        this.navigationGoalMap = sortByNewestDate(navigationGoalMap);
    }

    /**
     * Converts a map of strings that can be used in a JTable.
     *
     * @return a String matrix of navigationGoalMap (dimension = 3 by n)
     */
    public String[][] generateStringMatrix() {
        if (navigationGoalMap.isEmpty()) {
            // No navigation goals, return nothing.
            return null; //TODO better
        }

        ArrayList<String[]> stringMatrixList = new ArrayList<>();

        for (NavigationGoal navigationGoal : navigationGoalMap.values()) {
            String[] navigationGoalRule = {
                    Arrays.toString(navigationGoal.getUuid().toArray(new Byte[16])),
                    navigationGoal.getNavigationGoalStatus().name(),
                    navigationGoal.getLastUpdated().toString()
            };
            stringMatrixList.add(navigationGoalRule);
        }

        return stringMatrixList.toArray(new String[stringMatrixList.size()][3]); // three columns
    }

    public static HashMap<List<Byte>, NavigationGoal> sortByNewestDate(HashMap<List<Byte>, NavigationGoal> hm) {
        // Create a list from elements of HashMap
        List<Map.Entry<List<Byte>, NavigationGoal>> list = new LinkedList<>(hm.entrySet());

        // Sort the list
        list.sort(Map.Entry.comparingByValue());
        Collections.reverse(list);

        // put data from sorted list to hashmap
        HashMap<List<Byte>, NavigationGoal> temp = new LinkedHashMap<>();
        for (Map.Entry<List<Byte>, NavigationGoal> aa : list) {
            temp.put(aa.getKey(), aa.getValue());
        }
        return temp;
    }
}
