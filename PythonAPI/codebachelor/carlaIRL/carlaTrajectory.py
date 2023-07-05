import numpy as np
import pandas as pd
from DataFrameTrajectory import DataFrameTrajectory


class Trajectory:

    def __init__(self, df, stateTable):
        self.featureNames = df.columns.values

        # The initial state for this trajectory
        self.startState = self.findStateInStateTable(
            df.iloc[0, :5].to_numpy(dtype=np.int32), stateTable)

        # The terminal state for this trajectory
        self.terminalState = self.findStateInStateTable(
            df.iloc[-1, :5].to_numpy(dtype=np.int32), stateTable)

        # Transitions is a matrix with the transition observed during the trajectory
        # stateVisitationFrequency contains the frequency that a state has been visited
        self.transitions, self.stateVisitationFrequency = self.generateTrajectionTransition(
            df, stateTable)

    def generateTrajectionTransition(self, dfTrajectory, stateTable, debug=False):
        # Returns matrix that contains the state,action,state' for every step,
        # with row 0 the intial state.
        # Also returns state visitation frequencey for this trajectory.

        steps = dfTrajectory.to_numpy(dtype=np.int32)
        prevState = self.startState
        prevAction = steps[0, 5]
        steps = np.delete(steps, 0, axis=0)
        trajectoryTransition = np.empty((0, 3), dtype=int)

        # Create a matrix that contains the (state, action, next-state) transition
        for step in steps:
            newState = self.findStateInStateTable(step[:5], stateTable)
            trajectoryTransition = np.append(trajectoryTransition, np.array(
                [[prevState, prevAction, newState]]), axis=0)

            prevState = newState
            prevAction = step[5]

        vistedStates, stateVisitationFrequency = np.unique(
            trajectoryTransition[:, 2], axis=0, return_counts=True)

        # Add the visit of the initial state to the arrays
        vistedStates = np.append([1], vistedStates)
        stateVisitationFrequency = np.append([1], stateVisitationFrequency)

        trajectoryTransition, count = np.unique(
            trajectoryTransition, axis=0, return_counts=True)
        stateVisitationFrequency = dict(
            zip(vistedStates, stateVisitationFrequency))

        if debug:
            print("State feature names",
                  self.featureNames[:-1], "with action name", self.featureNames[-1])
            for index, i in enumerate(trajectoryTransition):
                print(i, "beginStateFeatures", stateTable.iloc[i[0]].tolist(), "- action", i[1],
                      "- endStateFeatures", stateTable.iloc[i[2]].tolist(), "count",  count[index])
            print("\n\n\n\n")

        return trajectoryTransition, stateVisitationFrequency

    def findStateInStateTable(self, state, stateTable):
        return stateTable.loc[(stateTable['isInDistance'] == (state[0])) & (stateTable['isRedLight'] == (state[1])) &
                              (stateTable['onIntersection'] == (state[2])) & (stateTable['passedIntersection'] == state[3]) & (stateTable['distanceToGoal'] == state[4])].index[0]



def main():
    x = DataFrameTrajectory()
    traj = Trajectory(x.dfs[20], x.stateTable)

    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
