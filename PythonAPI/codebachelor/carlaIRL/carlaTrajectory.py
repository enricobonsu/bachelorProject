import numpy as np
import pandas as pd
from DataFrameTrajectory import DataFrameTrajectory


class Trajectory:

    def __init__(self, df, stateTable):
        self.featureNames = df.columns.values
        self.startState = self.findStateInStateTable(
            df.iloc[0, :4].to_numpy(dtype=np.int32), stateTable)
        self.terminalState = self.findStateInStateTable(
            df.iloc[-1, :4].to_numpy(dtype=np.int32), stateTable)
        self.transitions, self.stateVisitationFrequency = self.generateTransition(df, stateTable)
        # print(self.transitions)
        # print(self.stateVisitationFrequency)


    def generateTransition(self, df, stateTable, debug=False):
        # Returns matrix that contains the state,action,state' for every step. with row 0 begin the start state.
        # Also returns state visitation frequencey for this trajectory.
        steps = df.to_numpy(dtype=np.int32)
        prevState = self.startState
        prevAction = steps[0, 4]
        steps = np.delete(steps, 0, axis=0)
        transition = np.empty((0, 3), dtype=int)
        for step in steps:
            newState = self.findStateInStateTable(step[:4], stateTable)
            transition = np.append(transition, np.array(
                [[prevState, prevAction, newState]]), axis=0)

            prevState = newState
            prevAction = step[4]
        vistedStates, stateVisitationFrequency = np.unique(transition[:,0], axis=0, return_counts=True)
        

        transition, count = np.unique(transition, axis=0, return_counts=True)
        stateVisitationFrequency = dict(zip(vistedStates, stateVisitationFrequency))

        if debug:
            for index, i in enumerate(transition):
                print(i, "beginState", stateTable[i[0]], "- action", i[1],
                      "- endState", stateTable[i[2]], "count",  count[index])
            print()
            print()

        return transition, stateVisitationFrequency

    def findStateInStateTable(self, state, stateTable):
        key = list(filter(lambda x: np.array_equal(
            stateTable[x], state), stateTable))[0]
        return key


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
