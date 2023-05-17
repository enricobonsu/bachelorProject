import numpy as np
import pandas as pd
from DataFrameTrajectory import DataFrameTrajectory
from carlaTrajectory import Trajectory


class Demonstration:
    def __init__(self):
        dfTrajectories = DataFrameTrajectory()
        self.trajectories = []
        self.stateTable = dfTrajectories.stateTable
        for df in dfTrajectories.dfs:
            self.trajectories.append(Trajectory(df, self.stateTable))

        self.observedTransitions = self.generateObservedTransition(
            self.trajectories)
        self.terminalStates = self.terminalState(self.trajectories)
        # features exclude the distance to the goal.
        self.features = len(dfTrajectories.stateTable[0])-1

        # contains probability of a given transition s,a,s'
        self.p_transition = self.generateProbTransition(
            self.observedTransitions)
        print(self.observedTransitions)
        
        

    def generateObservedTransition(self, trajectories):
        observedStateTransitions = dict()
        for traj in trajectories:
            for transition in traj.transitions:
                key = (transition[0], transition[1])
                if key in observedStateTransitions:
                    observedStateTransitions[key].add(transition[2])
                else:
                    observedStateTransitions[key] = {transition[2]}

        return observedStateTransitions

    def terminalState(self, trajectories):
        terminals = set()
        for traj in trajectories:
            terminals.add(traj.terminalState)
        return terminals

    def generateProbTransition(self, observedTransitions):
        n_actions= set()
        n_states = set()

        for state_action in observedTransitions.keys():
            n_actions.add(state_action[1])

        for state_action in observedTransitions.items():
            n_states.add(state_action[0][0])
            n_states.update(state_action[1])

        n_states = len(n_states)
        n_actions = len(n_actions)


        table = np.zeros(shape=(n_states, n_actions,n_states))
        for state_action_next in observedTransitions.items():
            probability = 1 / len(state_action_next[1])
            for next in state_action_next[1]:
                table[state_action_next[0][0],state_action_next[0][1],next] = probability
        #         print("(",state_action_next[0][0],state_action_next[0][1],next,") Has probability", probability)
        # print("table[0,0,0]", table[0,0,0])
        # exit()
        return table


def main():
    demo = Demonstration()
    # traj = Trajectory(x.dfs[0], x.stateTable)

    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
