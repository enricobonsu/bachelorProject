import numpy as np
import pandas as pd
from DataFrameTrajectory import DataFrameTrajectory
from carlaTrajectory import Trajectory


class Demonstration:
    def __init__(self):
        dfTrajectories = DataFrameTrajectory()
        self.trajectories = []
        self.stateTable = dfTrajectories.stateTable

        # dfTrajectories.dfs is a list of trajectories that are stores as df
        for df in dfTrajectories.dfs:
            self.trajectories.append(Trajectory(df, self.stateTable))

        self.transitionTable = self.generateTransitionTable()
        self.terminalStates = self.terminalState(self.trajectories)

        # contains probability of a given transition s,a,s'
        self.p_transition = self.generateProbTransition()
        
    def generateProbTransition(self, debug=True):
        n_states = self.stateTable.shape[0]
        
        stateActions = list(self.transitionTable.keys())
        possibleActions = set()
        for stateAction in stateActions:
            possibleActions.add(int(stateAction.split("-")[1]))
        

        n_actions = len(possibleActions)
        pTable = np.zeros(shape=(n_states, n_actions,n_states))
        
        for stateAction, outcomeStates in self.transitionTable.items():
            pair = stateAction.split("-")
            outcomes = set(outcomeStates)
            n_outcomes = len(outcomes)
            while outcomes:
                state = outcomes.pop()
                # print(self.stateTable)
                # probability = 0.5
                # if self.stateTable.iloc[int(pair[0])]['isInDistance'] == 1:
                #     if int(pair[0]) == state:
                #         probability -= 0.01
                #     else:
                #         probability += 0.01
                pTable[int(pair[0]),int(pair[1]), state] = 1/n_outcomes
                if debug:
                    print("(",pair[0],pair[1],state,") Has probability", 1/n_outcomes)
        
        if debug:
            countPossibleTransitions = 0
            for transitions in pTable:
                countPossibleTransitions += np.count_nonzero(transitions)
            print("Number of possible transitions", countPossibleTransitions)
        exit()
        return pTable



        
    # Generates a dict with the possible transitions for each state
    def generateTransitionTable(self, terminalState = 87):
        transitionDf = pd.read_csv("stateTransitions.csv")
    
        stateActionTransitions = dict()
        for _, row in transitionDf.iterrows():
            # Ignore transitions which are not observed during the demonstration
            if int(row['state-action'].split("-")[0]) >= terminalState:
                continue
            stateActionTransitions[row['state-action']] = set(eval(row['transitions']))

        return stateActionTransitions


    def terminalState(self, trajectories):
        terminals = set()
        for traj in trajectories:
            terminals.add(traj.terminalState)
        return terminals

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
