
import numpy as np
import pandas as pd
import glob


class DataFrameTrajectory(object):

    def __init__(self, prefix="traj", path=None):
        files = self.findTrajectories(prefix, path)

        def retrieveCsvNumber(fileName):
            """
            Retrieves the trajectory number from the csv file name
            """
            x = fileName.split(".")[0]
            x = x[4:]
            return int(x)

        files.sort(key=lambda x: retrieveCsvNumber(x))

        # dfs contains all trajectories, stored as a list of dfs
        self.dfs = []
        for file in files:
            self.dfs.append(self.csvToDf(file))
 
        self.stateTable = self.generateStateTable()


    # Generates the feature for each unique
    def generateStateTable(self):
        df = pd.read_csv("stateFeatures.csv")
        # df["isInDistance"] += 1 
        # df["isRedLight"] += 1 
        # df["passedIntersection"] += 1 
        stateDict = df.set_index('state')
        # print(stateDict)
        return stateDict


    def findTrajectories(self, prefix="traj", path=None):
        """
        Finds all csv files for a given prefix either in the current folder, or a provided path folder. 
        """
        path = prefix + r'*.csv'
        files = glob.glob(path)
        files.pop(0) # The first trajectory is always inconsistent.
        return files

    def csvToDf(self, file):
        df = pd.read_csv(file)

        df = df.round(0)
        # offset action, will now correspond correctly to s,a,s'
        df['stop'] = df['stop'].shift(-1)
        df = df.fillna(0)  # when on goal, we just keep going forward.

        # Let the trajectory observations start from the furthest point from the destination
        furthestPosition = df['distanceToGoal'].max()

        # Respawning takes two ticks before a transition takes place.
        indexFurthestPoint = df.loc[(df['distanceToGoal'] == furthestPosition)].index.max()

        df = df.loc[indexFurthestPoint:]
        columns_titles = ["isInDistance", "isRedLight","onIntersection",
                          "passedIntersection", "distanceToGoal", "stop"]
        df = df.reindex(columns=columns_titles) # make the action (stop) the last action
        # df["isInDistance"] += 1 
        # df["isRedLight"] += 1 
        # df["passedIntersection"] += 1 

        # print(df)
        # exit()
        return df


def main():
    traj = DataFrameTrajectory(prefix="traj")


if __name__ == '__main__':
    main()
