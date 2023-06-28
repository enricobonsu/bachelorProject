
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

        # df2 contains all trajectories as one large df.
        df2 = pd.DataFrame()
        for df in self.dfs:
            df2 = pd.concat([df2, df])

        self.stateTable = self.generateStateTable(df2)
        # print(self.stateTable)


    # Generates the feature for each unique
    def generateStateTable(self, df):
        np2 = df.drop_duplicates().to_numpy(dtype=np.int32)
        stateTable = dict()
        
        # The non observed states for the tf need to be provided.
        for state in range(np2.shape[0]):
            z = np2[state, :4]
            # print(z)
            # z[0] =+1
            # z[1] =+1
            # z[2] =+1
            # print(z)

            # exit()
            stateTable[state] = z # store features + distance as unique state.

        return stateTable


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
        df['distanceToGoal'] -= 25.0  # filter data
        df = df[df['distanceToGoal'] >= 0]
        # print(df)
        # print(df.loc[df["isInDistance"] == 1].idxmax()['isInDistance'])
        # print()
        # exit()
        # print((df.loc[df["isInDistance"] == 1].idxmin()['isInDistance']))
        # df = df.iloc[df.loc[df["isInDistance"] == 1].idxmax()['isInDistance']:]
        # df = df.loc[df["isInDistance"] == 1] 
        # print(df)
        # if file is not 'traj0.csv':
        #     # Because of the respawning the first location is stored twice.
        #     df = df.iloc[1:]
        columns_titles = ["isInDistance", "isRedLight",
                          "passedIntersection", "distanceToGoal", "stop"]
        df = df.reindex(columns=columns_titles) # make the action (stop) the last action
        # print(df)
        return df


def main():
    traj = DataFrameTrajectory(prefix="traj")


if __name__ == '__main__':
    main()
