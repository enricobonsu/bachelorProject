
import numpy as np
import pandas as pd
import glob

class DfToTrajectory:

    def __init__(self, prefix = "traj",path = None):
        files = self.findTrajectories(prefix)

        def retrieveCsvNumber(fileName):
            """
            Retrieves the trajectory number from the csv file name
            """
            x = fileName.split(".")[0]
            x = x[4:]
            return int(x)
        
        files.sort(key = lambda x: retrieveCsvNumber(x))
        self.readTrajectories(files)
    
    def findTrajectories(self,prefix = "traj", path = None):
        """
        Finds all csv files for a given prefix either in the current folder, or a provided path folder. 
        """
        path = prefix+ r'*.csv'
        files = glob.glob(path)
        return files

    def readTrajectories(self,files):
        df = pd.DataFrame()
        for file in files:
            tempdf =pd.read_csv(file)
            df = pd.concat([df, tempdf])
        df = df.round(0)

        self.df2 = df.drop_duplicates(keep='first')
        
        self.df2 = self.df2.drop([154]) # filter data
        # 2321
        

        self.df2['distanceToGoal'] -= 25.0 # filter data
        self.df2 = self.df2.sort_values(by=['distanceToGoal'],ascending = False)
        self.df2 = self.df2[self.df2['distanceToGoal'] >= 0]
        self.df2 = self.df2.drop([2321])
        self.df2 = self.df2.drop([1987])
        self.df2 = self.df2.drop([1989])
        # 1989
        

        print(self.df2)
        print(self.df2.loc[(self.df2['isInDistance'] == 0) & (self.df2['isRedLight'] == 0) & (self.df2['passedIntersection'] == 0)])

        print(self.df2.loc[(self.df2['isInDistance'] == 1)])

        self.df2 = self.df2.to_numpy()
        # self._t = transitions



def main(): 
    traj = DfToTrajectory(prefix="traj")

if __name__ == '__main__':
    main()

