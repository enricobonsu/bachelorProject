
import numpy as np
import pandas as pd

class Trajectory:

    def __init__(self):
        df = pd.DataFrame()
        for i in range (30):
            tempdf =pd.read_csv("traj"+str(i)+".csv")
            # print(tempdf)
            df = pd.concat([df, tempdf])
        # df = pd.read_csv("traj0.csv")
        df = df.round(0)

        self.df2 = df.drop_duplicates(keep='first')
        
        self.df2 = self.df2.drop([154]) # filter data
        2321
        

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
        # for i in self.df2:
        #     print(i[4])
        
        
        # self._t = transitions

    
        


def main(): 
    traj = Trajectory()

    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
