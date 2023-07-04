import numpy as np
import optimizer as O
import maxentCarla as M
import carlaworld as W
from carlaDemonstration import Demonstration
# import carlaDemonstration as demo


def maxent(demonstration):
    """
    Maximum Entropy Inverse Reinforcement Learning
    """
    # set up features: we use one feature vector per state
    # we hebben 3 features (distance, redlight, passedIntersection)
    terminalStates = demonstration.terminalStates
    p_transition = demonstration.p_transition

    # p_transition[list(demonstration.terminalStates),1,list(demonstration.terminalStates)] = 1 # terminal will always end into itself
    # p_transition[list(demonstration.terminalStates),0,list(demonstration.terminalStates)] = 1 # terminal will always end into itself

    trajectories = demonstration.trajectories
    stateToFeatures = demonstration.stateTable
    
    # choose our parameter initialization strategy:
    #   initialize parameters with constant
    init = O.Constant(0.00000000001)
    # choose our optimization strategy:
    #   we select exponentiated gradient descent with linear learning-rate decay
    optim = O.ExpSga(lr=O.linear_decay(lr0=0.001))
    # actually do some inverse reinforcement learning
    reward = M.irl(p_transition,stateToFeatures,
                   terminalStates, trajectories, optim, init)
    print()
    print()
    print(stateToFeatures)
    print(np.where(reward != 0)[0])
    return reward


def main():

    demonstration = Demonstration()
    maxent(demonstration)


    # set-up mdp
    # world, reward, terminal = setup_mdp()

    # world = W.CarlaWorld()
    # terminal = [0]  # state with 25m from end is the goal state.
    # trajectories = demo.Trajectory().df2

    # reward_maxent = maxent(world, terminal, trajectories)

    # # generate "expert" trajectories

    # # show the computed reward
    # ax = plt.figure(num='MaxEnt Reward').add_subplot(111)
    # P.plot_state_values(ax, world, reward_maxent, **style)
    # plt.draw()

    # # maximum casal entropy reinforcement learning (non-causal)
    # reward_maxcausal = maxent_causal(world, terminal, trajectories)

    # # show the computed reward
    # ax = plt.figure(num='MaxEnt Reward (Causal)').add_subplot(111)
    # P.plot_state_values(ax, world, reward_maxcausal, **style)
    # plt.draw()

    # plt.show()
    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
