import numpy as np
import optimizer as O
import maxentCarla as M
import carlaworld as W
import carlaTrajectory as demo


def maxent(world, terminal, trajectories):
    """
    Maximum Entropy Inverse Reinforcement Learning
    """
    # set up features: we use one feature vector per state
    # we hebben 3 features (distance, redlight, passedIntersection)
    features = np.zeros((89, 8))  # elke state moet nog zijn feature verhogen.
    for state in trajectories:
        if state[0] == 0 and state[1] == 0 and state[2] == 0:
            features[state[4]]
        elif state[0] == 0 and state[1] == 0 and state[2] == 1:
            features[state[4]] = features[state[4]] + 0.1
        elif state[0] == 0 and state[1] == 1 and state[2] == 0:
            features[state[4]] = features[state[4]] + 0.2
        elif state[0] == 0 and state[1] == 1 and state[2] == 1:
            features[state[4]] = features[state[4]] + 0.3
        elif state[0] == 1 and state[1] == 0 and state[2] == 0:
            features[state[4]] = features[state[4]] + 0.4
        elif state[0] == 1 and state[1] == 0 and state[2] == 1:
            features[state[4]] = features[state[4]] + 0.5
        elif state[0] == 1 and state[1] == 1 and state[2] == 0:
            features[state[4]] = features[state[4]] + 0.6
        elif state[0] == 1 and state[1] == 1 and state[2] == 1:
            features[state[4]] =

    print(features)
    # meter 53 tm 57 kunnen in de buurt zijn van het stoplicht.
    # elke state heeft 1 uit 8 mogelijke features
    # dus features = (aantal states, 2^3)
    # 2^3 = 8 state_features, dat zijn alle mogelijke combinaties van features.
    # eigenlijk zijn er nog minder, sinds niet alles te bereiken is.
    # choose our parameter initialization strategy:
    #   initialize parameters with constant
    init = O.Constant(1.0)
    # choose our optimization strategy:
    #   we select exponentiated gradient descent with linear learning-rate decay
    optim = O.ExpSga(lr=O.linear_decay(lr0=0.2))
    # actually do some inverse reinforcement learning
    reward = M.irl(world.p_transition, features,
                   terminal, trajectories, optim, init)
    return reward


def main():
    # set-up mdp
    # world, reward, terminal = setup_mdp()

    world = W.CarlaWorld()
    terminal = [0]  # state with 25m from end is the goal state.
    trajectories = demo.Trajectory().df2

    reward_maxent = maxent(world, terminal, trajectories)

    # generate "expert" trajectories

    # show the computed reward
    ax = plt.figure(num='MaxEnt Reward').add_subplot(111)
    P.plot_state_values(ax, world, reward_maxent, **style)
    plt.draw()

    # maximum casal entropy reinforcement learning (non-causal)
    reward_maxcausal = maxent_causal(world, terminal, trajectories)

    # show the computed reward
    ax = plt.figure(num='MaxEnt Reward (Causal)').add_subplot(111)
    P.plot_state_values(ax, world, reward_maxcausal, **style)
    plt.draw()

    plt.show()
    # while True:
    #     record.world.wait_for_tick()
    #     record.drawTrafficLightTriggers()
    #     # wp = record.map.get_waypoint(record.vehicle.get_location())
    #     # print(wp)
    #     record._affected_by_traffic_light(max_distance=2.0 + 0.3 * 30)


if __name__ == '__main__':
    main()
