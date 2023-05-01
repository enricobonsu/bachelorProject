import numpy as np


class CarlaWorld:

    def __init__(self, states=89):

        self.actions = [0, 1]  # stop or not

        self.n_states = states
        self.n_actions = len(self.actions)

        self.p_transition = self._transition_prob_table()

    def _transition_prob_table(self):
        """
        Builds the internal probability transition table.

        Returns:
            The probability transition table of the form

                [state_from, state_to, action]

            containing all transition probabilities. The individual
            transition probabilities are defined by `self._transition_prob'.
        """
        table = np.zeros(shape=(self.n_states, self.n_states, self.n_actions))

        # s1, s2, a = range(self.n_states), range(self.n_states), range(self.n_actions)
        for s_from in range(self.n_states):
            for s_to in range(self.n_states):
                for a in range(self.n_actions):
                    table[s_from, s_to, a] = self._transition_prob(
                        s_from, s_to, a)

        return table

    def _transition_prob(self, s_from, s_to, a):
        """
        Compute the transition probability for a single transition.

        Args:
            s_from: The state in which the transition originates.
            s_to: The target-state of the transition.
            a: The action via which the target state should be reached.

        Returns:
            The transition probability from `s_from` to `s_to` when taking
            action `a`.
        """

        # als we stoppen blijven we op dezelfde state
        if s_from == s_to and a == 1:
            return 1.0

        # als we niet stoppen zullen we 1 meter verder uitkomen
        if (s_from+1.0) == s_to and a == 0:  # wat zijn de waardes van die states?
            # Als we in de state waardoor we de volgende keer in de buurt van het licht zijn, dan kan het of rood of nietRood zijn.
            return 1.0

        # otherwise this transition is impossible
        return 0.0
