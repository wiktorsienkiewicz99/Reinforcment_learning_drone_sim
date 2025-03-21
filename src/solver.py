from abc import ABC, abstractmethod
import numpy as np
import random
import matplotlib.pyplot as plt

class Solver(ABC):
    """Abstract base class for optimization solvers."""

    @abstractmethod
    def get_parameters(self):
        """Returns a dictionary of solver hyperparameters."""
        pass

    @abstractmethod
    def solve(self, problem, pop0, *args, **kwargs):
        """Solves the given problem with the initial population."""
        pass

class DroneSolver(Solver):
    """
    Genetic algorithm solver for optimizing drone flight height.
    """

    def __init__(
        self,
        t_max: float,
        pc: float,
        pm: float,
        population_size: int,
        time_step: float,
        num_iterations: int,
    ):
        # Hyperparameters
        self.t_max = t_max                  # Max engine runtime
        self.pc = pc                        # Crossover probability
        self.pm = pm                        # Mutation probability
        self.population_size = population_size
        self.time_step = time_step
        self.num_iterations = num_iterations

        # Drone state and simulation setup
        self.velocity = 0.0
        self.acceleration = 0.0
        self.max_height = 0.0
        self.height = 0.0
        self.pop = []
        self.bit_chart_size = int(np.floor(self.t_max / self.time_step))

        # Score tracking
        self.average_score = 0.0
        self.average_scores = []

        # Constants
        self.gravity = -10.0
        self.crash_penalty = -1500
        self.score_offset = 1500
        self.crash_velocity = 20.0
        self.engine_acceleration = 30.0

    def get_parameters(self):
        return {
            "t_max": self.t_max,
            "pc": self.pc,
            "pm": self.pm,
            "population_size": self.population_size
        }

    def population_init(self):
        """Initializes a population of binary sequences (engine on/off decisions)."""
        self.pop = [
            np.random.randint(2, size=self.bit_chart_size)
            for _ in range(self.population_size)
        ]

    def roulette_wheel_selection(self, scores):
        """Selects one parent index using roulette wheel selection."""
        total_fitness = sum(scores)
        probabilities = [score / total_fitness for score in scores]
        chosen_score = np.random.choice(scores, p=probabilities)
        index = int(np.where(scores == chosen_score)[0][0])
        return index

    def mutation(self, individual):
        """Mutates an individual with probability `pm`."""
        for j in range(self.bit_chart_size):
            if random.random() < self.pm:
                individual[j] ^= 1  # Flip bit
        return individual

    def crossover(self, population, parent_indices):
        """Applies crossover and mutation to generate new population."""
        new_population = []
        for i in range(self.population_size - 1):
            p1 = population[parent_indices[i]].copy()
            p2 = population[parent_indices[i + 1]].copy()

            if random.random() < self.pc:
                cut_point = random.randint(0, self.bit_chart_size)
                p1[cut_point:], p2[cut_point:] = p2[cut_point:], p1[cut_point:]

            new_population.append(self.mutation(p1))
            new_population.append(self.mutation(p2))

        return new_population[:self.population_size]

    def target_function(self, individual):
        """
        Simulates drone flight for a given binary sequence.
        Returns the score (max height, penalized if crashed).
        """
        self.height = 0.0
        self.velocity = 0.0
        self.max_height = 0.0

        i = 0
        while i < len(individual) or (self.height > 0 and i >= len(individual)):
            friction = -0.1 * self.velocity
            if i < len(individual):  # During engine control time
                engine_on = individual[i] == 1
                self.acceleration = (
                    self.engine_acceleration * engine_on +
                    self.gravity + friction
                )
                i += 1
            else:  # Free fall
                self.acceleration = self.gravity + friction

            self.velocity += self.acceleration * self.time_step
            self.height += self.velocity * self.time_step
            self.height = max(0.0, self.height)

            if self.height + self.score_offset > self.max_height:
                self.max_height = self.height + self.score_offset

        if abs(self.velocity) >= self.crash_velocity:
            return self.max_height + self.crash_penalty
        else:
            return self.max_height

    def solve(self, problem, initial_population, passes, *args, **kwargs):
        """
        Solves the problem using a genetic algorithm.
        Tries different mutation probabilities across multiple passes.
        """
        self.population_init()
        init_pop = self.pop.copy()
        mutation_rates = []
        pass_avg_scores = []

        for p in range(passes):
            self.pop = init_pop.copy()
            self.pm = p * 0.01
            mutation_rates.append(self.pm)
            self.average_scores = []

            print(f"\nPass {p+1}, Mutation Rate = {self.pm:.2f}")

            for iteration in range(self.num_iterations):
                scores = [problem(individual) for individual in self.pop]
                self.average_score = sum(scores) / self.population_size
                self.average_scores.append(self.average_score)

                parents = [self.roulette_wheel_selection(scores) for _ in self.pop]
                self.pop = self.crossover(self.pop, parents)

            avg_pass_score = sum(self.average_scores) / len(self.average_scores)
            pass_avg_scores.append(avg_pass_score)

            print(f"Pass Average Score: {avg_pass_score:.2f}")

        return mutation_rates, pass_avg_scores

def main():
    drone_solver = DroneSolver(
        t_max=10.0,
        pc=0.95,
        pm=0.0,
        population_size=100,
        time_step=0.1,
        num_iterations=500
    )

    mutation_rates, scores = drone_solver.solve(
        problem=drone_solver.target_function,
        initial_population=drone_solver.pop,
        passes=25
    )

    plt.plot(scores)
    plt.ylim([min(scores), max(scores)])
    plt.xlabel("Pass number")
    plt.ylabel("Average Score")
    plt.title("Mutation Rate vs Performance")
    plt.grid(True)
    plt.show()

    print("Mutation rates:", mutation_rates)
    print("Scores:", scores)

if __name__ == "__main__":
    main()
