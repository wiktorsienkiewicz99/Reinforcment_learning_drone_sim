Drone Flight Optimization Using a Genetic Algorithm

Overview:

This project implements a genetic algorithm to optimize the vertical flight profile of a simulated drone. The objective is to determine the most efficient on/off engine sequence that maximizes altitude while avoiding a crash landing. Each sequence (individual in the population) is a binary array, where each bit indicates whether the drone's engine is on (1) or off (0) at a given time step.

The simulation includes physics-based calculations of height, velocity, and acceleration, taking into account gravity, thrust, and friction. A crash penalty is applied if the final landing speed exceeds a defined safety threshold.

The system evaluates the average performance of each generation and allows mutation rate tuning across multiple passes to compare its influence on optimization efficiency.
Detailed Task Description
Objective:

Optimize a drone’s vertical flight pattern using a genetic algorithm to reach the highest possible altitude without crashing.
Simulation Setup:

    Engine Control Input:
    Each individual in the genetic population is a binary sequence where:
        1 indicates the drone engine is turned on during that time step.
        0 indicates the engine is off.
        The length of this sequence is calculated from t_max / time_step.

    Physics Model:
        Gravity and friction are always applied.
        Engine thrust is applied when the engine is on.
        Height and velocity are updated at each time step using standard kinematic equations.
        The drone is constrained to non-negative altitude (cannot go underground).
        A crash is defined as landing with velocity exceeding the crash_velocity threshold.

    Fitness Function:
        The fitness score is based on the maximum height achieved during the flight.
        A constant value is added to shift scores into the positive range.
        A crash penalty is subtracted if landing speed exceeds the threshold.

Genetic Algorithm:

    Population Initialization:
    Randomly generate population_size binary sequences.

    Parent Selection:
    Uses roulette wheel selection based on fitness scores to probabilistically favor higher-performing individuals.

    Crossover:
        Performed with probability pc.
        A random crossover point is selected.
        Bits after the crossover point are exchanged between two parents.

    Mutation:
        Each bit in each child is flipped with probability pm.

    Generations and Passes:
        The population evolves over num_iterations generations.
        The entire experiment is repeated for multiple passes, each with a different mutation probability (pm = 0.00 to 0.24).

    Data Collection:
        The average score per generation is recorded.
        For each mutation rate pass, the average score across all generations is computed and plotted.

Output:

    Graph:
    A line plot visualizing how average performance varies with mutation rate.
        X-axis: Pass number (corresponds to mutation rate).
        Y-axis: Average fitness score across all generations in that pass.

    Console Logs:
        Mutation rate for each pass.
        Average score for each generation and pass.
        Whether a crash occurred in individual simulations.