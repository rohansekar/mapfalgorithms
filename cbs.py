import time as timer
import heapq
import random
import copy
from single_agent_planner import (
    compute_heuristics,
    a_star,
    get_location,
    get_sum_of_cost,
)


def detect_first_collision_for_path_pair(path1, path2):
    ##############################
    # Task 2.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    t = max(len(path1), len(path2))
    for i in range(t):
        if path1!=[] and path2!=[]:
            agent1_ver = get_location(path1, i)
            agent2_ver = get_location(path2, i)
            if agent1_ver == agent2_ver:
                return [agent1_ver], i
            if i > 0:
                agent1_edge = get_location(path1, i - 1)
                agent2_edge = get_location(path2, i - 1)
                if agent1_ver == agent2_edge and agent2_ver == agent1_edge:
                    return [agent1_ver, agent2_ver], i
    return None, None


def detect_collisions_among_all_paths(paths):
    ##############################
    # Task 2.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    all_collisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            coord, timestep = detect_first_collision_for_path_pair(paths[i], paths[j])
            if coord != None:
                all_collisions.append(
                    {"a1": i, "a2": j, "loc": coord, "timestep": timestep}
                )
    if all_collisions != None:
        return all_collisions
    return None


def standard_splitting(collision):
    ##############################
    # Task 2.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #
    #                         specified edge at the specified timestep
    constraints = []
    # print('%%%%%%%%%%%%%',collision)
    if len(collision) != 0:
        # print(collision)
        if len(collision["loc"]) == 1:
            constraints.append(
                {
                    "agent": collision["a1"],
                    "loc": [collision["loc"][0]],
                    "timestep": collision["timestep"],
                }
            )
            constraints.append(
                {
                    "agent": collision["a2"],
                    "loc": [collision["loc"][0]],
                    "timestep": collision["timestep"],
                }
            )
        elif len(collision["loc"]) == 2:
            constraints.append(
                {
                    "agent": collision["a1"],
                    "loc": [collision["loc"][0], collision["loc"][1]],
                    "timestep": collision["timestep"],
                }
            )
            constraints.append(
                {
                    "agent": collision["a2"],
                    "loc": [collision["loc"][1], collision["loc"][0]],
                    "timestep": collision["timestep"],
                }
            )
    return constraints


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(
            self.open_list,
            (node["cost"], len(node["collisions"]), self.num_of_generated, node),
        )
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        """Finds paths for all agents from their start locations to their goal locations"""

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {"cost": 0, "constraints": [], "paths": [], "collisions": []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(
                self.my_map,
                self.starts[i],
                self.goals[i],
                self.heuristics[i],
                i,
                root["constraints"],
            )
            if path is None:
                raise BaseException("No solutions")
            root["paths"].append(path)
        root["collisions"] = detect_collisions_among_all_paths(root["paths"])
        root["cost"] = get_sum_of_cost(root["paths"])
        self.push_node(root)

        # Task 2.1: Testing
        # print('&&&&&&&&&&&&&&&&&&&',type(root["collisions"]))

        # Task 2.2: Testing

        ##############################
        # Task 2.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # These are just to print debug output - can be modified once you implement the high-level search
        a = 0
        while len(self.open_list) > 0:
            curr = self.pop_node()
            curr["collisions"] = detect_collisions_among_all_paths(curr['paths'])
            if len(curr["collisions"]) == 0:
                self.print_results(curr)
                return curr["paths"]
            for j in curr["collisions"]:
                new_constraints = standard_splitting(j)
                for i in new_constraints:
                    child = copy.deepcopy(curr)
                    if i not in curr["constraints"]:
                        child["constraints"].append(i)
                    agent_id = i["agent"]
                    path = a_star(
                        self.my_map,
                        self.starts[agent_id],
                        self.goals[agent_id],
                        self.heuristics[agent_id],
                        agent_id,
                        child["constraints"],
                    )
                    a+=1
                    if a>500:
                        print('TimeOut')
                        self.print_results(curr)
                        return curr["paths"]
                    print("path", path)
                    if path is None:
                        return None
                    child["paths"][agent_id] = path
                    child["collisions"] = detect_collisions_among_all_paths(
                        child["paths"]
                    )
                    child["cost"] = get_sum_of_cost(child["paths"])
                    self.push_node(child)
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node["paths"])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
