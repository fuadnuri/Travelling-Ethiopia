
from search.astart import AStarSearch
from search.ucs import UCS
from search.advs import CoffeeAdversarialSearch
from search.bfs_dfs import Search

from data.graph1 import graph1
from data.graph2 import graph2
from data.graph3 import heuristics
from data.graph4 import coffee_tree, coffee_utilities
from data.graph5 import relaxed_adj

def main():
    searcher = UCS(graph2)

    # 1. BFS/DFS (Figure 1
    searcher_bfs_dfs = Search(graph1)
    print("Q1.2 - BFS Addis to Axum:", searcher_bfs_dfs.bfs("Addis Ababa", "Axum"))

    # 2.2 UCS to Lalibela (Figure 2)
    searcher_ucs = UCS(graph2)
    cost_ucs, path_ucs = searcher_ucs.search("Addis Ababa", "Lalibela")
    print(f"Q2.2 - UCS Addis to Lalibela: {path_ucs} (Cost: {cost_ucs})")

    # 2.3 Multiple Goals Local Optimum
    goals = ["Axum", "Gondar", "Lalibela", "Babile", "Jimma", "Bale", "Sof Oumer", "Arba Minch"]
    searcher_ucs_multi = UCS(graph2)
    total_cost, full_path = searcher_ucs_multi.multi_goal_search("Addis Ababa", goals) # type: ignore
    print(f"Q2.3 - Multiple Goals Path: {full_path} (Total Cost: {total_cost})")
    # 3. A* Search to Moyale (Figure 3)
    search_astar = AStarSearch(graph2, heuristics)
    path_a, cost_a = search_astar.search("Addis Ababa", "Moyale")
    print(f"Q3 - A* Addis to Moyale: {path_a} (Cost: {cost_a})")

    # 4. MiniMax (Figure 4)
    coffee_bot = CoffeeAdversarialSearch(coffee_tree, coffee_utilities)
    score, coffee_path = coffee_bot.minimax("Addis Ababa", 0, True)
    print(f"Q4 - MiniMax Optimal Path: {coffee_path} (Quality: {score})")

if __name__ == "__main__":
    main()
