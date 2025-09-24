# Grid City Delivery Agent: Mannan Arora, 24BAS10066

This project is for the Vityarthi flipped course of Fundamentals of AI and ML.
---

## Features

* *Complex Grid Environment:* The city grid supports:
    * Static walls (X)
    * Variable terrain costs (e.g., 2, 4)
    * Scheduled moving obstacles (vehicles)
    * Time-based traffic multipliers (rush hour)

* *Multiple Search Algorithms:*
    * *Uninformed:* Breadth-First Search (BFS), Uniform-Cost Search (UCS)
    * *Informed:* A* (A-Star) with the Manhattan distance heuristic
    * *Local Search:* Simulated Annealing

* *Dynamic Replanning:* Includes a simulation to show the agent adapting to a new, unexpected obstacle by finding a new path on the fly.

* *Performance Comparison:* Automatically runs all algorithms on a set of test maps and generates a dynamic analysis comparing their path cost, efficiency (nodes expanded), and speed.

---

## How to Run

1.  Save the code as a Python file (e.g., agent.py).
2.  Run it from your terminal:
    bash
    python agent.py
    
3.  That's it. All the maps are included in the script, so no extra files are needed.

To turn the visual map grids on or off in the output, simply change the SHOW_MAPS variable at the top of the main execution block in the script.

---

## Map Format

The maps are included as multi-line strings in the script. The format is simple:

* S - Start
* G - Goal
* X - Wall (impassable)
* . or 1 - Normal path (cost of 1)
* 2-9 - High-cost terrain (cost is the number)
* --- - A separator between the grid and the schedule below it.

After the separator, you can add lines for dynamic events:

* D,r,c,t1,t2,... - A *D*ynamic obstacle appears at (row r, col c) at time steps t1, t2, etc.
* T,r,c,t,m - *T*raffic at (row r, col c) at time t has a cost *m*ultiplier of m.
