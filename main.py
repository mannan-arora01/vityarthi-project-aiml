import time
import random
import math
from collections import deque

def load_map(map_data):
    grid = []
    moving_stuff = {}
    traffic = {}
    start_pos, goal_pos = None, None

    all_lines = map_data.strip().splitlines()
    
    grid_lines = []
    schedule_lines = []
    
    separator_found = False
    for line in all_lines:
        if line == '---':
            separator_found = True
            continue
        if separator_found:
            schedule_lines.append(line)
        else:
            grid_lines.append(line)

    for r, line in enumerate(grid_lines):
        row = []
        for c, char in enumerate(line.strip()):
            if char == 'S':
                start_pos = (r, c)
                row.append('1')
            elif char == 'G':
                goal_pos = (r, c)
                row.append('1')
            elif char == '.':
                row.append('1')
            else:
                row.append(char)
        grid.append(row)

    for line in schedule_lines:
        clean_line = line.strip()
        if not clean_line or clean_line.startswith('#'):
            continue
        
        parts = [p.strip() for p in clean_line.split(',')]
        type = parts[0]
        pos = (int(parts[1]), int(parts[2]))
        
        if type == 'D':
            for t in parts[3:]:
                time_step = int(t)
                if time_step not in moving_stuff:
                    moving_stuff[time_step] = []
                moving_stuff[time_step].append(pos)
        elif type == 'T':
            time_step = int(parts[3])
            multiplier = float(parts[4])
            if time_step not in traffic:
                traffic[time_step] = {}
            traffic[time_step][pos] = multiplier
            
    return grid, moving_stuff, traffic, start_pos, goal_pos

def show_grid(grid, path, start, goal):
    if not path:
        return
    
    grid_copy = [list(row) for row in grid]
    for r, c in path:
        if (r, c) != start and (r, c) != goal:
            grid_copy[r][c] = '*'
    grid_copy[start[0]][start[1]] = 'S'
    grid_copy[goal[0]][goal[1]] = 'G'
    
    print("  path map:")
    for row in grid_copy:
        print(f"    {' '.join(row)}")
    print()

def get_neighbors(pos):
    r, c = pos
    return [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]

def bfs(grid, start, goal):
    queue = deque([(start, [start])])
    visited = {start}
    nodes = 0
    while queue:
        nodes += 1
        pos, path = queue.popleft()
        if pos == goal:
            return path, len(path) - 1, nodes
        for neighbor in get_neighbors(pos):
            r, c = neighbor
            if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 'X' and neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))
    return None, 0, nodes

def ucs(grid, start, goal, max_fuel, traffic):
    frontier = [(0, start, [start])]
    visited = set()
    nodes = 0
    while frontier:
        frontier.sort() # sort to find the cheapest
        cost, pos, path = frontier.pop(0)
        nodes += 1
        if pos in visited:
            continue
        visited.add(pos)
        if pos == goal:
            return path, cost, nodes
        time = len(path) - 1
        for neighbor in get_neighbors(pos):
            r, c = neighbor
            if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 'X' and neighbor not in visited:
                terrain_cost = int(grid[r][c])
                traffic_mult = 1.0
                if time + 1 in traffic and neighbor in traffic[time+1]:
                    traffic_mult = traffic[time+1][neighbor]
                
                new_cost = cost + (terrain_cost * traffic_mult)
                if new_cost <= max_fuel:
                    frontier.append((new_cost, neighbor, path + [neighbor]))
    return None, 0, nodes

def astar(grid, start, goal, max_fuel, moving_stuff, traffic, time_offset=0):
    frontier = [(0 + abs(start[0]-goal[0]) + abs(start[1]-goal[1]), 0, start, [start])]
    visited = set()
    nodes = 0
    while frontier:
        frontier.sort() # sort to find the cheapest
        f, g, pos, path = frontier.pop(0)
        nodes += 1
        if pos in visited:
            continue
        visited.add(pos)
        if pos == goal:
            return path, g, nodes
        
        time = len(path) - 1 + time_offset
        for neighbor in get_neighbors(pos):
            r,c = neighbor
            # check everything inside the loop
            is_valid = 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] != 'X'
            is_safe = True
            if time + 1 in moving_stuff and neighbor in moving_stuff[time+1]:
                is_safe = False
            
            if is_valid and is_safe and neighbor not in visited:
                terrain_cost = int(grid[r][c])
                traffic_mult = 1.0
                if time + 1 in traffic and neighbor in traffic[time+1]:
                    traffic_mult = traffic[time+1][neighbor]
                
                new_g = g + (terrain_cost * traffic_mult)
                if new_g <= max_fuel:
                    h = abs(neighbor[0]-goal[0]) + abs(neighbor[1]-goal[1])
                    new_f = new_g + h
                    frontier.append((new_f, new_g, neighbor, path + [neighbor]))
    return None, 0, nodes

def sim_anneal(grid, start, goal, max_fuel, moving_stuff, traffic):
    initial_path, _, _ = astar(grid, start, goal, max_fuel, moving_stuff, traffic)
    if not initial_path: return None, 0, 0
    
    current_path = initial_path
    
    def get_cost(p):
        c = 0
        for i in range(len(p) - 1):
            curr, next_node = p[i], p[i+1]
            if next_node not in get_neighbors(curr) : return 9999
            r,col = next_node
            if not (0 <= r < len(grid) and 0 <= col < len(grid[0]) and grid[r][col] != 'X'): return 9999
            if i + 1 in moving_stuff and next_node in moving_stuff[i+1]: return 9999
            
            t_mult = 1.0
            if i + 1 in traffic and next_node in traffic[i+1]: t_mult = traffic[i+1][next_node]
            c += int(grid[r][col]) * t_mult
        return c if c <= max_fuel else 9999
        
    current_cost = get_cost(current_path)
    temp = 100.0
    nodes = 1

    for i in range(1000):
        if temp < 0.1: break
        nodes += 1
        new_path = list(current_path)
        if len(new_path) > 3:
            idx1 = random.randint(1, len(new_path) - 2)
            idx2 = random.randint(1, len(new_path) - 2)
            new_path[idx1], new_path[idx2] = new_path[idx2], new_path[idx1]
        
        new_cost = get_cost(new_path)
        if new_cost < current_cost or math.exp((current_cost - new_cost) / temp) > random.random():
            current_path, current_cost = new_path, new_cost
        temp *= 0.995
    return current_path, current_cost, nodes

def run_simulation(grid, start, goal, max_fuel):
    print("\n--- replanning simulation ---")
    pos = start
    total_path = [start]
    total_cost = 0
    total_nodes = 0
    time_step = 0
    sim_grid = [row[:] for row in grid]
    while pos != goal:
        if time_step == 3:
            block_pos = (2, 2)
            r,c = block_pos
            if 0 <= r < len(grid) and 0 <= c < len(grid[0]) and sim_grid[r][c] != 'X':
                print(f"\nlog: time {time_step:02d} | new wall at {block_pos}\n")
                sim_grid[r][c] = 'X'
        
        path, _, nodes = astar(sim_grid, pos, goal, max_fuel - total_cost, {}, {})
        total_nodes += nodes
        if not path:
            print(f"log: time {time_step:02d} | stuck at {pos}."); return

        next_pos = path[1]
        move_cost = int(sim_grid[next_pos[0]][next_pos[1]])
        if (total_cost + move_cost) > max_fuel:
            print(f"log: time {time_step:02d} | out of fuel."); return
            
        total_cost += move_cost
        pos = next_pos
        total_path.append(pos)
        time_step += 1
        print(f"log: time {time_step:02d} | moved to {pos}. cost: {total_cost:.1f}")

    print(f"\ngoal reached.")
    print(f"result: cost={total_cost}, nodes={total_nodes}")
    return total_path

def print_analysis(results):
    print("analysis:")
    successful_runs = {name: data for name, data in results.items() if data['path'] is not None}
    if not successful_runs:
        print("  no solution found."); return

    cost_algos = {n: d for n, d in successful_runs.items() if n != "bfs"}
    if cost_algos:
        min_cost = min(data['cost'] for data in cost_algos.values())
        best_algos = [name for name, data in cost_algos.items() if abs(data['cost'] - min_cost) < 0.01]
        print(f"  best cost: {min_cost:.1f} (by: {', '.join(best_algos)})")

    best_nodes_algo = min(successful_runs, key=lambda name: successful_runs[name]['nodes'])
    min_nodes = successful_runs[best_nodes_algo]['nodes']
    print(f"  least nodes: {best_nodes_algo} ({min_nodes} nodes)")

    fastest_algo = min(successful_runs, key=lambda name: successful_runs[name]['time'])
    min_time = successful_runs[fastest_algo]['time']
    print(f"  fastest time: {fastest_algo} ({min_time:.7f}s)")

if __name__ == "__main__":
    show_maps = True
    fuel = 300

    map1 = "S.X.\n.X.G\n....\n.X.X"
    map2 = "S.........\n.11111111.\n.1XXXXX1..\n.1X...X1.G\n.1XXXXX1..\n.11111111.\n.........."
    map3 = "S11111111111111.\n.44444444444441.\n.4............1.\n.4.111111111111.\n.4.1..........11\n.4.1.G.111111111\n.4.1..........11\n.4.111111111111.\n.4............1.\n.44444444444441.\n.11111111111111."
    map4 = "S.111....\n.X.X.222.\n...X.....\n.3.X.XXX.\n.3.X...XG\n.3.....X.\n.333333X.\n---\nD,2,4,3,8,13\nD,2,5,4,7,12\nT,6,1,5,4.0"
    
    maps = {"small": map1, "medium": map2, "large": map3, "dynamic": map4}
    
    print("vityarthi project: 24BAS10066")

    for name, map_data in maps.items():
        print(f"\nmap: {name}")
        grid, moving_stuff, traffic, start, goal = load_map(map_data)
        
        results = {}
        
        # run astar
        start_time = time.time()
        path, cost, nodes = astar(grid, start, goal, fuel, moving_stuff, traffic)
        results['astar'] = {'path': path, 'cost': cost, 'nodes': nodes, 'time': time.time()-start_time}
        print(f"astar: cost={cost:.1f}, nodes={nodes}, time={time.time()-start_time:.7f}s")
        if show_maps: show_grid(grid, path, start, goal)

        # run ucs
        start_time = time.time()
        path, cost, nodes = ucs(grid, start, goal, fuel, traffic)
        results['ucs'] = {'path': path, 'cost': cost, 'nodes': nodes, 'time': time.time()-start_time}
        print(f"ucs: cost={cost:.1f}, nodes={nodes}, time={time.time()-start_time:.7f}s")
        if show_maps: show_grid(grid, path, start, goal)
        
        # run sa
        start_time = time.time()
        path, cost, nodes = sim_anneal(grid, start, goal, fuel, moving_stuff, traffic)
        results['sim_anneal'] = {'path': path, 'cost': cost, 'nodes': nodes, 'time': time.time()-start_time}
        print(f"sim_anneal: cost={cost:.1f}, nodes={nodes}, time={time.time()-start_time:.7f}s")
        if show_maps: show_grid(grid, path, start, goal)

        # run bfs
        start_time = time.time()
        path, cost, nodes = bfs(grid, start, goal)
        results['bfs'] = {'path': path, 'cost': cost, 'nodes': nodes, 'time': time.time()-start_time}
        print(f"bfs: steps={cost}, nodes={nodes}, time={time.time()-start_time:.7f}s")
        if show_maps: show_grid(grid, path, start, goal)

        print_analysis(results)

    grid, _, _, start, goal = load_map(map2)
    final_path = run_simulation(grid, start, goal, fuel)
    if show_maps:
        show_grid(grid, final_path, start, goal)
