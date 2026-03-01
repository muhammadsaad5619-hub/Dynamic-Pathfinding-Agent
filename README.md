# Dynamic Pathfinding Agent

A grid-based pathfinding visualizer built with Python and Tkinter. Supports A* and Greedy Best-First Search with live animation, dynamic obstacles, and real-time metrics.

---

## Requirements

- Python 3.8 or higher
- No external libraries needed — uses only the standard library

---

## How to Run

```bash
python pathfinding_agent.py
```

---

## Controls

All controls are in the rows above and below the grid. Nothing is inside the canvas.

**Top row**
- Set Rows and Cols, then press **Apply Grid**
- Adjust the Density slider and press **Random Map** to generate a maze
- Press **Clear** to remove all walls

**Second row**
- Pick an algorithm: **A\*** or **GBFS**
- Pick a heuristic: **Manhattan**, **Euclidean**, or **Chebyshev**
- Toggle **Diagonal** movement on or off
- Toggle **Dynamic Obstacles** on or off

**Edit mode row**
- **Draw Walls** — click or drag on the grid to place/remove walls
- **Set Start** — click a cell to move the start position
- **Set Goal** — click a cell to move the goal position

**Bottom row**
- **RUN** — starts the search and animates the agent
- **STOP** — pauses the animation
- **RESET** — clears the path and visited colors, keeps the walls

---

## Grid Symbols (in the canvas)

| Symbol | Meaning |
|--------|---------|
| S | Start cell |
| G | Goal cell |
| Dark gray | Wall |
| Blue | Visited / explored node |
| Green | Final path |
| Purple | Agent current position |

---

## Algorithms

**A\*** — uses `f(n) = g(n) + h(n)`. Guarantees the shortest path. Slower than GBFS but always optimal.

**GBFS** — uses `f(n) = h(n)` only. Faster but does not guarantee the shortest path.

---

## Heuristics

| Heuristic | Formula | Best for |
|-----------|---------|---------|
| Manhattan | `\|Δrow\| + \|Δcol\|` | 4-directional movement |
| Euclidean | `sqrt(Δrow² + Δcol²)` | Diagonal movement |
| Chebyshev | `max(\|Δrow\|, \|Δcol\|)` | 8-directional movement |

---

## Dynamic Obstacles

When enabled, each animation step has a 5% chance of spawning a new wall on the grid. If the new wall blocks the agent's remaining path, the algorithm re-plans automatically from the agent's current position. If the wall does not block the path, nothing changes.

---

## Metrics (shown at the bottom)

- **Nodes** — total number of cells expanded during the search
- **Cost** — length of the final path (steps or diagonal distance)
- **Time** — how long the search computation took in milliseconds
- **Status** — current state (Ready / Running / Done / Re-planned / Blocked)

---

## File Structure

```
pathfinding_agent.py   # entire application, single file
README.md              # this file
```

---

## Quick Example

1. Run `python pathfinding_agent.py`
2. Press **Random Map** (density ~0.25 works well)
3. Select **A\*** and **Manhattan**
4. Press **RUN**
5. Watch the agent navigate to the goal
6. Press **RESET**, switch to **GBFS**, press **RUN** again to compare
