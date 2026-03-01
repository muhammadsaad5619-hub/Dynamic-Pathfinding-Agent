"""
Dynamic Pathfinding Agent — Simple & Clean
All controls are OUTSIDE the grid. Grid is just the visual canvas.
Run: python pathfinding_agent.py
"""

import tkinter as tk
import heapq, random, time, math

C = {
    "bg":      "#0f0f0f", "panel":  "#1a1a1a", "border": "#2a2a2a",
    "text":    "#eeeeee", "dim":    "#888888", "accent": "#00d4ff",
    "green":   "#00ff88", "red":    "#ff4466", "yellow": "#ffcc00",
    "blue":    "#4488ff", "purple": "#cc88ff", "wall":   "#333333",
    "empty":   "#1e1e1e", "btn":    "#222222", "btn_h":  "#2e2e2e",
}

CELL = 28

def manhattan(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
def euclidean(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])
def chebyshev(a,b): return max(abs(a[0]-b[0]),abs(a[1]-b[1]))
HEURISTICS = {"Manhattan":manhattan,"Euclidean":euclidean,"Chebyshev":chebyshev}

def get_nb(node,rows,cols,walls,diag):
    r,c=node
    dirs=[(-1,0),(1,0),(0,-1),(0,1)]
    if diag: dirs+=[(-1,-1),(-1,1),(1,-1),(1,1)]
    for dr,dc in dirs:
        nr,nc=r+dr,c+dc
        if 0<=nr<rows and 0<=nc<cols and (nr,nc) not in walls:
            yield (nr,nc)

def reconstruct(came,goal):
    p,n=[],goal
    while n is not None: p.append(n); n=came[n]
    return p[::-1]

def gbfs(start,goal,rows,cols,walls,hfn,diag):
    heap=[(hfn(start,goal),start)]; came={start:None}; vis=[]
    while heap:
        _,cur=heapq.heappop(heap); vis.append(cur)
        if cur==goal: return reconstruct(came,goal),vis
        for nb in get_nb(cur,rows,cols,walls,diag):
            if nb not in came:
                came[nb]=cur; heapq.heappush(heap,(hfn(nb,goal),nb))
    return None,vis

def astar(start,goal,rows,cols,walls,hfn,diag):
    g={start:0}; heap=[(hfn(start,goal),0,start)]; came={start:None}; vis=[]; closed=set()
    while heap:
        _,gc,cur=heapq.heappop(heap)
        if cur in closed: continue
        closed.add(cur); vis.append(cur)
        if cur==goal: return reconstruct(came,goal),vis
        for nb in get_nb(cur,rows,cols,walls,diag):
            cost=math.hypot(nb[0]-cur[0],nb[1]-cur[1]) if diag else 1
            ng=gc+cost
            if ng<g.get(nb,1e18):
                g[nb]=ng; came[nb]=cur
                heapq.heappush(heap,(ng+hfn(nb,goal),ng,nb))
    return None,vis


class App:
    def __init__(self, root):
        self.root=root; root.title("Pathfinding Agent"); root.configure(bg=C["bg"]); root.resizable(False,False)
        self.ROWS=15; self.COLS=20
        self.walls=set(); self.start=(0,0); self.goal=(14,19)
        self.path=[]; self.visited=[]; self.agent_pos=None; self.agent_step=0
        self.anim_id=None; self.running=False

        self.v_rows=tk.IntVar(value=15); self.v_cols=tk.IntVar(value=20)
        self.v_density=tk.DoubleVar(value=0.25); self.v_alg=tk.StringVar(value="A*")
        self.v_heur=tk.StringVar(value="Manhattan"); self.v_diag=tk.BooleanVar(value=False)
        self.v_dyn=tk.BooleanVar(value=False); self.v_mode=tk.StringVar(value="wall")
        self.v_nodes=tk.StringVar(value="-"); self.v_cost=tk.StringVar(value="-")
        self.v_time=tk.StringVar(value="-"); self.v_status=tk.StringVar(value="Ready")
        self._ui(); self._draw()

    def _lbl(self,p,t,padx=8):
        tk.Label(p,text=t,bg=p["bg"],fg=C["dim"],font=("Courier",9)).pack(side=tk.LEFT,padx=(padx,2))

    def _btn(self,p,t,cmd,bg=None,fg=None):
        bg=bg or C["btn"]; fg=fg or C["text"]
        tk.Button(p,text=t,command=cmd,bg=bg,fg=fg,relief="flat",
                  font=("Courier",9,"bold"),padx=10,pady=5,
                  activebackground=C["btn_h"],activeforeground=fg,cursor="hand2").pack(side=tk.LEFT,padx=3)

    def _sep(self,p):
        tk.Frame(p,bg=C["border"],width=1).pack(side=tk.LEFT,fill=tk.Y,padx=8)

    def _ui(self):
        # ROW 1: Title + Grid size + Map
        r1=tk.Frame(self.root,bg=C["panel"],pady=8); r1.pack(fill=tk.X)
        tk.Label(r1,text="⬛ PATHFINDING AGENT",bg=C["panel"],fg=C["accent"],
                 font=("Courier",12,"bold")).pack(side=tk.LEFT,padx=14)
        self._sep(r1)
        self._lbl(r1,"Rows")
        tk.Spinbox(r1,from_=5,to=30,textvariable=self.v_rows,width=4,
                   bg=C["btn"],fg=C["text"],insertbackground=C["text"],
                   buttonbackground=C["btn"],relief="flat",font=("Courier",10)).pack(side=tk.LEFT,padx=2)
        self._lbl(r1,"Cols")
        tk.Spinbox(r1,from_=5,to=40,textvariable=self.v_cols,width=4,
                   bg=C["btn"],fg=C["text"],insertbackground=C["text"],
                   buttonbackground=C["btn"],relief="flat",font=("Courier",10)).pack(side=tk.LEFT,padx=2)
        self._btn(r1,"Apply Grid",self._apply_grid)
        self._sep(r1)
        self._lbl(r1,"Density")
        tk.Scale(r1,variable=self.v_density,from_=0.05,to=0.55,resolution=0.05,
                 orient=tk.HORIZONTAL,length=90,bg=C["panel"],fg=C["text"],
                 troughcolor=C["btn"],highlightthickness=0,showvalue=True,
                 font=("Courier",8)).pack(side=tk.LEFT)
        self._btn(r1,"Random Map",self._gen_map)
        self._btn(r1,"Clear",self._clear)

        # ROW 2: Algorithm + Heuristic + Options
        r2=tk.Frame(self.root,bg=C["panel"],pady=5); r2.pack(fill=tk.X)
        self._lbl(r2,"Algorithm",14)
        for a in ["A*","GBFS"]:
            tk.Radiobutton(r2,text=a,variable=self.v_alg,value=a,
                           bg=C["panel"],fg=C["text"],selectcolor=C["btn_h"],
                           activebackground=C["panel"],activeforeground=C["accent"],
                           font=("Courier",10)).pack(side=tk.LEFT)
        self._sep(r2)
        self._lbl(r2,"Heuristic")
        for h in ["Manhattan","Euclidean","Chebyshev"]:
            tk.Radiobutton(r2,text=h,variable=self.v_heur,value=h,
                           bg=C["panel"],fg=C["text"],selectcolor=C["btn_h"],
                           activebackground=C["panel"],activeforeground=C["accent"],
                           font=("Courier",10)).pack(side=tk.LEFT)
        self._sep(r2)
        tk.Checkbutton(r2,text="Diagonal",variable=self.v_diag,
                       bg=C["panel"],fg=C["text"],selectcolor=C["btn_h"],
                       activebackground=C["panel"],font=("Courier",10)).pack(side=tk.LEFT)
        tk.Checkbutton(r2,text="Dynamic Obstacles",variable=self.v_dyn,
                       bg=C["panel"],fg=C["text"],selectcolor=C["btn_h"],
                       activebackground=C["panel"],font=("Courier",10)).pack(side=tk.LEFT,padx=6)

        # ROW 3: Edit mode
        r3=tk.Frame(self.root,bg=C["bg"],pady=5); r3.pack(fill=tk.X)
        self._lbl(r3,"Edit Mode:",14)
        for lbl,val in [("Draw Walls","wall"),("Set Start","start"),("Set Goal","goal")]:
            tk.Radiobutton(r3,text=lbl,variable=self.v_mode,value=val,
                           bg=C["bg"],fg=C["text"],selectcolor=C["btn"],
                           activebackground=C["bg"],activeforeground=C["accent"],
                           font=("Courier",10)).pack(side=tk.LEFT,padx=4)

        # CANVAS
        cf=tk.Frame(self.root,bg=C["bg"],padx=14,pady=4); cf.pack()
        self.canvas=tk.Canvas(cf,width=self.COLS*CELL,height=self.ROWS*CELL,
                               bg=C["bg"],highlightthickness=2,
                               highlightbackground=C["accent"])
        self.canvas.pack()
        self.canvas.bind("<Button-1>",self._click)
        self.canvas.bind("<B1-Motion>",self._drag)

        # ROW 4: Run controls + Metrics
        r4=tk.Frame(self.root,bg=C["panel"],pady=8); r4.pack(fill=tk.X)
        self._btn(r4,"▶ RUN",self._run,bg=C["green"],fg="#0f0f0f")
        self._btn(r4,"■ STOP",self._stop_btn,bg=C["red"],fg="#0f0f0f")
        self._btn(r4,"↺ RESET",self._reset)
        self._sep(r4)
        for lbl,var,col in [("Nodes:",self.v_nodes,C["blue"]),("Cost:",self.v_cost,C["yellow"]),
                             ("Time:",self.v_time,C["purple"]),("Status:",self.v_status,C["accent"])]:
            tk.Label(r4,text=lbl,bg=C["panel"],fg=C["dim"],font=("Courier",9)).pack(side=tk.LEFT,padx=(8,2))
            tk.Label(r4,textvariable=var,bg=C["panel"],fg=col,font=("Courier",9,"bold"),
                     width=11,anchor="w").pack(side=tk.LEFT)

        # ROW 5: Legend
        r5=tk.Frame(self.root,bg=C["bg"],pady=4); r5.pack(fill=tk.X,padx=14,pady=(0,8))
        for color,label in [(C["green"],"Start/Path"),(C["red"],"Goal"),(C["wall"],"Wall"),
                             (C["blue"],"Visited"),(C["yellow"],"Frontier"),(C["purple"],"Agent")]:
            dot=tk.Canvas(r5,width=12,height=12,bg=C["bg"],highlightthickness=0)
            dot.pack(side=tk.LEFT,padx=(4,2))
            dot.create_oval(1,1,11,11,fill=color,outline="")
            tk.Label(r5,text=label,bg=C["bg"],fg=C["dim"],font=("Courier",8)).pack(side=tk.LEFT,padx=(0,10))

    def _draw(self):
        self.canvas.delete("all")
        vis_set  = set(self.visited)
        path_set = set(self.path)
        for r in range(self.ROWS):
            for c in range(self.COLS):
                node=(r,c)
                x0,y0=c*CELL+1,r*CELL+1
                x1,y1=x0+CELL-2,y0+CELL-2
                if node==self.start:       color=C["green"]
                elif node==self.goal:      color=C["red"]
                elif node in self.walls:   color=C["wall"]
                elif node==self.agent_pos: color=C["purple"]
                elif node in path_set:     color=C["green"]
                elif node in vis_set:      color=C["blue"]
                else:                      color=C["empty"]
                self.canvas.create_rectangle(x0,y0,x1,y1,fill=color,outline=C["bg"],width=1)
        # Labels
        for node,lbl in [(self.start,"S"),(self.goal,"G")]:
            r,c=node
            self.canvas.create_text(c*CELL+CELL//2,r*CELL+CELL//2,text=lbl,
                                    fill="#0f0f0f",font=("Courier",9,"bold"))

    def _cell(self,event):
        r,c=event.y//CELL,event.x//CELL
        if 0<=r<self.ROWS and 0<=c<self.COLS: return (r,c)
        return None

    def _click(self,event):
        n=self._cell(event)
        if n: self._edit(n)

    def _drag(self,event):
        n=self._cell(event)
        if n and self.v_mode.get()=="wall": self._edit(n)

    def _edit(self,node):
        m=self.v_mode.get()
        if m=="start": self.start=node
        elif m=="goal": self.goal=node
        elif m=="wall":
            if node in (self.start,self.goal): return
            if node in self.walls: self.walls.discard(node)
            else: self.walls.add(node)
        self._draw()

    def _apply_grid(self):
        self._stop(); self.ROWS=self.v_rows.get(); self.COLS=self.v_cols.get()
        self.walls.clear(); self.path=[]; self.visited=[]; self.agent_pos=None
        self.start=(0,0); self.goal=(self.ROWS-1,self.COLS-1)
        self.canvas.configure(width=self.COLS*CELL,height=self.ROWS*CELL)
        self._draw(); self.root.geometry("")

    def _gen_map(self):
        self._stop(); self.walls.clear(); self.path=[]; self.visited=[]; self.agent_pos=None
        d=self.v_density.get()
        for r in range(self.ROWS):
            for c in range(self.COLS):
                n=(r,c)
                if n not in (self.start,self.goal) and random.random()<d:
                    self.walls.add(n)
        self._draw(); self.v_status.set("Map ready")

    def _clear(self):
        self._stop(); self.walls.clear(); self.path=[]; self.visited=[]; self.agent_pos=None
        self._draw(); self.v_status.set("Cleared")

    def _reset(self):
        self._stop(); self.path=[]; self.visited=[]; self.agent_pos=None
        self._draw(); self.v_nodes.set("-"); self.v_cost.set("-"); self.v_time.set("-"); self.v_status.set("Ready")

    def _run(self):
        self._stop(); self.path=[]; self.visited=[]; self.agent_pos=self.start; self.agent_step=0
        hfn=HEURISTICS[self.v_heur.get()]; diag=self.v_diag.get(); alg=self.v_alg.get()
        t0=time.perf_counter()
        if alg=="A*": path,vis=astar(self.start,self.goal,self.ROWS,self.COLS,self.walls,hfn,diag)
        else:         path,vis=gbfs(self.start,self.goal,self.ROWS,self.COLS,self.walls,hfn,diag)
        elapsed=(time.perf_counter()-t0)*1000
        if path is None: self.v_status.set("No path!"); return
        self.path=path; self.visited=vis
        cost=sum(math.hypot(path[i+1][0]-path[i][0],path[i+1][1]-path[i][1])
                 for i in range(len(path)-1)) if diag else len(path)-1
        self.v_nodes.set(str(len(vis))); self.v_cost.set(f"{cost:.1f}"); self.v_time.set(f"{elapsed:.1f}ms")
        self.v_status.set("Running..."); self.running=True; self._animate()

    def _animate(self):
        if not self.running or self.agent_step>=len(self.path):
            self.running=False; self.v_status.set("Done!" if self.agent_pos==self.goal else "Stopped")
            self._draw(); return
        self.agent_pos=self.path[self.agent_step]; self.agent_step+=1
        if self.v_dyn.get(): self._maybe_spawn()
        self._draw()
        self.anim_id=self.root.after(30,self._animate)

    def _maybe_spawn(self):
        if random.random()<0.05:
            node=(random.randint(0,self.ROWS-1),random.randint(0,self.COLS-1))
            if node in (self.start,self.goal,self.agent_pos): return
            if node not in self.walls:
                self.walls.add(node)
                if node in self.path[self.agent_step:]: self._replan()

    def _replan(self):
        hfn=HEURISTICS[self.v_heur.get()]; diag=self.v_diag.get(); alg=self.v_alg.get()
        if alg=="A*": path,vis=astar(self.agent_pos,self.goal,self.ROWS,self.COLS,self.walls,hfn,diag)
        else:         path,vis=gbfs(self.agent_pos,self.goal,self.ROWS,self.COLS,self.walls,hfn,diag)
        if path: self.path=path; self.agent_step=1; self.v_status.set("Re-planned!")
        else:    self._stop(); self.v_status.set("Blocked!")

    def _stop(self):
        self.running=False
        if self.anim_id: self.root.after_cancel(self.anim_id); self.anim_id=None

    def _stop_btn(self):
        self._stop(); self.v_status.set("Stopped")


if __name__=="__main__":
    root=tk.Tk()
    App(root)
    root.mainloop()
