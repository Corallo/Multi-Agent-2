using MyGraph;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using Debug = UnityEngine.Debug;

public class Planner5New {

 
    public List<HashSet<int>> sets = new List<HashSet<int>>();
    public TerrainManager terrain_manager;
    //private Graph DroneGraph;
    public GameObject[] friends;
    public GameObject[] enemies;
    private CarController m_Car;
    //public LayerMask mask;
    private float Margin = 0;
    public int lastEnemyNumber;
    public int st;
    public Graph DroneGraph;
    public bool firstTime;
    public Stopwatch DfsStopwatch;
    public void setEnemyNumber(TerrainManager _terrain_manager) {
        enemies = GameObject.FindGameObjectsWithTag("Enemy");
        lastEnemyNumber = enemies.Length;
        st = -1;
        firstTime = true;
        terrain_manager = _terrain_manager.GetComponent<TerrainManager>();

        var watch = System.Diagnostics.Stopwatch.StartNew();

        DroneGraph = new Graph();
        DroneGraph = generateGraph(DroneGraph);
        watch.Stop();

        var elapsedMs = watch.Elapsed;
        Debug.Log("Time to create Graph:" + elapsedMs.ToString());
    }
    public void checkReplan(TerrainManager _terrain_manager) {
        enemies = GameObject.FindGameObjectsWithTag("Enemy");
        if (lastEnemyNumber != enemies.Length) {
            lastEnemyNumber = enemies.Length;
            ComputePath(_terrain_manager);
        }
    }

    public void ComputePath(TerrainManager _terrain_manager)
    {
        st++;
        terrain_manager = _terrain_manager.GetComponent<TerrainManager>();

        var watch = System.Diagnostics.Stopwatch.StartNew();
        watch.Stop();
        var elapsedMs = watch.Elapsed;
        friends = GameObject.FindGameObjectsWithTag("Player");
        enemies = GameObject.FindGameObjectsWithTag("Enemy");
        m_Car = friends[0].GetComponent<CarController>();


        LayerMask mask = LayerMask.GetMask("CubeWalls");

        for (int i = 0; i < DroneGraph.getSize(); i++)
        {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Collider c = cube.GetComponent<Collider>();
            c.enabled = false;
            Vector3 position = DroneGraph.getNode(i).getPosition();


            cube.transform.position = new Vector3(position.x, 1, position.z);
            cube.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            for (int k = 0; k < DroneGraph.getAdjList(i).Count; k++)
            {
                //  Debug.DrawLine(DroneGraph.getNode(i).getPosition(), DroneGraph.getNode(DroneGraph.getAdjList(i)[k]).getPosition(), Color.yellow, 1f);
            }

        }

        /* GENERATE INITIAL POSITION NODES FOR CAR (FIND THE CLOSEST NODE TO THE CAR, THERE IS ARLEADY A FUNCTION FOR THAT)*/
        List<List<int>> car_targets = new List<List<int>>();
        /* GENERATE PERMUTATION OF INTERESTING NODES */
        List<int> best_path = new List<int>();

        watch = System.Diagnostics.Stopwatch.StartNew();
        List<int> path = new List<int>();

        List<int> point_of_interest = new List<int>();


        for (int i = 0; i < DroneGraph.getSize(); i++)
        {

            point_of_interest.Add(i);
        }


        watch = System.Diagnostics.Stopwatch.StartNew();
        Dictionary<Tuple<int, int>, float> CostMatrix = new Dictionary<Tuple<int, int>, float>();
        Dictionary<Tuple<int, int>, List<int>> PathMatrix = new Dictionary<Tuple<int, int>, List<int>>();
        int startNode = DroneGraph.FindClosestNode(friends[0].transform.position, DroneGraph).getId();
        List<List<int>> possiblePaths = new List<List<int>>();
        List<int> tmp_path = new List<int>();
        computeDanger(DroneGraph);

        smoothDanger(DroneGraph);


        for (int i = 0; i < DroneGraph.getSize(); i++)
        {
            DroneGraph.setColorOfNode(i, 0);
        }

        bool found = false;
        DfsStopwatch = System.Diagnostics.Stopwatch.StartNew();
        DfS(DroneGraph, startNode, possiblePaths, tmp_path, found);
        best_path = possiblePaths[0];
        foreach (var currentPath in possiblePaths)
        {
            if (DroneGraph.getNode(currentPath[currentPath.Count - 1]).getDanger() +
                DroneGraph.getNode(currentPath[currentPath.Count - 2]).getDanger() <
                DroneGraph.getNode(best_path[best_path.Count - 1]).getDanger() +
                DroneGraph.getNode(best_path[best_path.Count - 2]).getDanger())
            {

                best_path = currentPath;
            }
            else if (DroneGraph.getNode(currentPath[currentPath.Count - 1]).getDanger() +
                     DroneGraph.getNode(currentPath[currentPath.Count - 2]).getDanger() ==
                     DroneGraph.getNode(best_path[best_path.Count - 1]).getDanger() +
                     DroneGraph.getNode(best_path[best_path.Count - 2]).getDanger())
            {
                if (currentPath.Count < best_path.Count)
                {
                    best_path = currentPath;
                }
            }
        }

        //Debug.Log("Danger of this patH:" + BestCost.ToString());
        watch.Stop();
        elapsedMs = watch.Elapsed;
        Debug.Log("Time to precompute A Star" + elapsedMs.ToString());

        Color[] colors = {Color.cyan, Color.black, Color.blue, Color.magenta, Color.red, Color.grey};

        Vector3 old_wp = DroneGraph.getNode(best_path[0]).getPosition();
        int coloridx = UnityEngine.Random.Range(0, colors.Length - 1);

        foreach (var wp in best_path)
        {
            //Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
            Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), colors[coloridx], best_path.Count*10f);
            old_wp = DroneGraph.getNode(wp).getPosition();
        }

        /*
        foreach (var p in best_path) {
            Debug.Log(p.ToString() + ", \0");
        }
        */
        var my_path = pathHelp(DroneGraph, best_path);

        if (firstTime)
        {
            friends[0].GetComponent<CarAI5>().skipper = 50;
            if (friends.Length > 1)
            {
                friends[1].GetComponent<CarAI5>().skipper = 500;
            }

            if (friends.Length > 2)
            {

                friends[2].GetComponent<CarAI5>().skipper = 800;
            }

            firstTime = false;
        }
        else

        {
            friends[0].GetComponent<CarAI5>().skipper = 20;
            if (friends.Length > 1) {
                friends[1].GetComponent<CarAI5>().skipper = 40;
                friends[1].GetComponent<CarAI5>().wait = 100;
            }

            if (friends.Length > 2) {

                friends[2].GetComponent<CarAI5>().skipper = 60;
                friends[2].GetComponent<CarAI5>().wait = 200;
            }
        }

        foreach (var car in friends) {

            //car.GetComponent<CarAI3>().trajectory = car.GetComponent<CarAI3>().myController.makeTrajectory(myRealPath, maxVel / 2.236f, maxAcc); ;
            //car.GetComponent<CarAI3>().lastPointInPath = best_path[j][0];
            car.GetComponent<CarAI5>().my_path = my_path;
            car.GetComponent<CarAI5>().my_path_length = best_path.Count;
            car.GetComponent<CarAI5>().my_mask = mask;
            car.GetComponent<CarAI5>().run = 0;
            int lpip = -1;
            for (int w = -1; w < my_path.Count - 1; w++) {
                if (Vector3.Distance(car.transform.position, my_path[lpip + 1].getPosition()) >
                    Vector3.Distance(car.transform.position, my_path[w + 1].getPosition())) {
                    lpip = w;
                }
            }
            car.GetComponent<CarAI5>().lastPointInPath = lpip;
            //car.GetComponent<CarAI5>().skipper = 1000;

        }
        float[] offsets = { -7, 0, 7 };

        if (friends.Length == 3) {
            var StrongestCar = friends[0];
            var MiddleCar = friends[1];
            var LowCar = friends[2];
            /*
            if (friends[0].GetComponent<CarAI5>().health > friends[1].GetComponent<CarAI5>().health &&
                friends[0].GetComponent<CarAI5>().health > friends[2].GetComponent<CarAI5>().health)
            {
                StrongestCar = friends[0];
            }
            */
            if (StrongestCar.GetComponent<CarAI5>().health > LowCar.GetComponent<CarAI5>().health)
            {
                var tmp = StrongestCar;
                StrongestCar = LowCar;
                LowCar = tmp;
            }


            if (StrongestCar.GetComponent<CarAI5>().health > MiddleCar.GetComponent<CarAI5>().health)
            {
                var tmp = StrongestCar;
                StrongestCar = MiddleCar;
                MiddleCar = tmp;
            }


            if (MiddleCar.GetComponent<CarAI5>().health > LowCar.GetComponent<CarAI5>().health)
            {
                var tmp = MiddleCar;
                MiddleCar = LowCar;
                LowCar = tmp;
            }
            /*
            foreach (var car in friends) {
                if (StrongestCar.GetComponent<CarAI5>().health <= car.GetComponent<CarAI5>().health) {
                    if (LowCar.GetComponent<CarAI5>().health > MiddleCar.GetComponent<CarAI5>().health)
                    {
                        LowCar = MiddleCar;
                    }
                    if(MiddleCar.GetComponent<CarAI5>().health > StrongestCar.GetComponent<CarAI5>().health)
                    { MiddleCar = StrongestCar;}
                    StrongestCar = car;
                } else if (MiddleCar.GetComponent<CarAI5>().health <= car.GetComponent<CarAI5>().health) {
                    if (LowCar.GetComponent<CarAI5>().health > MiddleCar.GetComponent<CarAI5>().health) {
                        LowCar = MiddleCar;
                    }
                    MiddleCar = car;
                } else {
                    LowCar = car;
                }
            }

   
            */
            StrongestCar.GetComponent<CarAI5>().offset = offsets[0];
            MiddleCar.GetComponent<CarAI5>().offset = offsets[1];
            LowCar.GetComponent<CarAI5>().offset = offsets[2];
        } else if (friends.Length == 2) {
            var StrongestCar = friends[0];
            var MiddleCar = friends[1];


            if (StrongestCar.GetComponent<CarAI5>().health > MiddleCar.GetComponent<CarAI5>().health) {
                var tmp = StrongestCar;
                StrongestCar = MiddleCar;
                MiddleCar = tmp;
            }
            /*
            foreach (var car in friends) {
                if (StrongestCar.GetComponent<CarAI5>().health < car.GetComponent<CarAI5>().health) {
                    MiddleCar = StrongestCar;
                    StrongestCar = car;
                } else {
                    MiddleCar = car;
                }

              
            }*/
            StrongestCar.GetComponent<CarAI5>().offset = offsets[0];
            MiddleCar.GetComponent<CarAI5>().offset = offsets[1];
        } else {
            var StrongestCar = friends[0];
            StrongestCar.GetComponent<CarAI5>().offset = 0;
        }
    
    /*
    if (friends.Length == 3) {
        var bestCar = friends[0];
        /*foreach (var car in friends)
        {
            if (bestCar.GetComponent<CarAI5>().health < car.GetComponent<CarAI5>().health)
            {
                bestCar = car;
            }
        }

        //bestCar.GetComponent<CarAI5>().offset = 0f;
        bestCar.GetComponent<CarAI5>().leaderOffset = -2f;

        int h = 0;
        foreach (var car in friends) {
            if (bestCar != car) {
                if (h == 0) {
                   // car.GetComponent<CarAI5>().offset = 2.5f;
                    car.GetComponent<CarAI5>().leaderOffset = 1f;
                    h++;

                } else if (h == 1) {
                    //car.GetComponent<CarAI5>().offset = -2.5f;
                    car.GetComponent<CarAI5>().leaderOffset = 1f;
                }
            }
        }

    } else if (friends.Length == 2) {
        int h = 0;
        foreach (var car in friends) {

            if (h == 0) {
                car.GetComponent<CarAI5>().offset = 2.5f;
                car.GetComponent<CarAI5>().leaderOffset = 0f;
                h++;

            } else if (h == 1) {
                car.GetComponent<CarAI5>().offset = -2.5f;
                car.GetComponent<CarAI5>().leaderOffset = 0f;
            }
        }
    } else {
        friends[0].GetComponent<CarAI5>().offset = 0;
        friends[0].GetComponent<CarAI5>().leaderOffset = 0;
    }
    */
    /* if (friends.Length == 3)
     {
         for (int i = st; i < st + friends.Length; i++)
         {
             if (i == st)
             {
                 friends[i % friends.Length].GetComponent<CarAI5>().offset = 0f;
                 friends[i % friends.Length].GetComponent<CarAI5>().leaderOffset = 3f;

             }

             if (i == st + 1)
             {
                 friends[i % friends.Length].GetComponent<CarAI5>().offset = 3f;
                 friends[i % friends.Length].GetComponent<CarAI5>().leaderOffset = -1.5f;
             }

             if (i == st + 2)
             {
                 friends[i % friends.Length].GetComponent<CarAI5>().offset = -3f;
                 friends[i % friends.Length].GetComponent<CarAI5>().leaderOffset = -1.5f;
             }

         }
     }
     else if (friends.Length == 2)
     {
         for (int i = st; i < st + friends.Length; i++) {

             if (i == st ) {
                 friends[i % friends.Length].GetComponent<CarAI5>().offset = 3f;
                 friends[i % friends.Length].GetComponent<CarAI5>().leaderOffset = 0f;
             }

             if (i == st + 1) {
                 friends[i % friends.Length].GetComponent<CarAI5>().offset = -3f;
                 friends[i % friends.Length].GetComponent<CarAI5>().leaderOffset = 0f;
             }

         }
     }
     else
     {
         friends[0].GetComponent<CarAI5>().offset = 0f; 
         friends[0].GetComponent<CarAI5>().leaderOffset = 0f;
     }
     */
}

    //REMEMBER TO SET THE COLOR AT 0 AT THE BEGINING 
    public void DfS(Graph G, int position, List<List<int>> solutions, List<int> tmp_path, bool found) {
        tmp_path.Add(position);
        G.setColorOfNode(position, 1);
       if(DfsStopwatch.ElapsedMilliseconds>1000)// if (solutions.Count > 1000)
        {
            return;
        }
        if (G.getNode(position).getDanger() >= 1) { //END 
            if (found == true) {
                if (tmp_path.Count > 7)
                {
                    if (ComputeYellowAngle(G.getNode(tmp_path[tmp_path.Count - 1]).getPosition(),
                        G.getNode(tmp_path[tmp_path.Count - 2]).getPosition()) > 30 && ComputeYellowAngle(G.getNode(tmp_path[tmp_path.Count - 1]).getPosition(),
                        G.getNode(tmp_path[tmp_path.Count - 2]).getPosition()) < 150)
                    {
                        solutions.Add(new List<int>(tmp_path));
                    }
                }

                tmp_path.RemoveAt(tmp_path.Count - 1);
                G.setColorOfNode(position, 0);
                return;
            }

            found = true;
        } else {
            if (found)
            {
                found = false;
                tmp_path.RemoveAt(tmp_path.Count - 1);
                G.setColorOfNode(position, 0);
                return;
            }
        }
        foreach (int child in G.getAdjList(position)) //LOOP FOR CHILDREN
        {
            if (G.getNode(child).getColor() == 0) {
                DfS(G, child, solutions, tmp_path, found);
            }
        }

        tmp_path.RemoveAt(tmp_path.Count - 1);
         G.setColorOfNode(position, 0);
        found = false;
    }

    public void ASuperStar(Graph G, int start_pos, int idx_goal) {
        priorityQueue Q = new priorityQueue();

        for (int i = 0; i < G.getSize(); i++) {
            G.setColorOfNode(i, 0);
            G.getNode(i).setParent(-1);
        }
        G.setColorOfNode(start_pos, 1);
        int best_node;
        float best_cost;
        //Debug.Log("RUNNING A STAR!"+ start_pos.ToString() +" " + idx_goal.ToString());

        float total_cost;
        Q.enqueue(start_pos, 0);


        while (Q.getSize() != 0) {
            best_node = Q.dequeue();
            best_cost = Q.getCost(best_node);
            //Delete node
            Q.removeNode(best_node);

            if (G.getNode(best_node).getDanger() > 0) {
                //Debug.Log("path found");
                return;
            }

            foreach (int child in G.getAdjList(best_node)) {
                total_cost = computeCost(G, best_node, child, idx_goal) + best_cost;

                if (Q.isInQueue(child)) {
                    if (Q.getCost(child) > total_cost) {
                        G.getNode(child).setParent(best_node);
                        Q.updateCost(child, total_cost);
                    }
                } else {
                    if (G.getNode(child).getColor() == 0) {
                        Q.enqueue(child, total_cost);
                        G.getNode(child).setParent(best_node);
                        G.getNode(child).setColor(1);
                    }
                }


            }
        }

        Debug.Log("path not found!!!!");

    }

    public float computePathCost(Graph G, List<int> _path) {
        float cost = 0;
        for (int i = 0; i < _path.Count - 1; i++) {
            cost += Vector3.Distance(G.getNode(_path[i]).getPosition(), G.getNode(_path[i + 1]).getPosition()) *
                    (G.getNode(_path[i]).getDanger() + G.getNode(_path[i + 1]).getDanger());

        }
        return cost;

    }


    public List<int> getBestPath(Graph G, int idx_start, int idx_goal) {
        List<int> path = new List<int>();
        path.Add(idx_goal);
        int idx = idx_goal;
        //Debug.Log("Following path from A star");
        while (idx != idx_start) {
            if (G.getNode(idx).getParent() == -1) {
                // Debug.Log("I am in node" + idx.ToString());
            }
            idx = G.getNode(idx).getParent();
            path.Add(idx);
            if (path.Count > 10000) {
                Debug.Log("OMG WTF?");
                break;
            }
        }
        path.Reverse();
        //Debug.Log("Path found:");
        //var strings = string.Join(", ", path);
        //Debug.Log(strings);
        return path;
    }

    public class priorityQueue {
        List<int> values;
        List<float> priority;

        public priorityQueue() {
            values = new List<int>();
            priority = new List<float>();
        }
        public void enqueue(int _value, float _p) {
            values.Add(_value);
            priority.Add(_p);
        }
        public int getSize() {
            return values.Count;
        }
        public int dequeue() {
            int best_idx = priority.IndexOf(priority.Min());
            int best_node = values[best_idx];


            return best_node;
        }
        public void removeNode(int node) {
            int node_idx = values.IndexOf(node);
            priority.RemoveAt(node_idx);
            values.RemoveAt(node_idx);
        }
        public float getCost(int node) {
            int idx = values.IndexOf(node);
            return priority[idx];
        }
        public void updateCost(int node, float p) {
            int idx = values.IndexOf(node);
            priority[idx] = p;
        }
        public bool isInQueue(int node) {
            int idx = values.IndexOf(node);
            if (idx == -1) { return false; }
            return true;
        }
    }

    public float computeCost(Graph G, int parent, int child, int goal) {
        //REAL COST:
        LayerMask mask = LayerMask.GetMask("CubeWalls");
        float real_cost;
        float h_cost;
        float zero = 0;
        float k = 1 / zero;

        real_cost = Vector3.Distance(G.getNode(parent).getPosition(), G.getNode(child).getPosition()) * ((G.getNode(parent).getDanger() + G.getNode(child).getDanger()));




        //real_cost = Math.Max(real_cost, k * (real_cost - 10));

        RaycastHit rayHit;
        bool hit = Physics.SphereCast(G.getNode(child).getPosition(), Margin, G.getNode(goal).getPosition() - G.getNode(child).getPosition(), out rayHit, Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition()), mask);
        if (!hit) {
            h_cost = 0;
        } else {
            h_cost = Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition());
        }


        return real_cost; //3 * real_cost + h_cost; //+ (100f / G.getNode(child).getDistanceToWall());

    }

    public void computeDiStanceToWall(Graph G) {
        RaycastHit hit;
        LayerMask mask = LayerMask.GetMask("CubeWalls");
        List<Vector3> radiusHelpMatrix = new List<Vector3>();

        radiusHelpMatrix.Add(new Vector3(-1f, -1f, -1f));
        radiusHelpMatrix.Add(new Vector3(1f, 1f, 1f));
        radiusHelpMatrix.Add(new Vector3(1f, 1f, -1f));
        radiusHelpMatrix.Add(new Vector3(1f, -1f, 1f));
        radiusHelpMatrix.Add(new Vector3(1f, -1f, -1f));
        radiusHelpMatrix.Add(new Vector3(-1f, 1f, 1f));
        radiusHelpMatrix.Add(new Vector3(-1f, 1f, -1f));
        radiusHelpMatrix.Add(new Vector3(-1f, -1f, 1f));

        for (int i = 0; i < G.getSize(); i++) {
            float minDistance = 5000f;
            float actualDistance;
            for (int j = 0; j < radiusHelpMatrix.Count; j++) {
                Physics.SphereCast(G.getNode(i).getPosition(), 2, radiusHelpMatrix[j], out hit, 50f, mask);
                actualDistance = hit.distance;
                if (actualDistance != 0) {
                    if (minDistance > actualDistance) {
                        minDistance = actualDistance;
                    }
                }
            }
            G.getNode(i).setDistanceToWall(minDistance);
            //Debug.Log(minDistance);
        }
    }


    List<Node> pathHelp(Graph G, List<int> idList) {
        List<Node> nodeList = new List<Node>();
        foreach (int i in idList) {
            nodeList.Add(G.getNode(i));
        }
        return nodeList;
    }

    Graph generateGraph(Graph G) {
        LayerMask mask = LayerMask.GetMask("CubeWalls");
        int col = terrain_manager.myInfo.x_N;
        int row = terrain_manager.myInfo.z_N;

        for (int i = 0; i < row; i++) //Add all the center of blocks as node
        {
            for (int j = 0; j < col; j++) {
                if (terrain_manager.myInfo.traversability[i, j] == 0f) {
                    Vector3 center = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
                    G.addNode(new Node(center));
                }
            }
        }

        int size = G.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = i; j < size; j++) {
                if (i != j && Vector3.Distance(G.getNode(i).getPosition(), G.getNode(j).getPosition()) < 21f) {
                    G.addNode(new Node((G.getNode(i).getPosition() + G.getNode(j).getPosition()) / 2));
                }
            }
        }

        size = G.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i != j && Vector3.Distance(G.getNode(i).getPosition(), G.getNode(j).getPosition()) < 11f) {
                    G.addNode(new Node((G.getNode(i).getPosition() + G.getNode(j).getPosition()) / 2));
                }
            }
        }
        size = G.getSize();
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i != j && Vector3.Distance(G.getNode(i).getPosition(), G.getNode(j).getPosition()) < 6f) {
                    G.addNode(new Node((G.getNode(i).getPosition() + G.getNode(j).getPosition()) / 2));
                }
            }
        }
        /* int density = 5;
         float margin = 5;
         for (int i = (int)terrain_manager.myInfo.x_low + 10; i < (int)terrain_manager.myInfo.x_high; i += density) {
             for (int j = (int)terrain_manager.myInfo.z_low + 10; j < (int)terrain_manager.myInfo.x_high; j += density) {
                 Vector3 center = new Vector3(i, 0, j);
                 if (!position_collision(margin,center)){ 
                     G.addNode(new Node(center));
                 }
             }
         }
         */
        for (int i = 0; i < G.getSize(); i++) {
            Vector3 center = G.getNode(i).getPosition();
            List<int> neighbor = getNeighbor(center, G);
            foreach (int p in neighbor) {
                G.addEdge(p, i);
            }


        }
        return G;
    }
    bool position_collision(float radiusMargin, Vector3 position) {
        float[,] radiusHelpMatrix = new float[,] { { radiusMargin, radiusMargin }, { radiusMargin, -radiusMargin }, { -radiusMargin, radiusMargin }, { -radiusMargin, -radiusMargin }, { radiusMargin, 0.0f }, { -radiusMargin, 0.0f }, { 0.0f, radiusMargin }, { 0.0f, -radiusMargin } };
        for (int a = 0; a < 8; a = a + 1) {
            int i = terrain_manager.myInfo.get_i_index(position.x + radiusHelpMatrix[a, 0]);
            int j = terrain_manager.myInfo.get_j_index(position.z + radiusHelpMatrix[a, 1]);

            if (terrain_manager.myInfo.traversability[i, j] == 1.0f) {
                return true;
            }
        }
        return false;
    }
    List<int> getNeighbor(Vector3 center, Graph G) {
        float dist = 2.5f;
        List<Vector3> newPos = new List<Vector3> { center + new Vector3(dist, 0, 0), center + new Vector3(-dist, 0, 0), center + new Vector3(0, 0, dist), center + new Vector3(0, 0, -dist) };
        List<int> idxList = new List<int>();
        Node n;
        float distance;
        foreach (var p in newPos) {
            n = G.FindClosestNode(p, G);
            distance = Vector3.Distance(n.getPosition(), p);
            if (distance < 1) {
                idxList.Add(n.getId());
            }
        }
        return idxList;
    }
    List<Vector3> computeTargets(TerrainManager _terrain_manager) {
        LayerMask mask = LayerMask.GetMask("CubeWalls");
        terrain_manager = _terrain_manager.GetComponent<TerrainManager>();

        int col = terrain_manager.myInfo.x_N;
        int row = terrain_manager.myInfo.z_N;

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                if (terrain_manager.myInfo.traversability[i, j] == 0f) {
                    Vector3 center = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
                    sets.Add(new HashSet<int>());
                    for (int ii = 0; ii < row; ii++) {
                        for (int jj = 0; jj < col; jj++) {

                            if (terrain_manager.myInfo.traversability[ii, jj] == 0f) {
                                List<Vector3> targets = getCorner(ii, jj);
                                bool neverhit = true;
                                foreach (Vector3 target in targets) {
                                    bool hit = Physics.Raycast(center, target - center, Vector3.Distance(target, center) - 0.1f, mask);
                                    if (hit) { neverhit = false; }
                                }
                                if (neverhit == true) {
                                    sets[sets.Count - 1].Add(ii * col + jj);
                                }
                            }
                        }
                    }
                } else {
                    sets.Add(new HashSet<int>());
                }
            }
        }
        List<int> nodes = findBestSet();
        List<Vector3> Vnodes = new List<Vector3>();

        foreach (int n in nodes) // CONVERT TO Vector3
        {
            Vnodes.Add(new Vector3(terrain_manager.myInfo.get_x_pos(n / col), 0, terrain_manager.myInfo.get_z_pos(n % col)));
        }
        return Vnodes;
    }


    List<int> findBestSetSmart() {
        List<int> bestset = new List<int>();
        HashSet<int> missingPoints = new HashSet<int>();

        for (int i = 0; i < sets.Count; i++) {
            if (sets[i].Count != 0) {
                missingPoints.Add(i);

            }
        }

        while (missingPoints.Count != 0) {
            List<int> score = new List<int>();

            foreach (HashSet<int> set in sets) {
                score.Add(0);
                foreach (int x in set) {
                    if (missingPoints.Contains(x)) {
                        score[score.Count - 1]++;
                    }
                }
            }

            if (score.Max() == missingPoints.Count) {
                int bestidx = score.IndexOf(score.Max()); //FOND THE BEST; NOW WE ADD IT
                bestset.Add(bestidx);
                foreach (int x in sets[bestidx]) {
                    missingPoints.Remove(x);
                }
            } else {
                for (int i = 0; i < sets.Count; i++) {
                    List<int> tmp_list = new List<int>(sets[i]);
                    if (sets[i].Count == 0) { continue; }
                    for (int j = 0; j < sets.Count; j++) {
                        if (sets[j].Count == 0) { continue; }
                        tmp_list.AddRange(sets[j]);
                        int tmp_score = 0;
                        foreach (int x in tmp_list) {
                            if (missingPoints.Contains(x)) {
                                tmp_score++;
                            }
                        }

                        foreach (int x in sets[j]) { tmp_list.Remove(x); }

                    }
                }
            }


        }



        return bestset;
    }

    List<int> findBestSet() {
        var stringBuilder = new StringBuilder();
        foreach (var arrayElement in sets) {

            var strings = string.Join(", ", arrayElement);
            stringBuilder.AppendLine(strings);
        }

        File.AppendAllText(@"C:\Users\delli\Desktop\array.txt", stringBuilder.ToString());



        List<int> bestset = new List<int>();
        HashSet<int> missingPoints = new HashSet<int>();

        for (int i = 0; i < sets.Count; i++) {
            if (sets[i].Count != 0) {
                missingPoints.Add(i);

            }
        }
        while (missingPoints.Count != 0) {
            List<int> score = new List<int>();

            foreach (HashSet<int> set in sets) {
                score.Add(0);
                foreach (int x in set) {
                    if (missingPoints.Contains(x)) {
                        score[score.Count - 1]++;
                    }
                }
            }

            int bestidx = score.IndexOf(score.Max()); //FOND THE BEST; NOW WE ADD IT
            bestset.Add(bestidx);
            foreach (int x in sets[bestidx]) {
                missingPoints.Remove(x);
            }
        }

        return bestset;
    }

    List<Vector3> getCorner(int i, int j) {
        Vector3 center = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
        List<Vector3> myList = new List<Vector3>();
        int x = (int)(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / (2 * terrain_manager.myInfo.x_N);

        int z = (int)(terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / (2 * terrain_manager.myInfo.z_N);

        myList.Add(center + new Vector3(+x, 0, +z));

        myList.Add(center + new Vector3(+x, 0, -z));

        myList.Add(center + new Vector3(-x, 0, +z));

        myList.Add(center + new Vector3(-x, 0, -z));

        return myList;

    }
    List<Vector3> getCorner(Vector3 center) {
        List<Vector3> myList = new List<Vector3>(4);
        int x = (int)(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / (2 * terrain_manager.myInfo.x_N);

        int z = (int)(terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / (2 * terrain_manager.myInfo.z_N);

        myList[0] = center + new Vector3(+x, 0, +z);

        myList[1] = center + new Vector3(+x, 0, -z);

        myList[2] = center + new Vector3(-x, 0, +z);

        myList[3] = center + new Vector3(-x, 0, -z);

        return myList;

    }

    void computeDanger(Graph G) {
        LayerMask mask = LayerMask.GetMask("CubeWalls");
        for (int i = 0; i < G.getSize(); i++) {
            Node n = G.getNode(i);
            int danger = 0;
            foreach (var enemy in enemies) {
                //DroneGraph.FindClosestNode(car.transform.position, DroneGraph)

                if (!Physics.Raycast(n.getPosition(), enemy.transform.position - n.getPosition(),
                    Vector3.Distance(n.getPosition(), enemy.transform.position), mask)) {
                    danger++;
                }
            }
            n.setDanger(danger);
        }
    }

    void smoothDanger(Graph G) {
        List<float> newDanger = new List<float>();
        for (int i = 0; i < G.getSize(); i++) {
            float danger = 0;
            float m = G.getNode(i).getDanger();
            for (int j = 0; j < G.getAdjList(i).Count; j++) {
                danger += Math.Max(m, G.getNode(G.getAdjList(i)[j]).getDanger());
            }

            danger /= G.getAdjList(i).Count;
            newDanger.Add(danger);
        }

        for (int i = 0; i < G.getSize(); i++) {
            G.getNode(i).setDanger(newDanger[i]);
        }
    }
    float ComputeYellowAngle(Vector3 pos, Vector3 pos2)
    {
        float yellow_angle;
        var lastevil = enemies[0];
        var turret_mask = LayerMask.GetMask("CubeWalls");
        foreach (var enemy in enemies) {
            if (!Physics.Raycast(enemy.transform.position,
                pos - enemy.transform.position,
                Vector3.Distance(enemy.transform.position, pos),//it was my_path[my_path_length - 1]
                turret_mask)) {

                // Debug.DrawLine(enemy.transform.position, my_path[lastPointInPath + 1].getPosition(),Color.yellow,100);
                lastevil = enemy;
                break;
            }
        }

        yellow_angle = Vector3.Angle(lastevil.transform.position - pos2,
            pos - pos2);
        return yellow_angle;
    }
}