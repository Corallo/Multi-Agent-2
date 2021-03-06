﻿using MyGraph;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
public class Planner2 {

    public List<HashSet<int>> sets = new List<HashSet<int>>();
    public TerrainManager terrain_manager;
    //private Graph DroneGraph;
    public GameObject[] friends;
    public GameObject[] enemies;
    private CarController m_Car;
    //public LayerMask mask;
    private float Margin = 0;
    public void ComputePath(TerrainManager _terrain_manager) {
        terrain_manager = _terrain_manager.GetComponent<TerrainManager>();
        Graph DroneGraph = new Graph();
        var watch = System.Diagnostics.Stopwatch.StartNew();
        // the code that you want to measure comes here

        DroneGraph = generateGraph(DroneGraph);
        watch.Stop();
        var elapsedMs = watch.Elapsed;
        Debug.Log("Time to create Graph:" + elapsedMs.ToString());
        friends = GameObject.FindGameObjectsWithTag("Player");
        enemies = GameObject.FindGameObjectsWithTag("Enemy");
        m_Car = friends[0].GetComponent<CarController>();

        List<Node> my_path = new List<Node>();

        LayerMask mask = LayerMask.GetMask("CubeWalls");

        for (int i = 0; i < DroneGraph.getSize(); i++) {
            GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Collider c = cube.GetComponent<Collider>();
            c.enabled = false;
            Vector3 position = DroneGraph.getNode(i).getPosition();


            cube.transform.position = new Vector3(position.x, 1, position.z);
            cube.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            for (int k = 0; k < DroneGraph.getAdjList(i).Count; k++) {
                // Debug.DrawLine(DroneGraph.getNode(i).getPosition(), DroneGraph.getNode(DroneGraph.getAdjList(i)[k]).getPosition(), Color.yellow, 1f);
            }

        }

        /* GENERATE INITIAL POSITION NODES FOR CAR (FIND THE CLOSEST NODE TO THE CAR, THERE IS ARLEADY A FUNCTION FOR THAT)*/
        List<List<int>> car_targets = new List<List<int>>();
        /* GENERATE PERMUTATION OF INTERESTING NODES */
        List<List<int>> best_path = new List<List<int>>();

        watch = System.Diagnostics.Stopwatch.StartNew();
        List<Vector3> points_to_visit = computeTargets(_terrain_manager);
        Debug.Log("solution size: " + points_to_visit.Count.ToString());
        watch.Stop();
        elapsedMs = watch.Elapsed;
        Debug.Log("Time to find best sub-set" + elapsedMs.ToString());



        /* FIX THE START POINT = 0 IN A * */
        int NUMBER_OF_CARS = 3;
        int start_idx = 0;
        int goal_idx = 1; //= goalN.getId();
        List<int> path = new List<int>();

        List<int> point_of_interest = new List<int>();

        foreach (var car in friends) {
            point_of_interest.Add(DroneGraph.FindClosestNode(car.transform.position, DroneGraph).getId());
        }
        foreach (var checkpoint in points_to_visit) {
            point_of_interest.Add(DroneGraph.FindClosestNode(checkpoint, DroneGraph).getId());
        }
        watch = System.Diagnostics.Stopwatch.StartNew();
        Dictionary<Tuple<int, int>, float> CostMatrix = new Dictionary<Tuple<int, int>, float>();
        Dictionary<Tuple<int, int>, List<int>> PathMatrix = new Dictionary<Tuple<int, int>, List<int>>();
        for (int i = 0; i < point_of_interest.Count; i++) {
            for (int k = 0; k < point_of_interest.Count; k++) {
                ASuperStar(DroneGraph, point_of_interest[i], point_of_interest[k]);
                List<int> tmp_path = getBestPath(DroneGraph, point_of_interest[i], point_of_interest[k]);
                PathMatrix[new Tuple<int, int>(point_of_interest[i], point_of_interest[k])] = new List<int>(tmp_path); // ADD To the real path, the path to get the ith point 
                CostMatrix[new Tuple<int, int>(point_of_interest[i], point_of_interest[k])] = (computePathCost(DroneGraph, tmp_path));
            }
        }
        watch.Stop();
        elapsedMs = watch.Elapsed;
        Debug.Log("Time to precompute A Star" + elapsedMs.ToString());
        /* Debug.Log("Printing points of interest");
         var strings = string.Join(", ", point_of_interest);
         Debug.Log(strings);*/

        List<List<int>> this_path = new List<List<int>>();
        watch = System.Diagnostics.Stopwatch.StartNew();



        car_targets = OldTabuSearch.RunTabuSearch(3, point_of_interest, CostMatrix, PathMatrix, 100, 1000, 100);
        Debug.Log(car_targets.Count);

        List<float> this_cost = new List<float>();
        this_path.Clear();
        for (int c = 0; c < NUMBER_OF_CARS; c++) { //For all the car 
            path.Clear();
            Debug.Log(car_targets[c].Count);
            if (car_targets[c].Count == 1) {
                path.Add(car_targets[c][0]);
            }
            for (int i = 0; i < car_targets[c].Count - 1; i++) {        //for all the interesting points of that car   

                start_idx = car_targets[c][i];
                goal_idx = car_targets[c][i + 1];
                path.AddRange(PathMatrix[new Tuple<int, int>(start_idx, goal_idx)]);
            }
            this_path.Add(new List<int>(path)); //copy the path
                                                //Debug.Log(this_cost[c]);
        }

        /*  Debug.Log("Cost for this path:" + real_cost.ToString());
          foreach (var l in this_path)
          {
              strings = string.Join(", ", l);
              Debug.Log(strings);
          }*/

        best_path = new List<List<int>>(this_path);


        /*
        // Block of code to hopefully remove consecutive duplicates of nodes in the path that might cause issues.
        foreach (var List in best_path)
        {
            for (int i = List.Count - 2; i >= 0; i--)
            {
                if (List[i] == List[i + 1])
                {
                    List.RemoveAt(i);
                }
            }
        }
        */



        watch.Stop();
        elapsedMs = watch.Elapsed;
        Debug.Log("Time for permutation " + elapsedMs.ToString());

        // Block of code to hopefully remove consecutive duplicates of nodes in the path that might cause issues.
        /*  foreach (var List in best_path)
          {
              for (int i = List.Count - 2; i >= 0; i--)
              {
                  if (List[i] == List[i + 1])
                  {
                      List.RemoveAt(i);
                  }
              }
          }
          */


        int j = 0;
        foreach (var car in friends) {

            //car.GetComponent<CarAI3>().trajectory = car.GetComponent<CarAI3>().myController.makeTrajectory(myRealPath, maxVel / 2.236f, maxAcc); ;
            //car.GetComponent<CarAI3>().lastPointInPath = best_path[j][0];
            car.GetComponent<CarAI2>().my_path = pathHelp(DroneGraph, best_path[j]);
            car.GetComponent<CarAI2>().my_path_length = best_path[j].Count;
            car.GetComponent<CarAI2>().my_mask = mask;
            if (j == 0) {
                car.GetComponent<CarAI2>().color = "Cyan";
                car.GetComponent<CarAI2>().skipper = 0;
            } else if (j == 1) {
                car.GetComponent<CarAI2>().color = "Yellow";
                car.GetComponent<CarAI2>().skipper = 150;
            } else {
                car.GetComponent<CarAI2>().color = "Blue";
                car.GetComponent<CarAI2>().skipper = 250;
            }
            j++;
        }


        /*SUM COST OF A STAR */


        Vector3 old_wp = DroneGraph.getNode(best_path[0][0]).getPosition();

        foreach (var wp in best_path[0]) {
            //Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
            Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.cyan, 10000f);
            old_wp = DroneGraph.getNode(wp).getPosition();
        }

        old_wp = DroneGraph.getNode(best_path[1][0]).getPosition();
        foreach (var wp in best_path[1]) {
            //Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
            Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.yellow, 10000f);
            old_wp = DroneGraph.getNode(wp).getPosition();
        }
        old_wp = DroneGraph.getNode(best_path[2][0]).getPosition();

        foreach (var wp in best_path[2]) {
            //Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
            Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.blue, 10000f);
            old_wp = DroneGraph.getNode(wp).getPosition();
        }



        my_path = pathHelp(DroneGraph, path);


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

            if (idx_goal == best_node) {
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
            cost += Vector3.Distance(G.getNode(_path[i]).getPosition(), G.getNode(_path[i + 1]).getPosition());

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

        real_cost = Vector3.Distance(G.getNode(parent).getPosition(), G.getNode(child).getPosition());




        //real_cost = Math.Max(real_cost, k * (real_cost - 10));

        RaycastHit rayHit;
        bool hit = Physics.SphereCast(G.getNode(child).getPosition(), Margin, G.getNode(goal).getPosition() - G.getNode(child).getPosition(), out rayHit, Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition()), mask);
        if (!hit) {
            h_cost = 0;
        } else {
            h_cost = Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition());
        }


        return 3 * real_cost + h_cost; //+ (200f / G.getNode(child).getDistanceToWall());

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
        RaycastHit rayout;
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
        for (int i = 0; i < G.getSize(); i++) {
            Vector3 center = G.getNode(i).getPosition();
            for (int j = 0; j < G.getSize(); j++) {
                if (i != j) {
                    Vector3 target = G.getNode(j).getPosition();
                    bool hit = Physics.SphereCast(center, 2f, target - center, out rayout, Vector3.Distance(target, center), mask);
                    if (!hit == true) {
                        G.addEdge(i, j);
                    }
                }
            }
        }
        return G;
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
}