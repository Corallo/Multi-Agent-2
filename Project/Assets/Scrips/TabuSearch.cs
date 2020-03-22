using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class TabuSearch {

    public static List<List<int>> RunTabuSearch(int amountOfCars, List<int> nodeList, Dictionary<Tuple<int, int>, float> CostMatrix, Dictionary<Tuple<int, int>, List<int>> PathMatrix, int n, int number_of_iterations, int tabuSize, List<int> CarIndices) {
        List<List<int>> bestList = new List<List<int>>();
        float bestListScore = 99999999999999;
        //List<int> CarIndices = new List<int>();
        /*
        for (int i = 0; i < 3; i++) {
            for (int j = nodeList.Count - 1; j >= 3; j--) {
                if (CarIndices[i] == nodeList[j]) {
                    nodeList.RemoveAt(j);
                }
            }
        }*/
        /*CarIndices.Add(nodeList[0]);
        CarIndices.Add(nodeList[1]);
        CarIndices.Add(nodeList[2]);*/
        List<List<int>> bestCandidate = generateNeighbourhood(amountOfCars, CarIndices, nodeList, 1)[0];
        List<float> tabuCostList = new List<float>();
        List<List<List<int>>> tabuList = new List<List<List<int>>>();

        while (number_of_iterations-- >= 0) {
            float bestCandidateScore = 9999999999;
            List<int> startList = new List<int>();
            foreach (var car in bestCandidate) {
                startList.AddRange(car);
            }
            List<List<List<int>>> children = generateNeighbourhood(amountOfCars, CarIndices, startList, n);
            bestCandidate = children[0];
            foreach (var candidate in children) {

                float childCost = fitness(candidate, CostMatrix);
                if ((!contains(tabuCostList, tabuList, childCost, candidate)) && childCost < bestCandidateScore) {
                    bestCandidate = candidate; //DELETED THE NEW FROM HERE
                    bestCandidateScore = childCost;
                }

            }
            // Debug.Log(bestListScore - bestCandidateScore);
            if (bestCandidateScore < bestListScore) {
                bestList = new List<List<int>>(bestCandidate);
                bestListScore = bestCandidateScore;
                Debug.Log("NEW SOLUTION");
            }
            tabuCostList.Add(bestCandidateScore);
            tabuList.Add(new List<List<int>>(bestCandidate));

            if (tabuList.Count > tabuSize) {

                tabuList.RemoveAt(0);
                tabuCostList.RemoveAt(0);
            }

        }
        return bestList;
    }
    public static List<List<int>> RunTabuSearchSmall(int amountOfCars, List<int> nodeList, Dictionary<Tuple<int, int>, float> CostMatrix, Dictionary<Tuple<int, int>, List<int>> PathMatrix, int n, int number_of_iterations, int tabuSize) {
        List<List<int>> bestList = new List<List<int>>();
        float bestListScore = 99999999999999;
        List<int> CarIndices = new List<int>();

        for (int i = 0; i < 3; i++) {
            for (int j = nodeList.Count - 1; j >= 3; j--) {
                if (nodeList[i] == nodeList[j]) {
                    nodeList.RemoveAt(j);
                }
            }
        }
        CarIndices.Add(nodeList[0]);
        List<List<int>> bestCandidate = generateNeighbourhood(amountOfCars, CarIndices, nodeList, 1)[0];
        List<float> tabuCostList = new List<float>();
        List<List<List<int>>> tabuList = new List<List<List<int>>>();

        while (number_of_iterations-- >= 0) {
            float bestCandidateScore = 9999999999;
            List<int> startList = new List<int>();
            foreach (var car in bestCandidate) {
                startList.AddRange(car);
            }
            List<List<List<int>>> children = generateNeighbourhood(amountOfCars, CarIndices, startList, n);
            bestCandidate = children[0];
            foreach (var candidate in children) {

                float childCost = fitness(candidate, CostMatrix);
                if ((!contains(tabuCostList, tabuList, childCost, candidate)) && childCost < bestCandidateScore) {
                    bestCandidate = candidate; //DELETED THE NEW FROM HERE
                    bestCandidateScore = childCost;
                }

            }
            // Debug.Log(bestListScore - bestCandidateScore);
            if (bestCandidateScore < bestListScore) {
                bestList = new List<List<int>>(bestCandidate);
                bestListScore = bestCandidateScore;
                Debug.Log("NEW SOLUTION");
            }
            tabuCostList.Add(bestCandidateScore);
            tabuList.Add(new List<List<int>>(bestCandidate));

            if (tabuList.Count > tabuSize) {

                tabuList.RemoveAt(0);
                tabuCostList.RemoveAt(0);
            }

        }
        return bestList;
    }

    public static bool contains(List<float> tabuCostList, List<List<List<int>>> tabuList, float cost, List<List<int>> candidate) {
        for (int i = 0; i < tabuList.Count; i++) {
            if (Math.Abs(tabuCostList[i] - cost) < 0.1) {
                bool flag = false;
                for (int j = 0; j < tabuList[i].Count; j++) {
                    if (tabuList[i][j].SequenceEqual(candidate[j])) {
                        flag = true;
                    }
                }
                if (flag) {
                    return true;
                }

            }
        }
        return false;
    }
    public static float fitness(List<List<int>> child, Dictionary<Tuple<int, int>, float> CostMatrix) {

        //List<List<int>> this_path = new List<List<int>>();
        List<float> this_cost = new List<float>();
        // List<int> path = new List<int>();
        int start_idx;
        int goal_idx;
        //this_path.Clear();
        for (int c = 0; c < child.Count; c++) { //For all the car 
                                                //  path.Clear();
            /* if (child[c].Count == 1) {
                 path.Add(child[c][0]);
             }*/
            this_cost.Add(0);
            for (int i = 0; i < child[c].Count - 1; i++) {        //for all the interesting points of that car   

                start_idx = child[c][i];
                goal_idx = child[c][i + 1];
                //ASuperStar(DroneGraph, start_idx, goal_idx);
                //path.AddRange(getBestPath(DroneGraph, start_idx, goal_idx)); // ADD To the real path, the path to get the ith point 
                // path.AddRange(PathMatrix[new Tuple<int, int>(start_idx, goal_idx)]);
                this_cost[this_cost.Count - 1] += CostMatrix[new Tuple<int, int>(start_idx, goal_idx)];
            }
            //this_cost.Add(computePathCost(DroneGraph, path));
            //this_path.Add(new List<int>(path)); //copy the path
            //Debug.Log(this_cost[c]);
        }
        float real_cost = this_cost.Max();
        return real_cost;
    }
    public static List<List<List<int>>> generateNeighbourhood(int amountOfCars, List<int> CarIndices, List<int> nodeList, int n) {

        int length = nodeList.Count;

        List<List<List<int>>> children = new List<List<List<int>>>();

        List<Tuple<int, int>> swap_idx = generateSwap(nodeList.Count, n);
        foreach (var t in swap_idx) {
            List<List<int>> permutedList = new List<List<int>>();
            List<int> child = new List<int>(nodeList);
            child.Swap<int>(t);

            int lowIdx = 1000;
            foreach (int index in CarIndices) {
                int newIdx = child.IndexOf(index);
                if (newIdx < lowIdx) {
                    lowIdx = newIdx;
                }
            }

            List<int> subList = new List<int>();

            for (int i = lowIdx; i < (length + lowIdx); i++) {

                if (i == lowIdx) {
                    subList.Add(child[i % length]);
                } else if (CarIndices.Contains(child[i % length])) {
                    permutedList.Add(new List<int>(subList));
                    subList.Clear();
                    subList.Add(child[i % length]);
                } else {
                    subList.Add(child[i % length]);
                }
            }
            permutedList.Add(new List<int>(subList));


            children.Add(permutedList);
        }
        return children;
    }

    public static List<T> Swap<T>(this List<T> list, Tuple<int, int> idx) {
        T tmp = list[idx.Item1];
        list[idx.Item1] = list[idx.Item2];
        list[idx.Item2] = tmp;
        return list;
    }

    public static List<Tuple<int, int>> generateSwap(int length, int n) {
        UnityEngine.Random.seed = System.DateTime.Now.Millisecond;
        List<Tuple<int, int>> res = new List<Tuple<int, int>>();
        while (res.Count < n) {
            Tuple<int, int> t = Tuple.Create(UnityEngine.Random.Range(0, length), UnityEngine.Random.Range(0, length));
            bool flag = true;
            foreach (var element in res) {
                if (element == t) {
                    flag = false;
                }
            }
            if (flag) {
                res.Add(t);
            }
        }
        return res;
    }

}