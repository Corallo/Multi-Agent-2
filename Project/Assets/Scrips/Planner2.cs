using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class Planner2
{

    public List<HashSet<int>> sets = new List<HashSet<int>>();
    public TerrainManager terrain_manager;
    List<Vector3> plan(TerrainManager _terrain_manager)
    {
        
        terrain_manager = _terrain_manager.GetComponent<TerrainManager>();

        int col = terrain_manager.myInfo.x_N;
        int row = terrain_manager.myInfo.z_N;

        for (int i = 0; i < row; i++){
            for( int j = 0; j < col; j++){
                Vector3 center = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
                sets.Add(new HashSet<int>());
                for (int ii = 0; ii<row; ii++)
                {
                    for (int jj=0; jj < col; jj++)
                    {

                        List<Vector3> targets = getCorner(ii, jj);
                        bool neverhit = true;
                        foreach (Vector3 target in targets)
                        {
                            bool hit = Physics.Raycast(center, target - center, Vector3.Distance(target, center));
                            if (hit) { neverhit = false; }
                        }
                        if (neverhit == true)
                        {
                            sets[i * col + j].Add(ii * col + jj);
                        }
                    }
                }
            }
        }
        List<int> nodes = findBestSet();
        List<Vector3> Vnodes = new List<Vector3>(nodes.Count);
        int idx = 0;
        foreach (int n in nodes) // CONVERT TO Vector3
        {
            Vnodes[idx++] = new Vector3(terrain_manager.myInfo.get_x_pos(n/col), 0, terrain_manager.myInfo.get_z_pos(n%col));
        }
        return Vnodes;
    }




    List<int> findBestSet()
    {
        List<int> bestset = new List<int>();
        HashSet<int> missingPoints = new HashSet<int>();
        
        for (int i=0;i<sets.Count;i++)
        {
            missingPoints.Add(i);
        }
        while (missingPoints.Count != 0)
        {
            List<int> score = new List<int>(sets.Count);
            int idx = 0;
            foreach (HashSet<int> set in sets)
            {
                foreach (int x in set)
                {
                    if (missingPoints.Contains(x))
                    {
                        score[idx]++;
                    }
                }
                idx++;
            }

           int bestidx = score.IndexOf(score.Max()); //FOND THE BEST; NOW WE ADD IT
           bestset.Add(bestidx);
            foreach (int x in sets[bestidx])
            {
                    missingPoints.Remove(x);   
            }
        }

        return bestset;
    }

    List<Vector3> getCorner(int i, int j) {
        Vector3 center = new Vector3(terrain_manager.myInfo.get_x_pos(i), 0, terrain_manager.myInfo.get_z_pos(j));
        List<Vector3> myList = new List<Vector3>(4);
        int x = (int)(terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / (2*terrain_manager.myInfo.x_N);

        int z = (int)(terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / (2*terrain_manager.myInfo.z_N);

        myList[0] = center + new Vector3(+x, 0, +z);

        myList[1] = center + new Vector3(+x, 0, -z);

        myList[2] = center + new Vector3(-x, 0, +z);

        myList[3] = center + new Vector3(-x, 0, -z);

        return myList;

    }
}