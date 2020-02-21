using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class permutor
{

    public List<List<int>> permute(int amountOfCars, List<int> nodeList)
    {

        List<List<int>> permutedList = new List<List<int>>();
        List<int> CarIndices = new List<int>();
        CarIndices.Add(nodeList[0]);
        CarIndices.Add(nodeList[1]);
        CarIndices.Add(nodeList[2]);

        for(int i = 0; i < 3; i++)
        {
            for(int j = nodeList.Count-1; j >= 3; j--)
            {
                if (nodeList[i] == nodeList[j])
                {
                    nodeList.RemoveAt(j);
                }
            }
        }
        int length = nodeList.Count;

        for (int i = 0; i < length; i++)
        {
            Random.seed = System.DateTime.Now.Millisecond;
            int rand = UnityEngine.Random.Range(i, length);
            int temp = nodeList[i];
            nodeList[i] = nodeList[rand];
            nodeList[rand] = temp;
        }

        //Find earliest car index in the new permuted list
        int lowIdx = 1000;
        foreach (int index in CarIndices)
        {
            int newIdx = nodeList.IndexOf(index);
            if (newIdx < lowIdx)
            {
                lowIdx = newIdx;
            }
        }

        List<int> subList = new List<int>();

        for (int i = lowIdx; i < (length + lowIdx); i++)
        {

            if (i == lowIdx)
            {
                subList.Add(nodeList[i % length]);
            }

            else if (CarIndices.Contains(nodeList[i % length]))
            {
                permutedList.Add(new List<int>(subList));
                subList.Clear();
                subList.Add(nodeList[i % length]);
                }

            else
            {
                subList.Add(nodeList[i % length]);
            }
        }
        permutedList.Add(new List<int>(subList));
        // Loop through 
        return permutedList;
    }
}
