using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace perm {
    public class permutor {

        public List<List<int>> permute(int amountOfCars, List<int> nodeList) {
            List<List<int>> permutedList = new List<List<int>>();
            List<int> CarIndices = new List<int>();
            CarIndices.Add(nodeList[0]);
            CarIndices.Add(nodeList[1]);
            CarIndices.Add(nodeList[2]);
            int length = nodeList.Count;
            int last = length - 1; // Permutes the nodeList
            // MARCUS I AM INIZIALIZING AMOUNT TO 100 BECAUSE I DON'T KNOW WHAT IT IS, CHECK IT PLS
            int amount = 100;
            for (int i = 0; i < amount; i++) {
                int rand = UnityEngine.Random.Range(i, length);
                int temp = nodeList[i];
                nodeList[i] = nodeList[rand];
                nodeList[rand] = temp;
            }

            //Find earliest car index in the new permuted list
            int lowIdx = 1000;
            foreach (int index in CarIndices) {
                int newIdx = nodeList.IndexOf(index);
                if (newIdx < lowIdx) {
                    lowIdx = newIdx;
                }
            }

            List<int> subList = new List<int>();

            for (int i = lowIdx; i < (length + lowIdx); i++) {

                if (i == lowIdx) {
                    subList.Add(nodeList[i % length]);
                }

                else if (CarIndices.Contains(nodeList[i % length])) {
                    permutedList.Add(new List<int>(subList));
                    subList.Clear();
                }

                else {
                    subList.Add(nodeList[i % length]);
                }
            }
            // Loop through 
            return permutedList;
        }
    }
}