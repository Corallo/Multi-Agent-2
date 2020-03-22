using System;
using System.Collections.Generic;
using UnityEngine;

namespace MyGraph {
	public class Node {

		private int id;
		private double speedX, speedZ, AccellerationX, AccellerationZ, theta;
		private Vector3 position;
		private float x, z;
		private int color;
		private int parent;
		private float distanceToWall;
		private float danger;

		public float getDanger() {
			return danger;
		}

		public void setDanger(float d) {
			danger = d;
		}
		public float getDistanceToWall() {
			return distanceToWall;
		}
		public void setDistanceToWall(float _d) {
			distanceToWall = _d;
		}

		public int getParent() {
			return parent;
		}
		public void setParent(int p) {
			parent = p;
		}
		public void setColor(int _c) {
			color = _c;
		}
		public int getColor() { return color; }
		public Vector3 getPosition() {
			return position;
		}

		public float getPositionX() {
			return position.x;
		}
		public float getPositionZ() {
			return position.z;
		}
		public int getId() {
			return id;
		}
		public Node(float _x, float _z) {
			x = _x;
			z = _z;
			position = new Vector3(_x, 0, _z);
			id = -1;
		}
		public Node(Vector3 _position) {
			position = _position;
			id = -1;
		}
		public Node() {
			x = 0;
			z = 0;
			id = -1;
		}
		public void setId(int _id) {
			id = _id;
		}
		public void setPositionX(float _x) {
			x = _x;
			position.x = _x;
		}
		public void setPositionZ(float _z) {
			z = _z;
			position.z = _z;
		}
		public void setTheta(double _theta) {
			theta = _theta;
		}
		public double getTheta() {
			return theta;
		}
	}

	public class Graph {
		Dictionary<int, Node> nodes;
		Dictionary<int, List<int>> adjList;
		List<int> endNodes;

		int size;

		public Graph() {

			nodes = new Dictionary<int, Node>();
			adjList = new Dictionary<int, List<int>>();
			endNodes = new List<int>();
			size = 0;
		}

		// The following constructor has parameters for two of the three 
		// properties. 

		public void setColorOfNode(int _idx, int color) {
			nodes[_idx].setColor(color);
		}
		public int getColorOfNode(int _idx) {
			return nodes[_idx].getColor();
		}
		public Dictionary<int, Node> getNodes() {
			return nodes;
		}
		public int getSize() {
			return size;
		}
		public int addNode(Node _newNode) {
			int id = size++;
			_newNode.setId(id);
			_newNode.setColor(0);
			nodes.Add(id, _newNode);
			adjList.Add(id, new List<int>());
			return id;
		}
		public int addNode(Node _newNode, List<int> _adjList) {
			int id = size++;
			nodes.Add(id, _newNode);
			adjList.Add(id, _adjList);
			return id;
		}
		public Node getNode(int _id) {
			return nodes[_id];
		}
		public List<int> getAdjList(int _id) {
			return adjList[_id];
		}
		public void setAdjList(int _id, List<int> _adjList) {
			adjList[_id] = _adjList;
		}
		public void addEdge(int _idA, int _idB) {
			List<int> actualList;

			actualList = adjList[_idA];
			if (!actualList.Contains(_idB)) {
				actualList.Add(_idB);
				setAdjList(_idA, actualList);
			}
			actualList = adjList[_idB];
			if (!actualList.Contains(_idA)) {
				actualList.Add(_idA);
				setAdjList(_idB, actualList);
			}
		}

		public double computeAngle(Node _A, Node _B, Node _C) {
			double a_x = _A.getPositionX() - _B.getPositionX();
			double a_z = _A.getPositionZ() - _B.getPositionZ();
			double b_x = _B.getPositionX() - _C.getPositionX();
			double b_z = _B.getPositionX() - _C.getPositionZ();
			double num = a_x * b_x + a_z * b_z;
			double den = Math.Sqrt(a_x * a_x + a_z * a_z) * Math.Sqrt(b_x * b_x + b_z * b_z);
			double angle = Math.Acos(num / (den + 0.000001f));
			return angle;
		}


		public void setPathTheta(List<int> _path) {
			int A, B, C = 0;
			A = _path[0];
			nodes[A].setTheta(0);

			for (int i = 1; i < _path.Count - 1; i++) {
				A = _path[i - 1];
				B = _path[i];
				C = _path[i + 1];
				nodes[B].setTheta(computeAngle(nodes[A], nodes[B], nodes[C]));
			}
			nodes[C].setTheta(0);

		}



		public Node FindClosestNode(Vector3 target, Graph G) {

			Node temp;
			Node closest = nodes[0];//root
			float closestDistance = Vector3.Distance(closest.getPosition(), target);
			float checkDistance = 0f;

			for (int i = 0; i < G.getSize(); i++) {
				temp = G.getNode(i);
				checkDistance = Vector3.Distance(temp.getPosition(), target);
				if (checkDistance < closestDistance) {
					closestDistance = checkDistance;
					closest = temp;
				}
			}
			return closest;
		}


	}
}