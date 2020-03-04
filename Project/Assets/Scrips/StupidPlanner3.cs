using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MyGraph;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(CarController))]

public class StupidPlanner3  
{
	private CarController m_Car; // the car controller we want to use
	LayerMask mask;
	public GameObject terrain_manager_game_object;
	TerrainManager terrain_manager;

	public GameObject[] friends;
	public GameObject[] enemies;


	int counter = 0;
	//SphereCollider droneCollider;
	private float Margin = 0;
	private float sideMargin = 0;

	private Graph DroneGraph;
	//All edges have same length:
	public float edgeLength = 10.0f;
	public float edgeMinDist = 0.1f;
	public float addEdgeMaxLength = 12.0f;
	List<Node> my_path = new List<Node>();
	int randomTimer = 0;
	Vector3 goal_pos;
	int lastPointInPath = 0;
	public float newAngle = 0;
	private Boolean backing = false;

	public void ComputePath(TerrainManager _terrain_manager)
	{
		 float maxVel = 20f;
		 float maxAcc = 3f;
		float constAcc = 0.1f;
		mask = ~LayerMask.GetMask("Player"); 
		//TerrainManager t = terrain_manager.GetComponent<TerrainManager>();
		//terrain_manager = t;
		terrain_manager = _terrain_manager.GetComponent<TerrainManager>();
		//terrain_manager = new TerrainManager();

		// note that both arrays will have holes when objects are destroyed
		// but for initial planning they should work
		friends = GameObject.FindGameObjectsWithTag("Player");
		enemies = GameObject.FindGameObjectsWithTag("Enemy");
		m_Car = friends[0].GetComponent<CarController>();
		Debug.Log("TESTING!");
		_terrain_manager.done = true;

		Transform ColliderBottom = m_Car.transform.Find("Colliders").Find("ColliderBottom");
		Vector3 scale = ColliderBottom.localScale;
		Margin = scale.z / 1;
		sideMargin = scale.x / 1;


		// Plan your path here
		// Replace the code below that makes a random path
		// ...

		Vector3 start_pos = terrain_manager.myInfo.start_pos;
		goal_pos = terrain_manager.myInfo.goal_pos;

		Node StartNode = new Node(new Vector3(start_pos.x, 1, start_pos.z));

		DroneGraph = new Graph();
		DroneGraph.addNode(StartNode); 
		int n = 3000;
		RRG(n, DroneGraph);
		killFuckers(DroneGraph);


		computeDiStanceToWall(DroneGraph);
		for (int i = 0; i < DroneGraph.getSize(); i++)
		{
			GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
			Collider c = cube.GetComponent<Collider>();
			c.enabled = false;
			Vector3 position = DroneGraph.getNode(i).getPosition();
			//Debug.Log("Node n." + i.ToString());
			//foreach (object o in DroneGraph.getAdjList(i))
			//{
			//   Debug.Log(o);
			//}
			cube.transform.position = new Vector3(position.x, 1, position.z);
			cube.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
			for (int k = 0; k < DroneGraph.getAdjList(i).Count; k++)
			{
				Debug.DrawLine(DroneGraph.getNode(i).getPosition(), DroneGraph.getNode(DroneGraph.getAdjList(i)[k]).getPosition(), Color.red, 10f);
				//Debug.Log(DroneGraph.getNode(i).getPosition() +" - "+ DroneGraph.getNode(j).getPosition());
			}
			//Debug.Log("Adj list of node " + i.ToString());
			//var strings = string.Join(", ", DroneGraph.getAdjList(i));
			//Debug.Log(strings);

		}
		// Node goalN = DroneGraph.FindClosestNode(goal_pos, DroC: \Users\delli\Desktop\KTH\MultiAgent\Multi - Agent - 2 - master\Multi - Agent - 2 - master\Project\Assets\Scrips\CarAI1.csneGraph);
	

	
		/* GENERATE INITIAL POSITION NODES FOR CAR (FIND THE CLOSEST NODE TO THE CAR, THERE IS ARLEADY A FUNCTION FOR THAT)*/
		List<List<int>> car_targets = new List<List<int>>();
		/* GENERATE PERMUTATION OF INTERESTING NODES */
		List<List<int>> best_path = new List<List<int>>();

		double best_cost = 99999999999;

		/* FIX THE START POINT = 0 IN A * */
		int NUMBER_OF_CARS = 3;
		int start_idx = 0;
		int goal_idx = 1; //= goalN.getId();
		List<int> path = new List<int>();


		List<int> point_of_interest = new List<int>();

		foreach (var car in friends)
		{
			point_of_interest.Add(DroneGraph.FindClosestNode(car.transform.position, DroneGraph).getId());
		}
		foreach (var car in enemies)
		{
			point_of_interest.Add(DroneGraph.FindClosestNode(car.transform.position, DroneGraph).getId());
		}
		int time = 0;
		Debug.Log("Printing points of interest");
		var strings = string.Join(", ", point_of_interest);
		Debug.Log(strings);
		permutor p = new permutor();
		List<List<int>> this_path = new List<List<int>>();
		while (time < 1)
		{
			car_targets = p.permute(friends.Length, point_of_interest);

			Debug.Log("Printing car_targets");
			foreach (var l in car_targets) {
				strings = string.Join(", ", l);
				Debug.Log(strings);
				}
			List<double> this_cost = new List<double>();
			this_path.Clear();
			for (int c = 0; c < NUMBER_OF_CARS; c++)
			{ //For all the car 
				path.Clear();
				if (car_targets[c].Count == 1)
				{
					path.Add(car_targets[c][0]);
				}
				for (int i = 0; i < car_targets[c].Count - 1; i++)
				{        //for all the interesting points of that car   

					start_idx = car_targets[c][i];
					goal_idx = car_targets[c][i + 1];
					ASuperStar(DroneGraph, start_idx, goal_idx);
					path.AddRange(getBestPath(DroneGraph, start_idx, goal_idx)); // ADD To the real path, the path to get the ith point 

				}
				this_cost.Add(computePathCost(DroneGraph, path));
				this_path.Add( new List<int>(path)); //copy the path
				//Debug.Log(this_cost[c]);
			}
			double real_cost = this_cost.Max();
			Debug.Log("Cost for this path:" + real_cost.ToString() + " : "+ this_cost.Sum().ToString() +" "+ (real_cost- this_cost.Sum()).ToString());
			foreach (var l in this_path)
			{
				strings = string.Join(", ", l);
				Debug.Log(strings);
			}
			if (real_cost < best_cost)
			{
				Debug.Log("Found a better solution");
				best_cost = real_cost;
				best_path = new List<List<int>>(this_path);
			}
			time++;

			// Block of code to hopefully remove consecutive duplicates of nodes in the path that might cause issues.
			foreach (var List in best_path){
				for (int i = List.Count - 2; i >= 0; i--){
					if(List[i] == List[i+1]){
						List.RemoveAt(i);
					}
				}
			}


			
		}

		// Block of code to hopefully remove consecutive duplicates of nodes in the path that might cause issues.
		foreach (var List in best_path){
			for (int i = List.Count - 2; i >= 0; i--){
				if(List[i] == List[i+1]){
					List.RemoveAt(i);
				}
			}
		}


		
		int j = 0;
		foreach ( var car in friends)
		{
			
			//car.GetComponent<CarAI3>().trajectory = car.GetComponent<CarAI3>().myController.makeTrajectory(myRealPath, maxVel / 2.236f, maxAcc); ;
			//car.GetComponent<CarAI3>().lastPointInPath = best_path[j][0];
			car.GetComponent<CarAI3>().my_path = pathHelp(DroneGraph, best_path[j]);
			car.GetComponent<CarAI3>().my_path_length = best_path[j].Count;
			car.GetComponent<CarAI3>().my_mask = mask;
			if (j == 0){
				car.GetComponent<CarAI3>().color = "Cyan";
				car.GetComponent<CarAI3>().skipper = 0;
			}
			else if (j == 1){
				car.GetComponent<CarAI3>().color = "Yellow";
				car.GetComponent<CarAI3>().skipper = 300;
			}
			else{
				car.GetComponent<CarAI3>().color = "Blue";
				car.GetComponent<CarAI3>().skipper = 1000;
			}
			j++;
		}


		/*SUM COST OF A STAR */


		Vector3 old_wp = DroneGraph.getNode(best_path[0][0]).getPosition();
		
		foreach (var wp in best_path[0])
		{
			//Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
			Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.cyan, 10000f);
			old_wp = DroneGraph.getNode(wp).getPosition();
		}

		old_wp = DroneGraph.getNode(best_path[1][0]).getPosition();
		foreach (var wp in best_path[1])
		{
			//Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
			Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.yellow, 10000f);
			old_wp = DroneGraph.getNode(wp).getPosition();
		}
		old_wp = DroneGraph.getNode(best_path[2][0]).getPosition();

		foreach (var wp in best_path[2])
		{
			//Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
			Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.blue, 10000f);
			old_wp = DroneGraph.getNode(wp).getPosition();
		}



		my_path = pathHelp(DroneGraph, path);


	}
	List<Node> pathHelp(Graph G, List<int> idList)
	{
		List<Node> nodeList = new List<Node>();
		foreach (int i in idList)
		{
			nodeList.Add(G.getNode(i));
		}
		return nodeList;
	}

	bool position_collision(float Margin, Vector3 position)
	{
		float[,] radiusHelpMatrix = new float[,] { { Margin, Margin }, { Margin, -Margin }, { -Margin, Margin }, { -Margin, -Margin }, { Margin, 0.0f }, { -Margin, 0.0f }, { 0.0f, Margin }, { 0.0f, -Margin } };
		for (int a = 0; a < 8; a = a + 1)
		{
			int i = terrain_manager.myInfo.get_i_index(position.x + radiusHelpMatrix[a, 0]);
			int j = terrain_manager.myInfo.get_j_index(position.z + radiusHelpMatrix[a, 1]);

			if (terrain_manager.myInfo.traversability[i, j] == 1.0f)
			{
				return true;
			}
		}
		return false;
	}

	bool IsCollidingOnEdge(Vector3 from, Vector3 to)
	{
		from.y = 3;
		to.y = 3;

		RaycastHit rayHit;
		bool hit = Physics.SphereCast(from, Margin, to - from, out rayHit, Vector3.Distance(from, to),mask);
		if (hit)
		{
			return true;
		}
		return false;
	}


	Vector3 Pseudo_random(float Margin)
	{

		if (randomTimer == 10)
		{
			randomTimer = 0;
			return goal_pos;
		}
		randomTimer++;
		float cordx = 50.0f; float cordz = 50.0f;
		bool foundNonColidingPos = false;
		float[,] radiusHelpMatrix = new float[,] { { Margin, Margin }, { Margin, -Margin }, { -Margin, Margin }, { -Margin, -Margin }, { Margin, 0.0f }, { -Margin, 0.0f }, { 0.0f, Margin }, { 0.0f, -Margin } };
		while (foundNonColidingPos == false)
		{
			cordx = UnityEngine.Random.Range(terrain_manager.myInfo.x_low, terrain_manager.myInfo.x_high);
			cordz = UnityEngine.Random.Range(terrain_manager.myInfo.z_low, terrain_manager.myInfo.z_high);
			bool traversbel = true;
			for (int a = 0; a < 8; a = a + 1)
			{
				int i = terrain_manager.myInfo.get_i_index(cordx + radiusHelpMatrix[a, 0]);
				int j = terrain_manager.myInfo.get_j_index(cordz + radiusHelpMatrix[a, 1]);

				if (terrain_manager.myInfo.traversability[i, j] == 1.0f)
				{
					traversbel = false;

					break;
				}
			}
			foundNonColidingPos = traversbel;
		}

		//Debug.Log("x: "+cordx.ToString()+"| z: "+cordz.ToString());

		return new Vector3(cordx, 0, cordz);

	}
	public float computeAngle2(Node _A, Node _B, Node _C)
	{
		Vector3 aa = _A.getPosition() - _B.getPosition();
		Vector3 bb = _C.getPosition() - _B.getPosition();
		return Vector3.Angle(aa, bb);
	}

	public double computePathCost(Graph G, List<int> _path)
	{
		double cost = 0;
		for (int i = 0; i < _path.Count - 1; i++)
		{
			cost += Vector3.Distance(G.getNode(_path[i]).getPosition(), G.getNode(_path[i+1]).getPosition());

		}
		return cost;

	}

	public void RRG(int max_nodes, Graph G)
	{
		float edgeLength = 6.0f;
		float nodeMinDistance = 2.5f;
		float addEdgeMaxLength = 8.0f;


		int max_iter = 10000;
		Node close_node = null;
		Vector3 new_coord = new Vector3(0, 0, 0);
		for (int i = 0; i < max_nodes && max_iter > 0; i++)
		{
			//For all the new nodes;

			bool found = false;
			for (max_iter = 10000; !found && max_iter > 0; max_iter--)
			{
				Vector3 goal = Pseudo_random(Margin); //Find random point
				close_node = G.FindClosestNode(goal, G); //Find closest node
				float distance = Vector3.Distance(close_node.getPosition(), goal); //And compute the distance
				if (distance > edgeLength)
				{  //skip if B too close 
					new_coord = Vector3.Lerp(close_node.getPosition(), goal, edgeLength / distance);
					//distance = Vector3.Distance(new_coord, G.FindClosestNode(new_coord, G).getPosition());
					//if (distance > nodeMinDistance)
					//{  //skip if C too close to another point WE DONT FUCKING NEED THIS CHECK, BECAUSE CLOSE_NODE WILL ALWAISE BE THE CLOSEST TO  B
					if (!position_collision(Margin, new_coord) && !IsCollidingOnEdge(close_node.getPosition(), new_coord))
					{
						found = true;
					}
					//}
				}
			}
			//Debug.Log(max_iter);


			int idx = G.addNode(new Node(new_coord));
			G.addEdge(idx, close_node.getId());
			for (int j = 0; j < G.getSize() - 1; j++)
			{
				Node temp = G.getNode(j);
				float checkDistance = Vector3.Distance(temp.getPosition(), G.getNode(idx).getPosition());
				if (checkDistance < addEdgeMaxLength && j != idx && !IsCollidingOnEdge(temp.getPosition(), G.getNode(idx).getPosition()))
				{
					G.addEdge(j, idx);
				}
			}
		}
	}

	public void dfs_step(Graph G, int position, List<List<int>> paths, List<int> thisPath, int idx_goal)
	{

		G.getNode(position).setColor(1);
		thisPath.Add(position);
		if (counter >= 100)
		{
			return;
		}
		if (position == idx_goal)
		{
			counter++;
			Debug.Log(counter);
			var strings = string.Join(", ", thisPath);
			Debug.Log(strings);

			List<int> good_path = new List<int>(thisPath);
			paths.Add(good_path);
			thisPath.RemoveAt(thisPath.Count - 1);
			G.getNode(position).setColor(0);
			return;
		}
		List<int> child = G.getAdjList(position);
		for (int i = 0; i < child.Count; i++)
		{
			if (G.getNode(child[i]).getColor() == 0) //not visited
			{
				dfs_step(G, child[i], paths, thisPath, idx_goal);
			}
		}
		thisPath.RemoveAt(thisPath.Count - 1);
		G.getNode(position).setColor(0);

	}
	public List<List<int>> dfs(Graph G, int idx_goal)
	{

		List<List<int>> paths = new List<List<int>>();
		List<int> thisPath = new List<int>();
		dfs_step(G, 0, paths, thisPath, idx_goal);


		return paths;
	}

	public List<int> getBestPath(Graph G, int idx_start, int idx_goal)
	{
		List<int> path = new List<int>();
		path.Add(idx_goal);
		int idx = idx_goal;
		//Debug.Log("Following path from A star");
		while (idx != idx_start)
		{
			if (G.getNode(idx).getParent() == -1)
			{
				Debug.Log("I am in node" + idx.ToString());
			}
			idx = G.getNode(idx).getParent();
			path.Add(idx);
			if (path.Count > 10000)
			{
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

	public void killFuckers(Graph G)
	{
		int i, j, x;
		List<int> adj = new List<int>();
		for (i = 0; i < G.getSize(); i++)
		{
			adj = G.getAdjList(i);

			for (j = 0; j < adj.Count; j++)
			{
				//Debug.Log(Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition()));

				if (Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition()) > 10)
				{
					//Debug.Log("KILLLL!");
					x = adj[j];
					G.getAdjList(i).Remove(x);
					G.getAdjList(x).Remove(i);
					j--;
				}
			}
		}
	}
	public void ASuperStar(Graph G, int start_pos, int idx_goal)
	{
		priorityQueue Q = new priorityQueue();

		for( int i = 0; i< G.getSize(); i++)
		{
			G.setColorOfNode(i, 0);
			G.getNode(i).setParent(-1);
		}
		G.setColorOfNode(start_pos, 1);
		int best_node;
		float best_cost;
		//Debug.Log("RUNNING A STAR!"+ start_pos.ToString() +" " + idx_goal.ToString());

		float total_cost;
		Q.enqueue(start_pos, 0);
		

		while (Q.getSize() != 0)
		{
			best_node = Q.dequeue();
			best_cost = Q.getCost(best_node);
			//Delete node
			Q.removeNode(best_node);

			if (idx_goal == best_node)
			{
				//Debug.Log("path found");
				return;
			}

			foreach (int child in G.getAdjList(best_node))
			{
				total_cost = computeCost(G, best_node, child, idx_goal) + best_cost;

				if (Q.isInQueue(child))
				{
					if (Q.getCost(child) > total_cost)
					{
						G.getNode(child).setParent(best_node);
						Q.updateCost(child, total_cost);
					}
				}
				else
				{
					if (G.getNode(child).getColor() == 0)
					{
						Q.enqueue(child, total_cost);
						G.getNode(child).setParent(best_node);
						G.getNode(child).setColor(1);
					}
				}


			}
		}

		Debug.Log("path not found!!!!");

	}

	public class priorityQueue
	{
		List<int> values;
		List<float> priority;

		public priorityQueue()
		{
			values = new List<int>();
			priority = new List<float>();
		}
		public void enqueue(int _value, float _p)
		{
			values.Add(_value);
			priority.Add(_p);
		}
		public int getSize()
		{
			return values.Count;
		}
		public int dequeue()
		{
			int best_idx = priority.IndexOf(priority.Min());
			int best_node = values[best_idx];


			return best_node;
		}
		public void removeNode(int node)
		{
			int node_idx = values.IndexOf(node);
			priority.RemoveAt(node_idx);
			values.RemoveAt(node_idx);
		}
		public float getCost(int node)
		{
			int idx = values.IndexOf(node);
			return priority[idx];
		}
		public void updateCost(int node, float p)
		{
			int idx = values.IndexOf(node);
			priority[idx] = p;
		}
		public bool isInQueue(int node)
		{
			int idx = values.IndexOf(node);
			if (idx == -1) { return false; }
			return true;
		}
	}

	public float computeCost(Graph G, int parent, int child, int goal)
	{
		//REAL COST:
		float real_cost;
		float h_cost;
		float actual_angle = 0;
		float best_angle = 0;
		float max_speed = 15;
		float alpha = 1 / 10;
		float zero = 0;
		float k = 1/zero ;
		foreach (var n in G.getAdjList(child))
		{
			actual_angle = computeAngle2(G.getNode(parent), G.getNode(child), G.getNode(n));
			if (actual_angle > best_angle)
			{
				best_angle = actual_angle;
			}
		}
		//max_speed = 1 / (1 + ((180 - best_angle) * alpha) * ((180 - best_angle) * alpha) * ((180 - best_angle) * alpha) * ((180 - best_angle) * alpha));
		//max_speed = 15;
		real_cost = Vector3.Distance(G.getNode(parent).getPosition(), G.getNode(child).getPosition()); // max_speed; /// max_speed;

		
		
		
		//real_cost = Math.Max(real_cost, k * (real_cost - 10));

		RaycastHit rayHit;
		bool hit = Physics.SphereCast(G.getNode(child).getPosition(), Margin, G.getNode(goal).getPosition() - G.getNode(child).getPosition(), out rayHit, Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition()),mask);
		if (!hit)
		{
			h_cost = 0;
		}
		else
		{
			h_cost = Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition());
		}


		return 3 * real_cost + h_cost + (200f / G.getNode(child).getDistanceToWall());

	}

	public void computeDiStanceToWall(Graph G)
	{
		RaycastHit hit;

		List<Vector3> radiusHelpMatrix = new List<Vector3>();

		radiusHelpMatrix.Add(new Vector3(-1f, -1f, -1f));
		radiusHelpMatrix.Add(new Vector3(1f, 1f, 1f));
		radiusHelpMatrix.Add(new Vector3(1f, 1f, -1f));
		radiusHelpMatrix.Add(new Vector3(1f, -1f, 1f));
		radiusHelpMatrix.Add(new Vector3(1f, -1f, -1f));
		radiusHelpMatrix.Add(new Vector3(-1f, 1f, 1f));
		radiusHelpMatrix.Add(new Vector3(-1f, 1f, -1f));
		radiusHelpMatrix.Add(new Vector3(-1f, -1f, 1f));

		for (int i = 0; i < G.getSize(); i++)
		{
			float minDistance = 5000f;
			float actualDistance;
			for (int j = 0; j < radiusHelpMatrix.Count; j++)
			{
				Physics.SphereCast(G.getNode(i).getPosition(), 2, radiusHelpMatrix[j], out hit, 50f,mask);
				actualDistance = hit.distance;
				if (actualDistance != 0)
				{
					if (minDistance > actualDistance)
					{
						minDistance = actualDistance;
					}
				}
			}
			G.getNode(i).setDistanceToWall(minDistance);
			//Debug.Log(minDistance);
		}
	}
}
