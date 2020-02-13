using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        int counter=0;
        //SphereCollider droneCollider;
        private float Margin=0;
        private float sideMargin=0;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private Graph DroneGraph;
        //All edges have same length:
        public float edgeLength = 5.0f;
        public float edgeMinDist = 0.1f;
        public float addEdgeMaxLength =  10.0f;
        List<Node> my_path = new List<Node>();
        int randomTimer = 0;
        Vector3 goal_pos;
        int lastPointInPath=0;
        public float newAngle=0; 
        private Boolean backing=false;

        private void Start()
        {   
            

            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            Transform ColliderBottom=m_Car.transform.Find("Colliders").Find("ColliderBottom");
            Vector3 scale = ColliderBottom.localScale;
            Margin = scale.z/2;
            sideMargin = scale.x/2;
            

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
            for (int i=0; i< DroneGraph.getSize(); i++){
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
                    for (int j = 0; j < DroneGraph.getAdjList(i).Count; j++){
                        Debug.DrawLine(DroneGraph.getNode(i).getPosition(), DroneGraph.getNode(DroneGraph.getAdjList(i)[j]).getPosition(), Color.red, 100f);
                        //Debug.Log(DroneGraph.getNode(i).getPosition() +" - "+ DroneGraph.getNode(j).getPosition());
                }
                //Debug.Log("Adj list of node " + i.ToString());
                //var strings = string.Join(", ", DroneGraph.getAdjList(i));
                //Debug.Log(strings);

            }
            Node goalN=DroneGraph.FindClosestNode(goal_pos, DroneGraph);



	/* GENERATE INITIAL POSITION NODES FOR CAR (FIND THE CLOSEST NODE TO THE CAR, THERE IS ARLEADY A FUNCTION FOR THAT)*/
	List<List<int>> car_targets = new List<List<int>> ();
	/* GENERATE PERMUTATION OF INTERESTING NODES */
	List<List<int>> best_path = new List<List<int>>();

    double best_cost=99999999999;

	/* FIX THE START POINT = 0 IN A * */
	    int NUMBER_OF_CARS=3; 
	    int start_idx = 0;
            int goal_idx = goalN.getId();
            List<int> path = new List<int>();
            //paths=dfs(DroneGraph, goal_idx);
     /*RUN MULTIPLE TIME  A_START FOR ALL THE CAR, FOR ALL THE INTERESTING POINTS */
     time=0;
        while(time<100000){
            double this_cost=0;
            List<List<int>> this_path = new List<List<int>>();
	        for ( int c=0; c<NUMBER_OF_CARS; c++){ //For all the car 
		        for(int i=0;i<car_targets[c].size()-1;i++){        //for all the interesting points of that car   
                
                    start_idx = car_targets[c][i];
                    goal_idx = car_targets[c][i+1];
			        ASuperStar(DroneGraph,start_idx, goal_idx);
                    path=path.Concat(getBestPath(DroneGraph,start_idx, goal_idx)).ToList(); // ADD To the real path, the path to get the ith point 

                }
                this_cost += computePathCost(DroneGraph, path);
                this_path[c] = new List<int>(path) //copy the path
              }
              if(this_cost<best_cost){
                    best_cost = this_cost;
                    best_path = new List<List<int>>(this_path);
              }
              time++
         }
	/*SUM COST OF A STAR */


            Vector3 old_wp = start_pos;
            Debug.Log("Printing space now");
            foreach (var wp in path){
                //Debug.Log(Vector3.Distance(old_wp, DroneGraph.getNode(wp).getPosition()));
                Debug.DrawLine(old_wp, DroneGraph.getNode(wp).getPosition(), Color.cyan, 100f);
                old_wp = DroneGraph.getNode(wp).getPosition();
            }



        
            my_path = pathHelp(DroneGraph,path);


            
        }
        
        List<Node> pathHelp(Graph G,List<int> idList){
            List<Node> nodeList = new List<Node>();
            foreach(int i in idList){
                nodeList.Add(G.getNode(i));
            }
            return nodeList;
        }

        private void FixedUpdate(){
            // Execute your path here
            // ...

            CarController controller=transform.GetComponent<CarController>();
           
            if(lastPointInPath==my_path.Count-3){
                Vector3 carToTarget = transform.InverseTransformPoint(goal_pos);
                float newSteer = (carToTarget.x / carToTarget.magnitude);
                float newSpeed = 1f;
                m_Car.Move(newSteer,newSpeed, newSpeed, 0);

                
            }else{
                float curveAngel=Vector3.Angle(my_path[lastPointInPath].getPosition()-my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+1].getPosition()-my_path[lastPointInPath+2].getPosition()); 
                for(int i=lastPointInPath;i<my_path.Count;i=i+1){
                    if (3.0f+0.04*(controller.CurrentSpeed)>=Vector3.Distance(my_path[i].getPosition(),transform.position)){
                        lastPointInPath=i;
                    }
                }
                if(lastPointInPath!=my_path.Count-1){
                    
                    Vector3 target=my_path[lastPointInPath+1].getPosition();
                    float distanceToTargetTemp = Vector3.Distance(transform.position,target);
                    int targetId=0;
                    for(int i=lastPointInPath+2;i<my_path.Count;i=i+1){
                        float newDistance=Vector3.Distance(transform.position,my_path[i].getPosition());
                        if(distanceToTargetTemp>newDistance){
                            distanceToTargetTemp=newDistance;
                            target=my_path[i].getPosition();
                            targetId=i;
                        }
                    }
                    Vector3 carToTarget = transform.InverseTransformPoint(target);
                    float newSteer = (carToTarget.x / carToTarget.magnitude);
                    float newSpeed = 1f;
                    float handBreak = 0f;
                    Vector3 steeringPoint = new Vector3(0,0,1);
                    steeringPoint=(transform.rotation * steeringPoint);

                    float breakingDistance = (controller.CurrentSpeed*controller.CurrentSpeed)/(2);
                    float distanceToTarget = Vector3.Distance(transform.position,target);

                    RaycastHit rayHit;
                    bool hitBreak = Physics.SphereCast(transform.position,sideMargin, steeringPoint,out rayHit, 0.08f*breakingDistance);
                    bool hitBack = Physics.SphereCast(transform.position,sideMargin, steeringPoint,out rayHit, Margin);
                    bool hitContinueBack = Physics.SphereCast(transform.position,sideMargin, steeringPoint,out rayHit, Margin*3);
                    newAngle=Vector3.Angle(transform.position-my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+1].getPosition()-my_path[lastPointInPath+2].getPosition()); 
                    if(controller.AccelInput==0 && backing==false){
                        newSpeed = 1f/1+newAngle;
                    }

                    if(hitBack){
                        backing=true;
                        newSpeed=-1f;
                        if(controller.BrakeInput>0 && controller.AccelInput<=0){
                            newSteer=-newSteer;
                        }
                    }else if(hitBreak && backing==false){
                        newSpeed=-1;
                        handBreak = 1;
                        print("yes");

                    }
                    if(hitContinueBack && controller.AccelInput>=0 && backing==false){
                        newSteer= newSteer*2;
                        //print("nope");
                    } 
                    if(hitContinueBack && backing==true ){
                        newSpeed=-1f;
                        newSteer=-newSteer;
                        //print("yes");
                    }else{
                        backing=false;
                    }

                    if(controller.CurrentSpeed>150){
                        newSpeed=0;
                    }
                    m_Car.Move(newSteer,newSpeed, newSpeed, handBreak);
                    
                    
                    

                    //Debug.DrawLine(transform.position,my_path[lastPointInPath+1].getPosition(), Color.black);
                    //Debug.DrawLine(my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+2].getPosition(), Color.white);
                    Debug.DrawLine(transform.position,target, Color.black);
                    Debug.DrawLine(transform.position+steeringPoint*0.08f*breakingDistance,transform.position, Color.white);

                }
            }
            // this is how you control the car
            //m_Car.Move(0f, 0f, 0f, 0f);
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
            bool hit = Physics.SphereCast(from, Margin ,to - from, out rayHit,Vector3.Distance(from,to));
            if (hit) { 
                return true; }
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
            float[,] radiusHelpMatrix = new float[,] { { Margin, Margin }, { Margin, -Margin }, { -Margin, Margin}, { -Margin, -Margin }, { Margin, 0.0f }, { -Margin, 0.0f }, { 0.0f, Margin }, { 0.0f, -Margin } };
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

        public class Node
        {

            private int id;
            private double speedX, speedZ, AccellerationX, AccellerationZ, theta;
            private Vector3 position;
            private float x, z;
            private int color;
            private int parent;
            private float distanceToWall;
        
            public float getDistanceToWall()
            {
                return distanceToWall;
            }
            public void setDistanceToWall(float _d)
            {
                distanceToWall = _d;
            }
            
            public int getParent()
            {
                return parent;
            }
            public void setParent(int p)
            {
                parent = p;
            }
            public void setColor(int _c)
            {
                color = _c;
            }
            public int getColor() { return color; }
            public Vector3 getPosition()
            {
                return position;
            }

            public float getPositionX()
            {
                return position.x;
            }
            public float getPositionZ()
            {
                return position.z;
            }
            public int getId()
            {
                return id;
            }
            public Node(float _x, float _z)
            {
                x = _x;
                z = _z;
                position = new Vector3(_x, 0, _z);
                id = -1;
            }
            public Node(Vector3 _position)
            {
                position = _position;
                id = -1;
            }
            public Node()
            {
                x = 0;
                z = 0;
                id = -1;
            }
            public void setId(int _id)
            {
                id = _id;
            }
            public void setPositionX(float _x)
            {
                x = _x;
                position.x = _x;
            }
            public void setPositionZ(float _z)
            {
                z = _z;
                position.z = _z;
            }
            public void setTheta(double _theta)
            {
                theta = _theta;
            }
            public double getTheta()
            {
                return theta;
            }
        }

        public class Graph
        {
            Dictionary<int, Node> nodes;
            Dictionary<int, List<int>> adjList;
            List<int> endNodes;

            int size;

            public Graph() {

            nodes =  new Dictionary<int, Node>();
            adjList =  new Dictionary<int, List<int>>();
            endNodes = new List<int>();
            size = 0;
            }

            // The following constructor has parameters for two of the three 
            // properties. 

            public void setColorOfNode(int _idx, int color)
            {
                nodes[_idx].setColor(color);
            }
            public int getColorOfNode(int _idx)
            {
                return nodes[_idx].getColor();
            }
            public Dictionary<int, Node> getNodes()
            {
                return nodes;
            }
            public int getSize()
            {
                return size;
            }
            public int addNode(Node _newNode)
            {
                int id = size++;
                _newNode.setId(id);
                _newNode.setColor(0);
                nodes.Add(id, _newNode);
                adjList.Add(id, new List<int>());
                return id;
            }
            public int addNode(Node _newNode, List<int> _adjList)
            {
                int id = size++;
                nodes.Add(id, _newNode);
                adjList.Add(id, _adjList);
                return id;
            }
            public Node getNode(int _id)
            {
                return nodes[_id];
            }
            public List<int> getAdjList(int _id)
            {
                return adjList[_id];
            }
            public void setAdjList(int _id, List<int> _adjList)
            {
                adjList[_id]= _adjList;
            }
            public void addEdge(int _idA, int _idB)
            {
                List<int> actualList;

                actualList = adjList[_idA];
                if (!actualList.Contains(_idB)) { 
                    actualList.Add(_idB);
                    setAdjList(_idA, actualList);
                }
                actualList = adjList[_idB];
                if (!actualList.Contains(_idA))
                {
                    actualList.Add(_idA);
                    setAdjList(_idB, actualList);
                }
            }

            public double computeAngle(Node _A, Node _B, Node _C)
            {
                double a_x = _A.getPositionX() - _B.getPositionX();
                double a_z = _A.getPositionZ() - _B.getPositionZ();
                double b_x = _B.getPositionX() - _C.getPositionX();
                double b_z = _B.getPositionX() - _C.getPositionZ();
                double num = a_x * b_x + a_z * b_z;
                double den = Math.Sqrt(a_x * a_x + a_z * a_z) * Math.Sqrt(b_x * b_x + b_z * b_z);
                double angle = Math.Acos(num / (den + 0.000001f));
                return angle;
            }


            public void setPathTheta(List<int> _path)
            {
                int A, B, C = 0;
                A = _path[0];
                nodes[A].setTheta(0);

                for (int i = 1; i < _path.Count - 1; i++)
                {
                    A = _path[i - 1];
                    B = _path[i];
                    C = _path[i + 1];
                    nodes[B].setTheta(computeAngle(nodes[A], nodes[B], nodes[C]));
                }
                nodes[C].setTheta(0);

            }
            public double computePathCost(Graph G,List<int> _path)
            {
                double cost = 0;
                for( int i=0;i<_path.size()-1;i++){
                    cost += Vector3.Distance(G.getNode(_path[i]).getPosition(), G.getNode(_path[i]).getPosition())
                }
                return cost;

            }


            public Node FindClosestNode(Vector3 target,Graph G)
            {

                Node temp;
                Node closest = nodes[0];//root
                float closestDistance = Vector3.Distance(closest.getPosition(), target);
                float checkDistance = 0f;
                
                for (int i=0; i < G.getSize(); i++)
                {
                    temp = G.getNode(i);
                    checkDistance = Vector3.Distance(temp.getPosition(), target);
                    if (checkDistance < closestDistance)
                    {
                        closestDistance = checkDistance;
                        closest = temp;
                    }
                }
                return closest;
            }


        }
        public float computeAngle2(Node _A, Node _B, Node _C)
        {
            Vector3 aa = _A.getPosition() - _B.getPosition();
            Vector3 bb = _C.getPosition() - _B.getPosition();
            return Vector3.Angle(aa, bb);
        }

        public void RRG(int max_nodes,Graph G)
        {
            //float edgeLength = 5.0f;
            //float nodeMinDistance = 2.5f;
            //float addEdgeMaxLength = 10.0f;
            print(edgeLength);
            print(addEdgeMaxLength);

            int max_iter=10000;
            Node close_node=null;
            Vector3 new_coord= new Vector3(0,0,0);
            for (int i = 0; i<max_nodes && max_iter>0; i++)
            {
                //For all the new nodes;

                bool found = false;
                for (max_iter = 10000; !found && max_iter>0; max_iter--)
                {
                    Vector3 goal = Pseudo_random(Margin); //Find random point
                    close_node = G.FindClosestNode(goal,G); //Find closest node
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
                for ( int j =0; j<G.getSize()-1; j++)
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

        public void dfs_step(Graph G, int position, List<List<int>> paths,List<int> thisPath, int idx_goal)
        {
            
            G.getNode(position).setColor(1);
            thisPath.Add(position);
            if (counter>=100)
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
            for(int i = 0; i < child.Count; i++)
            {
                if (G.getNode(child[i]).getColor() == 0) //not visited
                {
                    dfs_step(G, child[i], paths, thisPath, idx_goal);
                }
            }
            thisPath.RemoveAt(thisPath.Count - 1);
            G.getNode(position).setColor(0);

        }
        public List<List<int>>  dfs(Graph G,int idx_goal)
        {

            List<List<int>> paths = new List<List<int>>();
            List<int> thisPath = new List<int>();
            dfs_step(G, 0, paths,thisPath, idx_goal);


            return paths;
        }

        public List<int> getBestPath(Graph G,int, idx_start,int idx_goal)
        {
            List<int> path = new List<int>();
            path.Add(idx_goal);
            int idx = idx_goal;
            while( idx != idx_start)
            {
                idx = G.getNode(idx).getParent();
                path.Add(idx);
            }
            path.Reverse();
            return path;
        }

        public void killFuckers(Graph G)
        {
            int i,j,x;
            List<int> adj = new List<int>();
            for (i = 0;i  < G.getSize(); i++)
            {
                adj = G.getAdjList(i);

                for (j=0;j<adj.Count;j++)
                {
                    Debug.Log(Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition()));

                    if (Vector3.Distance(G.getNode(i).getPosition(), G.getNode(adj[j]).getPosition())> 10)
                    {
                        Debug.Log("KILLLL!");
                        x = adj[j];
                        G.getAdjList(i).Remove(x);
                        G.getAdjList(x).Remove(i);
                        j--;
                    }
                }
            }
        }
        public void ASuperStar(Graph G,int start_pos int idx_goal)
        {
            priorityQueue Q = new priorityQueue();


            int best_node;
            float best_cost;


            float total_cost;
            Q.enqueue(start_pos, 0);

            while(Q.getSize()!=0)
            {
                best_node = Q.dequeue();
                best_cost = Q.getCost(best_node);
                //Delete node
                Q.removeNode(best_node);

                if (idx_goal == best_node){
                    return;
                }

                foreach (int child in G.getAdjList(best_node)){
                    total_cost = computeCost(G, best_node, child, idx_goal) + best_cost;

                    if (Q.isInQueue(child))
                    {
                        if (Q.getCost(child) > total_cost)
                        {
                            G.getNode(child).setParent(best_node);
                            Q.updateCost(child,total_cost);
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
            float best_angle=0;
            float max_speed = 15;
            float alpha = 1 / 10;
            
            foreach (var n in G.getAdjList(child))
            {
                actual_angle = computeAngle2(G.getNode(parent), G.getNode(child), G.getNode(n));
                if (actual_angle > best_angle)
                {
                    best_angle = actual_angle;
                }
            }
            max_speed = 1 / (1 +  ((180 - best_angle)*alpha) * ((180 - best_angle) * alpha)*((180 - best_angle)*alpha) * ((180 - best_angle) * alpha));
            //max_speed = 15;
            real_cost = Vector3.Distance(G.getNode(parent).getPosition(), G.getNode(child).getPosition())/max_speed; /// max_speed;


            RaycastHit rayHit;
            bool hit = Physics.SphereCast(G.getNode(child).getPosition(), Margin, G.getNode(goal).getPosition()- G.getNode(child).getPosition(), out rayHit, Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition()));
            if (!hit)
            {
                h_cost = 0;
            }
            else
            {
                h_cost = Vector3.Distance(G.getNode(child).getPosition(), G.getNode(goal).getPosition());
            }


            return 3*real_cost+h_cost + (200f / G.getNode(child).getDistanceToWall());

        }

        public void computeDiStanceToWall(Graph G)
        {
            RaycastHit hit;
            float radiusMargin = 1f;

            List<Vector3> radiusHelpMatrix = new List<Vector3>();

            radiusHelpMatrix.Add(new Vector3(-1f, -1f, -1f));
            radiusHelpMatrix.Add(new Vector3(1f, 1f, 1f));
            radiusHelpMatrix.Add(new Vector3(1f, 1f, -1f));
            radiusHelpMatrix.Add(new Vector3(1f, -1f, 1f));
            radiusHelpMatrix.Add(new Vector3(1f, -1f, -1f));
            radiusHelpMatrix.Add(new Vector3(-1f, 1f, 1f));
            radiusHelpMatrix.Add(new Vector3(-1f, 1f, -1f));
            radiusHelpMatrix.Add(new Vector3(-1f, -1f, 1f));

            for (int i =0;i < G.getSize(); i++)
            {
                float minDistance = 5000f;
                float actualDistance; 
                for (int j = 0; j < radiusHelpMatrix.Count; j++)
                {
                    Physics.SphereCast(G.getNode(i).getPosition(),2, radiusHelpMatrix[j], out hit, 50f);
                    actualDistance=hit.distance;
                    if (actualDistance != 0)
                    {
                        if (minDistance > actualDistance)
                        {
                            minDistance = actualDistance;
                        }
                    }
                }
                G.getNode(i).setDistanceToWall(minDistance);
                Debug.Log(minDistance);
            }
        }

        public List<List<int>> generatePermutation(int n , List<int> v){

            List<int> cars;

            

        }


    }
}
