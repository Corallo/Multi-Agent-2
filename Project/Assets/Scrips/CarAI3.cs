using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MyGraph;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI3 : MonoBehaviour
    {
        public CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

		public String color = "";

		public int my_path_length;

        public LayerMask my_mask;

        public int skipper;

        int counter = 0;
        //SphereCollider droneCollider;
        private float Margin = 0;
        private float sideMargin = 0;

        private Graph DroneGraph;
        //All edges have same length:
        public float edgeLength = 10.0f;
        public float edgeMinDist = 0.1f;
        public float addEdgeMaxLength = 12.0f;
        public List<Node> my_path = new List<Node>();
        int randomTimer = 0;
        Vector3 goal_pos;
        public int lastPointInPath = 0;
        public float newAngle = 0;
        private Boolean backing = false;

        private void Start()
        {
            // get the car controller
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            foreach (GameObject obj in enemies)
            {
                Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
            }
            m_Car = GetComponent<CarController>();
            Debug.Log("Printing the path!");



            // Plan your path here
            // ...
        }

        private void FixedUpdate()
        {


            //if (my_path.Count == 0 || my_path.Count == 1) { return; }
            /*foreach (Node n in my_path)
            {
                Debug.Log(n.getId());
            }*/
            CarController controller = transform.GetComponent<CarController>();
            /*if (lastPointInPath == my_path.Count - 3)
            {
				Debug.Log("Goal position in sight!");
				Debug.Log(goal_pos);
                Vector3 carToTarget = transform.InverseTransformPoint(goal_pos);
                float newSteer = (carToTarget.x / carToTarget.magnitude);
                float newSpeed = 1f;
                m_Car.Move(newSteer, newSpeed, newSpeed, 0);
            }*/

            if (skipper > 0)
            {
                skipper--;
                m_Car.Move(0f, 1f, 1f, 0f);
                return;

            }

			if (my_path.Count == 0 || my_path.Count == 1) {
                m_Car.Move(0f, 1f, 1f, 0f);
                //return; 
            }
            
            else
            {
                
                //float curveAngel = Vector3.Angle(my_path[lastPointInPath].getPosition() - my_path[lastPointInPath + 1].getPosition(), my_path[lastPointInPath + 1].getPosition() - my_path[lastPointInPath + 2].getPosition());
                /*int oldLastPointInPath = lastPointInPath;
                for (int i = lastPointInPath; i < my_path.Count; i = i + 1)
                {
                    if (3.0f + 0.04 * (controller.CurrentSpeed) >= Vector3.Distance(my_path[i].getPosition(), transform.position)) //0.04
                    //if (10.0f >= Vector3.Distance(my_path[i].getPosition(), transform.position)) 
                    {
                        //lastPointInPath = i;
                        if(oldLastPointInPath + 20 > i){
                            lastPointInPath = i;
                        }
                    }
                }*/

                /*
                if (Vector3.Distance(transform.position, my_path[lastPointInPath].getPosition())<1f)
                {
                    lastPointInPath++;
                }
                */

                if (lastPointInPath == my_path_length - 1){
                    return;
                }


                if (lastPointInPath != my_path.Count - 1)
                {

                    Vector3 target = my_path[lastPointInPath + 1].getPosition();
                    float distanceToTargetTemp = Vector3.Distance(transform.position, target);
                    int targetId = 0;
                    /*for (int i = lastPointInPath + 2; i < my_path.Count; i = i + 1)
                    {
                        float newDistance = Vector3.Distance(transform.position, my_path[i].getPosition());
                        if (distanceToTargetTemp > newDistance)
                        {
                            distanceToTargetTemp = newDistance;
                            target = my_path[i].getPosition();
                            targetId = i;
                        }
                    }*/
                    if(distanceToTargetTemp < 6.0f){
                        lastPointInPath++;
                        target = my_path[lastPointInPath + 1].getPosition();
                    }

                    Vector3 carToTarget = transform.InverseTransformPoint(target);
                    float newSteer = (carToTarget.x / carToTarget.magnitude);
                    float newSpeed = 1f;
                    float handBreak = 0f;
                    Vector3 steeringPoint = new Vector3(0, 0, 1);
                    steeringPoint = (transform.rotation * steeringPoint);

                    float breakingDistance = (controller.CurrentSpeed * controller.CurrentSpeed) / (2);
                    float distanceToTarget = Vector3.Distance(transform.position, target);

                    RaycastHit rayHit;
                    bool hitBreak = Physics.SphereCast(transform.position, sideMargin, steeringPoint, out rayHit, 0.08f * breakingDistance, my_mask);
                    bool hitBack = Physics.SphereCast(transform.position, sideMargin, steeringPoint, out rayHit, Margin, my_mask);
                    bool hitContinueBack = Physics.SphereCast(transform.position, sideMargin, steeringPoint, out rayHit, Margin * 3, my_mask);
                    //newAngle = Vector3.Angle(transform.position - my_path[lastPointInPath + 1].getPosition(), my_path[lastPointInPath + 1].getPosition() - my_path[lastPointInPath + 2].getPosition());

                    /*if (controller.AccelInput == 0 && backing == false)
                    {
                        newSpeed = 1f / 1 + newAngle;
                    }*/

                    if (hitBack)
                    {
                        backing = true;
                        newSpeed = -1f;
                        if (controller.BrakeInput > 0 && controller.AccelInput <= 0)
                        {
                            newSteer = -newSteer;
                        }
                    }
                    else if (hitBreak && backing == false)
                    {
                        Debug.Log("Backing up");
                        newSpeed = -1;
                        handBreak = 1;
                        // print("yes");

                    }
                    if (hitContinueBack && controller.AccelInput >= 0 && backing == false)
                    {
                        newSteer = newSteer * 2;
                        //print("nope");
                    }
                    if (hitContinueBack && backing == true)
                    {
                        newSpeed = -1f;
                        newSteer = -newSteer;
                        //print("yes");
                    }
                    else
                    {
                        backing = false;
                    }

                    if (controller.CurrentSpeed > 20) //Default 20
                    {
                        newSpeed = 0;
                    }

					if (carToTarget.z/carToTarget.magnitude < 0){ // The point is behind the car
						newSpeed = -1f;
						newSteer = (-carToTarget.x/carToTarget.magnitude);
						if(carToTarget.z/carToTarget.magnitude == -1){
							newSpeed = -1f;
							newSteer = 1f; // Test. Unlikely to happen.
						}
					}

                    m_Car.Move(newSteer, newSpeed, newSpeed, handBreak);




                    //Debug.DrawLine(transform.position,my_path[lastPointInPath+1].getPosition(), Color.black);
                    //Debug.DrawLine(my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+2].getPosition(), Color.white);
                    Debug.DrawLine(transform.position, target, Color.black);
                    Debug.DrawLine(transform.position + steeringPoint * 0.08f * breakingDistance, transform.position, Color.white);

                }
            }
            // this is how you control the car
            //m_Car.Move(0f, 0f, 0f, 0f);
        }




    }
}


