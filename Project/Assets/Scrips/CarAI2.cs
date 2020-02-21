using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using MyGraph;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI2 : MonoBehaviour
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
        public float Margin = 1f;
        public float sideMargin = 1f;

        private Graph DroneGraph;
        //All edges have same length:
        public float edgeLength = 10.0f;
        public float edgeMinDist = 0.1f;
        public float addEdgeMaxLength = 12.0f;
        public List<Node> my_path = new List<Node>();
        int randomTimer = 0;
        Vector3 goal_pos;
        public int lastPointInPath ;
        public float newAngle = 0;
        private Boolean backing = false;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            lastPointInPath = -1;
            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem



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

            Margin = 1;
            sideMargin = 1;

            if (skipper > 0)
            {
                skipper--;
                m_Car.Move(0f, 0f, 0f, 0f);
                return;

            }

            if (my_path.Count == 0 || my_path.Count == 1)
            {
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

                if (lastPointInPath == my_path_length - 1)
                {
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
                    if (distanceToTargetTemp < 4f)
                    { // EQUAL TO DISTANCE OF NODES
                        lastPointInPath++;
                        target = my_path[lastPointInPath + 1].getPosition();
                    }

                    Vector3 carToTarget = transform.InverseTransformPoint(target);
                    float newSteer = (carToTarget.x / carToTarget.magnitude);
                    float newSpeed = 1f;
                    float handBreak = 0f;
                    Vector3 steeringPoint = new Vector3(0, 0, 1);
                    steeringPoint = (transform.rotation * steeringPoint);

                    bool carInFront = false;
                    bool carBehind = false;
                    
                    Vector3 rayFront = transform.position + steeringPoint;

                    float breakingDistance = (controller.CurrentSpeed * controller.CurrentSpeed) / (2);
                    float distanceToTarget = Vector3.Distance(transform.position, target);

                    RaycastHit rayHit;
                    bool hitBreak = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, 0.08f * breakingDistance);


                    if(hitBreak && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player"){
                        Debug.Log("Player Detected.");
                        carInFront = true;
                    }

                    bool hitBack = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin);

                    if(hitBack && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player"){
                        Debug.Log("Player Detected.");
                        carInFront = true;
                    }

                    bool hitContinueBack = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin * 3);

                    if(hitContinueBack && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player"){
                        Debug.Log("Player Detected.");
                        carInFront = true;
                    }

                    RaycastHit rayBackHit;
                    bool hitFront = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, out rayBackHit, 1);

                    if(hitFront && rayBackHit.collider.attachedRigidbody != null && rayBackHit.collider.attachedRigidbody.tag == "Player"){
                        carBehind = true;
                    }
                    
                    bool hitBreak_r = Physics.Raycast(transform.position,  steeringPoint,  0.08f * breakingDistance);
                    bool hitBack_r = Physics.Raycast(transform.position,  steeringPoint,  Margin);
                    bool hitContinueBack_r = Physics.Raycast(transform.position,  steeringPoint,  Margin * 3);
                    
                    //newAngle = Vector3.Angle(transform.position - my_path[lastPointInPath + 1].getPosition(), my_path[lastPointInPath + 1].getPosition() - my_path[lastPointInPath + 2].getPosition());

                    /*if (controller.AccelInput == 0 && backing == false)
                    {
                        newSpeed = 1f / 1 + newAngle;
                    }*/
                    if (hitFront )
                    {
                        newSpeed = 1f;
                        //Debug.Log("Ups");
                    }
                    else if (hitBack || hitBack_r)
                    {
                        //Debug.Log("AAAA");
                        backing = true;
                        newSpeed = -1f;
                        if (controller.BrakeInput > 0 && controller.AccelInput <= 0)
                        {
                            newSteer = -newSteer;
                        }
                    }
                    else if ((hitBreak||hitBreak_r) && backing == false)
                    {
                        //Debug.Log("Backing up");
                        newSpeed = -1;
                        //handBreak = 1;
                        // print("yes");

                    }
                    if ((hitContinueBack||hitContinueBack_r) && controller.AccelInput >= 0 && backing == false)
                    {
                        //Debug.Log("BBBB");
                        newSteer = newSteer * 2;
                        //print("nope");
                    }
                    if ((hitContinueBack||hitContinueBack_r) && backing == true)
                    {
                        //Debug.Log("CCCC");
                        newSpeed = -1f;
                        newSteer = -newSteer;
                        //print("yes");
                    }
                    else
                    {
                        backing = false;
                        //Debug.Log("DDDD");
                    }

                    if (controller.CurrentSpeed > 20) //Default 20
                    {
                        newSpeed = 0;
                    }

                    if (carToTarget.z / carToTarget.magnitude < 0)
                    { // The point is behind the car
                        newSpeed = -1f;
                        newSteer = (-carToTarget.x / carToTarget.magnitude);
                        if (carToTarget.z / carToTarget.magnitude == -1)
                        {
                            newSpeed = -1f;
                            newSteer = 1f; // Test. Unlikely to happen.
                        }
                    }

                    if (carInFront){
                        Debug.Log("C A R ! ! ! ");
                        newSteer = 1f;
                    }

                    if(carBehind){
                        newSteer = -1f;
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
