using MyGraph;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Networking.NetworkSystem;

namespace UnityStandardAssets.Vehicles.Car {
    [RequireComponent(typeof(CarController))]
    public class CarAI5 : MonoBehaviour
    {
        public CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        public String color = "";

        public int my_path_length;
        public int run;
        public LayerMask my_mask;

        public int skipper;

        int counter = 0;
        //SphereCollider droneCollider;

        public float sideMargin;
        public int myIdx;

        private Graph DroneGraph;

        //All edges have same length:
        public float edgeLength = 10.0f;
        public float edgeMinDist = 0.1f;
        public float addEdgeMaxLength = 12.0f;
        public List<Node> my_path = new List<Node>();
        int randomTimer = 0;
        Vector3 goal_pos;
        public int lastPointInPath;
        public float newAngle = 0;
        private Boolean backing = false;
        public float leaderOffset;
        public float health;
        public float offset;
        public int backingCount;
        public CarController controller;
        public Vector3 vel;
        public Vector3 carVelocity;
        public float orientation;
        public Vector3 target;
        public float yellow_angle;
        public int wait;
        public float newSpeed;
        public int stuckChecker;
        public Vector3 position;
        public int unstuckRun;
        public int safe;
        public int maxSpeed;
        private void Start()
        {

            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            lastPointInPath = 0;
            friends = GameObject.FindGameObjectsWithTag("Player");
            //skipper = 75;
            skipper = 1;
            health = 200;
            run = 0;
            stuckChecker = 1000;
            unstuckRun = 0;
            maxSpeed = 8;
        }


        private void FixedUpdate()
        {



            controller = transform.GetComponent<CarController>();
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            // my_mask = LayerMask.GetMask("CubeWalls");
            friends = GameObject.FindGameObjectsWithTag("Player");

            vel = m_Car.GetComponent<Rigidbody>().velocity;
            if (run > 0)
            {
                run--;

                target = my_path[my_path_length - 1].getPosition();
                Vector3 pretarget = my_path[my_path_length - 2].getPosition();
                //target = target + createPerpendicularV(target, pretarget) * offset +
                //         (target - pretarget).normalized * leaderOffset;
                Vector3 carToTargetRUN = transform.InverseTransformPoint(target);

                //
                m_Car.Move(carToTargetRUN.x / carToTargetRUN.magnitude, 1f, 0f, 0f);
                return;
            }

            if (unstuckRun > 0)
            {
                unstuckRun--;
                m_Car.Move(0, 1f, 0f, 0f);
                return;
            }

            if (skipper > 0)
            {
                skipper--;
                m_Car.Move(0f, 0f, -1f, 0f);
                return;

            }
            
            if (wait > 0 && wait%2==0)
            {
                wait--;
                m_Car.Move(0f, 1f, 0f, 0f);
                return;
            }
            if (wait > 0 && wait % 2 == 1) {
                wait--;
                m_Car.Move(0f, 0f, -1f, 0f);
                return;
            }
            /*
            if (wait == 0)
            {
                m_Car.Move(0f, 0f, 0f, -1f);
                wait--;
            }*/

            /*
            if (stuckChecker == 0)
            {
                if (Vector3.Distance(position, m_Car.transform.position) < 1f)
                {
                    unstuckRun = 5;
                }
                else
                {
                    position = m_Car.transform.position;
                }

                stuckChecker = 1000;
            }
            else
            {
                stuckChecker--;
            }
            */

            if (my_path.Count == 0)
            {
                //m_Car.Move(0f, 0f, 0f, 1f);
                Debug.Log("Empty path");
                return;
            }

            int i = 0;
            Vector3 mean = new Vector3(0, 0, 0);
            foreach (var friend in friends)
            {
                if (friend == this.gameObject)
                {
                    myIdx = i;
                }

                mean += friend.transform.position;
                i++;
            }

            if (myIdx == 0)
            {
                //Maybe we want to compute this only once in the planner
            }


            mean /= friends.Length;
            Vector3 carToMean = transform.InverseTransformPoint(mean);
            Vector3 MyPosition = transform.position;
            health = m_Car.GetComponent<Destructable>().health;
            Debug.Log("MY healt is " + health.ToString());
            float distanceToTargetTemp;


            var lastevil = enemies[0];
            var turret_mask = LayerMask.GetMask("CubeWalls");
            foreach (var enemy in enemies) {
                if (!Physics.Raycast(enemy.transform.position,
                    my_path[my_path_length -1].getPosition() - enemy.transform.position,
                    Vector3.Distance(enemy.transform.position, my_path[my_path_length - 1].getPosition()),//it was my_path[my_path_length - 1]
                    turret_mask)) {

                    Debug.DrawRay(enemy.transform.position, (my_path[my_path_length - 1].getPosition() - enemy.transform.position) * 1000, Color.yellow, 20f);
                    // Debug.DrawLine(enemy.transform.position, my_path[lastPointInPath + 1].getPosition(),Color.yellow,100);
                    lastevil = enemy;
                    break;
                }
            }

            yellow_angle = Vector3.Angle(lastevil.transform.position - my_path[my_path_length - 2].getPosition(),
                my_path[my_path_length - 1].getPosition() - my_path[my_path_length - 2].getPosition());

            var evil = enemies[0];
            Vector3 blueVector=new Vector3(0,0,0);
            foreach (var enemy in enemies) {
                if (!Physics.Raycast(enemy.transform.position,
                    my_path[lastPointInPath + 1].getPosition() - enemy.transform.position,
                    Vector3.Distance(enemy.transform.position, my_path[lastPointInPath + 1].getPosition()),//it was my_path[my_path_length - 1]
                    turret_mask)) {

                    Debug.DrawRay(enemy.transform.position, (my_path[lastPointInPath + 1].getPosition() - enemy.transform.position) * 1000, Color.yellow, 20f);
                    // Debug.DrawLine(enemy.transform.position, my_path[lastPointInPath + 1].getPosition(),Color.yellow,100);
                    evil = enemy;
                    break;
                }
            }

            int stopPhase = 2;
            /*if (yellow_angle < 25) {
                stopPhase = 10;
            }*/

            if (lastPointInPath > my_path_length - ((stopPhase+4+safe))) // maybe last 5 points
            {
                

                blueVector = evil.transform.position - my_path[lastPointInPath+1].getPosition();
                target = my_path[lastPointInPath + 1].getPosition();
                //Vector3 pretarget = my_path[lastPointInPath].getPosition();
                target = target + blueVector.normalized * offset; //+ (target - pretarget).normalized * leaderOffset;
                
            } else
            {
                    target = my_path[lastPointInPath + 1].getPosition();
                    //Vector3 pretarget = my_path[lastPointInPath].getPosition();
                    //target = target + blueVector.normalized * offset; //+ (target - pretarget).normalized * leaderOffset;
                    /*target = target + createPerpendicularV(target, pretarget) * offset +
                             (target - pretarget).normalized * leaderOffset;*/

            }

            maxSpeed = 8;
            if (lastPointInPath != -1 && lastPointInPath+10 < my_path_length)
            {
                maxSpeed = 20;
                Vector3 longVector = my_path[lastPointInPath].getPosition() - my_path[lastPointInPath + 10].getPosition();
                for (i = lastPointInPath+1; i < lastPointInPath + 10; i++)
                {
                    if (Vector3.Angle(my_path[lastPointInPath].getPosition() - my_path[i].getPosition(), longVector) >   1f && 
                        Vector3.Angle(my_path[lastPointInPath].getPosition() - my_path[i].getPosition(), longVector) < 179f)
                    {
                        maxSpeed = 8;
                    }
                }
            }

            Vector3 carToTarget = transform.InverseTransformPoint(target);
            float newSteer = (carToTarget.x / carToTarget.magnitude);
            newSpeed = 1;

            if (backingCount > 0) {
                backingCount--;
                m_Car.Move((-carToTarget.x / carToTarget.magnitude), 0, -1, 0);
                return;
            }

            bool allowNewPoint;
            distanceToTargetTemp = Vector3.Distance(MyPosition, target);

            if (distanceToTargetTemp < 6f)
            {
                // EQUAL TO DISTANCE OF NODES

                if (lastPointInPath < my_path.Count - (2+safe) )
                {
                    allowNewPoint = true;
                   /* foreach (var friend in friends)
                    {
                        if (friend.GetComponent<CarAI5>().lastPointInPath < lastPointInPath)
                        {
                            allowNewPoint = false;
                        }
                    }*/

                    if (allowNewPoint)
                    {
                        lastPointInPath++;
                    }
                    else
                    {
                        Debug.Log(myIdx.ToString() + " I would like to go to next point, buy i am not allowed to");
                        if (controller.CurrentSpeed > 1)
                        {
                            newSpeed = -1;
                        }
                        else
                        {
                            newSpeed = 0.2f;
                        }
                    }

                }
            }

            carToTarget = transform.InverseTransformPoint(target);
             newSteer = (carToTarget.x / carToTarget.magnitude);
             newSpeed = 1;

            //distanceToTargetTemp = Vector3.Distance(mean, target);


            //Decide if it is the moment to go to the next point
     
            // EQUAL TO DISTANCE OF NODES


                float handBreak = 0f;
                Vector3 steeringPoint = new Vector3(0, 0, 1);
                steeringPoint = (transform.rotation * steeringPoint);

                Vector3 rayFront = transform.position + steeringPoint;

                float breakingDistance = (controller.CurrentSpeed * controller.CurrentSpeed) / (2);
                float distanceToTarget = Vector3.Distance(transform.position, target);

                RaycastHit rayHit;
                /*
                 bool hitBreak = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, 0.08f * breakingDistance);
    
    
                 if (hitBreak && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                     // Debug.Log("Player Detected.");
                     carInFront = true;
                 }
    
                 bool hitBack = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin);
    
                 if (hitBack && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                     // Debug.Log("Player Detected.");
                     carInFront = true;
                 }
    
                 */
                    float Margin = 2f;
                float sideMargin = 1.5f;
                //my_mask = LayerMask.GetMask("CubeWalls");
                my_mask = Physics.DefaultRaycastLayers;
                bool hitFront = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin, my_mask);
                bool hitFrontRay = Physics.Raycast(rayFront, steeringPoint, Margin, my_mask);
                // if (hitFront && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                //    carInFront = true;
                //}

                RaycastHit rayBackHit;
                bool hitBack = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, out rayBackHit,
                    Margin,
                    my_mask);
                bool hitBackRay = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, Margin, my_mask);
                //if (hitBack && rayBackHit.collider.attachedRigidbody != null && rayBackHit.collider.attachedRigidbody.tag == "Player") {
                //      carBehind = true;
                // }

                //bool hitBreak_r = Physics.Raycast(transform.position, steeringPoint, 0.08f * breakingDistance, my_mask);
                // bool hitBack_r = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);
                bool hitFront2 = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);
                bool hitFrontRay2 = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);


                if (hitBack || hitBackRay)
                {
                    Debug.Log(myIdx.ToString() + " I will hit something back, so I am going haead");
                    newSpeed = 0.2f;
                }

                if (hitFront || hitFront2 || hitFront2 || hitFrontRay2)
                {
                    Debug.Log(myIdx.ToString() + " I will hit something haed so i am going back");
                    newSpeed = -0.2f;
                    newSteer = -newSteer;
                    backingCount = 20;
                }
                /*else if (hitBack || hitBack_r) {
                    //Debug.Log("AAAA");
                    backing = true;
                    newSpeed = -1f;
                    if (controller.BrakeInput > 0 && controller.AccelInput <= 0) {
                        newSteer = -newSteer;
                    }
                } else if ((hitBreak || hitBreak_r) && backing == false) {
                    //Debug.Log("Backing up");
                    newSpeed = -1;
                    //handBreak = 1;
                    // print("yes");
    
                }
                if ((hitContinueBack || hitContinueBack_r) && controller.AccelInput >= 0 && backing == false) {
                    //Debug.Log("BBBB");
                    newSteer = newSteer * 2;
                    //print("nope");
                }
                if ((hitContinueBack || hitContinueBack_r) && backing == true) {
                    //Debug.Log("CCCC");
                    newSpeed = -1f;
                    newSteer = -newSteer;
                    //print("yes");
                } else {
                    backing = false;
                    //Debug.Log("DDDD");
                }
    
                */

                if (controller.CurrentSpeed > maxSpeed) //10*((health/200)+1)) //Default 20
                {
                    Debug.Log(myIdx.ToString() + " Slowing down");
                    newSpeed = 0;
                }

                if (carToTarget.z / carToTarget.magnitude < 0)
                {
                    // The point is behind the car
                    newSpeed = -0.2f;
                    newSteer = (-carToTarget.x / carToTarget.magnitude);
                    Debug.Log(myIdx.ToString() + " point is behind me");
                    if (carToTarget.z / carToTarget.magnitude == -1)
                    {
                        newSpeed = -0.2f;
                        newSteer = 1f; // Test. Unlikely to happen.
                    }
                }
            /*
            if (carInFront) {
                //Debug.Log("C A R ! ! ! ");
                //newSteer = 1f;
                //newSpeed = -1f;
            }

            if (carBehind) {
                //newSteer = -1f;
                //newSpeed = 1f;
            }
            */
            /*
            if (carToMean.z < -3 && carToTarget.z > 0) {
                Debug.Log(myIdx.ToString() + " Waiting for others car");
                if (controller.CurrentSpeed > 1)
                {
                    newSpeed = -1;
                }else{
                    newSpeed = 0;
                }

            }
            */
            

            carVelocity = transform.InverseTransformPoint(vel);
            orientation = Vector3.Dot(steeringPoint, vel);

            //safe = 0;

            if (lastPointInPath >= my_path_length - (stopPhase+safe))
            {
                if (orientation > 0 && carToTarget.magnitude < 10f) //Default 20  !(carToMean.z > 0.1)  
                {

                    Debug.Log(myIdx.ToString() + " Decelerating to wait my friends");
                   
                        newSpeed = -1f;
                }else if (orientation < 0 && carToTarget.magnitude < 10f) { //&& 

                    newSpeed = 1f;
                    Debug.Log(myIdx.ToString() + " Decelerating to wait my friends");

                }

                if (controller.CurrentSpeed < 1 && carToTarget.magnitude < 10f)//carToMean.z > -0.8
                {
                    if (myIdx == 0)
                    {
                        bool go = true;
                        foreach (var car in friends)
                        {
                            if (car.GetComponent<CarAI5>().controller.CurrentSpeed > 0.8 || lastPointInPath!=car.GetComponent<CarAI5>().lastPointInPath)
                            {
                                go = false;
                            }
                        }

                        if (go)
                        {
                            foreach (var car in friends)
                            {
                                car.GetComponent<CarAI5>().run = 50;
                            }
                        }
                    }
                }
            

            /*
            if (carToMean.z >= -4f && carToMean.z < 0.5f)
            {
                newSpeed *= 0.5f;
            }*/
            }

            float footbrake = 0;
            if (newSpeed < 0) {
                footbrake = newSpeed;
                newSpeed = 0;
            }

            m_Car.Move(newSteer, newSpeed, footbrake, handBreak);




                //Debug.DrawLine(transform.position,my_path[lastPointInPath+1].getPosition(), Color.black);
                //Debug.DrawLine(my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+2].getPosition(), Color.white);
                Debug.DrawLine(transform.position, target, Color.black);
                Debug.DrawLine(transform.position + steeringPoint * 0.08f * breakingDistance, transform.position,
                    Color.white);


            }
            // this is how you control the car
            //m_Car.Move(0f, 0f, 0f, 0f);

            Vector3 createPerpendicularV(Vector3 a, Vector3 b)
            {

                Vector2 a2 = new Vector2(a.x, a.z);
                Vector2 b2 = new Vector2(b.x, b.z);
                Vector2 c = b2 - a2;

                c = Vector2.Perpendicular(c);
                Vector3 c3 = new Vector3(c.x, 0, c.y);
                c3 = Vector3.Normalize(c3);
                return c3;
            }


        }

    }

