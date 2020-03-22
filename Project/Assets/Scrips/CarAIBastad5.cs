using MyGraph;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car {
    [RequireComponent(typeof(CarController))]
    public class CarAIbastard5 : MonoBehaviour {
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
        private void Start() {

            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            lastPointInPath = -1;
            friends = GameObject.FindGameObjectsWithTag("Player");
            //skipper = 75;
            skipper = 1;
            health = 200;
        }


        private void FixedUpdate() {



            CarController controller = transform.GetComponent<CarController>();

            // my_mask = LayerMask.GetMask("CubeWalls");
            friends = GameObject.FindGameObjectsWithTag("Player");




            if (skipper > 0) {
                skipper--;
                m_Car.Move(1f, 0f, -1f, 0f);
                return;

            }

            if (backingCount > 0) {
                backingCount--;
                m_Car.Move(0, 0, -1, 0);
                return;
            }

            if (my_path.Count == 0) {
                m_Car.Move(0f, 0f, 0f, 1f);
                return;
            }

            int i = 0;
            Vector3 mean = new Vector3(0, 0, 0);
            foreach (var friend in friends) {
                if (friend == this.gameObject) {
                    myIdx = i;
                }

                mean += friend.transform.position;
                i++;
            }

            mean /= friends.Length;
            Vector3 carToMean = transform.InverseTransformPoint(mean);
            Vector3 MyPosition = transform.position;
            health = m_Car.GetComponent<Destructable>().health;
            Debug.Log("MY healt is " + health.ToString());
            float distanceToTargetTemp;
            Vector3 target;



            //Compute the target
            if (lastPointInPath != -1) {
                target = my_path[lastPointInPath + 1].getPosition();
                Vector3 pretarget = my_path[lastPointInPath].getPosition();
                target = target + createPerpendicularV(target, pretarget) * offset +
                         (target - pretarget).normalized * leaderOffset;

            } else {
                target = my_path[lastPointInPath + 1].getPosition();

            }

            Vector3 carToTarget = transform.InverseTransformPoint(target);
            float newSteer = (carToTarget.x / carToTarget.magnitude);
            float newSpeed = 1;

            bool allowNewPoint;
            distanceToTargetTemp = Vector3.Distance(MyPosition, target);
            //distanceToTargetTemp = Vector3.Distance(mean, target);


            //Decide if it is the moment to go to the next point
            if (distanceToTargetTemp < 6f) {
                // EQUAL TO DISTANCE OF NODES

                if (lastPointInPath < my_path.Count - 2) {
                    allowNewPoint = true;
                    foreach (var friend in friends) {
                        if (friend.GetComponent<CarAI5>().lastPointInPath < lastPointInPath) {
                            allowNewPoint = false;
                        }
                    }

                    if (allowNewPoint) {
                        lastPointInPath++;
                    } else {
                        Debug.Log(myIdx.ToString() + " I would like to go to next point, buy i am not allowed to");
                        if (controller.CurrentSpeed > 1) {
                            newSpeed = -1;
                        } else {
                            newSpeed = 0.2f;
                        }
                    }
                    /*if (myIdx == 0) {
                        foreach (var friend in friends) {

                            friend.GetComponent<CarAI5>().lastPointInPath++;

                        }
                    }*/


                }
            }






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
            float Margin = 0.5f;
            float sideMargin = 1.5f;
            //my_mask = LayerMask.GetMask("CubeWalls");
            my_mask = Physics.DefaultRaycastLayers;
            bool hitFront = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin, my_mask);
            bool hitFrontRay = Physics.Raycast(rayFront, steeringPoint, Margin, my_mask);
            // if (hitFront && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
            //    carInFront = true;
            //}

            RaycastHit rayBackHit;
            bool hitBack = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, out rayBackHit, Margin,
                my_mask);
            bool hitBackRay = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, Margin, my_mask);
            //if (hitBack && rayBackHit.collider.attachedRigidbody != null && rayBackHit.collider.attachedRigidbody.tag == "Player") {
            //      carBehind = true;
            // }

            //bool hitBreak_r = Physics.Raycast(transform.position, steeringPoint, 0.08f * breakingDistance, my_mask);
            // bool hitBack_r = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);
            bool hitFront2 = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);
            bool hitFrontRay2 = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);


            if (hitBack || hitBackRay) {
                Debug.Log(myIdx.ToString() + " I will hit something back, so I am going haead");
                newSpeed = 0.2f;
            }

            if (hitFront || hitFront2 || hitFront2 || hitFrontRay2) {
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

            if (controller.CurrentSpeed > 20) //10*((health/200)+1)) //Default 20
            {
                Debug.Log(myIdx.ToString() + " Slowing down");
                newSpeed = 0;
            }

            if (carToTarget.z / carToTarget.magnitude < 0) {
                // The point is behind the car
                newSpeed = -0.2f;
                newSteer = (-carToTarget.x / carToTarget.magnitude);
                Debug.Log(myIdx.ToString() + " point is behind me");
                if (carToTarget.z / carToTarget.magnitude == -1) {
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

            if (lastPointInPath == my_path.Count - 2) {
                Debug.Log("hand braking");
                //handBreak = 1 - distanceToTarget / 6;
                newSpeed = 0;
            }


            m_Car.Move(newSteer, newSpeed, newSpeed, handBreak);




            //Debug.DrawLine(transform.position,my_path[lastPointInPath+1].getPosition(), Color.black);
            //Debug.DrawLine(my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+2].getPosition(), Color.white);
            Debug.DrawLine(transform.position, target, Color.black);
            Debug.DrawLine(transform.position + steeringPoint * 0.08f * breakingDistance, transform.position, Color.white);


        }
        // this is how you control the car
        //m_Car.Move(0f, 0f, 0f, 0f);

        Vector3 createPerpendicularV(Vector3 a, Vector3 b) {

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
