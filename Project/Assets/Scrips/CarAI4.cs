using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car {
    [RequireComponent(typeof(CarController))]
    public class CarAI4 : MonoBehaviour {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;
        public GameObject leader;
        public List<Vector3> positions;
        int myIdx;

        public int my_path_length;

        public LayerMask my_mask;

        public int skipper;

        int counter = 0;
        //SphereCollider droneCollider;
        public float Margin = 1f;
        public float sideMargin = 1f;

        //All edges have same length:
        public float edgeLength = 10.0f;
        public float edgeMinDist = 0.1f;
        public float addEdgeMaxLength = 12.0f;
        int randomTimer = 0;
        Vector3 goal_pos;
        public int lastPointInPath;
        public float newAngle = 0;
        private Boolean backing = false;
        public List<Vector3> myPath;
        public float dist;
        private void Start() {
            // get the car controller
            dist = 1;
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            leader = friends[0];
            List<Vector3> positions = new List<Vector3>();
            int i = 0;
            foreach (var friend in friends) {
                if (friend == this.gameObject) {
                    myIdx = i;
                }
                i++;
            }

            myPath = new List<Vector3>();
            // Plan your path here
            // ...


        }


        private void FixedUpdate()
        {
            float maxDistnce;
            float allowedVelocity;
            float width = 20;
            if (positions.Count <= 2) {
                positions.Add(leader.transform.position);
            } else {
                positions.RemoveAt(0);
                positions.Add(leader.transform.position);
            }

            if (positions.Count < 2)
            {
                return;
            }

            generatePath(positions, myPath, myIdx);

            if (myPath.Count < 50)
            {
                return;
            }


            for (int i = 0; i < myPath.Count-1; i++)
            {
                Debug.DrawLine(myPath[i], myPath[i+1], Color.cyan);
            }
            /*
            RaycastHit raycastHitleft;
            RaycastHit raycastHitright;
            bool hitleft;
            bool hitright;
            float dl = width;
            float dr = width;

            Vector3 line = createPerpendicularV(positions[0], positions[1]);
            Vector3 goal = positions[0];
           
            hitleft = Physics.Raycast(positions[0], line, out raycastHitleft, width, my_mask);
            hitright = Physics.Raycast(positions[0], line, out raycastHitright, width, my_mask);
            Debug.DrawLine(positions[0], positions[0]+line * width, Color.red);
            Debug.DrawLine(positions[0], positions[0]+line * -width, Color.red);


            if (hitleft) {
                Vector3 closest = raycastHitleft.collider.ClosestPointOnBounds(positions[0]);
                dl = Vector3.Distance(closest, positions[0]);

            }
            if (hitright) {
                Vector3 closest = raycastHitright.collider.ClosestPointOnBounds(positions[0]);
                dr = Vector3.Distance(closest, positions[0]);

            }
            if (dl < dr) {
                width = dl;
            } else if (dr > dl) {
                width = dr;
            } else if (dl == dr) {
                width = Math.Min(width, dl);
            }

            if (myIdx == 2) {
                goal = positions[0] + width * line;
            } else if (myIdx == 1) {
                goal = positions[0] + 0.5f * width * line;
            } else if (myIdx == 3) {
                goal = positions[0] - 0.5f * width * line;
            } else if (myIdx == 4) {
                goal = positions[0] - width * line;
            }
            */
            my_mask = LayerMask.GetMask("CubeWalls");

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

            if (skipper > 0) {
                skipper--;
                m_Car.Move(0f, 0f, 0f, 0f);
                return;

            }



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



            Vector3 target = myPath[0];
            //float distanceToTargetTemp = Vector3.Distance(transform.position, target);
            int targetId = 0;



            Vector3 carToTarget = transform.InverseTransformPoint(target);
            float newSteer = (carToTarget.x / carToTarget.magnitude);
            float newSpeed = 1f;
            float handBreak = 0f;

           // for (int i = 1; i < myPath.Count - 1; i++) {
                float angle = Vector3.Angle(myPath[0] - myPath[25], myPath[49] - myPath[25]);
                if (angle < 90f) //Change me
                {
                    Debug.Log("Ok stop now");
                    newSpeed=-1f;
                    m_Car.Move(newSteer, newSpeed, newSpeed, handBreak);

            }
            //}
            Vector3 steeringPoint = new Vector3(0, 0, 1);
            steeringPoint = (transform.rotation * steeringPoint);

            bool carInFront = false;
            bool carBehind = false;
            
            Vector3 rayFront = transform.position + steeringPoint;

            float breakingDistance = (controller.CurrentSpeed * controller.CurrentSpeed) / (2);
            float distanceToTarget = Vector3.Distance(transform.position, target);
            if (distanceToTarget < 10f) {
                newSpeed= 1f - ((10f - distanceToTarget)/10f);
            }
            RaycastHit rayHit;
            bool hitBreak = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, 0.08f * breakingDistance,my_mask);


            if (hitBreak && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                Debug.Log("Player Detected.");
                carInFront = true;
            }

            bool hitBack = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin, my_mask);

            if (hitBack && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                Debug.Log("Player Detected.");
                carInFront = true;
            }

            bool hitContinueBack = Physics.SphereCast(rayFront, sideMargin, steeringPoint, out rayHit, Margin * 3, my_mask);

            if (hitContinueBack && rayHit.collider.attachedRigidbody != null && rayHit.collider.attachedRigidbody.tag == "Player") {
                Debug.Log("Player Detected.");
                carInFront = true;
            }

            RaycastHit rayBackHit;
            bool hitFront = Physics.Raycast(transform.position - steeringPoint, -steeringPoint, out rayBackHit, 1, my_mask);

            if (hitFront && rayBackHit.collider.attachedRigidbody != null && rayBackHit.collider.attachedRigidbody.tag == "Player") {
                carBehind = true;
            }

            bool hitBreak_r = Physics.Raycast(transform.position, steeringPoint, 0.08f * breakingDistance, my_mask);
            bool hitBack_r = Physics.Raycast(transform.position, steeringPoint, Margin, my_mask);
            bool hitContinueBack_r = Physics.Raycast(transform.position, steeringPoint, Margin * 3, my_mask);

            //newAngle = Vector3.Angle(transform.position - my_path[lastPointInPath + 1].getPosition(), my_path[lastPointInPath + 1].getPosition() - my_path[lastPointInPath + 2].getPosition());

            /*if (controller.AccelInput == 0 && backing == false)
            {
                newSpeed = 1f / 1 + newAngle;
            }*/
            if (hitFront) {
                newSpeed = newSpeed;
                //Debug.Log("Ups");
            } else if (hitBack || hitBack_r) {
                //Debug.Log("AAAA");
                backing = true;
                newSpeed = 0f;//-1f;
                if (controller.BrakeInput > 0 && controller.AccelInput <= 0) {
                    newSteer = -newSteer;
                }
            } else if ((hitBreak || hitBreak_r) && backing == false) {
                //Debug.Log("Backing up");
                newSpeed = 0f; //-1;
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
                newSpeed = 0f;//-1f;
                newSteer = -newSteer;
                //print("yes");
            } else {
                backing = false;
                //Debug.Log("DDDD");
            }

            //friends = GameObject.FindGameObjectsWithTag("Player");
            maxDistnce = 0;
            for (int i=1; i < friends.Length;i++ )
            {
                var f = friends[i];
                var my_car = f.GetComponent<CarAI4>();
                Debug.Log(my_car);
                Debug.Log("Get ready for it:");
                float ithdist =my_car.dist;
                Debug.Log(ithdist);
                if (maxDistnce < f.GetComponent<CarAI4>().dist)
                {
                    maxDistnce = f.GetComponent<CarAI4>().dist;
                }
            }
            

            allowedVelocity = 150f *(dist/maxDistnce);
            if (controller.CurrentSpeed > allowedVelocity) //Default 20
            {
                newSpeed = 0;
            }

            if (carToTarget.z / carToTarget.magnitude < 0) { // The point is behind the car
                Debug.Log("IT IS BEHIND");
                newSpeed = 0f;//-1f; //it was -1
                newSteer = 0f; //(-carToTarget.x / carToTarget.magnitude);
                if (carToTarget.z / carToTarget.magnitude == -1)
                {
                    newSpeed = 0f;//-1f; //it was -1
                    newSteer = 0f; //1f; // Test. Unlikely to happen.
                }
            }

            if (carInFront) {
                Debug.Log("C A R ! ! ! ");
                newSteer = 1f;
            }

            if (carBehind) {
                newSteer = -1f;
            }

            m_Car.Move(newSteer, newSpeed, newSpeed, handBreak);




            //Debug.DrawLine(transform.position,my_path[lastPointInPath+1].getPosition(), Color.black);
            //Debug.DrawLine(my_path[lastPointInPath+1].getPosition(),my_path[lastPointInPath+2].getPosition(), Color.white);
            Debug.DrawLine(transform.position, target, Color.black);
            Debug.DrawLine(transform.position + steeringPoint * 0.08f * breakingDistance, transform.position, Color.white);



            // this is how you control the car
            //m_Car.Move(0f, 0f, 0f, 0f);
        }

        public List<Vector3> generatePath(List<Vector3> positions, List<Vector3> myPath, int myIdx)
        {
            RaycastHit raycastHitleft;
            RaycastHit raycastHitright;
            bool hitleft;
            bool hitright;
            float width = 20;
            float dl = width;
            float dr = width;

            Vector3 line = createPerpendicularV(positions[0], positions[1]);
            Vector3 goal = positions[0];
            hitleft = Physics.Raycast(positions[0], line, out raycastHitleft, width, my_mask);
            hitright = Physics.Raycast(positions[0], line, out raycastHitright, width, my_mask);
            Debug.DrawLine(positions[0], positions[0] + line * width, Color.red);
            Debug.DrawLine(positions[0], positions[0] + line * -width, Color.red);


            if (hitleft) {
                Vector3 closest = raycastHitleft.collider.ClosestPointOnBounds(positions[0]);
                dl = Vector3.Distance(closest, positions[0]);

            }
            if (hitright) {
                Vector3 closest = raycastHitright.collider.ClosestPointOnBounds(positions[0]);
                dr = Vector3.Distance(closest, positions[0]);

            }
            if (dl < dr) {
                width = dl;
            } else if (dr > dl) {
                width = dr;
            } else if (dl == dr) {
                width = Math.Min(width, dl);
            }

            if (myIdx == 2) {
                goal = positions[0] + width * line;
            } else if (myIdx == 1) {
                goal = positions[0] + 0.5f * width * line;
            } else if (myIdx == 3) {
                goal = positions[0] - 0.5f * width * line;
            } else if (myIdx == 4) {
                goal = positions[0] - width * line;
            }

            myPath.Add((goal));
            if (myPath.Count > 50)
            {
                myPath.RemoveAt(0);
            }

            computeDistance();
            return myPath;
        }

        public void computeDistance()
        {
            dist = 0;
            for (int i = 0; i < myPath.Count-1; i++)
            {
                dist += Vector3.Distance(myPath[i], myPath[i + 1]);
            }
        }        

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
    

