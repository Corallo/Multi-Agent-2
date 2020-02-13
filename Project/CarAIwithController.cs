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
        public GameObject terrain_manager_game_object;
        public RRTDrone myrrt;
        TerrainManager terrain_manager;

        static float maxVel = 20f;
        static float maxAcc = 3f;
        float constAcc = 0.1f;

        public RTController mycontroller = new RTController();
        public RTController.TachiController distanceController = new RTController.TachiController(20f, 30f);
        //public RTController.PIDController SpeedController = new RTController.PIDController(100f, 20f, 0f);
        public RTController.PIDController SteeringController = new RTController.PIDController(0.6f/maxVel, 0f, 0f);

        List<RTController.refSignal> trajectory = new List<RTController.refSignal>();

        private void Start()
        {
            m_Car = GetComponent<CarController>();
            myrrt = new RRTDrone();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();


            Vector3 start_pos = terrain_manager.myInfo.start_pos;
            Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
            //Vector3 goal_pos = new Vector3(180, 0, 180);


            List<Vector3> my_path = new List<Vector3>();

            // Plan your path here
            myrrt.makePath(my_path, start_pos, transform.forward*5, goal_pos, terrain_manager, 3f);
            //my_path.Add(start_pos);  my_path.Add(goal_pos);
            Debug.Log("length= " + mycontroller.pathLength(my_path));


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(old_wp, wp, Color.yellow, 100f);
                old_wp = wp;
            }

            trajectory = mycontroller.makeTrajectory(my_path, maxVel / 2.236f, maxAcc);
        }


        int iteration = 0;
        private void FixedUpdate()
        {
            float x = transform.position.x;
            float z = transform.position.z;
            float v = m_Car.CurrentSpeed;


            RTController.refSignal reference = trajectory[iteration];

            Debug.DrawLine(transform.position, reference.pos, Color.white, 0.1f);


            Vector3 targetDir = reference.pos - transform.position;
            float errteta = Vector3.SignedAngle(targetDir, transform.forward, transform.up);
            float frontDistance = targetDir.magnitude * Mathf.Cos(errteta);

            if (Mathf.Abs(errteta) > 40f && iteration < trajectory.Count - 2) iteration++;
            if (targetDir.magnitude > 5f && iteration >= 2) iteration=iteration-1;

            //float acc = SpeedController.computeControl(maxVel, v);
            float acc = distanceController.computeControl(constAcc, maxVel, 0f, v, frontDistance);
            float steering = SteeringController.computeControl(0f, errteta);


            // Move(float steering, float accel, float footbrake, float handbrake)
            m_Car.Move(steering, acc, 0f, 0f);

            Debug.Log(" errteta " + errteta.ToString("F1") + " steer "+ steering.ToString("F1") + " v " + v.ToString("F1") + " vmax " + maxVel.ToString("F1") + " verr " + (v - maxVel).ToString("F1") + " frontD " + frontDistance.ToString("F1") + " acc " + acc);

            if (iteration < trajectory.Count - 2) iteration++;
        }
    }
}
