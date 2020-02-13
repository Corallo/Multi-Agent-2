using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RTController
{ 
    public List<refSignal> makeTrajectory(List<Vector3> path, float maxSpeed, float maxAcc)
    {
        List<refSignal> trajectory = new List<refSignal>();
        List<float> s = new List<float>();
        List<float> sdot = new List<float>();
        List<float> sdotdot = new List<float>();

        float accTime = maxSpeed / maxAcc;
        float accSpace = 0.5f * maxAcc * accTime*accTime;
        float length = pathLength(path);
        float cruiseTime= (length-accSpace)/ maxSpeed;
        float totalTime = cruiseTime + accTime;

        //Debug.Log("total time " + totalTime + " cruise time " + cruiseTime + " acc space " + accSpace + " cruise space " + maxSpeed*cruiseTime + " length= " + length);

        for (float t=0; t<=totalTime*1.1; t=t+ Time.fixedDeltaTime)
        {
            if (t <= accTime) {
                sdotdot.Add(maxAcc);
                sdot.Add(maxAcc * t);
                s.Add(0.5f * maxAcc * t*t);
            }
            else if (t > accTime && t <= totalTime-accTime)
            {
                sdotdot.Add(0f);
                sdot.Add(maxSpeed);
                s.Add(accSpace+maxSpeed*(t-accTime));
            }
            else if (t > totalTime-accTime && t<=totalTime)
            {
                sdotdot.Add(0f);
                sdot.Add(maxSpeed);
                s.Add(accSpace + maxSpeed * (t - accTime));
            }
            else
            {
                sdotdot.Add(0f);
                sdot.Add(0f);
                s.Add(s[s.Count-1]);
            }

            //Debug.Log("s = "+s[s.Count-1]+" sdot= "+sdot[sdot.Count-1] + " sdd= "+ sdotdot[sdotdot.Count-1]);
        }


        //Debug.Log("s final " + s[s.Count - 1]);
        //Debug.Log("sdot final " + sdot[s.Count - 1]);
        //Debug.Log("sdotdot final " + sdotdot[s.Count - 1]);

        // augmentedPath has one point for frame
        List<Vector3> augmentedPath = new List<Vector3>();

        for (int i=0; i < s.Count-1; i++)
        {
            Vector3 temp = interpol(path, s[i]*(path.Count-1)/s[s.Count-1]);

            augmentedPath.Add(temp);
        }

        // from s(t) and x(s) calculate x(t), v(t) and a(t)
        for (int i = 0; i < s.Count - 1; i++)
        {
            refSignal temp = new refSignal();
            temp.time = i * Time.fixedDeltaTime;

            temp.pos = augmentedPath[i];

            if (1<=i && (s[i] - s[i - 1])!=0)
            { 
                temp.speed = (augmentedPath[i] - augmentedPath[i - 1]) / (s[i] - s[i - 1]) * sdot[i];
            }
            else if(1<=i)
            {
                temp.speed = (augmentedPath[i] - augmentedPath[i - 1]) / Time.fixedDeltaTime;
            }
            else
            {
                temp.speed = new Vector3(0, 0, 0);
            }

            if (1 <= i)
            {
                //temp.acc = (temp.speed - trajectory[i - 1].speed) / (sdot[i] - sdot[i - 1]) * sdotdot[i];
                temp.acc = (temp.speed - trajectory[i - 1].speed) / Time.fixedDeltaTime;
            }
            else
            {
                temp.acc = new Vector3(0, 0, 0);
            }


            if (i<augmentedPath.Count-1) {
                if (augmentedPath[i + 1] == augmentedPath[i])
                {
                    temp.direction = trajectory[i - 1].direction;
                }
                else
                {
                    temp.direction = Mathf.Atan2(augmentedPath[i + 1].z - augmentedPath[i].z, augmentedPath[i + 1].x - augmentedPath[i].x);
                    temp.direction = temp.direction * Mathf.Rad2Deg;
                }
            }

            if (i > 0)
            {
                temp.dirder = (temp.direction - trajectory[i - 1].direction) / Time.fixedDeltaTime;
            }

            Debug.Log("time = " + temp.time + " p = " + temp.pos + " v = " + temp.speed + " a = " + temp.acc + " dir = " + temp.direction + " dirder = " + temp.dirder);

            trajectory.Add(temp);
        }

        return trajectory;
    }

    public float pathLength(List<Vector3> path)
    {
        float lenght = 0f;
        for (int i = 0; i < path.Count - 1; i++)
        {
            lenght = lenght + Vector3.Distance(path[i], path[i + 1]);
        }

        return lenght;
    }

    public void computeDirection(List<Vector3> path, List<float> direction)
    {
        direction.Clear();
        float dir;
        for (int i = 0; i < path.Count - 1; i++)
        {
            dir =Mathf.Atan2(path[i+1].z-path[i].z, path[i + 1].x - path[i].x);
            direction.Add(dir);
        }
    }

    public List<indexedPosition> indexedPath(List<Vector3> path)
    {
        List<indexedPosition> iPath = new List<indexedPosition>();
        indexedPosition ipos = new indexedPosition();
        float length = 0;

        for (int i = 0; i < path.Count - 1; i++)
        {
            ipos.length = length + Vector3.Distance(path[i], path[i + 1]);
            ipos.pos = path[i];
            iPath.Add(ipos);
        }

        return iPath;
    }

    public Vector3 interpol(List<Vector3> path, float s)
    {
        //Debug.Log("s= "+s + " count " + path.Count);
        Vector3 pos;
        Vector3 posp= path[(int)Mathf.Max( Mathf.Floor(s), 0)];
        Vector3 poss= path[(int)Mathf.Min( Mathf.Ceil(s), path.Count-1)];
        float deltas = s - Mathf.Floor(s);

        //pos = path[(int)Mathf.Floor(s)] + (path[(int)Mathf.Floor(s)] - path[(int)Mathf.Ceil(s)]) * (s- Mathf.Floor(s));
        pos = posp * (1 - deltas) + poss * deltas; 

        return pos;
    }

    public class PIDController
    {
        public float Kp;
        public float Ki;
        public float Kd;

        private float integral;

        public PIDController(float KP, float KI, float KD)
        {
            this.Kp = KP;
            this.Ki = KI;
            this.Kd = KD;
            this.integral = 0;
        }

        public float computeControl(float reference, float measure)
        {
            float u;
            float err = reference - measure;
            integral = integral + err*Time.fixedDeltaTime;

            u = err * Kp + integral * Ki;
            return u;
        }
    }

    public class SMcontroller
    {
        public float[] P= {0f,0f,0f,0f };
        public float mu;

        public SMcontroller(float MU,float P1, float P2, float P3, float P4)
        {
            this.mu = MU; 
            this.P[0] =P1;
            this.P[1] = P2;
            this.P[2] = P3;
            this.P[3] = P4;
        }

        public float[] computeControl(float Xref, float Yref, float Vref, float Tetaref,float x, float y, float v, float teta)
        {
            float sigma = P[0] * (x-Xref) + P[1] * (y-Yref) + P[2] * (v-Vref) + P[3] * (teta-Tetaref);
            float Pfx = P[0] * v*Mathf.Cos(teta) + P[1] * v*Mathf.Sin(teta) - P[2] * v;
            float Pg1 = P[2];
            float Pg2 = P[3]*v;
            float u1 = -Pfx / Pg1 - mu / Pg1 * Mathf.Sign(sigma);
            float u2 = -Pfx / Pg2 - mu / Pg2 * Mathf.Sign(sigma);
            float[] u= {u1, u2};
            return u;
        }
    }



    public class TachiController
    {
        public float Kp;
        public float Kd;

        public TachiController(float KP, float KD)
        {
            this.Kp = KP;
            this.Kd = KD;
        }

        public float computeControl(float refAcc, float refSpeed, float refPos, float speed, float pos)
        {
            float u =refAcc+ (refSpeed-speed)*Kd + (refPos - pos) * Kp;
            return u;
        }
    }


    public struct refSignal
    {
        public float time;
        public Vector3 pos;
        public Vector3 speed;
        public Vector3 acc;
        public float direction;
        public float dirder;
    }

    public struct indexedPosition
    {
        public float length;
        public Vector3 pos;
    }

};
