using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Net.Sockets;
using System.Linq;
using System.Diagnostics;


[RequireComponent(typeof(LineRenderer))]
public class PositionController : MonoBehaviour
{
    public Rigidbody ball;

    //Remember to drag the camera to this field in the inspector
    public Transform cameraTransform;
    public Vector3 cameraRelative;
    public Vector3 worldPosition;
    public Vector3 filteredPosition;
    public Vector3 previousPosition;

    float D = 1.225f * 0.5f * 3.1415f * 0.09725f * 0.09725f / 2f;   // Using average radius of all 4 balls
    float mass = 0.0837f;
    public float step;
    public float step2;
    int count = 0;

    public int resolution = 1;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

    List<float> position;
    //UKF filter = PositionController.AddComponent<filter>();
    public UKF filter;

    Vector3 Acceleration;
    Vector3 Velocity;
    Vector3 NextVelocity;
    Vector3 NextPosition;
    Vector3 Predict;
    Vector3 upperLimit;
    Vector3 lowerLimit;
    Vector3 TempVelocity;
    Vector3 Force;
    Vector3[] ProjectilePath = new Vector3[1];
    float interval = 0.1f;
    float oldSize = 0.243f;

    Stopwatch time = new Stopwatch();
    Stopwatch time2 = new Stopwatch();

    bool previouslyDetected;
    bool previouslyPredicted;

    void Awake()
    {
        Path = GetComponent<LineRenderer>();
        filter = GetComponent<UKF>();
        try
        {
            tcp_socket = new TcpClient(host, port);
            net_stream = tcp_socket.GetStream();
            socket_reader = new StreamReader(net_stream);

            socket_ready = true;
        }
        catch (Exception e)
        {
            // Something went wrong
            UnityEngine.Debug.Log("Socket error: " + e);
        }
    }

    private void Start()
    {
        ball = GetComponent<Rigidbody>();
        ball.useGravity = true;
        ball.isKinematic = true;
        previouslyDetected = false;
        //StartCoroutine("Prediction");
        UnityEngine.Debug.Log("start position: " + Convert.ToString(transform.position.x) + ", " + Convert.ToString(transform.position.y) + ", " + Convert.ToString(transform.position.z));
    }

    void Update()
    {

        string received_data = ReadSocket();
        
        if (received_data != "")
        {
            // convert string into 3 floats
            position = received_data.Split(',').Select(float.Parse).ToList();

            if (position[3] == 1)
            {
                previouslyPredicted = false;
                time2.Reset();

                if (!previouslyDetected)
                {
                    time.Start();
                    previouslyDetected = true;
                }
                else
                {
                    time.Stop();
                    step = Convert.ToSingle(time.Elapsed.TotalMilliseconds)/1000f;
                    //UnityEngine.Debug.Log("Step: " + Convert.ToString(time.Elapsed.TotalMilliseconds) + "ms");
                    time.Reset();
                    time.Start();
                }

                count = 0;
                ball.isKinematic = true;

                previousPosition = transform.position;

                upperLimit = new Vector3(oldSize * 1.2f, oldSize * 1.2f, oldSize * 1.2f);
                lowerLimit = new Vector3(oldSize * .8f, oldSize * .8f, oldSize * .8f);
                if (position[4] != 0 && transform.localScale.sqrMagnitude < upperLimit.sqrMagnitude && transform.localScale.sqrMagnitude > lowerLimit.sqrMagnitude)
                {
                    float newSize = 2f * position[4] / 1000f;
                    transform.localScale = new Vector3(newSize, newSize, newSize);
                    oldSize = newSize;
                    //UnityEngine.Debug.Log(Convert.ToString(position[4]) + "," + Convert.ToString(transform.localScale.x) + "\n");
                }

                // Vector of ball's position relative to the camera
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);
                UnityEngine.Debug.Log("World: " + Convert.ToString(worldPosition.x) + ", " + Convert.ToString(worldPosition.y) + ", " + Convert.ToString(worldPosition.z));

                // Enter the noisy world position into the Unscented Kalman Filter
                filter.UpdateFilter(worldPosition, Velocity, Acceleration, step);

                // Obtain filtered world position
                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);

                //Debug.Log("Filtered: " + Convert.ToString(filteredPosition.x) + ", " + Convert.ToString(filteredPosition.y) + ", " + Convert.ToString(filteredPosition.z));

                TempVelocity = (filteredPosition - previousPosition) / step;
                if (TempVelocity.magnitude < 10)
                {
                    Velocity = TempVelocity;
                    ball.velocity = Velocity;
                    Acceleration = -Vector3.Scale(Velocity, Velocity) * D / mass + Physics.gravity;
                }

                UnityEngine.Debug.Log("Velocity: " + Convert.ToString(Velocity.x) + ", " + Convert.ToString(Velocity.y) + ", " + Convert.ToString(Velocity.z));

                UnityEngine.Debug.Log("Acceleration: " + Convert.ToString(Acceleration.x) + ", " + Convert.ToString(Acceleration.y) + ", " + Convert.ToString(Acceleration.z));
                ball.MovePosition(filteredPosition);
            }
            else if (position[3] == 0)
            {
                previouslyDetected = false;
                time.Reset();

                if (!previouslyPredicted)
                {
                    time2.Start();
                    previouslyPredicted = true;
                }
                else
                {
                    time2.Stop();
                    step2 = Convert.ToSingle(time2.Elapsed.TotalMilliseconds) / 1000f;
                    //UnityEngine.Debug.Log("Step: " + Convert.ToString(time2.Elapsed.TotalMilliseconds) + "ms");
                    time2.Reset();
                    time2.Start();
                }

                count++;
                //Debug.Log(Convert.ToString(count));
                //Debug.Log(Convert.ToString(ball.isKinematic));

                // Wait 3 frames 
                if (count == 2)
                {
                    //Predict = filteredPosition + 0.011f * Velocity + 0.5f * Acceleration * 0.011f * 0.011f;
                    ////update velocity and acceleration
                    //filter.UpdateFilter(Predict, Velocity, Acceleration, step);
                    //filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                    //filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                    //filteredPosition.z = Convert.ToSingle(filter.getState()[2]);
                    //UnityEngine.Debug.Log("Predicted position: " + Convert.ToString(Predict.x) + ", " + Convert.ToString(Predict.y) + ", " + Convert.ToString(Predict.z));
                    //UnityEngine.Debug.Log("Velocity: " + Convert.ToString(Velocity.x) + ", " + Convert.ToString(Velocity.y) + ", " + Convert.ToString(Velocity.z));
                    //UnityEngine.Debug.Log("Acceleration: " + Convert.ToString(Acceleration.x) + ", " + Convert.ToString(Acceleration.y) + ", " + Convert.ToString(Acceleration.z));

                    //ball.useGravity = true;
                    

                    TempVelocity += Acceleration * step2;
                    Acceleration = -Vector3.Scale(TempVelocity, TempVelocity) * D / mass + Physics.gravity;
                    UnityEngine.Debug.Log("Predicted Acc: " + Convert.ToString(Acceleration.x) + ", " + Convert.ToString(Acceleration.y) + ", " + Convert.ToString(Acceleration.z));
                    ball.velocity = TempVelocity;
                    UnityEngine.Debug.Log("Predicted Velocity: " + Convert.ToString(ball.velocity.x) + ", " + Convert.ToString(ball.velocity.y) + ", " + Convert.ToString(ball.velocity.z));
                    ball.isKinematic = false;
                    //Force = -Vector3.Scale(TempVelocity, TempVelocity) * D;
                    //UnityEngine.Debug.Log("Force: " + Convert.ToString(Force.x) + ", " + Convert.ToString(Force.y) + ", " + Convert.ToString(Force.z));
                    //ball.AddForce(Force, ForceMode.Impulse);

                    //Acceleration = -Vector3.Scale(Velocity, Velocity) * D / mass + Physics.gravity;

                    count = 0;

                    //----- THIS NEEDS TO INPUT THE NEXT "PREDICTED" POSITION INTO THE FILTER ------
                    //Predict = transform.position + interval * Velocity + 0.5f * Acceleration * interval * interval;
                    //Debug.Log("Filtered: " + Convert.ToString(filteredPosition.x) + ", " + Convert.ToString(filteredPosition.y) + ", " + Convert.ToString(filteredPosition.z));
                    // Set the ball's position to the filtered world position
                    //transform.position = filteredPosition;

                }
            }
            //Debug.Log(Convert.ToString(worldPosition.x) + " , " + Convert.ToString(worldPosition.y) + " , " + Convert.ToString(worldPosition.z) + "\n");
            //Debug.Log(Convert.ToString(filteredPosition.x) + " , " + Convert.ToString(filteredPosition.y) + " , " + Convert.ToString(filteredPosition.z) + "\n");
        }
    }

    void OnApplicationQuit()
    {
        if (!socket_ready)
            return;
        socket_reader.Close();
        tcp_socket.Close();
        socket_ready = false;
    }

    public String ReadSocket()
    {
        if (!socket_ready)
            return "";

        if (net_stream.DataAvailable)
            return socket_reader.ReadLine();

        return "";
    }

    /*
    private IEnumerator Prediction()
    {
        
        Vector3 position1;
        Vector3 position2;
        

        float D = 1.225f * 0.5f * 3.1415f * 0.121435f * 0.121435f / 2f;
        float mass = 0.0837f;
        Path.SetVertexCount(resolution);
        while (true)
        {
            position1 = transform.position;
            yield return new WaitForSeconds(interval); // wait half a second
            position2 = transform.position;
            Velocity = (position2 - position1) / interval;
            //Debug.Log("Velocity: " + Convert.ToString(Velocity.x) + ", " + Convert.ToString(Velocity.y) + ", " + Convert.ToString(Velocity.z));

            //convert position to speed
            for (int i = 0; i < resolution; i++)
            {
                Acceleration = -Vector3.Scale(Velocity, Velocity) * D / mass + Physics.gravity;
                NextVelocity = Velocity + Acceleration * interval;
                NextPosition = position2 + interval * Velocity + 0.5f * Acceleration * interval * interval;
                ProjectilePath[i] = NextPosition;
                position2 = NextPosition;
                Velocity = NextVelocity;
            }
            Path.SetPositions(ProjectilePath);
            
            
           //Debug.Log("Acceleration: " + Convert.ToString(Acceleration.x) + ", " + Convert.ToString(Acceleration.y) + ", " + Convert.ToString(Acceleration.z));
        }
    }
    */
}