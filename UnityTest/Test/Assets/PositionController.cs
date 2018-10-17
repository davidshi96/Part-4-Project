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
    float step = 1.0f;
    float step2 = 1.0f;
    int count = 0;

    public int resolution = 5;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

    List<float> position;
    public UKF filter;

    Vector3 Acceleration = new Vector3(0.0f, 0.0f, 0.0f);
    Vector3 Velocity = new Vector3(0.0f, 0.0f, 0.0f);
    Vector3 Predict;
    Vector3 upperLimit;
    Vector3 lowerLimit;
    Vector3 TempVelocity;
    Vector3 Force;
    Vector3[] ProjectilePath = new Vector3[5];
    float interval = 0.1f;
    float oldSize = 0.243f;

    // Stopwatches to measure frame rate 
    Stopwatch time = new Stopwatch(); // Frame rate when ball is detected
    Stopwatch time2 = new Stopwatch(); // Frame rate when ball is not detected

    bool previouslyDetected;
    bool previouslyPredicted;

    //Write some text to the test.txt file
    StreamWriter writer1;
    StreamWriter writer2;

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
        StartCoroutine("Prediction");

        // ----- WRITING DATA INTO TEXT FILES -----
        if (File.Exists(Application.dataPath + "/prefiltered.txt"))
        {
            writer1 = new StreamWriter(Application.dataPath + "/prefiltered.txt", append: true);
        }

        if (File.Exists(Application.dataPath + "/filtered.txt"))
        {
            writer2 = new StreamWriter(Application.dataPath + "/filtered.txt", append: true);
        }
    }

    void Update()
    {
        // Read data from socket connection
        string received_data = ReadSocket();
        
        if (received_data != "")
        {
            // convert string into floats
            position = received_data.Split(',').Select(float.Parse).ToList();

            // If ball has been detected
            if (position[3] == 1)
            {
                // Start timer for ball detection, end timer for ball prediction
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
                    step = Convert.ToSingle(time.Elapsed.TotalMilliseconds)/1000f; // record time step for the frame
                    time.Reset();
                    time.Start();
                }

                count = 0;
                ball.isKinematic = true;

                previousPosition = GetComponent<Rigidbody>().position;

                // Updating size of ball
                upperLimit = new Vector3(oldSize * 1.3f, oldSize * 1.3f, oldSize * 1.3f);
                lowerLimit = new Vector3(oldSize * .7f, oldSize * .7f, oldSize * .7f);
                if (position[4] != 0 && transform.localScale.sqrMagnitude < upperLimit.sqrMagnitude && transform.localScale.sqrMagnitude > lowerLimit.sqrMagnitude)
                {
                    float newSize = 2f * position[4] / 1000f;
                    transform.localScale = new Vector3(newSize, newSize, newSize);
                    oldSize = newSize;
                    D = 1.225f * 0.5f * 3.1415f * (newSize/2) * (newSize / 2) / 2f;
                    //UnityEngine.Debug.Log(Convert.ToString(position[4]) + "," + Convert.ToString(transform.localScale.x) + "\n");
                }

                // Vector of ball's position relative to the camera
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);
                writer1.WriteLine(Convert.ToString(worldPosition.x));
                
                // Enter the noisy world position into the Unscented Kalman Filter

                filter.UpdateFilter(worldPosition, Velocity, Acceleration, step);

                // Obtain filtered world position
                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);
                writer2.WriteLine(Convert.ToString(filteredPosition.x));
                
                // Calculating temporary velocity value
                TempVelocity = (filteredPosition - previousPosition) / step;

                // Making sure temp velocity value is not a false reading (too high) before setting it to ball velocity
                if (TempVelocity.magnitude < 10)
                {
                    Velocity = TempVelocity;
                    ball.velocity = Velocity;
                    Acceleration = -Vector3.Scale(Velocity, Velocity) * D / mass + Physics.gravity;
                }

                // Set ball's position to filtered position
                ball.MovePosition(filteredPosition);
            }

            // If ball has not been detected
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
                    time2.Reset();
                    time2.Start();
                }

                count++;

                // Wait 3 frames to ensure ball has left camera FOV and not just skipped detection
                if (count == 2)
                {                 
                    TempVelocity += Acceleration * step2;
                    Acceleration = -Vector3.Scale(TempVelocity, TempVelocity) * D / mass + Physics.gravity;
                    if (TempVelocity.magnitude < 1000)
                    {
                        ball.velocity = TempVelocity;
                    }
                    ball.isKinematic = false;
                    count = 0;
                }
            }
        }
    }

    void OnApplicationQuit()
    {
        if (!socket_ready)
            return;
        socket_reader.Close();
        tcp_socket.Close();
        socket_ready = false;
        writer1.Close();
        writer2.Close();
    }

    // Function to read information from socket connection
    public String ReadSocket()
    {
        if (!socket_ready)
            return "";

        if (net_stream.DataAvailable)
            return socket_reader.ReadLine();

        return "";
    }

    
    private IEnumerator Prediction()
    {
        
        Vector3 position1;
        Vector3 position2;
        Vector3 a;
        Vector3 v;
        Vector3 NextVelocity;
        Vector3 NextPosition;
        Path.SetVertexCount(resolution);
        while (true)
        {
            position1 = GetComponent<Rigidbody>().position;
            yield return new WaitForSeconds(interval); // wait half a second
            position2 = GetComponent<Rigidbody>().position;
            v = (position2 - position1) / interval;
            //Debug.Log("Velocity: " + Convert.ToString(Velocity.x) + ", " + Convert.ToString(Velocity.y) + ", " + Convert.ToString(Velocity.z));

            //convert position to speed
            for (int i = 0; i < resolution; i++)
            {
                a = -Vector3.Scale(v, v) * D / mass + Physics.gravity;
                NextVelocity = v + a * interval;
                NextPosition = position2 + interval * v + 0.5f * a * interval * interval;
                ProjectilePath[i] = NextPosition;
                position2 = NextPosition;
                v = NextVelocity;
            }
            Path.SetPositions(ProjectilePath);
            
            
           //Debug.Log("Acceleration: " + Convert.ToString(Acceleration.x) + ", " + Convert.ToString(Acceleration.y) + ", " + Convert.ToString(Acceleration.z));
        }
    }
    
}