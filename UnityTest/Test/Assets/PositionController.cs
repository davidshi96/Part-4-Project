/*
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Net.Sockets;
using System.Linq;
using UnscentedKalmanFilter;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Factorization;


[RequireComponent(typeof(LineRenderer))]
public class PositionController : MonoBehaviour {

    //Remember to drag the camera to this field in the inspector
    public Transform cameraTransform;
    public Vector3 cameraRelative;
    public Vector3 worldPosition;
    public Vector3 filteredPosition;

    public int resolution = 5;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

    UKF filterX = new UKF();
    UKF filterY = new UKF();
    UKF filterZ = new UKF();

    List<double> measurementsX = new List<double>();
    List<double> statesX = new List<double>();
    List<double> measurementsY = new List<double>();
    List<double> statesY = new List<double>();
    List<double> measurementsZ = new List<double>();
    List<double> statesZ = new List<double>();

    void Awake()
    {
        
        Path = GetComponent<LineRenderer>();
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
            Debug.Log("Socket error: " + e);
        }
    }

    private void Start()
    {
        StartCoroutine("Prediction");

    }

    void Update()
    {

        string received_data = ReadSocket();
        
        if (received_data != "")
        {
            // convert string into 3 floats
            var position = received_data.Split(',').Select(float.Parse).ToList();
            //Debug.Log(Convert.ToString(position[0]) + " , " + Convert.ToString(position[1]) + " , " + Convert.ToString(position[2]));
            
            //Vector of ball's position relative to the camera
            if (position[3] == 1)
            {
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;   // Offset to account for body of camera and distance from surface to centre of ball (radius)

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);

                // Add world position into measurement lists for UKF
                measurementsX.Add(worldPosition.x);
                measurementsY.Add(worldPosition.y);
                measurementsZ.Add(worldPosition.z);

                Debug.Log(Convert.ToString(measurementsX[measurementsX.Count - 1]) + " , " + Convert.ToString(worldPosition.x) + "\n");

                // Update UKF
                filterX.Update(new[] { measurementsX[measurementsX.Count - 1] });
                filterY.Update(new[] { measurementsX[measurementsY.Count - 1] });
                filterZ.Update(new[] { measurementsX[measurementsZ.Count - 1] });

                double cov = filterX.getCovariance()[0,0];
                Debug.Log(Convert.ToString(cov) + "\n");

                // Add new filtered position into state lists
                statesX.Add(filterX.getState()[0]);
                statesY.Add(filterY.getState()[0]);
                statesZ.Add(filterZ.getState()[0]);

                Debug.Log(Convert.ToString(statesX[statesX.Count - 1]));

                // Set filtered position
                filteredPosition.x = Convert.ToSingle(statesX[statesX.Count - 1]);
                filteredPosition.y = Convert.ToSingle(statesY[statesY.Count - 1]);
                filteredPosition.z = Convert.ToSingle(statesZ[statesZ.Count - 1]);

                // Set the ball's position to the calculated filtered position
                transform.position = filteredPosition;
                
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
    }

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
        float interval = 0.1f;
        Vector3 Acceleration;
        Vector3 position1;
        Vector3 position2;
        Vector3 Velocity;
        Vector3 NextPosition;
        Vector3 NextVelocity;
        
        Vector3[] ProjectilePath = new Vector3[resolution];
        
        float D = 1.225f * 0.5f * 3.1415f * 0.121435f*0.121435f / 2f;   // diameter
        float mass = 0.561f;
        Path.SetVertexCount(resolution);
        while (true)
        {
            position1 = transform.position;
            yield return new WaitForSeconds(interval); // wait half a second
            position2 = transform.position;
            Velocity = (position2 - position1) / interval;

            //convert position to speed
            for (int i = 0; i < resolution; i++)
            {
                Acceleration = - Vector3.Scale(Velocity, Velocity) * D / mass + Physics.gravity;
                NextVelocity = Velocity + Acceleration * interval;
                NextPosition = position2 + interval * Velocity + 0.5f * Acceleration * interval * interval;
                ProjectilePath[i] = NextPosition;
                position2 = NextPosition;
                Velocity = NextVelocity;
            }
            Path.SetPositions(ProjectilePath);
        }
    }
}*/





//OLD VERSION

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Net.Sockets;
using System.Linq;


[RequireComponent(typeof(LineRenderer))]
public class PositionController : MonoBehaviour
{

    

    //Remember to drag the camera to this field in the inspector
    public Transform cameraTransform;
    public Vector3 cameraRelative;
    public Vector3 worldPosition;
    public Vector3 filteredPosition;

    public int resolution = 5;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

    public UKF filter = new UKF();

    Vector3 Acceleration;
    Vector3 Velocity;
    float interval = 0.1f;

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
            Debug.Log("Socket error: " + e);
        }
    }

    private void Start()
    {
        StartCoroutine("Prediction");
    }

    void Update()
    {

        string received_data = ReadSocket();

        if (received_data != "")
        {
            // convert string into 3 floats
            var position = received_data.Split(',').Select(float.Parse).ToList();
            //Vector of ball's position relative to the camera
            if (position[3] == 1)
            {
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);

                //Debug.Log(Convert.ToString(worldPosition.x) + " , " + Convert.ToString(worldPosition.y) + " , " + Convert.ToString(worldPosition.z) + "\n");

                // Enter the noisy world position into the Unscented Kalman Filter
                filter.UpdateFilter(worldPosition, Velocity, Acceleration, interval);

                // Obtain filtered world position
                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);

                Debug.Log(Convert.ToString(filteredPosition.x) + " , " + Convert.ToString(filteredPosition.y) + " , " + Convert.ToString(filteredPosition.z) + "\n");

                // Set the ball's position to the filtered world position
                transform.position = filteredPosition;
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
    }

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
        Vector3 NextPosition;
        Vector3 NextVelocity;

        Vector3[] ProjectilePath = new Vector3[resolution];

        float D = 1.225f * 0.5f * 3.1415f * 0.121435f * 0.121435f / 2f;
        float mass = 0.561f;
        Path.SetVertexCount(resolution);
        while (true)
        {
            position1 = transform.position;
            yield return new WaitForSeconds(interval); // wait half a second
            position2 = transform.position;
            Velocity = (position2 - position1) / interval;

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
        }
    }
}