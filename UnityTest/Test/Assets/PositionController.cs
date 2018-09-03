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

    public int resolution = 1;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

    //UKF filter = PositionController.AddComponent<filter>();
    public UKF filter;

    Vector3 Acceleration;
    Vector3 Velocity;
    Vector3 Predict;
    Vector3 upperLimit;
    Vector3 lowerLimit;
    float interval = 0.1f;
    float oldSize = 0.243f;

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
                upperLimit = new Vector3(oldSize * 1.1f, oldSize * 1.1f, oldSize * 1.1f);
                lowerLimit = new Vector3(oldSize * .9f, oldSize * .9f, oldSize * .9f);
                if (position[4] !=0 && transform.localScale.sqrMagnitude < upperLimit.sqrMagnitude && transform.localScale.sqrMagnitude > lowerLimit.sqrMagnitude)
                {
                    float newSize = 2f * position[4] / 1000f;
                    transform.localScale = new Vector3(newSize, newSize, newSize);
                    oldSize = newSize;
                    Debug.Log(Convert.ToString(position[4]) + "," + Convert.ToString(transform.localScale.x) + "\n");
                }
                
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);

                
                // Enter the noisy world position into the Unscented Kalman Filter
                filter.UpdateFilter(worldPosition, Velocity, Acceleration, interval);

                // Obtain filtered world position
                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);

                // Set the ball's position to the filtered world position
                transform.position = filteredPosition;

            }
            else if (position[3] == 0 && Convert.ToSingle(filter.getState()[0]) != 0 && Convert.ToSingle(filter.getState()[1]) != 0 && Convert.ToSingle(filter.getState()[2]) != 0)
            {
                //----- THIS NEEDS TO INPUT THE NEXT "PREDICTED" POSITION INTO THE FILTER ------

                /*
                // Obtain filtered world position
                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);
                //predicted next position
                Predict = filteredPosition + interval * Velocity + 0.5f * Acceleration * interval * interval;

                filter.UpdateFilter(Predict, Velocity, Acceleration, interval);

                filteredPosition.x = Convert.ToSingle(filter.getState()[0]);
                filteredPosition.y = Convert.ToSingle(filter.getState()[1]);
                filteredPosition.z = Convert.ToSingle(filter.getState()[2]);

                // Set the ball's position to the filtered world position
                transform.position = filteredPosition;
                */

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

    private IEnumerator Prediction()
    {
        
        Vector3 position1;
        Vector3 position2;
        Vector3 NextVelocity;
        Vector3 NextPosition;
        Vector3[] ProjectilePath = new Vector3[resolution];

        float D = 1.225f * 0.5f * 3.1415f * 0.121435f * 0.121435f / 2f;
        float mass = 0.0837f;
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