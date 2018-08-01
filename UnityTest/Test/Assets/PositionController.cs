using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Net.Sockets;
using System.Linq;

[RequireComponent(typeof(LineRenderer))]
public class PositionController : MonoBehaviour {

    //Remember to drag the camera to this field in the inspector
    public Transform cameraTransform;
    public Vector3 cameraRelative;
    public Vector3 worldPosition;

    public int resolution = 5;

    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    LineRenderer Path;

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
            //Vector of ball's position relative to the camera
            if (position[3] == 1)
            {
                cameraRelative.x = position[0] / 1000f;
                cameraRelative.y = position[1] / 1000f;
                cameraRelative.z = position[2] / 1000f + 0.0345f;

                // Calculates ball's position world position from its local position
                worldPosition = cameraTransform.TransformPoint(cameraRelative);

                // Set the ball's position to the calculated world position
                transform.position = worldPosition;
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
        
        float D = 1.225f * 0.5f * 3.1415f * 0.121435f*0.121435f / 2f;
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
}