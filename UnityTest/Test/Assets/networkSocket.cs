using UnityEngine;
using System.Collections;
using System;
using System.IO;
using System.Net.Sockets;

public class networkSocket : MonoBehaviour
{
    public String host = "localhost";
    public Int32 port = 54000;

    internal Boolean socket_ready = false;
    internal String input_buffer = "";
    TcpClient tcp_socket;
    NetworkStream net_stream;

    StreamReader socket_reader;

    void Update()
    {
        string received_data = readSocket();

        if (received_data != "")
        {
        	// Do something with the received data,
        	// print it in the log for now
            Debug.Log(received_data);
        }
    }


    void Awake()
    {
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

    void OnApplicationQuit()
    {
        if (!socket_ready)
            return;
        socket_reader.Close();
        tcp_socket.Close();
        socket_ready = false;
    }

    public String readSocket()
    {
        if (!socket_ready)
            return "";

        if (net_stream.DataAvailable)
            return socket_reader.ReadLine();

        return "";
    }

}