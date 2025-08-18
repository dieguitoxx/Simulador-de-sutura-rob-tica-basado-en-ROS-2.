using UnityEngine;
using System.Net.Sockets;
using System.IO;
using UnityEngine.UI;
using System;

public class mov1 : MonoBehaviour
{
    TcpListener server;
    TcpClient client;
    StreamReader reader;
    NetworkStream stream;
    StreamWriter writer;
	

	
    public void Start()
    {
        // Establece la conexión TCP/IP con Matlab
        server = new TcpListener(System.Net.IPAddress.Parse("127.0.0.1"), 55001);
        server.Start();
        Debug.Log("Esperando conexión...");
        client = server.AcceptTcpClient();
        Debug.Log("Conectado a Matlab.");
        stream = client.GetStream();
        reader = new StreamReader(stream);
        writer = new StreamWriter(stream);
		InvokeRepeating("UpdatePosiciones", 0.0f, 0.001f);
		
    }

    void UpdatePosiciones() //Posiciones
    { 
	  
	  
        while (stream.DataAvailable)
        {
            // Lee los datos de posición enviados desde Matlab
            string dataString = reader.ReadLine();
            string[] data = dataString.Split(',');

            // Actualiza las posiciones de los objetos en la escena
          //  GameObject.Find("Objeto1").transform.position = new Vector3(double.Parse(data[0]), double.Parse(data[1]), double.Parse(data[2]));
		// GameObject.Find("Objeto1").transform.position = new Vector3((float)double.Parse(data[0]), (float)double.Parse(data[1]), (float)double.Parse(data[2]));
         // GameObject.Find("Objeto1").transform.position = new Vector3(float.Parse(data[0]), float.Parse(data[1]), float.Parse(data[2]));
		   // GameObject.Find("Objeto1").transform.position = new Vector3(float.Parse(data[0], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[1], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[2], System.Globalization.CultureInfo.InvariantCulture));
		   GameObject.Find("Objeto1").transform.position = new Vector3(float.Parse(data[0], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[1], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[2], System.Globalization.CultureInfo.InvariantCulture));
		   GameObject.Find("Objeto2").transform.position = new Vector3(float.Parse(data[3], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[4], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[5], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto3").transform.position = new Vector3(float.Parse(data[6], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[7], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[8], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto4").transform.position = new Vector3(float.Parse(data[9], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[10], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[11], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto5").transform.position = new Vector3(float.Parse(data[12], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[13], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[14], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto6").transform.position = new Vector3(float.Parse(data[15], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[16], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[17], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto7").transform.position = new Vector3(float.Parse(data[18], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[19], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[20], System.Globalization.CultureInfo.InvariantCulture));
           GameObject.Find("Objeto8").transform.position = new Vector3(float.Parse(data[21], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[22], System.Globalization.CultureInfo.InvariantCulture), float.Parse(data[23], System.Globalization.CultureInfo.InvariantCulture));

		   
		    //GameObject.Find("Objeto2").transform.position = new Vector3(float.Parse(data[3]), float.Parse(data[4]), float.Parse(data[5]));
            //GameObject.Find("Objeto3").transform.position = new Vector3(float.Parse(data[6]), float.Parse(data[7]), float.Parse(data[8]));
            //GameObject.Find("Objeto4").transform.position = new Vector3(float.Parse(data[9]), float.Parse(data[10]), float.Parse(data[11]));
            //GameObject.Find("Objeto5").transform.position = new Vector3(float.Parse(data[12]), float.Parse(data[13]), float.Parse(data[14]));
            //GameObject.Find("Objeto6").transform.position = new Vector3(float.Parse(data[15]), float.Parse(data[16]), float.Parse(data[17]));
			//GameObject.Find("Objeto7").transform.position = new Vector3(float.Parse(data[18]), float.Parse(data[19]), float.Parse(data[20]));
            //GameObject.Find("Objeto8").transform.position = new Vector3(float.Parse(data[21]), float.Parse(data[22]), float.Parse(data[23]));
        }
    }

    void OnApplicationQuit()
    {
        // Cierra la conexión TCP/IP cuando la aplicación se cierra
        reader.Close();
        writer.Close();
        client.Close();
        server.Stop();
    }
}
