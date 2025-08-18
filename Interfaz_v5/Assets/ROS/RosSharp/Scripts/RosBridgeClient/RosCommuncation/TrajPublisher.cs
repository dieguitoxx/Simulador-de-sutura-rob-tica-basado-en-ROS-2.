/*******************
Autores:    Juan David Ruiz (juandavidrf@unicauca.edu.co)
            Sebastian Montenegro (exlogam@unicauca.edu.co)
*******************/

/********************* Librerias ********************/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;


namespace RosSharp.RosBridgeClient
{
    public class TrajPublisher : UnityPublisher<MessageTypes.CartesianControl.CartesianTrajectory>
    {
        /**************** Variables ******************/
        private MessageTypes.CartesianControl.CartesianTrajectory message;            //Mensaje ROS de tipo CartesianTrajectory
        

        /**************** Funciones ******************/
        protected override void Start()                             //Inicializa el publicador
        {
            base.Start();
            InitializeMessage();
        }

        public void Reactivar()                                     //Reactiva el cliente ROS para publicar
        {
            base.Start();
        }

        private void InitializeMessage()                            //Inicializa el mensaje ROS
        {
            message = new MessageTypes.CartesianControl.CartesianTrajectory();
        }

        public void Trayectoria(float[,] Coordenadas, int NumMuestras, float velocidad)   //Organiza y publica la trayectoria
        {
            message.points = new MessageTypes.CartesianControl.CartesianTrajectoryPoint[NumMuestras];
            for (int j = 0; j < NumMuestras; j++)
            {
                message.points[j] = new MessageTypes.CartesianControl.CartesianTrajectoryPoint();

                message.points[j].pose.position.x = Coordenadas[j,0];
                message.points[j].pose.position.y = Coordenadas[j,1];
                message.points[j].pose.position.z = Coordenadas[j,2];
                message.points[j].pose.orientation.x = 0;
                message.points[j].pose.orientation.y = 1;
                message.points[j].pose.orientation.z = 0;
                message.points[j].pose.orientation.w = 0;
            }
            message.points[0].twist.linear.x = velocidad;
            
            UpdateMessage();
        }

        private void UpdateMessage()                                //Publica el mensaje en el nodo de ROS
        {
            Publish(message);
        }
    }
}