using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class VisibilitySubscriber : UnitySubscriber<MessageTypes.Std.Bool>
    {
        public GameObject objectToControl; // El objeto que quieres hacer visible o invisible

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(MessageTypes.Std.Bool message)
        {
            objectToControl.SetActive(message.data);


	   Debug.Log("Mensaje recibido: " + message.data);
           objectToControl.SetActive(message.data);


        }


    }
}

