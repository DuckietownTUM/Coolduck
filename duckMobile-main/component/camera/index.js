import React from 'react';
import {View} from "react-native";
import {useState} from "@types/react";
import ROSLIB from "roslib";

const Camera = () => {
    const [status, setStatus] = useState("Not connected")
    const [message, setMessage] = useState("No message")

    const ros = new ROSLIB.Ros({encoding: 'ascii'})


    function connect() {
        // ros.connect("ws://ubuntu.local:9090")
        ros.connect("ws://coolduck.local:9001")
        ros.on('error', function (error) {
            console.log(error)
            setStatus(error)
        })

        // Find out exactly when we made a connection.
        ros.on('connection', function () {
            // console.log('Connected!')
            setStatus("Connected!")
        })

        ros.on('close', function () {
            // console.log('Connection closed')
            setStatus("Connection closed")
        })
    }

    let listener = new ROSLIB.Topic({
        ros: ros,
        name: '/chatter',
        messageType: 'std_msgs/String'
    });

    listener.subscribe(function (message) {
        // console.log('Received message on ' + listener.name + ': ' + message.data);
        setMessage(message.data)
        // listener.unsubscribe();
    });

    return (
       <View>

       </View>
    );
};

export default Camera;
