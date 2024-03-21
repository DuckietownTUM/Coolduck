import React, {useState} from 'react'
import ROSLIB from 'roslib'
import {Button, Text, View} from 'react-native';

function LoginPage() {
    const [status, setStatus] = useState("Not connected")
    const [message, setMessage] = useState("No message")

    const ros = new ROSLIB.Ros({encoding: 'ascii'})


    function connect() {
        // ros.connect("ws://ubuntu.local:9090")
        ros.connect("ws://coolduck.local:9001")
        ros.on('error', function (error) {
            console.log(error)
            setStatus("error")
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
        <View style={{flex: 1, justifyContent: "center", alignItems: "center"}}>
            {/*<Text>Open up App.js to start working on your app!</Text>*/}
            <Text>{status}</Text>

            <Text>{message}</Text>

            <Button title="Connect" onPress={connect}/>
        </View>


    )
}

export default LoginPage
