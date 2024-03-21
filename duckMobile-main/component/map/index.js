import {StatusBar} from 'expo-status-bar';
import {StyleSheet, Text, View, Dimensions, Button} from 'react-native';
import DrawPath from "./DrawPath";

import React, {useEffect} from 'react';
import ROSLIB from "roslib";
import {useState} from "react";
import {Image} from "react-native-svg";


const deviceWidth = Dimensions.get('window').width;
const deviceHeight = Dimensions.get('window').height;
const remainingHeight = deviceHeight - 300;


function splitStringByTripleAt(str) {
    // 使用split方法以@@@为分隔符将字符串分割成数组
    const parts = str.split("@@@");

    // 返回分割后的数组
    return parts;
}


function convertStringToList(inputString) {
    // Trim the input string to remove leading and trailing spaces and the exclamation mark
    const trimmedString = inputString.trim().slice(1, -2);

    // Split the string into an array of strings representing each point
    const pointStrings = trimmedString.split("),");

    // Map each point string to an object with x and y properties
    const pointObjects = pointStrings.map(pointStr => {
        // Clean the string and split into x and y components
        const [x, y] = pointStr.trim().slice(1).split(",");
        return {
            x: parseFloat(x),
            y: parseFloat(y),
            direction: 0
        };
    });

    return pointObjects;
}

const Map = () => {

    const [status, setStatus] = useState("Not connected")
    const [message, setMessage] = useState("[(0, 0)]!@@@[(0, 1)]!")
    const [list, setList] = useState([[{x: 0, y: 0, direction: 0},{x: 0, y: 1, direction: 0}],
        [{x: 0, y: 0, direction: 0},{x: 1, y: 1, direction: 0}]])

    const ros = new ROSLIB.Ros({encoding: 'ascii'})


    function connect() {
        // ros.connect("ws://ubuntu.local:9090")
        ros.connect("ws://coolduck.local:9001")
        // ros.connect("ws://y33tb0t.local:9001")
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

    //sample data
    // const string =
    // "[(0.0, 0.0), (0.311785617576267, 0.0),\n" +
    // "(0.7794638453586629, 0.00043098143749590874),\n" +
    // "(0.8562581286472232, 0.0006079030784014749),\n" +
    // "(1.2471087583103788, -0.004494966344433108),\n" +
    // "(1.6379900420038935, -0.0030541184892564277),\n" +
    // "(1.7938503267625523, 0.0001301344534835238),\n" +
    // "(1.8729313932875227, 0.0017822280012229044),\n" +
    // "(1.949674147434936, 0.004600548944687392)]!"

    useEffect(() => {
        const processedList = splitStringByTripleAt(message).map(convertStringToList);
        setList(processedList);
    }, [message])
    //
    // let list1 = [[
    //     {
    //         x: 0,
    //         y: 0,
    //         direction: 100
    //     },
    //     {
    //         x: 50,
    //         y: 0,
    //         direction: 100
    //     },
    //     {
    //         x: 300,
    //         y: 0,
    //         direction: 0
    //     },
    //     {
    //         x: 300,
    //         y: 250,
    //         direction: 90
    //     },
    //     {
    //         x: -300,
    //         y: 250,
    //         direction: undefined
    //     },  {
    //         x: -300,
    //         y: 0,
    //         direction: undefined
    //     }
    // ],[{
    //     x: 0,
    //     y: 0,
    //     direction: 100
    // },{
    //     x: 10,
    //     y: 0,
    //     direction: 100
    // },{
    //     x: 20,
    //     y: 30,
    //     direction: 0
    // },{
    //     x: 100,
    //     y: 100,
    //     direction: 90
    // },{
    //     x: 150,
    //     y: 250,
    //     direction: undefined
    // }, {
    //     x: -300,
    //     y: 0,
    //     direction: undefined
    // }]];

    const l2 = [[{x: 0, y: 0, direction: 0},{x: 0, y: 100, direction: 0}],
        [{x: 0, y: 0, direction: 0},{x: 100, y: 100, direction: 0}]]

    return (
        <View style={styles.container} ref={(view) => this.myView = view}>
            <Text>{status}</Text>
            {/*<Text>{message}</Text>*/}
            {/*<Text> {list1}</Text>*/}
            <Button title="Connect" onPress={connect}/>
            {/*{list.map((item, index) => (*/}
            {/*    <Text key={index}>*/}
            {/*        x: {item.x}, y: {item.y}, direction: {item.direction}*/}
            {/*    </Text>*/}
            {/*))}*/}
            <DrawPath data={list}></DrawPath>



        </View>
    );
};

export default Map;

// export default function App() {
//     let list = [
//         {
//             x:50,
//             y:100,
//             direction:100
//         },
//         {
//             x:100,
//             y:100,
//             direction:30
//         },
//         {
//             x:120,
//             y:200,
//             direction:50
//         },
//         {
//             x:120,
//             y:300,
//             direction:50
//         },
//         {
//             x:200,
//             y:300,
//             direction:60
//         }
//     ]
//     return (
//         <View style={styles.container}>
//             <DrawPath data={list}></DrawPath>
//         </View>
//     );
// }

const styles = StyleSheet.create({
    container: {
        flex: 1,
        // backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
        paddingTop: 20,
        height: remainingHeight
    },
});