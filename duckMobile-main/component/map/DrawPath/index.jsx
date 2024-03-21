import {StatusBar} from "expo-status-bar";
import React, {useEffect, useState} from "react";
import {StyleSheet, View, Dimensions} from "react-native";

import Svg, {
    Circle,
    Ellipse,
    G,
    Text,
    TSpan,
    TextPath,
    Path,
    Polygon,
    Polyline,
    Line,
    Rect,
    Use,
    Image,
    Symbol,
    Defs,
    LinearGradient,
    RadialGradient,
    Stop,
    ClipPath,
    Pattern,
    Mask,
} from "react-native-svg";


const windowWidth = Dimensions.get("window").width;
const windowHeight = Dimensions.get("window").height;


function calculateAngle(x1, y1, x2, y2) {

    const dx = x2 - x1;
    const dy = y2 - y1;

    const radians = Math.atan2(dy, dx);


    const degrees = radians * (180 / Math.PI);


    const normalizedDegrees = (degrees + 360) % 360;

    return normalizedDegrees;
}

export default function DrawPath({data}) {



    // if (isNaN(data)) {
    //     data = [[{x: 0, y: 0, direction: 0},{x: 10,y:150,direction: 0}], [{x: 100, y: 100, direction: 0}]];
    //         // data = [[
    //         //     {
    //         //         x: 0,
    //         //         y: 0,
    //         //         direction: 100
    //         //     },
    //         //     {
    //         //         x: 50,
    //         //         y: 0,
    //         //         direction: 100
    //         //     },
    //         //     {
    //         //         x: 300,
    //         //         y: 0,
    //         //         direction: 0
    //         //     },
    //         //     {
    //         //         x: 300,
    //         //         y: 250,
    //         //         direction: 90
    //         //     },
    //         //     {
    //         //         x: -300,
    //         //         y: 250,
    //         //         direction: undefined
    //         //     },  {
    //         //         x: -300,
    //         //         y: 0,
    //         //         direction: undefined
    //         //     }
    //         // ],[{
    //         //     x: 0,
    //         //     y: 0,
    //         //     direction: 100
    //         // },{
    //         //     x: 10,
    //         //     y: 0,
    //         //     direction: 100
    //         // },{
    //         //     x: 20,
    //         //     y: 30,
    //         //     direction: 0
    //         // },{
    //         //     x: 100,
    //         //     y: 100,
    //         //     direction: 90
    //         // },{
    //         //     x: 150,
    //         //     y: 250,
    //         //     direction: undefined
    //         // }, {
    //         //     x: 150,
    //         //     y: 0,
    //         //     direction: undefined
    //         // }]];
    // }
    // if (isNaN(data[data.length - 1]) || isNaN(data[data.length - 1]) || data[data.length - 1].x === undefined || data[data.length - 1].y === undefined) {
    //     data = [{x: 0, y: 0, direction: 0}];
    //     return  <Text>"222"</Text>;
    // }
    let factorX = 1
    let factorY = 1
    // let factorX = 8
    // let factorY = 4


    let data0 = data[0].map((item) => {
        return {x: item.y * factorX + 260, y: item.x * factorY + 240};
    });
    if ((data0.length > 1) && (data0[data0.length - 1].direction === undefined)) {
        data0[data0.length - 1].direction = calculateAngle(data0[data0.length - 2].x, data0[data0.length - 2].y, data0[data0.length - 1].x, data0[data0.length - 1].y) + 90;
    }

    let data1 = data[1].map((item) => {
        return {x: item.y * factorX + 260, y: item.x * factorY + 240};
    });
    if ((data1.length > 1) && (data1[data1.length - 1].direction === undefined)) {
        data1[data1.length - 1].direction = calculateAngle(data1[data1.length - 2].x, data1[data1.length - 2].y, data1[data1.length - 1].x, data1[data1.length - 1].y) + 90;
    }


    let lastPoint = null;
    let carPoint = data0[data0.length - 1];
    // let carPoints = `${carPoint.x - 5} ${carPoint.y - 10},${carPoint.x + 5} ${
    //     carPoint.y - 10
    // },${carPoint.x + 15} ${carPoint.y + 20},${carPoint.x - 15} ${
    //     carPoint.y + 20
    // }`;
    let carPoints = `${carPoint.x - 1.25} ${carPoint.y - 2.5},${carPoint.x + 1.25} ${
        carPoint.y - 2.5
    },${carPoint.x + 3.75} ${carPoint.y + 5},${carPoint.x - 3.75} ${
        carPoint.y + 5
    }`;


    return (
        <View style={styles.container}>
            <Svg height="100%" width="100%">
                {data0.slice(0, -1).map((item, i) => {
                    return (
                        <Rect
                            key={i + "rect"}
                            x={item.x - 2.5}
                            y={item.y - 2.5}
                            direction={45}

                            width={2.5}
                            height={2.5}
                            stroke="#fff"
                            strokeWidth="5"
                            fill="#fff"
                        />
                    );
                })}
                {/*{data0.map((item, i) => {*/}
                {/*    let e;*/}
                {/*    if (lastPoint) {*/}
                {/*        e = (*/}
                {/*            <Line*/}
                {/*                key={i + "line"}*/}
                {/*                x1={lastPoint.x}*/}
                {/*                y1={lastPoint.y}*/}
                {/*                x2={item.x}*/}
                {/*                y2={item.y}*/}
                {/*                stroke="#fff"*/}
                {/*                strokeWidth="1"*/}
                {/*            />*/}
                {/*        );*/}
                {/*    } else {*/}
                {/*        e = <></>;*/}
                {/*    }*/}
                {/*    lastPoint = item;*/}
                {/*    return e;*/}
                {/*})}*/}

                {/*//*/}
                {data1.slice(0, -1).map((item, i) => {
                    return (
                        <Rect
                            key={i + "rect"}
                            x={item.x - 2.5}
                            y={item.y - 2.5}
                            // opacity="0.5"
                            // rotation="30"
                            // origin={(item.x - 2.5)+ "," + (item.y - 2.5)}
                            width={2.5}
                            height={2.5}
                            stroke="#33FFCC"
                            strokeWidth="5"
                            fill="#fff"
                        />
                    );
                })}
                {/*{data1.map((item, i) => {*/}
                {/*    let e;*/}
                {/*    if (lastPoint) {*/}
                {/*        e = (*/}
                {/*            <Line*/}
                {/*                key={i + "line"}*/}
                {/*                x1={lastPoint.x}*/}
                {/*                y1={lastPoint.y}*/}
                {/*                x2={item.x}*/}
                {/*                y2={item.y}*/}
                {/*                stroke="#33FFCC"*/}
                {/*                strokeWidth="1"*/}
                {/*            />*/}
                {/*        );*/}
                {/*    } else {*/}
                {/*        e = <></>;*/}
                {/*    }*/}
                {/*    lastPoint = item;*/}
                {/*    return e;*/}
                {/*})}*/}

                <Polygon
                    rotation={carPoint.direction}
                    origin={carPoint.x + "," + carPoint.y}
                    points={carPoints}
                    fill="#ffff00"
                    stroke="#fff"
                    strokeWidth="1"
                />



                <Line x1={0} y1={0} x2={"100%"} y2={0} stroke="#fff" strokeWidth="1"/>
                <Line x1={0} y1={0} x2={0} y2={"100%"} stroke="#fff" strokeWidth="1"/>
                <Line x1={"100%"} y1={"100%"} x2={"100%"} y2={0} stroke="#fff" strokeWidth="1"/>
                <Line x1={"100%"} y1={"100%"} x2={0} y2={"100%"} stroke="#fff" strokeWidth="1"/>
            </Svg>
            {/*<Text style={{ color: "#fff", position: "absolute", x: 60, y: 60 }}>*/}
            {/*console.log("data0", data0);*/}
            {/*</Text>*/}
            {/*<Text>y</Text>*/}
        </View>
    );
}

const styles = StyleSheet.create({
    container: {
        display: "flex",
        flexDirection: "column",
        flex: 1,
        padding: 20,
        width: "90%",
        height: "90%",
        backgroundColor: "#FF7754",
    },
});
