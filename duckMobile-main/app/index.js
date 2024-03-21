import {View, Text} from "react-native";
import {SafeAreaView} from "react-native";
import {Stack, useRouter} from "expo-router";
import {COLORS, SIZES, FONT, SHADOWS} from "../constants";
import {ScrollView} from "react-native";
import Menu from "../component/home/mainMenu/Menu";
import {Canvas} from "@react-three/fiber/native";

import {useState} from "react";
const Home = () => {
    const router = useRouter();
   const [loginInfo, setLoginInfo] = useState(false);
   // const

    return (
        <SafeAreaView style={{flex: 1, background: COLORS.lightWhite}}>
            <Stack.Screen
                options={{
                    headerStyle: {
                        backgroundColor: COLORS.lightWhite,
                    },
                    headerShadowVisible: false,
                    headerTitle: ''
                }}
            />
            <ScrollView showsVerticalScrollIndicator={false}>
                    <View style={{
                        flex: 1,
                        padding: SIZES.medium}}>
                        {/*<Text>Open up App.js to start working on your app!</Text>*/}
                    <Menu/>
                    </View>
            </ScrollView>
        </SafeAreaView>
        // <>
        //
        //     <View>
        //         <Welcome/>
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //         {/*<Text>Open up App.js to start working on your app!</Text>*/}
        //
        //     </View>
        //
        //     {/*<Canvas>*/}
        //     {/*    <ambientLight/>*/}
        //     {/*    <sphereGeometry/>*/}
        //     {/*    <pointLight position={[10, 10, 10]}/>*/}
        //     {/*    <mesh>*/}
        //     {/*        <sphereGeometry/>*/}
        //     {/*        <meshStandardMaterial color="orange"/>*/}
        //     {/*    </mesh>*/}
        //     {/*</Canvas>*/}
        // </>
    )
}
export default Home;