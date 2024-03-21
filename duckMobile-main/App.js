import {StatusBar} from 'expo-status-bar';
import {StyleSheet, Text, View, ScrollView, SafeAreaView} from 'react-native';
import {useState} from "react";
import ROSLIB from 'roslib'
import LoginPage from "./component/master";
import {Stack, useRouter} from 'expo-router'
import {COLORS} from "./constants";
import ControlPad from "./component/control";

export default function App() {

    const router = useRouter();

    return (
        <SafeAreaView style={{flex: 1, background: COLORS.lightWhite}}>
            <View style={styles.container}>
                {/*<Stack.Screen*/}

                {/*/>*/}
                <Text>Open up App.js to start working on your app!</Text>
                {/*/!*<SendMessage/>*!/*/}
                {/*<LoginPage/>*/}
                {/*<StatusBar style="auto"/>*/}
            </View>

        </SafeAreaView>


    );
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
    },
});
