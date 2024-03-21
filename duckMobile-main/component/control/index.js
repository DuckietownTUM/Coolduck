// // import module
// import AxisPad from 'react-native-axis-pad';
// import React from 'react';
// import {Text, View} from "react-native";
//
// const ControlPad = () => {
//     return (
//         // render
//         <View
//             style={{
//                 flex: 1,
//                 justifyContent: 'center',
//                 alignItems: 'center',
//             }}>
//             <AxisPad
//                 resetOnRelease={true}
//                 autoCenter={true}
//                 onValue={({ x, y }) => {
//                     // values are between -1 and 1
//                     console.log(x, y);
//                 }}>
//                 <Text>Optional Component</Text>
//             </AxisPad>
//
//         </View>
//
//     );
// };
//
// export default ControlPad;

import { GestureHandlerRootView } from "react-native-gesture-handler";
import { ReactNativeJoystick } from "@korsolutions/react-native-joystick";
import React from 'react';
import {StyleSheet, View} from "react-native";
// import styles from "../home/mainMenu/mainMenu.style";




const ControlPad = () => {
    return (
        <View style={styles.container}>
            <GestureHandlerRootView>
                <ReactNativeJoystick color="#06b6d4" radius={75} onMove={(data) => console.log(data)} />
            </GestureHandlerRootView>

        </View>

    );
};


export default ControlPad;


const styles = StyleSheet.create({
    container: {
        flex: 1,
        // backgroundColor: '#fff',
        alignItems: 'center',
        justifyContent: 'center',
        paddingTop: 20,
        height: 500
    },
});
