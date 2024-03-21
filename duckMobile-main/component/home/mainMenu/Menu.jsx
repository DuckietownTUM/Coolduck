import {useState} from "react";
import {FlatList, Image, Text, TextInput, TouchableOpacity, View,} from "react-native";
import {useRouter} from "expo-router";

import styles from "./Menu.style";
import {icons, SIZES} from "../../../constants";
import {Canvas} from "@react-three/fiber";
import LoginPage from "../../master";
import Map from "../../map";
import ControlPad from "../../control";
import filter from "../../../assets/icons/filter.png";
import chevronRight from "../../../assets/icons/chevron-right.png";


const NodeTypes = ["Device Status", "Map", "Joystick", "SSH", "Camera"];


const Menu = ({loginInfo, setLoginInfo, handleClick}) => {
    const router = useRouter();
    const [activeFunctionType, setActiveFunctionType] = useState("Device Status");

    const displayTabContent = (activeFunctionType) => {
        switch (activeFunctionType) {
            case "Device Status":
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.headText}>Device Status</Text>
                        <LoginPage/>

                        {/*// <>*/}

                        {/*/!*<Canvas width="150" height="150">*!/*/}
                        {/*/!*    <ambientLight/>*!/*/}
                        {/*/!*    <sphereGeometry/>*!/*/}
                        {/*/!*    <pointLight position={[10, 10, 10]}/>*!/*/}
                        {/*/!*    <mesh>*!/*/}

                        {/*/!*        <meshStandardMaterial color="orange"/>*!/*/}
                        {/*/!*        <Box position={[0, -1.2, 0]}/>*!/*/}
                        {/*/!*        <Box position={[0, 1.2, 0]}/>*!/*/}
                        {/*/!*    </mesh>*!/*/}
                        {/*/!*</Canvas>*!/*/}
                        {/*// </>*/}
                    </View>

                );
            case "Map":
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.headText}>Map</Text>

                        {/*<Text style={styles.tabText}>Feature 1</Text>*/}
                        <Map></Map>

                    </View>
                );
            case "Joystick":
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.headText}>Joystick</Text>
                        <ControlPad></ControlPad>

                    </View>
                );
            case "SSH":
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.headText}>SSH</Text>
                    </View>
                );
            case "Camera":
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.headText}>Camera</Text>
                    </View>
                );
            default:
                return (
                    <View style={styles.tabsContainer}>
                        <Text style={styles.tabText()}>Device Status 1</Text>
                    </View>
                );
        }
    }

    return (
        <View>
            <View style={styles.container}>
                <Text style={styles.userName}>Hello</Text>
                <Text style={styles.welcomeMessage}>Connect your ROS device </Text>
            </View>

            <View style={styles.searchContainer}>
                <View style={styles.searchWrapper}>
                    <TextInput
                        style={styles.searchInput}
                        value={loginInfo}
                        onChangeText={(text) => setLoginInfo(text)}
                        placeholder='localhost:8080'
                    />
                </View>
                {/*<View style={styles.searchWrapper}>*/}
                {/*    <TextInput*/}
                {/*        style={styles.searchInput}*/}
                {/*        value={loginInfo}*/}
                {/*        onChangeText={(text) => setLoginInfo(text)}*/}
                {/*        placeholder='password'*/}
                {/*    />*/}
                {/*</View>*/}

                <TouchableOpacity style={styles.searchBtn} onPress={handleClick}>
                    <Image
                        source={icons.left}

                        resizeMode='contain'
                        style={styles.searchBtnImage}
                    />
                </TouchableOpacity>
            </View>

            <View style={styles.tabsContainer}

            >
                <FlatList
                    showsHorizontalScrollIndicator={false}
                    data={NodeTypes}
                    renderItem={({item}) => (
                        <TouchableOpacity
                            style={styles.tab(activeFunctionType, item)}
                            onPress={() => {
                                setActiveFunctionType(item);
                                // router.push(`/search/${item}`);
                            }}
                        >
                            <Text style={styles.tabText(activeFunctionType, item)}>{item}</Text>
                        </TouchableOpacity>
                    )}
                    keyExtractor={(item) => item}
                    contentContainerStyle={{columnGap: SIZES.small}}
                    horizontal
                />
                {displayTabContent(activeFunctionType)}
            </View>

        </View>
        // <Canvas>
        //     <ambientLight/>
        //     <sphereGeometry/>
        //     <pointLight position={[10, 10, 10]}/>
        //     <mesh>
        //
        //         <meshStandardMaterial color="orange"/>
        //         <Box position={[0, -1.2, 0]}/>
        //         <Box position={[0, 1.2, 0]}/>
        //     </mesh>
        // </Canvas>

    );
};

export default Menu;
