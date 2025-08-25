import { useEffect, useState } from "react";
import { NavLink } from "react-router-dom";
import { time } from "three/tsl";

export default function Plotter() {

    const [points, setPoints]  = useState([])

    useEffect(() => {
        const socket = new WebSocket('ws://localhost:9000/bot/plot'); 

        socket.onopen = () => {
            console.log("Connecting to WebSocket...");
        }

        socket.onmessage = async (event) => {
            console.log("Received message:", event.data);
            let message = JSON.parse(event.data)
            setPoints(points.concat(message));
        };
  

    }, [])

    return (
        <>
            <h1>
                Plotter
            </h1>

            {
                points && points.length != 0 && points.map((point, index) => {
                    <span key={point["timestamp"] + index}>
                        {"x = " + point["x"] + ", y = " + point["y"] + ", z = " + point["z"] + ", timestamp = " + point["timestamp"]}
                    </span> 
                })
            }

            <NavLink to= "/">
                <button>End</button>
            </NavLink>
        </>
    )
}