import React, {createContext, useState, useRef, useEffect} from 'react';
import {io} from 'socket.io-client';
import Peer from 'simple-peer';

const SocketContext = createContext();

// const socket = io('https://videochat-new.herokuapp.com/');
const socket = io('http://localhost:5000/');

const ContextProvider = ({children}) => {
    const [stream, setStream] = useState(null);
    const [me, setMe] = useState('');
    const [call, setCall] = useState({});
    const [receive, setreceive] = useState(false);
    const [ended, setEnded] = useState(false);
    const [name, setName] = useState('');
    
    const myVideo = useRef();
    const userVideo = useRef();
    const connectionRef = useRef();
    // const id = props.match.params.roomID;
    useEffect(() => {
        navigator.mediaDevices.getUserMedia({video: true, audio: true})
            .then((currentStream) => {
            setStream(currentStream);
            // myVideo.current.srcObject = currentStream;
            // socket.on('me', (id) => setMe(id));
            // socket.current.emit("join", id);
            });
        
        socket.on('createPeer', ({from, name: callerName, signal}) => {
            setCall({isreceiveCall:true, from, name:callerName, signal})
        });
    // must have empty dependency otherwise it will always run
    }, []);

    return (
        <SocketContext.Provider value={{
            call,
            receive,
            userVideo,
            stream,
            name,
            setName,
            ended,
            me

        }}>
            {children}
        </SocketContext.Provider>
    );
}

export {ContextProvider, SocketContext}