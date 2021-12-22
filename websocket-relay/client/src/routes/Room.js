import {Grid, Typography, Paper} from '@material-ui/core';
import {makeStyles} from '@material-ui/core/styles';
import React, {createContext, useState, useRef, useEffect} from 'react';
import {io} from 'socket.io-client';
import Peer from 'simple-peer';
import VideoPlayer from '../components/VideoPlayer';
import styled from "styled-components";
const StyledVideo = styled.video`
`;

const SocketContext = createContext();

const socket = io('https://videochat-new.herokuapp.com/');
// const socket = io('http://localhost:5000/');
// const socket = io('/');

const useStyles = makeStyles((theme) => ({
    video: {
      width: '550px',
      [theme.breakpoints.down('xs')]: {
        width: '300px',
      },
    },
    gridContainer: {
      justifyContent: 'center',
      [theme.breakpoints.down('xs')]: {
        flexDirection: 'column',
      },
    },
    paper: {
      padding: '10px',
      border: '2px solid black',
      margin: '10px',
    },
}));

const Container = styled.div`
    padding: 20px;
    display: flex;
    height: 100vh;
    width: 90%;
    margin: auto;
    flex-wrap: wrap;
`;


const Room = (props) => {

    const classes = useStyles();

    const roomID = props.match.params.roomID

    const [stream, setStream] = useState(null);
    const [me, setMe] = useState('');
    const [call, setCall] = useState({});
    const [receive, setreceive] = useState(false);
    const [ended, setEnded] = useState(false);
    const [name, setName] = useState('');
    
    const myVideo = useRef();
    const userVideo = useRef();
    const connectionRef = useRef();

    const [peers, setPeers] = useState([]);
    const peersRef = useRef([]);
    
    const videoConstraints = {
        height: window.innerHeight/2,
        width: window.innerWidth
    };
    useEffect(() => {
        console.log("connecting");
        navigator.mediaDevices.getUserMedia({video: videoConstraints, audio: true})
            .then((currentStream) => {
            setStream(currentStream);
            userVideo.current.srcObject = currentStream;
            // socket.on('me', (id) => setMe(id));
            console.log(roomID);
            socket.emit("join", roomID);
            console.log("joining");
            socket.on("users", users => {
                const peers = [];
                users.forEach(userID => {
                    const peer = createPeer(userID, socket.id, currentStream);
                    peersRef.current.push({
                        peerID: userID,
                        peer,
                    })
                    peers.push(peer);
                })
                console.log(peers);
                setPeers(peers);
            })

            socket.on("joined", data => {
                console.log("successfully joined this room");
                const peer = addPeer(data.signal, data.callerID, currentStream);
                peersRef.current.push({
                    peerID: data.callerID,
                    peer,
                })

                setPeers(users => [...users, peer]);
            });
            
            socket.on("receive", data => {
                const item = peersRef.current.find(p => p.peerID === data.id);
                item.peer.signal(data.signal);
            });
            
        })
    // must have empty dependency otherwise it will always run
    }, []);
    
    function createPeer(signalTo, callerID, stream) {
        const peer = new Peer({
            initiator: true,  
            config: { iceServers: [{ urls: 'stun:stun.l.google.com:19302' },  
            {
                url: 'turn:numb.viagenie.ca',
                credential: 'muazkh',
                username: 'webrtc@live.com'
            },
            {
            urls: 'turn:turn.anyfirewall.com:443?transport=tcp',
            credential: 'webrtc',
            username: 'webrtc'
        }] } ,
        trickle: false, stream});

        peer.on("signal", signal => {
            console.log("preparing to send");
            socket.emit("send", {signalTo, signal, callerID})
            // peer.signal(signal);
        })

        // peer.on('signal', (data) => {
        //     socket.emit('createPeer', {userToSignal, callerID, signalData: data, from: me, name});
        // });
        console.log("create peer");
        
        return peer;
    }

    function addPeer(incomingSignal, callerID, stream) {
        // setreceive(true);
        const peer = new Peer({
            initiator: false,  
            config: { 
                iceServers: [
                    { urls: 'stun:stun.l.google.com:19302' 
                    }, 
                    {
                        url: 'turn:numb.viagenie.ca',
                        credential: 'muazkh',
                        username: 'webrtc@live.com'
                    },
                    {
                    urls: 'turn:turn.anyfirewall.com:443?transport=tcp',
                    credential: 'webrtc',
                    username: 'webrtc'
        }] },trickle: false, stream});


        peer.on("signal", signal => {
            console.log("preparing to return signal");
            socket.emit("return", { signal, callerID })
        })
        console.log(incomingSignal);
        peer.signal(incomingSignal);
        return peer;
    }


    const leave = () => {
        setEnded(true);
        // socket.destroy();
        window.location.reload();
    }

    return (
        <Container>
            <StyledVideo muted ref={userVideo} autoPlay playsInline />
            {peers.map((peer, index) => {
                return (
                    <VideoPlayer key={index} peer={peer} />
                );
            })}
        </Container>
        // <Grid container className={classes.gridContainer}>
        //     {
        //         <Paper className={classes.paper}>
        //         <Grid item xs={12} md={6}>
        //             <Typography variant="h5" gutterBottom>
        //                 {name || 'Me'}
        //             </Typography>
        //             <StyledVideo playsInline muted ref={userVideo} autoPlay className={classes.video}/>
        //             {peers.map((peer, index) => {
        //                 return (
        //                 <Video key={index} peer={peer} />
        //                 );
        //             })}
        //         </Grid>
        //         </Paper>
        //     }           

            
        // </Grid>
    )
}

export default Room