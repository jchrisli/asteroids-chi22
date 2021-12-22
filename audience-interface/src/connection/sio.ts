import { io } from 'socket.io-client';
// Note: use this for deployment
const wsAddr = 'https://videochat-new.herokuapp.com/users';

//const wsAddr = 'http://localhost:5000/users';
const sio = io(wsAddr);
// TODO: maybe set up some logic for connection status --> how to share the information?
// Some kind of global object updated in closures?

export default sio;
