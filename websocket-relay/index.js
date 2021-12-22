// require('dotenv').config();
const app = require("express")();
const server = require("http").createServer(app);
const cors = require("cors");
const io = require("socket.io")(server, {
    cors: {
        origin: "*",
        methods: ["GET", "POST"]
    }
});

const users = {};
let max = 4;
const socketToRoom = {};
const ROBOT_NSP = '/robots'
const USER_NSP = '/users'

app.use(cors());
const PORT = process.env.PORT || 5000;

const debugLog = function(msg) {
    //console.log(`process.env.PORT is False? ${process.env.PORT}`);
    if(!process.env.PORT) {
        console.log(msg);
    }
}

app.get("/", (req, res)=>{
    res.send("Server is running");
});

// These main namespace methods will be removed eventually
io.on('connection', (socket) => {
    // socket.emit('me', socket.id);

    socket.on("join", roomID => {
        console.log("join new room");
        if (users[roomID]) {
            console.log(users[roomID]);
            const length = users[roomID].length;
            if (length === max) {
                socket.emit("full");
                return;
            }
            users[roomID].push(socket.id);
        } else {
            console.log("set users for this room");
            users[roomID] = [socket.id];
        }
        socketToRoom[socket.id] = roomID;
        const usersInThisRoom = users[roomID].filter(id => id !== socket.id);

        socket.emit("users", usersInThisRoom);
        console.log(usersInThisRoom);
    });

    socket.on('disconnect', ()=> {
        socket.broadcast.emit("ended");
    });

    socket.on("send", (data)=> {
        console.log("sending");
        io.to(data.signalTo).emit("joined",{ signal: data.signal, callerID: data.callerID });
    });

    socket.on("return", (data) => {
        io.to(data.callerID).emit("receive",{ signal: data.signal, id: socket.id });
    });

    // socket.on("pinpoint", (data) => {
    //     console.log(`Received pinpoint message x: ${data.x} y: ${data.y} w: ${data.w} h: ${data.h}`);
    //     socket.broadcast.emit("robot-go", {x: data.x, y: data.y, w: data.w, h: data.h});
    // });
});

// User (desktop browser UI) logic
io.of(USER_NSP).on('connection', (socket) => {
    debugLog('New connection!');

    const relayToRobots = (evtName, data) => {
        debugLog(`Relaying event ${evtName} data ${data} to robots`);
        io.of(ROBOT_NSP).emit(evtName, data);
    };

    socket.on("robot-go", (data) => {
        //console.log(`Received pinpoint message x: ${data.x} y: ${data.y} w: ${data.w} h: ${data.h}`);
        relayToRobots('robot-go', data);
    });

    socket.on('robot-select', (data) => {
        relayToRobots('robot-select', data);
    });

    socket.on('robot-rc', (data) => {
        relayToRobots('robot-rc', data);
    });
});

io.of(ROBOT_NSP).on('connection', (socket) => {
    debugLog(`New connection on ${ROBOT_NSP}`);
    socket.on('robot-update', (data) => {
        io.of(USER_NSP).emit('robot-update', data);
    });

    socket.on('workspace-update', (data) => {
        io.of(USER_NSP).emit('workspace-update', data);
    });

    socket.on('spotlight', (data) => {
        io.of(USER_NSP).emit('spotlight', data);
    });

    socket.on('pin', (data) => {
        io.of(USER_NSP).emit('pin', data);
    });
});

server.listen(PORT, ()=>console.log(`Server listening on port: ${PORT}`));