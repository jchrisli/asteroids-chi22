# UI-WebSocket servers protocols
## Robot control
Always use _username_ to represent a user, and _id_ to represent a robot

The following commands will simply be forwarded to the robot controller.
* __robot-go__ \{username: string, x: number, y: number, heading: number, w: number, h: number \}
    * w, h: width and height of the video HTML element
    * w, y: raw pointer click position
* __robot-select__ \{username: string, id: number\}