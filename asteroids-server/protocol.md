# Robot-WebSocket server (UI) protocols
## Robot -> WebSocket server
* __robot-update__ \[\{ x: number, y: number, id: number, users: \[string\], heading: number, pinned: boolean \}\]
* __workspace-update__ \[\{ x1: number, y1: number, x2: number, y2:number, id: number \}\]
* __spotlight__ \{id: number, on: bool\}

## WebSocket server (UI) -> Robot:
* __robot-select__ \{ username: string, id: number \}
* __robot-go__ \{ username: string, x: number, y: number, w: number, h: number, workspace: number \} 
* __robot-rc__ \{ username: string, id: number, direction: string \}
    * direction is one of _f_, _b_, _r_, _l_, _s_ (stop) 
