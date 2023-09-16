import asyncio
from fastapi import FastAPI, Request, HTTPException, WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
#from starlette.requests import Request
import json, socket
import uvicorn

current_pose = [0.0,0.0,0.0]
goal = [0.0,0.0,0.0]

app = FastAPI()

# Serve static files (HTML, CSS, JS)
app.mount("/static", StaticFiles(directory="static"), name="static")

# Set of connected WebSocket clients
websocket_clients = set()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global current_pose, goal
    await websocket.accept()
    websocket_clients.add(websocket)
    try:
        while True:
            await websocket.receive_text()  # Handle incoming messages if needed
            #await websocket.send_json({"current_pose": current_pose, "goal": goal})
            await websocket.send_text(json.dumps({"current_pose": current_pose, "goal": goal}))
    finally:
        websocket_clients.remove(websocket)

async def update_data():
    # Update the current pose and goal data
    global current_pose, goal
    while True:

        # Notify all connected clients with the updated data
        for client in websocket_clients:
            #await client.send_json({"current_pose": current_pose, "goal": goal})
            await client.send_text(json.dumps({"current_pose": current_pose, "goal": goal}))

        await asyncio.sleep(0.5)

@app.on_event("startup")
async def startup_event():
    # Start a background task to update the data periodically
    asyncio.create_task(update_data())

@app.get("/")
async def index():
    return {"data": "OK"}

@app.get("/robot_pose")
async def robot_pose():
    global current_pose
    return {"data": current_pose}

@app.get("/goal")
async def goal_func():
    global goal
    res = {"data": None}
    print(goal)
    if goal != None:
        res = {"data": goal}
    goal = None
    return res

@app.post('/set_goal')
async def set_goal(info: Request): # var_name: var_type
    global goal
    req = await info.json()
    print(req)
    if len(req) == 3:
        status = 200
        goal = req
    else:
        raise HTTPException(status_code=400, detail="Invalid input format")

    return {
        "status" : status,
        "data" : "SUCCESS"
    }

@app.post('/set_robotstate')
async def set_robotstate(info: Request):
    global current_pose
    req = await info.json()
    #print(req)
    if len(req) == 3:
        status = 200
        current_pose = req
        print(current_pose)
    else:
        raise HTTPException(status_code=400, detail="Invalid input format")

    return {
        "status" : status,
        "data" : "SUCCESS"
    }
