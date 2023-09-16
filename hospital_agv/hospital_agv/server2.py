import asyncio
from fastapi import FastAPI, Request, Response
from fastapi.responses import HTMLResponse, StreamingResponse
import random

app = FastAPI()

# Global variables for current pose and goal
current_pose = [0.0, 0.0, 0.0]
goal = [0.0, 0.0, 0.0]

# Set of connected SSE clients
sse_clients = set()


@app.get("/")
async def index(request: Request):
    with open("static/index.html") as file:
        html_content = file.read()

    return HTMLResponse(content=html_content, media_type="text/html")


@app.get("/stream")
async def stream_endpoint(request: Request):
    async def event_generator():
        while True:
            yield f"data: {current_pose}\n\n"
            await asyncio.sleep(0.5)

    response = StreamingResponse(event_generator(), media_type="text/event-stream")
    response.headers["Cache-Control"] = "no-cache"
    response.headers["Connection"] = "keep-alive"

    await response.__dict__["_send_body"](response.__dict__["_iterable"])

    return response


async def update_data():
    while True:
        # Generate random pose and goal data
        global current_pose, goal
        current_pose = [round(random.uniform(-10.0, 10.0), 2) for _ in range(3)]
        goal = [round(random.uniform(-10.0, 10.0), 2) for _ in range(3)]

        # Notify all connected SSE clients with the updated data
        for client in sse_clients:
            await client.send(f"data: {current_pose}\n\n")

        await asyncio.sleep(0.5)


@app.on_event("startup")
async def startup_event():
    # Start a background task to update the data periodically
    asyncio.create_task(update_data())


@app.middleware("http")
async def add_sse_header(request: Request, call_next):
    response = await call_next(request)
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Access-Control-Max-Age"] = "3600"
    return response


@app.middleware("http")
async def add_sse_clients(request: Request, call_next):
    response = await call_next(request)
    if request.url.path == "/stream":
        sse_clients.add(response)
    return response


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
