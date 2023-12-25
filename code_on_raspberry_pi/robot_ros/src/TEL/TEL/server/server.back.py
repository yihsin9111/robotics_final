import asyncio
# import websockets
# import websocket_server
from websocket_server import WebsocketServer, WebSocketHandler

import os
import time
from dotenv import load_dotenv

from .routes import Routes
from .types.state import State

# async def main():
#     await websockets.serve(main_handle, 'localhost', PORT)
#
#
# async def main_handle(websocket, path):
#     route = path.strip('/')
#
#     if route in Routes:
#         state = states[route]
#         await Routes[route](websocket, state)
#


def new_client(client, server):
    handler: WebSocketHandler = client["handler"]
    print(handler.read_bytes(2))


if __name__ == '__main__':

    load_dotenv()
    PORT = int(os.getenv('PORT') or 8000)

    server = WebsocketServer(host='localhost', port=PORT)

    server.set_fn_new_client(new_client)
    server.run_forever()


#     states = dict()
#     for route_name, route in Routes.items():
#         states[route_name] = State(route.initial_state)
#
#     asyncio.get_event_loop().run_until_complete(main())
#     asyncio.get_event_loop().run_forever()
