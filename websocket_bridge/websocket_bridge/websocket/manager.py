import json

class WebSocketManager:
    def __init__(self):
        self.clients = set()

    def register(self, client):
        self.clients.add(client)

    def unregister(self, client):
        self.clients.discard(client)

    def broadcast(self, channel, payload):
        message = json.dumps({'channel': channel, 'data': payload})
        for client in list(self.clients):
            if client.ws_connection and not client.ws_connection.is_closing():
                client.write_message(message)

