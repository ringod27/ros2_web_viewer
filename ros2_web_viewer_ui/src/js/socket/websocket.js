import { createSceneEntity, updateSceneEntity } from '../viewer/pointcloud.js';

let socket;
let reconnectInterval = 3000;

function connectWebSocket(url) {
  socket = new WebSocket('ws://localhost:9090');
  socket.onopen = () => {
    console.log('[WebSocket] Connected');
  };

  socket.onmessage = (event) => {
  
    const topic_msg = JSON.parse(event.data);
    
    const topic_name = topic_msg.data.topic;
    const type = topic_msg.data.type;

    if (!createSceneEntity(topic_name, type)) {
      updateSceneEntity(topic_msg.data);
    }
  };

  socket.onclose = () => {
      console.warn("WebSocket closed. Reconnecting in", reconnectInterval / 1000, "s");
      setTimeout(() => connectWebSocket(url), reconnectInterval);
  };

  socket.onerror = (error) => console.error('[WebSocket] Error:', error);

}

connectWebSocket("ws://localhost:9090");

export function sendMessageToSocket(msg) {
  socket.send(JSON.stringify(msg));
}


