// src/socket/websocket.js
import { createSceneEntity, updateSceneEntity } from '../viewer/pointcloud.js';

const socket = new WebSocket('ws://localhost:9090');

socket.onopen = () => {
  console.log('[WebSocket] Connected');
};

socket.onmessage = (event) => {
 
  const topic_msg = JSON.parse(event.data);

  // console.log(topic_msg.data);
  
  const topic_name = topic_msg.data.topic;
  const type = topic_msg.data.type;

  // console.log(topic_name);
  // console.log(type);


  // if (!topic_name || !type) return;

  // if (!topic_msg.data) return;

  // if (!topic_msg.data.topic) return;

  // if (!topic_msg.data.type) return;

  // if (!topic_msg.data._data_uint16) return;

  // if (!topic_msg.data._data_uint16.points) return;

  // if (!topic_msg.data._data_uint16.bounds) return;

  if (!createSceneEntity(topic_name, type)) {
    updateSceneEntity(topic_msg.data);
  }
};

socket.onclose = () => console.log('[WebSocket] Disconnected');
socket.onerror = (error) => console.error('[WebSocket] Error:', error);

export function sendMessageToSocket(msg) {
  socket.send(JSON.stringify(msg));
}
