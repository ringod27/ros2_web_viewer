import { initViewer } from './js/viewer/initViewer.js';
import { sendMessageToSocket } from './js/socket/websocket.js';
import { deleteSceneEntity } from './js/viewer/pointcloud.js';

initViewer();

document.getElementById("cbPointCloud").addEventListener("change", (e) => {
  const msg = {
    action: e.target.checked ? "subscribe" : "unsubscribe",
    type: "pointcloud",
    topic: "/sensing/lidar/hesai/pandar"
  };
  if (!e.target.checked) deleteSceneEntity(msg.topic);
  sendMessageToSocket(msg);
});

document.getElementById("cbGNSS").addEventListener("change", (e) => {
  const msg = {
    action: e.target.checked ? "subscribe" : "unsubscribe",
    type: "navsatfix",
    topic: "/ublox_gps_node/fix"
  };
  if (!e.target.checked) deleteSceneEntity(msg.topic);
  sendMessageToSocket(msg);
});

const pausePlayBtn = document.getElementById("btn-pause-play");
const topicCheckboxes = document.querySelectorAll(".topic-checkbox");

function togglePausePlay(isPaused) {
  if (isPaused) {
    pausePlayBtn.textContent = "Play";
    pausePlayBtn.style.background = "green";
  } else {
    pausePlayBtn.textContent = "Pause";
    pausePlayBtn.style.background = "rgb(165, 30, 30)";
  }
}

function showPausePlayButton() {
  const anyChecked = Array.from(topicCheckboxes).some(cb => cb.checked);
  pausePlayBtn.style.display = anyChecked ? "inline-block" : "none";

  const subscribeTopics = document.querySelectorAll('.topic-checkbox:checked');

  if (subscribeTopics.length > 0) {
    paused = false;
    togglePausePlay(paused);
  }

}

// Attach event listeners
topicCheckboxes.forEach(cb => cb.addEventListener("change", showPausePlayButton));

// Example usage:
let paused = false;
pausePlayBtn.addEventListener("click", () => {
  const subscribeTopics = document.querySelectorAll('.topic-checkbox:checked');
  
  subscribeTopics.forEach((checkbox) => {
    const topicType = checkbox.dataset.topicType;
    const topicName = checkbox.dataset.topicName;

      const msg = {
        action: paused ? "subscribe" : "unsubscribe",
        type: topicType,
        topic: topicName
      };

      sendMessageToSocket(msg);
      
  });

  paused = !paused;
  togglePausePlay(paused);
});



