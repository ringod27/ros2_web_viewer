import { initViewer } from './js/viewer/initViewer.js';
import { sendMessageToSocket } from './js/socket/websocket.js';
import { deleteSceneEntity } from './js/viewer/pointcloud.js';
import { TOPIC_LIST } from './js/enum.js';

initViewer();


// UI events
document.getElementById("cbPointCloud").addEventListener("change", (e) => {
  e.preventDefault();
  const topic_name = TOPIC_LIST.get('POINTS');
  const msg = {
    action: e.target.checked ? "subscribe" : "unsubscribe",
    type: "pointcloud",
    topic: topic_name
  };
  if (!e.target.checked) deleteSceneEntity(msg.topic);
  sendMessageToSocket(msg);
});

document.getElementById("cbGNSS").addEventListener("change", (e) => {
  e.preventDefault();
  const topic_name = TOPIC_LIST.get('FIX');
  const msg = {
    action: e.target.checked ? "subscribe" : "unsubscribe",
    type: "navsatfix",
    topic: topic_name
  };
  if (!e.target.checked) deleteSceneEntity(msg.topic);
  sendMessageToSocket(msg);
});

document.getElementById("cbObstacle").addEventListener("change", (e) => {
  e.preventDefault();
  // const msg = {
  //   action: e.target.checked ? "subscribe" : "unsubscribe",
  //   type: "obstacle",
  //   topic: "/obstacle"
  // };
  // if (!e.target.checked) deleteSceneEntity(msg.topic);
  // sendMessageToSocket(msg);
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

topicCheckboxes.forEach(cb => cb.addEventListener("change", showPausePlayButton));

let paused = false;
pausePlayBtn.addEventListener("click", () => {
  const subscribeTopics = document.querySelectorAll('.topic-checkbox:checked');
  
  subscribeTopics.forEach((checkbox) => {
    const topicType = checkbox.dataset.topicType;
    const topicName = TOPIC_LIST.get(checkbox.dataset.topicName);

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

// // UI events



