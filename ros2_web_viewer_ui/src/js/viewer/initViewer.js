// src/viewer/initViewer.js
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export let renderer, camera, controls, scene;

export function initViewer(canvasId = 'viewer') {
  const canvas = document.getElementById(canvasId);
  renderer = new THREE.WebGLRenderer({ antialias: true, canvas });
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x212121);

  const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambientLight);

  const fov = 75;
  const aspect = 2;
  const near = 0.0001;
  const far = 10000;
  camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
  camera.position.set(0, 0, 50);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.15;

  scene.rotateOnAxis(new THREE.Vector3(1, 0, 0), -Math.PI / 2);

  // const geometry = new THREE.SphereGeometry(0.1, 16, 16);
  // const material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
  // const marker = new THREE.Mesh(geometry, material);
  // marker.position.set(0,0,0);
  // scene.add(marker);

  renderer.setAnimationLoop(() => {
    if (resizeRendererToDisplaySize()) {
      camera.aspect = canvas.clientWidth / canvas.clientHeight;
      camera.updateProjectionMatrix();
    }
    controls.update();
    renderer.render(scene, camera);
  });
}

function resizeRendererToDisplaySize() {
  const canvas = renderer.domElement;
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const needResize = canvas.width !== width || canvas.height !== height;
  if (needResize) {
    renderer.setSize(width, height, false);
  }
  return needResize;
}