import * as THREE from 'three';
import { scene } from './initViewer.js';
import { parsebase64 } from './utils.js';

const sceneEntities = new Map();
window.sceneEntities = sceneEntities;
const z_min_slider = document.getElementById('z-min-slider');
const z_max_slider = document.getElementById('z-max-slider');
const thresholdEnabled = document.getElementById('cbObstacle');

export function createSceneEntity(topic_name, topic_type) {
  if (!sceneEntities.has(topic_name) && topic_type === "pointcloud") {
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({ size: 0.05, vertexColors: true });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
    sceneEntities.set(topic_name, points);
  } else if(!sceneEntities.has(topic_name) && topic_type === "navsatfix") {
    const marker = createWireframeBoxMarker(0.5, 1.2, 0.5, 0x00ff00);
    marker.rotation.x = -Math.PI / 2;
    scene.add(marker);
    sceneEntities.set(topic_name, marker);
  } else if(!sceneEntities.has(topic_name) && topic_type === "obstacle") {
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({ size: 0.05, vertexColors: true });
    const points = new THREE.Points(geometry, material);
    scene.add(points);
    sceneEntities.set(topic_name, points);
  }
}

export function updateSceneEntity(msg) {
  const object = sceneEntities.get(msg.topic);
  if (object && msg.type === "pointcloud") {
    updatePointcloud2(object, msg);
  } else if (object && msg.type === "navsatfix") {
    const {x, y, z} = msg.point;
    object.position.set(x, y, 0);
  } else if (object && msg.type === "obstacle") {
    updateObstacle(object, msg);
  }
}

export function deleteSceneEntity(topic_name) {
  const object = sceneEntities.get(topic_name);
  if (object) {
    if(object.geometry) {
      object.geometry.dispose();
      object.material.dispose();
      
    } else {
      object.traverse((child) => {
        // Only dispose if the child is renderable
        if (child.isMesh || child.isLine || child.isPoints) {
          if (child.geometry) child.geometry.dispose();

          if (child.material) {
            if (Array.isArray(child.material)) {
              child.material.forEach(mat => mat.dispose());
            } else {
              child.material.dispose();
            }
          }
        }
      });

    }
    scene.remove(object);
    sceneEntities.delete(topic_name);
  }
}

function updatePointcloud2(object, msg) {
  const bounds = msg._data_uint16.bounds;
  const data = parsebase64(msg._data_uint16.points);
  const view = new DataView(data);
  const numPoints = data.byteLength / 6;

  const xrange = bounds[1] - bounds[0];
  const xmin = bounds[0];
  const yrange = bounds[3] - bounds[2];
  const ymin = bounds[2];
  const zrange = bounds[5] - bounds[4];
  const zmin = bounds[4];

  const zThresholdMin = z_min_slider.value || -0.5;
  const zThresholdMax = z_max_slider.value || 2.0;

  const positions = new Float32Array(numPoints * 3);
  const colors = new Float32Array(numPoints * 3);

  for (let i = 0; i < numPoints; i++) {
    const offset = i * 6;

    const x = (view.getUint16(offset, true) / 65535) * xrange + xmin;
    const y = (view.getUint16(offset + 2, true) / 65535) * yrange + ymin;
    const z = (view.getUint16(offset + 4, true) / 65535) * zrange + zmin;

    const idx = i * 3;
    positions[idx] = x;
    positions[idx + 1] = y;
    positions[idx + 2] = z;

    if (thresholdEnabled.checked && z >= zThresholdMin && z <= zThresholdMax) {
      // Color red if Z-threshold is enabled and point is in range
      colors[idx] = 1.0;
      colors[idx + 1] = 0.0;
      colors[idx + 2] = 0.0;
    } else {
      // Default height-based HSL color
      const intensity = (z - zmin) / zrange;
      const color = new THREE.Color();
      color.setHSL((1.0 - intensity) * 0.7, 1.0, 0.5);
      colors[idx] = color.r;
      colors[idx + 1] = color.g;
      colors[idx + 2] = color.b;
    }
  }

  object.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  object.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  object.geometry.computeBoundingSphere();
}

function createWireframeBoxMarker(width, height, depth, borderColor = 0xff0000, fillColor = 0x00ff00, opacity = 0.2) {
  const geometry = new THREE.BoxGeometry(width, height, depth);

  const edges = new THREE.EdgesGeometry(geometry);
  const lineMaterial = new THREE.LineBasicMaterial({ color: borderColor });
  const wireframe = new THREE.LineSegments(edges, lineMaterial);

  const fillMaterial = new THREE.MeshBasicMaterial({
    color: fillColor,
    transparent: true,
    opacity: opacity,
    depthWrite: false,
  });
  const boxMesh = new THREE.Mesh(geometry, fillMaterial);

  const group = new THREE.Group();
  group.add(boxMesh);
  group.add(wireframe);

  return group;
}
