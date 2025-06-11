// src/viewer/pointcloud.js
import * as THREE from 'three';
import { scene } from './initViewer.js';
import { parsebase64 } from './utils.js';

const sceneEntities = new Map();

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

    // const geometry = new THREE.SphereGeometry(0.1, 16, 16);
    // const material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
    // const marker = new THREE.Mesh(geometry, material);
    
    scene.add(marker);
    sceneEntities.set(topic_name, marker);
  }
}

export function updateSceneEntity(msg) {
  const object = sceneEntities.get(msg.topic);
  if (object && msg.type === "pointcloud") {
    updatePointcloud2(object, msg);
  } else {
    const {x, y, z} = msg.point;
    object.position.set(x, y, 0);
  }
}

export function deleteSceneEntity(topic_name) {
  const object = sceneEntities.get(topic_name);
  if (object) {
    object.geometry.dispose();
    object.material.dispose();
    scene.remove(object);
    sceneEntities.delete(topic_name);
  }
}

function updatePointcloud2(object, msg) {
  const bounds = msg._data_uint16.bounds;
  const data = parsebase64(msg._data_uint16.points);
  const view = new DataView(data);
  const numPoints = data.byteLength / 6;

  const positions = new Float32Array(numPoints * 3);
  const colors = new Float32Array(numPoints * 3);

  const xrange = bounds[1] - bounds[0];
  const xmin = bounds[0];
  const yrange = bounds[3] - bounds[2];
  const ymin = bounds[2];
  const zrange = bounds[5] - bounds[4];
  const zmin = bounds[4];

  for (let i = 0; i < numPoints; i++) {
    const offset = i * 6;

    const x = (view.getUint16(offset, true) / 65535) * xrange + xmin;
    const y = (view.getUint16(offset + 2, true) / 65535) * yrange + ymin;
    const z = (view.getUint16(offset + 4, true) / 65535) * zrange + zmin;

    positions[i * 3] = x;
    positions[i * 3 + 1] = y;
    positions[i * 3 + 2] = z;

    const intensity = (z - zmin) / zrange;
    const color = new THREE.Color();
    color.setHSL((1.0 - intensity) * 0.7, 1.0, 0.5);
    colors[i * 3] = color.r;
    colors[i * 3 + 1] = color.g;
    colors[i * 3 + 2] = color.b;
  }

  object.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  object.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  object.geometry.computeBoundingSphere();
}

function createWireframeBoxMarker(width, height, depth, borderColor = 0xff0000) {
  // Create box geometry
  const geometry = new THREE.BoxGeometry(width, height, depth);

  // Create edges geometry from the box geometry
  const edges = new THREE.EdgesGeometry(geometry);

  // Create line segments to display edges
  const material = new THREE.LineBasicMaterial({ color: borderColor });
  const wireframe = new THREE.LineSegments(edges, material);

  return wireframe;
}