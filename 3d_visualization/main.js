//Import the THREE.js library
import * as THREE from "https://cdn.skypack.dev/three@0.129.0/build/three.module.js";
// To allow for the camera to move around the scene
import { OrbitControls } from "https://cdn.skypack.dev/three@0.129.0/examples/jsm/controls/OrbitControls.js";
// To allow for importing the .gltf file
import { GLTFLoader } from "https://cdn.skypack.dev/three@0.129.0/examples/jsm/loaders/GLTFLoader.js";


const container = document.getElementById('container3D');
const width = container.clientWidth;
const height = container.clientHeight;

let scene, camera, renderer, cylinder, controls;

// Fonction pour créer un axe simple sans flèche
function createAxis(color, start, end) {
  const geometry = new THREE.BufferGeometry().setFromPoints([start, end]);
  const material = new THREE.LineBasicMaterial({
    color: color,
    depthTest: true,  // Test de profondeur activé pour que les axes respectent la profondeur
    transparent: true,
    opacity: 1.0
  });

  return new THREE.Line(geometry, material);
}
// Fonction pour créer une grille restreinte au quadrant positif d’un plan donné
function createPlaneGrid(width, height, step, color, plane) {
  const geometry = new THREE.BufferGeometry();
  const vertices = [];

  for (let i = 0; i <= width; i += step) {
    for (let j = 0; j <= height; j += step) {
      if (plane === 'XY') {
        vertices.push(i, 0, 0, i, height, 0); // lignes parallèles à Y
        vertices.push(0, j, 0, width, j, 0); // lignes parallèles à X
      }
      else if (plane === 'XZ') {
        vertices.push(i, 0, 0, i, 0, height); // lignes parallèles à Z
        vertices.push(0, 0, j, width, 0, j); // lignes parallèles à X
      }
      else if (plane === 'YZ') {
        vertices.push(0, i, 0, 0, i, height); // lignes parallèles à Z
        vertices.push(0, 0, j, 0, height, j); // lignes parallèles à Y
      }
    }
  }

  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
  const material = new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: 0.25 });
  return new THREE.LineSegments(geometry, material);
}

window.addEventListener('DOMContentLoaded', () => {
  // Initialisation de la scène
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  renderer = new THREE.WebGLRenderer();
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap; // Optionnel : ombres douces
  renderer.setSize(window.innerWidth, window.innerHeight);
  container.appendChild(renderer.domElement);

  // Lumières
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1.2);
  directionalLight.position.set(10, 10, 10);
  scene.add(directionalLight);
  const ambientLight = new THREE.AmbientLight(0xcccccc, 0.4); // lumière douce et neutre
  scene.add(ambientLight);

  // Cylindre (objet mobile)
  const loader = new GLTFLoader();
  loader.load(
    'models/can/simple_cola_can/scene.gltf',  // Remplace par le chemin vers ton fichier .glb
    function (gltf) {
      const model = gltf.scene;

      // Assurez-vous que le pivot est bien centré dans Blender, sinon on peut le recentrer ici :
      model.traverse(child => {
        if (child.isMesh) {
          child.castShadow = true;
        }
      });

      model.position.set(3, 3, 3); // même position que l'ancien cylindre
      scene.add(model);
      cylinder = model; // pour réutiliser la variable "cylinder" dans la rotation plus tard
    },
    undefined,
    function (error) {
      console.error('Erreur de chargement du modèle 3D', error);
    }
  );

  // Ajout des trois plans de grille dans le quadrant positif du repère
  const axisLength2 = 7;
  const step = 0.5;

  // Plan XY (Z=0)
  const gridXY = createPlaneGrid(axisLength2, axisLength2, step, 0x999999, 'XY');
  scene.add(gridXY);

  // Plan XZ (Y=0)
  const gridXZ = createPlaneGrid(axisLength2, axisLength2, step, 0x999999, 'XZ');
  scene.add(gridXZ);

  // Plan YZ (X=0)
  const gridYZ = createPlaneGrid(axisLength2, axisLength2, step, 0x999999, 'YZ');
  scene.add(gridYZ);

  // Repère orthonormé
  const axisLength = 7;
  const xAxis = createAxis(0xF52F57, new THREE.Vector3(0, 0, 0), new THREE.Vector3(axisLength, 0, 0)); // Axe X rouge
  const yAxis = createAxis(0xF52F57, new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, axisLength, 0)); // Axe Y vert
  const zAxis = createAxis(0xF52F57, new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, axisLength)); // Axe Z bleu

  scene.add(xAxis);
  scene.add(yAxis);
  scene.add(zAxis);

  // Caméra
  camera.position.set(9, 7.5, 9);
  controls = new OrbitControls(camera, renderer.domElement);
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  // Resize
  window.addEventListener('resize', () => {
    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  });

  // Fonction de rendu
  function render() {
    renderer.render(scene, camera);
    requestAnimationFrame(render);
  }

  render();
});

// Rafraîchir la page
document.getElementById('refreshBtn').addEventListener('click', () => {
  location.reload();
});

// Affichage des données de rotation
const gxElem = document.getElementById('gx');
const gyElem = document.getElementById('gy');
const gzElem = document.getElementById('gz');

// Données du graphique
const ctx = document.getElementById('rotationChart').getContext('2d');
const chartData = {
  labels: [],
  datasets: [
    {
      label: 'gx',
      data: [],
      borderColor: '#F52F57',
      fill: false,
    },
    {
      label: 'gy',
      data: [],
      borderColor: '#F3752B',
      fill: false,
    },
    {
      label: 'gz',
      data: [],
      borderColor: '#EDEDF4',
      fill: false,
    },
  ],
};

const rotationChart = new Chart(ctx, {
  type: 'line',
  data: chartData,
  options: {
    animation: false,
    scales: {
      x: {
        title: { display: true, text: 'Temps (s)' }
      },
      y: {
        title: { display: true, text: 'Angle (°)' },
        suggestedMin: -180,
        suggestedMax: 180
      }
    }
  }
});

// Récupération des données de rotation
setInterval(async () => {
  try {
    const response = await fetch('http://localhost:5000/orientation');
    const data = await response.json();

    if (cylinder) {
      cylinder.rotation.x = data.gx * Math.PI / 180;
      cylinder.rotation.y = data.gy * Math.PI / 180;
      cylinder.rotation.z = data.gz * Math.PI / 180;
    }

    // Affichage dans l'encadré
    gxElem.textContent = data.gx.toFixed(2);
    gyElem.textContent = data.gy.toFixed(2);
    gzElem.textContent = data.gz.toFixed(2);

    const now = new Date().toLocaleTimeString();
    rotationChart.data.labels.push(now);
    rotationChart.data.datasets[0].data.push(data.gx);
    rotationChart.data.datasets[1].data.push(data.gy);
    rotationChart.data.datasets[2].data.push(data.gz);

    // Garde uniquement les 50 dernières valeurs
    if (rotationChart.data.labels.length > 50) {
      rotationChart.data.labels.shift();
      rotationChart.data.datasets.forEach(ds => ds.data.shift());
    }

    rotationChart.update();
    
  } catch (err) {
    console.error("Erreur lors du fetch des données", err);
  }
}, 200);