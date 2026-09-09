const video = document.querySelector('#comparison-source');
const canvas = document.querySelector('#comparison-canvas');
const context = canvas.getContext('2d');
const stage = document.querySelector('#comparison-stage');
const slider = document.querySelector('#comparison-slider');
const timeline = document.querySelector('#playback-position');
const playButton = document.querySelector('#play-toggle');
const sceneButtons = [...document.querySelectorAll('[data-scene]')];
const status = document.querySelector('#media-status');
const scenes = {
  peg: { title: 'Peg insertion', description: 'A robotic arm with a gripper grasps and lifts a round peg, then inserts it into a closely fitting hole. The peg uses a custom signed distance field; the tray and hole use mesh collision geometry.', anchor: 'peg-insertion', label: 'Peg insertion' },
  gear: { title: 'Gear assembly', description: 'A robotic arm with a gripper places the middle gear on its shaft, releases and grasps it again, then turns it in both directions. The gears use parameterized signed distance fields; the base and shafts use mesh collision geometry.', anchor: 'gear-assembly', label: 'Gear assembly' }
};
scenes['peg-teleop'] = { ...scenes.peg, title: 'Peg insertion · Teleop', label: 'Peg insertion teleoperation', description: 'Teleoperation of the peg insertion scenario.' };
scenes['gear-teleop'] = { ...scenes.gear, title: 'Gear assembly · Teleop', label: 'Gear assembly teleoperation', description: 'Teleoperation of the gear assembly scenario.' };

let currentScene = sceneButtons[0];
let version = 0;
let posters = [];
let playRequested = !matchMedia('(prefers-reduced-motion: reduce)').matches;
let sourceReady = false;
let videoFrameRequest = null;
let animationFrameRequest = null;

function syncSplit(value = slider.value) {
  const percent = Math.max(0, Math.min(100, Number(value) || 0));
  slider.value = String(percent);
  stage.style.setProperty('--split', `${percent}%`);
  slider.setAttribute('aria-valuetext', `${percent} percent surface, ${100 - percent} percent contact points`);
  return percent;
}

function sourceMatchesCurrentScene() {
  return video.currentSrc === new URL(currentScene.dataset.video, document.baseURI).href;
}

function draw() {
  const width = canvas.width, height = canvas.height;
  const split = Math.round(width * syncSplit() / 100);
  context.clearRect(0, 0, width, height);
  if (sourceReady && sourceMatchesCurrentScene() && video.readyState >= 2 && !video.seeking) {
    // Both views are encoded side by side in the same frame.
    const half = video.videoWidth / 2;
    if (split > 0) context.drawImage(video, 0, 0, half * split / width, video.videoHeight, 0, 0, split, height);
    if (split < width) context.drawImage(video, half + half * split / width, 0, half * (width - split) / width, video.videoHeight, split, 0, width - split, height);
  } else if (posters.length === 2 && posters.every(image => image.complete && image.naturalWidth)) {
    if (split > 0) context.drawImage(posters[0], 0, 0, posters[0].naturalWidth * split / width, posters[0].naturalHeight, 0, 0, split, height);
    if (split < width) context.drawImage(posters[1], posters[1].naturalWidth * split / width, 0, posters[1].naturalWidth * (width - split) / width, posters[1].naturalHeight, split, 0, width - split, height);
  }
}
function updateSplit() {
  syncSplit();
  draw();
}
slider.addEventListener('input', updateSplit);
function drag(event) {
  const rect = stage.getBoundingClientRect();
  slider.value = Math.round(Math.max(0, Math.min(100, (event.clientX - rect.left) / rect.width * 100)));
  updateSplit();
}
slider.addEventListener('pointerdown', event => {
  event.preventDefault();
  slider.focus();
  slider.setPointerCapture(event.pointerId);
  drag(event);
});
slider.addEventListener('pointermove', event => { if (slider.hasPointerCapture(event.pointerId)) drag(event); });
slider.addEventListener('pointerup', event => { if (slider.hasPointerCapture(event.pointerId)) slider.releasePointerCapture(event.pointerId); });

function stopFrameLoop() {
  if (videoFrameRequest !== null && 'cancelVideoFrameCallback' in video) {
    video.cancelVideoFrameCallback(videoFrameRequest);
    videoFrameRequest = null;
  }
  if (animationFrameRequest !== null) {
    cancelAnimationFrame(animationFrameRequest);
    animationFrameRequest = null;
  }
}

function startFrameLoop() {
  stopFrameLoop();
  if (!sourceReady || video.paused) return;
  if ('requestVideoFrameCallback' in video) {
    const frame = () => {
      videoFrameRequest = null;
      draw();
      if (!video.paused && sourceReady) videoFrameRequest = video.requestVideoFrameCallback(frame);
    };
    videoFrameRequest = video.requestVideoFrameCallback(frame);
  } else {
    const frame = () => {
      animationFrameRequest = null;
      draw();
      if (!video.paused && sourceReady) animationFrameRequest = requestAnimationFrame(frame);
    };
    animationFrameRequest = requestAnimationFrame(frame);
  }
}

function loadScene(button) {
  const token = ++version;
  stopFrameLoop();
  sourceReady = false;
  currentScene = button;
  status.textContent = '';
  for (const item of sceneButtons) item.setAttribute('aria-pressed', String(item === button));
  const scene = scenes[button.dataset.scene];
  document.querySelector('#scene-title').textContent = scene.title;
  document.querySelector('#scene-description').textContent = scene.description;
  document.querySelector('#scene-guide').href = `docs/examples/index.html#${scene.anchor}`;
  canvas.setAttribute('aria-label', `${scene.label}: surface rendering on the left, contact points on the right`);
  posters = [button.dataset.cleanPoster, button.dataset.contactPoster].map(path => {
    const image = new Image();
    image.onload = () => { if (token === version) draw(); };
    image.src = path;
    return image;
  });
  syncSplit(50);
  timeline.value = 0;
  video.pause();
  video.src = button.dataset.video;
  video.load();
  draw();
}
for (const button of sceneButtons) button.addEventListener('click', () => { if (button !== currentScene) loadScene(button); });
function syncControls() {
  playButton.textContent = video.paused ? 'Play' : 'Pause';
  playButton.setAttribute('aria-label', `${video.paused ? 'Play' : 'Pause'} recording`);
  timeline.value = video.currentTime;
}
playButton.addEventListener('click', () => {
  playRequested = video.paused;
  if (playRequested) video.play().catch(() => { playRequested = false; syncControls(); });
  else video.pause();
});
timeline.addEventListener('input', () => { if (video.readyState >= 1) video.currentTime = Number(timeline.value); });
video.addEventListener('loadedmetadata', () => {
  if (!sourceMatchesCurrentScene()) return;
  timeline.max = video.duration;
  timeline.value = 0;
  syncControls();
});
function handleVideoReady() {
  if (!sourceMatchesCurrentScene()) return;
  sourceReady = true;
  draw();
  if (playRequested) video.play().catch(() => { playRequested = false; syncControls(); });
}
video.addEventListener('loadeddata', handleVideoReady);
video.addEventListener('canplay', handleVideoReady);
video.addEventListener('emptied', () => { sourceReady = false; stopFrameLoop(); draw(); });
video.addEventListener('play', () => { syncControls(); startFrameLoop(); });
video.addEventListener('pause', () => { syncControls(); stopFrameLoop(); draw(); });
video.addEventListener('timeupdate', syncControls);
video.addEventListener('seeked', () => { draw(); syncControls(); });
video.addEventListener('error', () => {
  sourceReady = false;
  stopFrameLoop();
  status.textContent = 'The recording could not be loaded. Try reloading the page.';
});
window.addEventListener('pageshow', () => { syncSplit(50); draw(); });
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) { draw(); startFrameLoop(); }
});

loadScene(currentScene);
