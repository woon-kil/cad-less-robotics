#!/usr/bin/env node
// 로컬 Chrome으로 이미 만든 P-025 화면을 찍는다. 화면 파일은 바꾸지 않는다.

import { spawn } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import { setTimeout as delay } from "node:timers/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..");
const outDir = process.argv[2]
  ? path.resolve(root, process.argv[2])
  : path.join(root, "evidence", "P-025", "visual_render");
const pageUrl = "file://" + path.join(root, "evidence", "P-025", "viewer", "index.html");
const chrome = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";
const port = 9333;

const chromeProc = spawn(chrome, [
  "--headless=new",
  "--disable-gpu",
  "--hide-scrollbars",
  `--remote-debugging-port=${port}`,
  "--user-data-dir=/tmp/p025-chrome-profile",
  "about:blank",
], { stdio: "ignore" });

let nextId = 1;
const pending = new Map();

function send(ws, method, params = {}) {
  const id = nextId++;
  ws.send(JSON.stringify({ id, method, params }));
  return new Promise((resolve, reject) => {
    pending.set(id, { resolve, reject });
  });
}

async function connect(wsUrl) {
  const ws = new WebSocket(wsUrl);
  await new Promise((resolve, reject) => {
    ws.addEventListener("open", resolve);
    ws.addEventListener("error", reject);
  });
  ws.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.id && pending.has(message.id)) {
      const waiter = pending.get(message.id);
      pending.delete(message.id);
      if (message.error) waiter.reject(new Error(JSON.stringify(message.error)));
      else waiter.resolve(message.result);
    }
  });
  return ws;
}

async function shot(ws, file, clip) {
  const params = { format: "png" };
  if (clip) {
    params.clip = {
      x: Math.max(0, clip.x),
      y: Math.max(0, clip.y),
      width: Math.max(1, clip.width),
      height: Math.max(1, clip.height),
      scale: 1,
    };
    params.captureBeyondViewport = true;
  }
  const result = await send(ws, "Page.captureScreenshot", params);
  await writeFile(file, Buffer.from(result.data, "base64"));
  console.log(file);
}

async function box(ws, selector) {
  const expression = `(() => {
    const node = document.querySelector(${JSON.stringify(selector)});
    const rect = node.getBoundingClientRect();
    return {
      x: rect.x + window.scrollX,
      y: rect.y + window.scrollY,
      width: rect.width,
      height: rect.height,
    };
  })()`;
  const result = await send(ws, "Runtime.evaluate", { expression, returnByValue: true });
  return result.result.value;
}

async function main() {
  await mkdir(outDir, { recursive: true });
  let version;
  for (let attempt = 0; attempt < 40; attempt += 1) {
    try {
      version = await fetch(`http://127.0.0.1:${port}/json/version`).then((response) => response.json());
      break;
    } catch {
      await delay(100);
    }
  }
  if (!version) throw new Error("chrome debugging port did not open");
  const browser = await connect(version.webSocketDebuggerUrl);
  const target = await send(browser, "Target.createTarget", { url: "about:blank" });
  const pages = await fetch(`http://127.0.0.1:${port}/json/list`).then((response) => response.json());
  const page = pages.find((item) => item.id === target.targetId);
  const ws = await connect(page.webSocketDebuggerUrl);
  await send(ws, "Page.enable");
  await send(ws, "Runtime.enable");

  async function load(width, height, mobile) {
    await send(ws, "Emulation.setDeviceMetricsOverride", {
      width,
      height,
      deviceScaleFactor: 1,
      mobile,
    });
    await send(ws, "Page.navigate", { url: pageUrl });
    await delay(800);
  }

  await load(1440, 900, false);
  await shot(ws, path.join(outDir, "VR-01-overview-desktop-1440x900.png"));
  const scene1 = await box(ws, "#scene1");
  const scene3 = await box(ws, "#scene3");
  await shot(ws, path.join(outDir, "VR-02-scene1-3.png"), {
    x: scene1.x,
    y: scene1.y,
    width: scene1.width,
    height: scene3.y + scene3.height - scene1.y,
  });
  await shot(ws, path.join(outDir, "VR-03-scene4.png"), await box(ws, "#scene4"));
  await shot(ws, path.join(outDir, "VR-04-scene5.png"), await box(ws, "#scene5"));
  const scope = await box(ws, "#scope");
  const boundary = await box(ws, "#boundary");
  await shot(ws, path.join(outDir, "VR-05-boundary-and-prohibited.png"), {
    x: Math.min(scope.x, boundary.x),
    y: scope.y,
    width: Math.max(scope.width, boundary.width),
    height: boundary.y + boundary.height - scope.y,
  });

  await load(1280, 800, false);
  await shot(ws, path.join(outDir, "VR-01b-overview-laptop-1280x800.png"));
  await shot(ws, path.join(outDir, "VR-04c-scene5-laptop-1280x800.png"), await box(ws, "#scene5"));

  await load(390, 844, true);
  await shot(ws, path.join(outDir, "VR-01c-overview-mobile-390x844.png"));
  await shot(ws, path.join(outDir, "VR-04b-scene5-mobile.png"), await box(ws, "#scene5"));
  await shot(ws, path.join(outDir, "VR-05b-boundary-mobile.png"), await box(ws, "#boundary"));
}

main()
  .catch((error) => {
    console.error(error);
    process.exitCode = 1;
  })
  .finally(() => {
    chromeProc.kill("SIGKILL");
  });
