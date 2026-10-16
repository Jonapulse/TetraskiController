#!/usr/bin/env node
/**
 * Builds both firmwares and publishes ONE GitHub release with three assets:
 *   firmware.bin    radio controller image
 *   ski_fw.bin      ski controller image (raw binary, what the radio relays)
 *   manifest.json   {"radio":"1.0.10","ski":"2.6.1"}, what the app compares against
 *
 * Plain Node (18 or newer), no dependencies, same as radio-controller/scripts/release.js.
 *
 * Usage:
 *   node scripts/publish-release.js --repo owner/name --dry-run   # build + manifest only
 *   node scripts/publish-release.js --repo owner/name             # build, then publish
 *
 * Options:
 *   --repo owner/name   required, there is no default so nothing is published by accident
 *   --tag vX.Y.Z        release tag, defaults to v<radio version>
 *   --skip-build        reuse the .pio build output that is already there
 *   --dry-run           write dist/ and print the plan, never talk to GitHub
 *
 * Publishing needs a token in the GITHUB_TOKEN environment variable. Create a
 * fine-grained personal access token on github.com (Settings > Developer
 * settings) limited to the target repo, with "Contents: Read and write".
 *
 * Test builds can not reach the real repo: publishing to REAL_REPO is refused
 * unless both versions are plain X.Y.Z (so nothing like 0.2.0-bench) and the
 * radio was built with COMMS 0.
 */

const fs = require('fs');
const os = require('os');
const path = require('path');
const { execFileSync } = require('child_process');

const REAL_REPO = 'Jonapulse/TetraskiController';

// The radio stores the ski image in a 128 KB LittleFS partition and refuses
// anything that leaves less than its safety margin free (see skiStoreBegin()
// in SkiUpdateService.ino).
const SKI_IMAGE_MAX_BYTES = 114688;

const ROOT = path.resolve(__dirname, '..');
const RADIO_DIR = path.join(ROOT, 'radio-controller');
const SKI_DIR = path.join(ROOT, 'ski-controller');
const DIST_DIR = path.join(ROOT, 'dist');

const args = process.argv.slice(2);
const dryRun = args.includes('--dry-run');
const skipBuild = args.includes('--skip-build');
function option(name) {
  const i = args.indexOf(name);
  return i >= 0 ? args[i + 1] : undefined;
}

function fail(msg) {
  console.error(`\nError: ${msg}`);
  process.exit(1);
}

const repo = option('--repo');
if (!repo || !/^[\w.-]+\/[\w.-]+$/.test(repo)) {
  fail('Pass the target repo as --repo owner/name (use a scratch repo for test builds).');
}
const isRealRepo = repo.toLowerCase() === REAL_REPO.toLowerCase();

// ─── Versions, read from the sources ────────────────────────────────────────
const radioSource = fs.readFileSync(path.join(RADIO_DIR, 'src', 'TetraEMGControl.ino'), 'utf8');
const radioMatch = radioSource.match(/#define\s+FIRMWARE_VERSION\s+"([^"]+)"/);
if (!radioMatch) fail('Could not find FIRMWARE_VERSION in TetraEMGControl.ino');
const radioVersion = radioMatch[1];
const comms = (radioSource.match(/#define\s+COMMS\s+(\d+)/) || [])[1];

// The ski's version is VERSION_MAJOR/MINOR/REV in main.cpp, the same numbers
// it reports to the radio. Checked against the built image below.
function readSkiVersion() {
  const text = fs.readFileSync(path.join(SKI_DIR, 'src', 'main.cpp'), 'utf8');
  const parts = ['MAJOR', 'MINOR', 'REV'].map(name => {
    const m = text.match(new RegExp(`#define\\s+VERSION_${name}\\s+(\\d+)`));
    if (!m) fail(`Could not find VERSION_${name} in ski-controller/src/main.cpp`);
    return Number(m[1]);
  });
  return parts.join('.');
}
const skiVersion = readSkiVersion();

const tag = option('--tag') || `v${radioVersion}`;

if (isRealRepo) {
  const plain = /^\d+\.\d+\.\d+$/;
  if (!plain.test(radioVersion) || !plain.test(skiVersion)) {
    fail(`Refusing to publish radio ${radioVersion} / ski ${skiVersion} to ${REAL_REPO}: `
      + 'only plain X.Y.Z versions go there. Use a scratch repo for test builds.');
  }
  if (comms !== '0') {
    fail(`Refusing to publish to ${REAL_REPO} with COMMS ${comms}: debug text would share the wire with the ski.`);
  }
}

// ─── Build ──────────────────────────────────────────────────────────────────
function findTool(name, candidates) {
  for (const candidate of candidates) {
    for (const ext of ['', '.exe']) {
      if (fs.existsSync(candidate + ext)) return candidate + ext;
    }
  }
  return name;  // hope it is on PATH
}
const pioHome = path.join(os.homedir(), '.platformio');
const pio = findTool('pio', [
  path.join(pioHome, 'penv', 'Scripts', 'pio'),
  path.join(pioHome, 'penv', 'bin', 'pio'),
]);
const objcopy = findTool('arm-none-eabi-objcopy', [
  path.join(pioHome, 'packages', 'toolchain-gccarmnoneeabi-teensy', 'bin', 'arm-none-eabi-objcopy'),
]);

function run(cmd, cmdArgs, cwd) {
  console.log(`> ${path.basename(cmd)} ${cmdArgs.join(' ')}   (in ${path.relative(ROOT, cwd) || '.'})`);
  try {
    execFileSync(cmd, cmdArgs, { cwd, stdio: 'inherit' });
  } catch (e) {
    fail(`${path.basename(cmd)} failed: ${e.message}`);
  }
}

if (!skipBuild) {
  run(pio, ['run'], RADIO_DIR);
  run(pio, ['run', '-e', 'teensy41'], SKI_DIR);  // the update path only exists on the Teensy 4.1 build
}

const radioBin = path.join(RADIO_DIR, '.pio', 'build', 'esp32dev', 'firmware.bin');
const skiHex = path.join(SKI_DIR, '.pio', 'build', 'teensy41', 'firmware.hex');
if (!fs.existsSync(radioBin)) fail(`Missing ${radioBin}, build the radio first.`);
if (!fs.existsSync(skiHex)) fail(`Missing ${skiHex}, build the ski controller first.`);

fs.mkdirSync(DIST_DIR, { recursive: true });
const assets = {
  'firmware.bin': path.join(DIST_DIR, 'firmware.bin'),
  'ski_fw.bin': path.join(DIST_DIR, 'ski_fw.bin'),
  'manifest.json': path.join(DIST_DIR, 'manifest.json'),
};
fs.copyFileSync(radioBin, assets['firmware.bin']);
// Teensy builds produce Intel HEX, the radio relays a raw binary
run(objcopy, ['-I', 'ihex', '-O', 'binary', '--gap-fill', '0xFF', skiHex, assets['ski_fw.bin']], SKI_DIR);
fs.writeFileSync(assets['manifest.json'], JSON.stringify({ radio: radioVersion, ski: skiVersion }) + '\n');

// ─── Checks on what was built ───────────────────────────────────────────────
// The same test the radio applies before it relays: an image that does not
// contain its own version string is the wrong image for this manifest.
const radioImage = fs.readFileSync(assets['firmware.bin']);
const skiImage = fs.readFileSync(assets['ski_fw.bin']);
if (!radioImage.includes(Buffer.from(radioVersion, 'ascii'))) {
  fail(`firmware.bin does not contain the version string "${radioVersion}". Stale build?`);
}
if (!skiImage.includes(Buffer.from(skiVersion, 'ascii'))) {
  fail(`ski_fw.bin does not contain the version string "${skiVersion}". Stale build?`);
}
if (skiImage.length > SKI_IMAGE_MAX_BYTES) {
  fail(`ski_fw.bin is ${skiImage.length} bytes, the radio can store at most ${SKI_IMAGE_MAX_BYTES}.`);
}

console.log(`\nRepo:          ${repo}${isRealRepo ? '  (the REAL repo)' : ''}`);
console.log(`Tag:           ${tag}`);
console.log(`manifest.json: ${fs.readFileSync(assets['manifest.json'], 'utf8').trim()}`);
console.log(`firmware.bin:  ${radioImage.length} bytes (radio ${radioVersion}, COMMS ${comms})`);
console.log(`ski_fw.bin:    ${skiImage.length} bytes (ski ${skiVersion})`);
console.log(`Assets are in  ${path.relative(ROOT, DIST_DIR)}${path.sep}`);

if (dryRun) {
  console.log('\n--dry-run: nothing was sent to GitHub.');
  process.exit(0);
}

// ─── Publish ────────────────────────────────────────────────────────────────
const token = process.env.GITHUB_TOKEN;
if (!token) fail('Set GITHUB_TOKEN to a token that can write to the repo (see the top of this script).');

async function github(method, url, body, contentType) {
  const res = await fetch(url, {
    method,
    headers: {
      Authorization: `Bearer ${token}`,
      Accept: 'application/vnd.github+json',
      'X-GitHub-Api-Version': '2022-11-28',
      'User-Agent': 'tetraski-publish-release',
      ...(body ? { 'Content-Type': contentType || 'application/json' } : {}),
    },
    body,
  });
  const text = await res.text();
  if (!res.ok) fail(`GitHub ${method} ${url} returned ${res.status}: ${text}`);
  return text ? JSON.parse(text) : {};
}

(async () => {
  // Created as a draft and published only once all three assets are up, so
  // the app can never see a "latest" release with a file missing.
  const release = await github('POST', `https://api.github.com/repos/${repo}/releases`, JSON.stringify({
    tag_name: tag,
    name: tag,
    body: `Radio ${radioVersion}, ski controller ${skiVersion}`,
    draft: true,
  }));
  console.log(`\nCreated draft release ${release.id}`);

  const uploadUrl = release.upload_url.replace(/\{.*$/, '');
  for (const [name, file] of Object.entries(assets)) {
    await github('POST', `${uploadUrl}?name=${encodeURIComponent(name)}`, fs.readFileSync(file), 'application/octet-stream');
    console.log(`Uploaded ${name}`);
  }

  const published = await github('PATCH', `https://api.github.com/repos/${repo}/releases/${release.id}`,
    JSON.stringify({ draft: false, make_latest: 'true' }));
  console.log(`\nPublished: ${published.html_url}`);
})();
