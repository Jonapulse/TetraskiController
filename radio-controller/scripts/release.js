#!/usr/bin/env node
/**
 * Automates the TetraController release sequence:
 *   1. Bump FIRMWARE_VERSION (patch by default; --minor / --major for those)
 *   2. Set COMMS to 0
 *   3. Commit, tag (vX.Y.Z), and push
 *
 * Plain Node, no dependencies — runs identically on Mac and Windows via
 * `node release.js`, unlike a .bat/.sh pair that has to be kept in sync.
 *
 * Usage:
 *   node release.js                # bump patch (X.Y.Z -> X.Y.Z+1)
 *   node release.js --minor        # bump minor (X.Y.Z -> X.Y+1.0)
 *   node release.js --major        # bump major (X.Y.Z -> X+1.0.0)
 *   node release.js --dry-run      # show the plan, change nothing
 *   node release.js --no-push      # commit + tag locally, skip pushing
 *
 * Refuses to run if the working tree has ANY uncommitted changes, tracked
 * or staged — this keeps the release commit containing exactly the version
 * bump and nothing else that happened to be sitting around.
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

const args = process.argv.slice(2);
const dryRun = args.includes('--dry-run');
const noPush = args.includes('--no-push');
const bumpType = args.includes('--major') ? 'major'
  : args.includes('--minor') ? 'minor'
  : 'patch';

function run(cmd) {
  console.log(`> ${cmd}`);
  if (!dryRun) execSync(cmd, { stdio: 'inherit' });
}

function fail(msg) {
  console.error(`\nError: ${msg}`);
  process.exit(1);
}

// ─── Preconditions ─────────────────────────────────────────────────────────
let status;
try {
  status = execSync('git status --porcelain', { encoding: 'utf8' });
} catch (e) {
  fail(`Not a git repository (or git isn't on PATH): ${e.message}`);
}
if (status.trim() !== '') {
  console.error('Working tree is not clean — commit or stash these first:\n');
  console.error(status);
  fail('Refusing to run on a dirty working tree (see comment at top of this script for why).');
}

let trackedFiles;
try {
  trackedFiles = execSync('git ls-files', { encoding: 'utf8' }).split('\n');
} catch (e) {
  fail(`Could not list tracked files: ${e.message}`);
}
const relPath = trackedFiles.find(f => path.basename(f) === 'TetraEMGControl.ino');
if (!relPath) fail('Could not find TetraEMGControl.ino among tracked files.');
const firmwareFile = path.resolve(relPath);

// ─── Read + bump version ────────────────────────────────────────────────────
let content = fs.readFileSync(firmwareFile, 'utf8');

const versionMatch = content.match(/#define\s+FIRMWARE_VERSION\s+"(\d+)\.(\d+)\.(\d+)"/);
if (!versionMatch) fail(`Could not find FIRMWARE_VERSION in ${relPath}`);
let [, majorStr, minorStr, patchStr] = versionMatch;
let major = Number(majorStr), minor = Number(minorStr), patch = Number(patchStr);
const oldVersion = `${major}.${minor}.${patch}`;

if (bumpType === 'major') { major += 1; minor = 0; patch = 0; }
else if (bumpType === 'minor') { minor += 1; patch = 0; }
else { patch += 1; }
const newVersion = `${major}.${minor}.${patch}`;

content = content.replace(
  /#define\s+FIRMWARE_VERSION\s+"\d+\.\d+\.\d+"/,
  `#define FIRMWARE_VERSION "${newVersion}"`
);

// ─── Set COMMS to 0 ─────────────────────────────────────────────────────────
const commsMatch = content.match(/#define\s+COMMS\s+(\d+)/);
if (!commsMatch) fail(`Could not find #define COMMS in ${relPath}`);
const oldComms = commsMatch[1];
content = content.replace(/#define\s+COMMS\s+\d+/, '#define COMMS 0');

// ─── Report the plan ────────────────────────────────────────────────────────
console.log(`File:            ${relPath}`);
console.log(`FIRMWARE_VERSION: ${oldVersion} -> ${newVersion}  (${bumpType} bump)`);
console.log(`COMMS:            ${oldComms} -> 0`);
console.log(`Tag:              v${newVersion}`);
console.log(`Push:             ${noPush ? 'no (--no-push)' : 'yes'}`);
console.log(`After tag+push:   COMMS -> 1 again (written but left unstaged)`);

if (dryRun) {
  console.log('\n--dry-run: nothing written, committed, tagged, or pushed.');
  process.exit(0);
}

// ─── Execute ────────────────────────────────────────────────────────────────
fs.writeFileSync(firmwareFile, content, 'utf8');

run(`git add "${relPath}"`);
run(`git commit -m "Release v${newVersion}"`);
run(`git tag v${newVersion}`);

if (!noPush) {
  run('git push origin HEAD');
  run(`git push origin v${newVersion}`);
  console.log(`\nv${newVersion} committed, tagged, and pushed. Check the Actions tab.`);
} else {
  console.log(`\nv${newVersion} committed and tagged locally. Push manually when ready:`);
  console.log(`  git push origin HEAD && git push origin v${newVersion}`);
}

// ─── Flip COMMS back to 1 for continued dev — written to disk only, left
// unstaged. Not committed: that's a deliberate choice so the tag/release
// commit stays exactly "the version bump," nothing more.
console.log('\nFlipping COMMS back to 1 (unstaged)...');
let postContent = fs.readFileSync(firmwareFile, 'utf8');
postContent = postContent.replace(/#define\s+COMMS\s+\d+/, '#define COMMS 1');
fs.writeFileSync(firmwareFile, postContent, 'utf8');
console.log('Done.');
