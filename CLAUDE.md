# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is Vector?

Vector is an AI-native desktop IDE for designing, editing, and simulating robots using natural language. Users describe changes in plain English via a command bar, and the app produces structured diffs on URDF/MJCF/SDF robot model files. The project is in early prototype stage (Phase 1 MVP).

## Architecture

**Tauri desktop app** with three layers:

1. **`src-tauri/`** — Rust backend (Tauri 2.x). Handles file I/O, process management, IPC to frontend. Entry point: `src-tauri/src/lib.rs` → `main.rs`.

2. **`src/`** — TypeScript frontend (Vite + vanilla TS, no framework). Contains:
   - `index.html` — Full UI layout: activity bar, sidebar (explorer/presets/validation panels), editor panel, Three.js 3D viewport, command bar, status bar. All UI is currently in this single HTML file.
   - `src/main.ts` — All frontend logic: Three.js 3D rendering of robot models, URDF XML syntax highlighting, editor interaction, command bar, viewport controls, simulation bar UI.
   - `src/style.css` — Styling.

3. **`core/`** (planned, not yet implemented) — Python process for AI layer (Claude API), URDF/MJCF parsing (`yourdfpy`, `mujoco`), kinematic graph (`networkx`), validation, CAD generation (`cadquery`), and simulation bridge.

**Data flow:** Command bar → Tauri IPC → (future) Python core → Claude API → structured diff → editor renders diff + Three.js viewport updates.

**Key detail:** The AI operates on an internal kinematic graph (links as nodes, joints as edges via `networkx`), not raw XML. Edits mutate the graph first, then serialize back to URDF/MJCF.

## Build & Development Commands

```bash
# Prerequisites: Rust toolchain, Node.js

# Install frontend dependencies
cd src && npm install

# Run in development mode (starts Vite dev server + Tauri window)
cd src-tauri && cargo tauri dev

# Build for production
cd src-tauri && cargo tauri build

# Frontend only (no Tauri shell)
cd src && npm run dev        # Vite dev server on port 1420
cd src && npm run build      # TypeScript check + Vite build

# Type checking
cd src && npx tsc --noEmit
```

Note: `tauri.conf.json` runs `cd src && npm run dev` automatically as `beforeDevCommand`.

## Key Technical Details

- Frontend uses **Three.js** for 3D robot visualization (not Babylon.js) — the dependency is already installed.
- The editor is a custom `<textarea>` with manual syntax highlighting, not Monaco (Monaco integration is planned per STACK.md).
- A sample 3-DOF robot arm URDF is hardcoded in `main.ts` for the prototype.
- The Tauri backend is minimal — default scaffold with only the `tauri-plugin-log` plugin.
- IPC between frontend and the planned Python core will use JSON-RPC (protocol TBD: stdio vs socket vs WebSocket).
- Robot model formats supported: URDF (primary for MVP), MJCF, SDF.
