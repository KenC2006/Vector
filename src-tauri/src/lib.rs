use serde_json::{json, Value};
use std::collections::HashMap;
use std::fs;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tauri::{AppHandle, Emitter, Manager};
use tauri_plugin_dialog::DialogExt;

/// Find the project root directory (where core/ lives)
fn find_project_root() -> std::path::PathBuf {
    std::env::current_dir()
        .ok()
        .and_then(|p| {
            let mut path = p;
            while !path.join("core").exists() {
                if !path.pop() {
                    break;
                }
            }
            if path.join("core").exists() {
                Some(path)
            } else {
                None
            }
        })
        .unwrap_or_else(|| std::env::current_dir().unwrap())
}

// ── Python core processes ────────────────────────────────────────────────────
//
// The Python core (core/server.py) serves line-delimited JSON-RPC on stdio and
// handles requests one at a time. Two processes run it:
//
//   * "main" — the simulator and every fast RPC (sim_*, validate, design
//     compile/import, the instant baseline controller). It holds MuJoCo state.
//   * "ai"   — long model calls (ai_design, controller agent), which take
//     minutes. On its own process they cannot hold up sim_step or any other
//     UI RPC; cancelling one kills only this process (the sim keeps running).
//
// Each process is a `CoreClient`: requests are multiplexed by JSON-RPC id with
// no lock held while a caller waits, so concurrent callers queue inside Python
// rather than on a Rust mutex. A reader thread owns stdout and routes each
// response to the caller waiting on its id, forwards notifications (no id,
// e.g. `ai_progress`) to the UI as Tauri events, and logs and skips any line
// that is not a JSON object (stray prints can never be taken for a response).

type Reply = Result<Value, String>;

const NOT_RUNNING: &str = "Core process not running. Call start_core first.";
/// Startup ping budget: first import of mujoco / the SDK can be slow.
const PING_TIMEOUT: Duration = Duration::from_secs(60);

struct CoreClient {
    label: &'static str,
    pid: u32,
    child: Mutex<Child>,
    stdin: Mutex<BufWriter<ChildStdin>>,
    /// In-flight requests: id -> the waiting caller. Guarded together with
    /// `alive` so no request can be registered after the reader drained it.
    pending: Arc<Mutex<HashMap<u64, mpsc::Sender<Reply>>>>,
    alive: Arc<AtomicBool>,
    next_id: AtomicU64,
}

impl CoreClient {
    /// Spawn `python -m core.server`; notifications go to `emit`.
    fn spawn(
        label: &'static str,
        emit: impl Fn(&str, Value) + Send + 'static,
    ) -> Result<Arc<Self>, String> {
        let project_root = find_project_root();
        let launch = |python: &str| {
            Command::new(python)
                .arg("-m")
                .arg("core.server")
                .env("PYTHONUTF8", "1")
                .stdin(Stdio::piped())
                .stdout(Stdio::piped())
                .stderr(Stdio::inherit())
                .current_dir(&project_root)
                .spawn()
        };
        // Try "python" first, then "python3" (macOS/Linux often only have python3).
        let mut child = launch("python")
            .or_else(|_| {
                eprintln!("[Core] 'python' not found, trying 'python3'...");
                launch("python3")
            })
            .map_err(|e| {
                format!(
                    "Failed to spawn Python process (tried 'python' and 'python3'): {}",
                    e
                )
            })?;
        let stdin = child
            .stdin
            .take()
            .ok_or("Failed to open stdin for Python process")?;
        let stdout = child
            .stdout
            .take()
            .ok_or("Failed to open stdout from Python process")?;

        let pending: Arc<Mutex<HashMap<u64, mpsc::Sender<Reply>>>> = Arc::default();
        let alive = Arc::new(AtomicBool::new(true));
        {
            let (pending, alive) = (pending.clone(), alive.clone());
            let spawned = std::thread::Builder::new()
                .name(format!("core-{}-reader", label))
                .spawn(move || read_responses(label, stdout, &pending, &alive, emit));
            if let Err(e) = spawned {
                child.kill().ok();
                return Err(format!("Failed to start core reader thread: {}", e));
            }
        }
        Ok(Arc::new(CoreClient {
            label,
            pid: child.id(),
            child: Mutex::new(child),
            stdin: Mutex::new(BufWriter::new(stdin)),
            pending,
            alive,
            next_id: AtomicU64::new(1),
        }))
    }

    fn is_alive(&self) -> bool {
        if !self.alive.load(Ordering::SeqCst) {
            return false;
        }
        match self.child.lock() {
            Ok(mut child) => matches!(child.try_wait(), Ok(None)),
            Err(_) => false,
        }
    }

    fn exited_message(&self) -> String {
        format!(
            "Python core ({}) exited before answering (crashed, stopped or cancelled)",
            self.label
        )
    }

    /// Send one request and block until its response (matched by id) arrives.
    fn request(&self, method: &str, params: Value, timeout: Option<Duration>) -> Reply {
        let id = self.next_id.fetch_add(1, Ordering::SeqCst);
        let (tx, rx) = mpsc::channel();
        {
            let mut pending = self
                .pending
                .lock()
                .map_err(|_| "Core request table poisoned".to_string())?;
            if !self.alive.load(Ordering::SeqCst) {
                return Err(self.exited_message());
            }
            pending.insert(id, tx);
        }
        let line = json!({ "jsonrpc": "2.0", "method": method, "params": params, "id": id })
            .to_string();
        let written = match self.stdin.lock() {
            Ok(mut w) => w
                .write_all(line.as_bytes())
                .and_then(|_| w.write_all(b"\n"))
                .and_then(|_| w.flush())
                .map_err(|e| format!("Failed to write to Python core ({}): {}", self.label, e)),
            Err(_) => Err("Core stdin poisoned".to_string()),
        };
        if let Err(e) = written {
            self.forget(id);
            return Err(e);
        }
        let reply = match timeout {
            Some(t) => rx.recv_timeout(t).map_err(|e| match e {
                mpsc::RecvTimeoutError::Timeout => format!(
                    "Python core ({}) did not answer '{}' within {}s",
                    self.label,
                    method,
                    t.as_secs()
                ),
                mpsc::RecvTimeoutError::Disconnected => self.exited_message(),
            }),
            None => rx.recv().map_err(|_| self.exited_message()),
        };
        reply.unwrap_or_else(|e| {
            self.forget(id);
            Err(e)
        })
    }

    fn forget(&self, id: u64) {
        if let Ok(mut pending) = self.pending.lock() {
            pending.remove(&id);
        }
    }

    /// Kill the process; in-flight requests fail once its stdout closes.
    fn kill(&self) {
        if let Ok(mut child) = self.child.lock() {
            child.kill().ok();
            child.wait().ok();
        }
    }
}

/// Reader thread: route every stdout line of one core process until it closes.
fn read_responses(
    label: &str,
    stdout: impl Read,
    pending: &Mutex<HashMap<u64, mpsc::Sender<Reply>>>,
    alive: &AtomicBool,
    emit: impl Fn(&str, Value),
) {
    let mut reader = BufReader::new(stdout);
    let mut buf = Vec::new();
    loop {
        buf.clear();
        match reader.read_until(b'\n', &mut buf) {
            Ok(0) | Err(_) => break,
            Ok(_) => {}
        }
        let text = String::from_utf8_lossy(&buf);
        let line = text.trim();
        if line.is_empty() {
            continue;
        }
        let msg: Value = match serde_json::from_str::<Value>(line) {
            Ok(v) if v.is_object() => v,
            _ => {
                eprintln!("[core:{}] ignoring non-JSON stdout: {}", label, line);
                continue;
            }
        };
        let Some(id) = msg.get("id").and_then(Value::as_u64) else {
            if let Some(method) = msg.get("method").and_then(Value::as_str) {
                // Notification (e.g. ai_progress): forward to the UI.
                let params = msg.get("params").cloned().unwrap_or_else(|| json!({}));
                emit(method, params);
            } else {
                eprintln!("[core:{}] ignoring response with no request id: {}", label, line);
            }
            continue;
        };
        let reply = if let Some(result) = msg.get("result") {
            Ok(result.clone())
        } else if let Some(error) = msg.get("error") {
            Err(format!("Python error: {}", error))
        } else {
            Err("Invalid response format".to_string())
        };
        let waiter = pending.lock().ok().and_then(|mut p| p.remove(&id));
        match waiter {
            Some(tx) => {
                let _ = tx.send(reply);
            }
            None => eprintln!("[core:{}] ignoring response for unknown request id {}", label, id),
        }
    }
    // stdout closed: the process is gone. Fail everything still waiting
    // (dropping the senders wakes the callers).
    match pending.lock() {
        Ok(mut p) => {
            alive.store(false, Ordering::SeqCst);
            p.clear();
        }
        Err(_) => alive.store(false, Ordering::SeqCst),
    }
    eprintln!("[core:{}] process output closed", label);
}

/// Spawn a core process and wait until it answers a ping.
fn spawn_verified(label: &'static str, app: &AppHandle) -> Result<Arc<CoreClient>, String> {
    let app = app.clone();
    let client = CoreClient::spawn(label, move |method, params| {
        let _ = app.emit(method, params);
    })?;
    match client.request("ping", json!({}), Some(PING_TIMEOUT)) {
        Ok(_) => Ok(client),
        Err(e) => {
            client.kill();
            Err(format!(
                "Core process ({}) started but failed health check: {}",
                label, e
            ))
        }
    }
}

#[derive(Default)]
struct CoreSlot(Mutex<Option<Arc<CoreClient>>>);

impl CoreSlot {
    fn live(&self) -> Option<Arc<CoreClient>> {
        let client = self.0.lock().ok().and_then(|g| g.clone())?;
        client.is_alive().then_some(client)
    }

    fn replace(&self, client: Option<Arc<CoreClient>>) -> Option<Arc<CoreClient>> {
        match self.0.lock() {
            Ok(mut g) => std::mem::replace(&mut *g, client),
            Err(_) => None,
        }
    }
}

/// Application state: the two core processes.
#[derive(Default)]
pub struct AppState {
    main: CoreSlot,
    ai: CoreSlot,
    /// Serialises (re)spawning so concurrent start_core calls don't race.
    spawn_lock: Mutex<()>,
}

fn start_cores(app: &AppHandle) -> Result<String, String> {
    let state = app.state::<AppState>();
    let _guard = state.spawn_lock.lock().map_err(|e| e.to_string())?;
    let need_main = state.main.live().is_none();
    let need_ai = state.ai.live().is_none();
    if !need_main && !need_ai {
        return Err("Core process already running".to_string());
    }
    // Boot both in parallel (each spends a second or two importing).
    let (main, ai) = std::thread::scope(|s| {
        let main = s.spawn(|| need_main.then(|| spawn_verified("main", app)));
        let ai = s.spawn(|| need_ai.then(|| spawn_verified("ai", app)));
        (
            main.join().unwrap_or_else(|_| Some(Err("main core spawn panicked".into()))),
            ai.join().unwrap_or_else(|_| Some(Err("ai core spawn panicked".into()))),
        )
    });
    if let Some(ai) = ai {
        match ai {
            Ok(client) => {
                if let Some(old) = state.ai.replace(Some(client)) {
                    old.kill();
                }
            }
            // Not fatal: the AI core is spawned again on the next AI request.
            Err(e) => eprintln!("[Core] {}", e),
        }
    }
    if let Some(main) = main {
        let client = main?;
        if let Some(old) = state.main.replace(Some(client)) {
            old.kill();
        }
    }
    Ok("Core process started and verified".to_string())
}

/// The AI core, respawned on demand (after a cancel or crash).
fn ensure_ai_core(app: &AppHandle) -> Result<Arc<CoreClient>, String> {
    let state = app.state::<AppState>();
    if let Some(client) = state.ai.live() {
        return Ok(client);
    }
    let _guard = state.spawn_lock.lock().map_err(|e| e.to_string())?;
    if let Some(client) = state.ai.live() {
        return Ok(client);
    }
    let client = spawn_verified("ai", app)?;
    if let Some(old) = state.ai.replace(Some(client.clone())) {
        old.kill();
    }
    Ok(client)
}

async fn blocking(f: impl FnOnce() -> Reply + Send + 'static) -> Reply {
    tauri::async_runtime::spawn_blocking(f)
        .await
        .map_err(|e| format!("Core request task failed: {}", e))?
}

/// Fast RPC on the main (simulator) core.
async fn call_main(app: &AppHandle, method: &'static str, params: Value) -> Reply {
    let client = app.state::<AppState>().main.live().ok_or(NOT_RUNNING)?;
    blocking(move || client.request(method, params, None)).await
}

/// Long AI RPC on the AI core; its ai_progress notifications stream as events.
async fn call_ai(app: &AppHandle, method: &'static str, params: Value) -> Reply {
    let app = app.clone();
    blocking(move || ensure_ai_core(&app)?.request(method, params, None)).await
}

/// Start the Python core processes (errors "already running" when both are up;
/// after a cancel it restarts just the AI core).
#[tauri::command]
async fn start_core(app: AppHandle) -> Result<String, String> {
    blocking(move || start_cores(&app).map(Value::String))
        .await
        .map(|v| v.as_str().unwrap_or_default().to_string())
}

fn kill_process_tree(pid: u32) -> Result<(), String> {
    #[cfg(windows)]
    {
        let status = Command::new("taskkill")
            .arg("/PID")
            .arg(pid.to_string())
            .arg("/T")
            .arg("/F")
            .status()
            .map_err(|e| format!("Failed to run taskkill for Python core: {}", e))?;
        if status.success() {
            Ok(())
        } else {
            Err(format!("taskkill failed for Python core pid {}", pid))
        }
    }

    #[cfg(not(windows))]
    {
        let status = Command::new("kill")
            .arg("-TERM")
            .arg(pid.to_string())
            .status()
            .map_err(|e| format!("Failed to signal Python core: {}", e))?;
        if status.success() {
            Ok(())
        } else {
            Err(format!("kill failed for Python core pid {}", pid))
        }
    }
}

/// Interrupt the running AI request by killing the AI core process (never
/// waits on it). The simulator core is untouched; the next start_core or AI
/// request brings up a fresh AI core.
#[tauri::command]
async fn cancel_core_request(app: AppHandle) -> Result<String, String> {
    let Some(client) = app.state::<AppState>().ai.replace(None) else {
        return Ok("No Python core process to cancel".to_string());
    };
    let pid = client.pid;
    blocking(move || {
        let tree = kill_process_tree(pid);
        client.kill();
        tree.map(|_| Value::Null)
    })
    .await?;
    Ok("Python core request cancelled".to_string())
}

/// Stop the simulator core (releases MuJoCo state). A running AI call on the
/// AI core is not affected.
#[tauri::command]
async fn stop_core(app: AppHandle) -> Result<String, String> {
    if let Some(client) = app.state::<AppState>().main.replace(None) {
        blocking(move || {
            client.kill();
            Ok(Value::Null)
        })
        .await?;
    }
    Ok("Core process stopped".to_string())
}

/// Validate URDF content from a string (for real-time editor validation)
#[tauri::command]
async fn validate_urdf_content(app: AppHandle, urdf_content: String) -> Result<Value, String> {
    call_main(
        &app,
        "validate_urdf_content",
        json!({ "urdf_content": urdf_content }),
    )
    .await
}

/// Compile a design (explicit-pose part list) to URDF, optionally with the
/// geometry critic. Fast; runs on the main core so it never waits behind AI.
#[tauri::command]
async fn design_compile(app: AppHandle, design: Value, check: Option<bool>) -> Result<Value, String> {
    call_main(
        &app,
        "design_compile",
        json!({ "design": design, "check": check.unwrap_or(false) }),
    )
    .await
}

/// URDF -> design (the embedded design, or one rebuilt from the geometry).
#[tauri::command]
async fn design_import(app: AppHandle, urdf_content: String) -> Result<Value, String> {
    call_main(&app, "design_import", json!({ "urdf_content": urdf_content })).await
}

/// Load a robot model for simulation
#[tauri::command]
async fn sim_load(
    app: AppHandle,
    path: String,
    free_base: Option<bool>,
    seed: Option<u64>,
    terrain_config: Option<Value>,
) -> Result<Value, String> {
    let mut params = json!({ "path": path, "free_base": free_base.unwrap_or(false) });
    if let Some(s) = seed {
        params["seed"] = json!(s);
    }
    if let Some(config) = terrain_config {
        params["terrain_config"] = config;
    }
    call_main(&app, "sim_load", params).await
}

/// Step the simulation forward and return the resulting state
#[tauri::command]
async fn sim_step(app: AppHandle, n_steps: Option<u32>) -> Result<Value, String> {
    call_main(&app, "sim_step", json!({ "n_steps": n_steps.unwrap_or(1) })).await
}

/// Reset the simulation
#[tauri::command]
async fn sim_reset(app: AppHandle) -> Result<String, String> {
    let result = call_main(&app, "sim_reset", json!({})).await?;
    Ok(format!("Reset: {:?}", result))
}

/// Get the current simulation state (joint positions, velocities, etc.)
#[tauri::command]
async fn sim_get_state(app: AppHandle) -> Result<Value, String> {
    call_main(&app, "sim_get_state", json!({})).await
}

/// Set gravity vector ([gx, gy, gz], URDF/MuJoCo Z-up, default [0,0,-9.81])
#[tauri::command]
async fn sim_set_gravity(app: AppHandle, gravity: Vec<f64>) -> Result<String, String> {
    let result = call_main(&app, "sim_set_gravity", json!({ "gravity": gravity })).await?;
    Ok(format!("Gravity set: {:?}", result))
}

/// Compile and install a Python step-callback script (Phase C script runner)
#[tauri::command]
async fn sim_set_script(app: AppHandle, code: String) -> Result<Value, String> {
    call_main(&app, "sim_set_script", json!({ "code": code })).await
}

/// Generate a sim controller: `quick` = instant measured baseline (main core),
/// otherwise the controller agent writes, tests in MuJoCo and revises on the
/// AI core (streams ai_progress).
#[tauri::command]
async fn ai_gen_sim_script(
    app: AppHandle,
    prompt: String,
    urdf_content: String,
    current_script: Option<String>,
    terrain_config: Option<Value>,
    quick: Option<bool>,
) -> Result<Value, String> {
    let quick = quick.unwrap_or(false);
    let mut params = json!({
        "prompt": prompt,
        "urdf_content": urdf_content,
        "current_script": current_script.unwrap_or_default(),
        "quick": quick,
    });
    if let Some(config) = terrain_config {
        params["terrain_config"] = config;
    }
    if quick {
        call_main(&app, "ai_gen_sim_script", params).await
    } else {
        call_ai(&app, "ai_gen_sim_script", params).await
    }
}

/// Operator command the running controller sees as state["cmd"] (keyboard teleop)
#[tauri::command]
async fn sim_set_command(
    app: AppHandle,
    active: bool,
    vx: f64,
    vy: f64,
    yaw_rate: f64,
) -> Result<Value, String> {
    call_main(
        &app,
        "sim_set_command",
        json!({ "active": active, "vx": vx, "vy": vy, "yaw_rate": yaw_rate }),
    )
    .await
}

/// Explicit-pose robot designer: runs the whole design -> build -> critique ->
/// revise loop in the Python AI core and returns the finished URDF. Progress
/// is streamed as `ai_progress` events.
#[tauri::command]
async fn ai_design(
    app: AppHandle,
    prompt: String,
    urdf_content: Option<String>,
    images: Option<Vec<Value>>,
) -> Result<Value, String> {
    call_ai(
        &app,
        "ai_design",
        json!({
            "prompt": prompt,
            "urdf_content": urdf_content.unwrap_or_default(),
            "images": images.unwrap_or_default(),
        }),
    )
    .await
}

/// Write editor URDF to a staging file for `sim_load`. If `neighbor_urdf_path` is set (path to an
/// on-disk URDF), the staging file is written in the same directory so mesh `filename="meshes/..."`
/// resolves like the neighbor file. Otherwise uses the system temp directory.
#[tauri::command]
async fn write_sim_staging_urdf(
    content: String,
    neighbor_urdf_path: Option<String>,
) -> Result<String, String> {
    let dest = if let Some(ref p) = neighbor_urdf_path {
        let trimmed = p.trim();
        if trimmed.is_empty() {
            return Err("neighbor_urdf_path is empty".to_string());
        }
        let path = Path::new(trimmed);
        let parent = path
            .parent()
            .ok_or_else(|| format!("Could not get parent directory of {}", trimmed))?;
        parent.join(".vector_sim_staging.urdf")
    } else {
        let nanos = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos();
        std::env::temp_dir().join(format!("vector_sim_{nanos}.urdf"))
    };

    fs::write(&dest, content.as_bytes())
        .map_err(|e| format!("Failed to write staging URDF: {}", e))?;

    dest.to_str()
        .ok_or_else(|| "Staging path is not valid UTF-8".to_string())
        .map(|s| s.to_string())
}

/// Best-effort cleanup of a staging URDF written by `write_sim_staging_urdf`.
#[tauri::command]
async fn remove_sim_staging_urdf(path: String) -> Result<(), String> {
    match fs::remove_file(&path) {
        Ok(()) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(e) => Err(format!("Failed to remove staging URDF: {}", e)),
    }
}

/// Save file to disk
#[tauri::command]
async fn save_file(path: String, content: String) -> Result<String, String> {
    fs::write(&path, &content).map_err(|e| format!("Failed to write file: {}", e))?;
    Ok(path)
}

/// Read file from disk
#[tauri::command]
async fn open_file(path: String) -> Result<String, String> {
    fs::read_to_string(&path).map_err(|e| format!("Failed to read file: {}", e))
}

/// Read a binary file and return its contents as a Vec<u8> (for mesh loading)
#[tauri::command]
async fn read_binary_file(path: String) -> Result<Vec<u8>, String> {
    fs::read(&path).map_err(|e| format!("Failed to read binary file: {}", e))
}

/// Create a new file at `path` with optional initial content. Errors if the
/// file already exists unless `overwrite` is true. Parent directory must exist.
#[tauri::command]
async fn create_file(path: String, content: Option<String>, overwrite: Option<bool>) -> Result<String, String> {
    let p = std::path::Path::new(&path);
    if p.exists() && !overwrite.unwrap_or(false) {
        return Err(format!("File already exists: {}", path));
    }
    fs::write(&path, content.unwrap_or_default().as_bytes())
        .map_err(|e| format!("Failed to create file: {}", e))?;
    Ok(path)
}

/// Create a directory (and parents). No-op if it already exists.
#[tauri::command]
async fn create_directory(path: String) -> Result<String, String> {
    fs::create_dir_all(&path).map_err(|e| format!("Failed to create directory: {}", e))?;
    Ok(path)
}

/// Delete a file or directory. Directories require `recursive=true`.
#[tauri::command]
async fn delete_path(path: String, recursive: Option<bool>) -> Result<(), String> {
    let p = std::path::Path::new(&path);
    if !p.exists() {
        return Ok(());
    }
    if p.is_dir() {
        if recursive.unwrap_or(false) {
            fs::remove_dir_all(&path).map_err(|e| format!("Failed to delete directory: {}", e))
        } else {
            fs::remove_dir(&path).map_err(|e| format!("Failed to delete directory (not empty?): {}", e))
        }
    } else {
        fs::remove_file(&path).map_err(|e| format!("Failed to delete file: {}", e))
    }
}

/// Rename / move a path. Errors if `new_path` already exists.
#[tauri::command]
async fn rename_path(old_path: String, new_path: String) -> Result<String, String> {
    let np = std::path::Path::new(&new_path);
    if np.exists() {
        return Err(format!("Destination already exists: {}", new_path));
    }
    fs::rename(&old_path, &new_path).map_err(|e| format!("Failed to rename: {}", e))?;
    Ok(new_path)
}

/// Check whether a path exists, and whether it's a directory.
#[tauri::command]
async fn path_exists(path: String) -> Result<serde_json::Value, String> {
    let p = std::path::Path::new(&path);
    let exists = p.exists();
    let is_dir = exists && p.is_dir();
    Ok(json!({ "exists": exists, "isDir": is_dir }))
}

/// Open folder dialog and return the selected directory path
#[tauri::command]
async fn open_folder_dialog(app: tauri::AppHandle) -> Result<Option<String>, String> {
    let path: Option<tauri_plugin_dialog::FilePath> = app.dialog().file().blocking_pick_folder();

    Ok(path.map(|p| p.to_string()))
}

/// List files in a directory. `max_depth` defaults to 0 (single level —
/// children are loaded lazily when folders are expanded).
#[tauri::command]
async fn list_directory(path: String, max_depth: Option<u32>) -> Result<Vec<serde_json::Value>, String> {
    let mut entries = Vec::new();
    let depth = max_depth.unwrap_or(0);
    list_dir_recursive(&std::path::Path::new(&path), &path, 0, depth, &mut entries)
        .map_err(|e| format!("Failed to list directory: {}", e))?;
    Ok(entries)
}

fn list_dir_recursive(
    dir: &std::path::Path,
    root: &str,
    depth: u32,
    max_depth: u32,
    entries: &mut Vec<serde_json::Value>,
) -> std::io::Result<()> {
    if depth > max_depth {
        return Ok(());
    }
    let mut items: Vec<_> = fs::read_dir(dir)?.collect::<Result<Vec<_>, _>>()?;
    items.sort_by_key(|e| e.file_name());

    for entry in items {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        let full_path = path.to_string_lossy().to_string();
        let is_dir = path.is_dir();

        // Skip hidden files and common non-relevant dirs
        if name.starts_with('.')
            || name == "node_modules"
            || name == "target"
            || name == "__pycache__"
        {
            continue;
        }

        entries.push(json!({
            "name": name,
            "path": full_path,
            "isDir": is_dir,
            "depth": depth,
        }));

        if is_dir {
            list_dir_recursive(&path, root, depth + 1, max_depth, entries)?;
        }
    }
    Ok(())
}

/// Open file dialog and return selected file path
#[tauri::command]
async fn open_file_dialog(app: tauri::AppHandle) -> Result<Option<String>, String> {
    let path: Option<tauri_plugin_dialog::FilePath> = app
        .dialog()
        .file()
        .add_filter(
            "Robot Files",
            &["urdf", "xacro", "mjcf", "sdf", "xml", "step", "stp", "iges"],
        )
        .add_filter("All Files", &["*"])
        .blocking_pick_file();

    Ok(path.map(|p| p.to_string()))
}

/// Save file dialog and return selected save path
#[tauri::command]
async fn save_file_dialog(
    app: tauri::AppHandle,
    default_name: Option<String>,
    filters: Option<Vec<(String, Vec<String>)>>,
) -> Result<Option<String>, String> {
    let mut dialog = app.dialog().file();

    // Callers can pass a custom filter list (e.g. STL export). When omitted,
    // fall back to the URDF-centric default that pre-existed this parameter.
    if let Some(custom) = filters {
        for (label, exts) in &custom {
            let ext_refs: Vec<&str> = exts.iter().map(|s| s.as_str()).collect();
            dialog = dialog.add_filter(label, &ext_refs);
        }
        dialog = dialog.add_filter("All Files", &["*"]);
    } else {
        dialog = dialog
            .add_filter("URDF Files", &["urdf"])
            .add_filter("MJCF Files", &["mjcf"])
            .add_filter("SDF Files", &["sdf"])
            .add_filter("XML Files", &["xml"])
            .add_filter("All Files", &["*"]);
    }

    if let Some(name) = default_name {
        dialog = dialog.set_file_name(&name);
    }

    let path: Option<tauri_plugin_dialog::FilePath> = dialog.blocking_save_file();

    Ok(path.map(|p| p.to_string()))
}

// ── Git Commands ─────────────────────────────────────────────────────────────

/// Get the git repo root directory
fn git_repo_root() -> std::path::PathBuf {
    // Try `git rev-parse --show-toplevel` first for accuracy
    if let Ok(output) = Command::new("git")
        .args(["rev-parse", "--show-toplevel"])
        .current_dir(&find_project_root())
        .output()
    {
        if output.status.success() {
            let root = String::from_utf8_lossy(&output.stdout).trim().to_string();
            if !root.is_empty() {
                return std::path::PathBuf::from(root);
            }
        }
    }
    find_project_root()
}

/// Get current git branch
#[tauri::command]
async fn git_branch() -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["branch", "--show-current"])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to execute git: {}", e))?;

    if !output.status.success() {
        return Err("Not a git repository".to_string());
    }

    let branch = String::from_utf8_lossy(&output.stdout).trim().to_string();
    Ok(if branch.is_empty() {
        "HEAD".to_string()
    } else {
        branch
    })
}

/// Get git status as structured data
#[tauri::command]
async fn git_status() -> Result<serde_json::Value, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["status", "--porcelain"])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to execute git: {}", e))?;

    if !output.status.success() {
        return Err("Not a git repository".to_string());
    }

    let status_text = String::from_utf8_lossy(&output.stdout);
    let mut staged = vec![];
    let mut unstaged = vec![];

    for line in status_text.lines() {
        if line.len() < 4 {
            continue;
        }

        let index_char = line.as_bytes()[0] as char;
        let work_char = line.as_bytes()[1] as char;
        let file_path = line[3..].to_string();

        // Staged: index char is not ' ' and not '?'
        if index_char != ' ' && index_char != '?' {
            staged.push(json!({
                "path": file_path.clone(),
                "status": match index_char {
                    'M' => "modified",
                    'A' => "added",
                    'D' => "deleted",
                    'R' => "renamed",
                    'U' => "unmerged",
                    _ => "modified"
                }
            }));
        }

        // Unstaged: work char is not ' ', or untracked (??)
        if work_char != ' ' || index_char == '?' {
            unstaged.push(json!({
                "path": file_path,
                "status": if index_char == '?' { "untracked" } else {
                    match work_char {
                        'M' => "modified",
                        'D' => "deleted",
                        _ => "modified"
                    }
                }
            }));
        }
    }

    Ok(json!({
        "staged": staged,
        "unstaged": unstaged
    }))
}

/// Stage a file
#[tauri::command]
async fn git_stage(file_path: String) -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["add", &file_path])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to stage: {}", e))?;

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to stage: {}", err));
    }

    Ok(format!("Staged: {}", file_path))
}

/// Unstage a file
#[tauri::command]
async fn git_unstage(file_path: String) -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["reset", "HEAD", &file_path])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to unstage: {}", e))?;

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to unstage: {}", err));
    }

    Ok(format!("Unstaged: {}", file_path))
}

/// Discard changes to a file
#[tauri::command]
async fn git_discard(file_path: String) -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["checkout", "--", &file_path])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to discard: {}", e))?;

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to discard: {}", err));
    }

    Ok(format!("Discarded: {}", file_path))
}

/// Commit with a message
#[tauri::command]
async fn git_commit(message: String) -> Result<String, String> {
    if message.is_empty() {
        return Err("Commit message cannot be empty".to_string());
    }

    let root = git_repo_root();
    let output = Command::new("git")
        .args(["commit", "-m", &message])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to commit: {}", e))?;

    if !output.status.success() {
        let err = String::from_utf8_lossy(&output.stderr);
        return Err(format!("Failed to commit: {}", err));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    Ok(stdout.to_string())
}

/// Push to remote
#[tauri::command]
async fn git_push() -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .arg("push")
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to push: {}", e))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        return Err(format!("{}{}", stderr, stdout));
    }

    Ok(format!("{}{}", stdout, stderr))
}

/// Pull from remote
#[tauri::command]
async fn git_pull() -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .arg("pull")
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to pull: {}", e))?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    if !output.status.success() {
        return Err(format!("{}{}", stderr, stdout));
    }

    Ok(format!("{}{}", stdout, stderr))
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(AppState::default())
        .setup(|app| {
            if cfg!(debug_assertions) {
                app.handle().plugin(
                    tauri_plugin_log::Builder::default()
                        .level(log::LevelFilter::Info)
                        .build(),
                )?;
            }
            app.handle().plugin(tauri_plugin_dialog::init())?;
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            start_core,
            stop_core,
            cancel_core_request,
            validate_urdf_content,
            design_compile,
            design_import,
            sim_load,
            sim_step,
            sim_reset,
            sim_get_state,
            sim_set_gravity,
            sim_set_script,
            ai_design,
            ai_gen_sim_script,
            sim_set_command,
            save_file,
            open_file,
            read_binary_file,
            write_sim_staging_urdf,
            remove_sim_staging_urdf,
            open_file_dialog,
            open_folder_dialog,
            list_directory,
            save_file_dialog,
            create_file,
            create_directory,
            delete_path,
            rename_path,
            path_exists,
            git_branch,
            git_status,
            git_stage,
            git_unstage,
            git_discard,
            git_commit,
            git_push,
            git_pull
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reader_routes_responses_by_id_and_skips_noise() {
        let pending: Mutex<HashMap<u64, mpsc::Sender<Reply>>> = Mutex::default();
        let alive = AtomicBool::new(true);
        let (tx1, rx1) = mpsc::channel();
        let (tx2, rx2) = mpsc::channel();
        let (tx3, rx3) = mpsc::channel();
        {
            let mut p = pending.lock().unwrap();
            p.insert(1, tx1);
            p.insert(2, tx2);
            p.insert(3, tx3);
        }
        // Out-of-order responses, a stray print, a notification, a response
        // for an unknown id, a parse-error response (id null), then EOF with
        // request 3 still unanswered.
        let stream = concat!(
            "hello from a stray print
",
            "{\"jsonrpc\":\"2.0\",\"method\":\"ai_progress\",\"params\":{\"stage\":\"x\"}}
",
            "{\"jsonrpc\":\"2.0\",\"result\":\"two\",\"id\":2}
",
            "{\"jsonrpc\":\"2.0\",\"result\":\"late\",\"id\":99}
",
            "{\"jsonrpc\":\"2.0\",\"error\":{\"code\":-32700},\"id\":null}
",
            "[1, 2]
",
            "{\"jsonrpc\":\"2.0\",\"error\":{\"message\":\"bad\"},\"id\":1}
",
        );
        let events = std::cell::RefCell::new(Vec::new());
        read_responses("test", stream.as_bytes(), &pending, &alive, |m, p| {
            events.borrow_mut().push((m.to_string(), p))
        });
        assert_eq!(rx2.recv().unwrap(), Ok(json!("two")));
        assert!(rx1.recv().unwrap().unwrap_err().contains("bad"));
        assert!(rx3.recv().is_err(), "unanswered request must fail at EOF");
        assert!(!alive.load(Ordering::SeqCst));
        assert!(pending.lock().unwrap().is_empty());
        let events = events.into_inner();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].0, "ai_progress");
        assert_eq!(events[0].1, json!({ "stage": "x" }));
    }

    /// End-to-end against the real Python core: concurrent callers each get
    /// their own response. Needs Python + the core's deps, so run explicitly:
    /// `cargo test --lib -- --ignored`.
    #[test]
    #[ignore]
    fn python_core_multiplexes_concurrent_requests() {
        let client = CoreClient::spawn("test", |_, _| {}).expect("spawn python core");
        client
            .request("ping", json!({}), Some(PING_TIMEOUT))
            .expect("ping");
        let handles: Vec<_> = (0..8)
            .map(|i| {
                let client = client.clone();
                std::thread::spawn(move || {
                    if i % 2 == 0 {
                        client.request("ping", json!({}), Some(PING_TIMEOUT))
                    } else {
                        client.request(&format!("no_such_method_{}", i), json!({}), Some(PING_TIMEOUT))
                    }
                })
            })
            .collect();
        for (i, h) in handles.into_iter().enumerate() {
            let reply = h.join().unwrap();
            if i % 2 == 0 {
                assert!(reply.is_ok(), "ping {} failed: {:?}", i, reply);
            } else {
                let err = reply.unwrap_err();
                assert!(err.contains(&format!("no_such_method_{}", i)), "{}", err);
            }
        }
        client.kill();
        let after = client.request("ping", json!({}), Some(Duration::from_secs(5)));
        assert!(after.is_err());
    }
}
