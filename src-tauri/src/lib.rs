use serde_json::json;
use std::io::{BufRead, BufReader, Write};
use std::path::Path;
use std::process::{Command, Child, Stdio};
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};
use tauri::{State, Emitter, AppHandle};
use std::fs;
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

/// Persistent Python core process state
struct CoreProcess {
    child: Child,
    stdin: std::io::BufWriter<std::process::ChildStdin>,
    stdout: BufReader<std::process::ChildStdout>,
}

/// Application state containing the persistent core process
pub struct AppState {
    core: Mutex<Option<CoreProcess>>,
}

impl CoreProcess {
    /// Spawn a new persistent Python core process
    fn spawn() -> Result<Self, String> {
        let project_root = find_project_root();

        // Try "python" first, then "python3" as fallback (macOS/Linux often only have python3)
        let mut child = Command::new("python")
            .arg("-m")
            .arg("core.server")
            .env("PYTHONUTF8", "1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .current_dir(&project_root)
            .spawn()
            .or_else(|_| {
                eprintln!("[Core] 'python' not found, trying 'python3'...");
                Command::new("python3")
                    .arg("-m")
                    .arg("core.server")
                    .env("PYTHONUTF8", "1")
                    .stdin(Stdio::piped())
                    .stdout(Stdio::piped())
                    .stderr(Stdio::inherit())
                    .current_dir(&project_root)
                    .spawn()
            })
            .map_err(|e| format!("Failed to spawn Python process (tried 'python' and 'python3'): {}", e))?;

        let stdin = std::io::BufWriter::new(
            child
                .stdin
                .take()
                .ok_or("Failed to open stdin for Python process")?,
        );

        let stdout = BufReader::new(
            child
                .stdout
                .take()
                .ok_or("Failed to open stdout from Python process")?,
        );

        Ok(CoreProcess {
            child,
            stdin,
            stdout,
        })
    }

    /// Check if the child process is still alive
    fn is_alive(&mut self) -> bool {
        match self.child.try_wait() {
            Ok(Some(_)) => false,  // Process has exited
            Ok(None) => true,      // Still running
            Err(_) => false,       // Error checking — assume dead
        }
    }

    /// Send a JSON-RPC request and read the response
    fn send_rpc(&mut self, method: &str, params: serde_json::Value, id: u32) -> Result<serde_json::Value, String> {
        // Check if the Python process is still alive
        if !self.is_alive() {
            return Err("Python core process has exited unexpectedly".to_string());
        }

        let request = json!({
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": id
        });

        // Write request
        self.stdin
            .write_all(request.to_string().as_bytes())
            .map_err(|e| format!("Failed to write to Python process: {}", e))?;
        self.stdin
            .write_all(b"\n")
            .map_err(|e| format!("Failed to write newline: {}", e))?;
        self.stdin
            .flush()
            .map_err(|e| format!("Failed to flush stdin: {}", e))?;

        // Read response
        let mut response_line = String::new();
        self.stdout
            .read_line(&mut response_line)
            .map_err(|e| format!("Failed to read response: {}", e))?;

        if response_line.is_empty() {
            return Err("No response from Python process (process may have crashed)".to_string());
        }

        // Skip any non-JSON lines (e.g., Python print() output that leaked to stdout)
        // Keep reading until we get a valid JSON-RPC response
        let mut attempts = 0;
        while attempts < 10 {
            let trimmed = response_line.trim();
            if trimmed.starts_with('{') {
                break;
            }
            // Not JSON — this is a stray print() from Python, skip it
            eprintln!("[send_rpc] Skipping non-JSON line from Python: {}", trimmed);
            response_line.clear();
            self.stdout
                .read_line(&mut response_line)
                .map_err(|e| format!("Failed to read response: {}", e))?;
            if response_line.is_empty() {
                return Err("No response from Python process after skipping non-JSON output".to_string());
            }
            attempts += 1;
        }

        // Parse response
        let response: serde_json::Value = serde_json::from_str(&response_line)
            .map_err(|e| format!("Failed to parse response JSON: {} — raw: {}", e, response_line.trim()))?;

        // Extract result or error
        if let Some(result) = response.get("result") {
            Ok(result.clone())
        } else if let Some(error) = response.get("error") {
            Err(format!("Python error: {}", error))
        } else {
            Err("Invalid response format".to_string())
        }
    }

    /// Send a JSON-RPC request, forwarding any notification lines as Tauri events.
    /// Notifications are JSON lines with "method" but no "id" field.
    fn send_rpc_streaming(&mut self, method: &str, params: serde_json::Value, id: u32, app: &AppHandle) -> Result<serde_json::Value, String> {
        if !self.is_alive() {
            return Err("Python core process has exited unexpectedly".to_string());
        }

        let request = json!({
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": id
        });

        self.stdin
            .write_all(request.to_string().as_bytes())
            .map_err(|e| format!("Failed to write to Python process: {}", e))?;
        self.stdin
            .write_all(b"\n")
            .map_err(|e| format!("Failed to write newline: {}", e))?;
        self.stdin
            .flush()
            .map_err(|e| format!("Failed to flush stdin: {}", e))?;

        // Read lines until we get the final response (has "id" or "result"/"error")
        let mut attempts = 0;
        loop {
            let mut line = String::new();
            self.stdout
                .read_line(&mut line)
                .map_err(|e| format!("Failed to read response: {}", e))?;

            if line.is_empty() {
                return Err("No response from Python process (process may have crashed)".to_string());
            }

            let trimmed = line.trim();
            if !trimmed.starts_with('{') {
                eprintln!("[send_rpc_streaming] Skipping non-JSON: {}", trimmed);
                attempts += 1;
                if attempts > 100 { return Err("Too many non-JSON lines".to_string()); }
                continue;
            }

            // Parse the JSON
            let parsed: serde_json::Value = match serde_json::from_str(trimmed) {
                Ok(v) => v,
                Err(e) => {
                    eprintln!("[send_rpc_streaming] Bad JSON: {} — {}", e, trimmed);
                    attempts += 1;
                    if attempts > 100 { return Err("Too many bad JSON lines".to_string()); }
                    continue;
                }
            };

            // Check if this is a notification (has "method" but no "id")
            if parsed.get("method").is_some() && parsed.get("id").is_none() {
                // Forward as Tauri event
                let method_name = parsed["method"].as_str().unwrap_or("unknown");
                let params = parsed.get("params").cloned().unwrap_or(json!({}));
                let _ = app.emit(method_name, params);
                continue;
            }

            // This is the final response
            if let Some(result) = parsed.get("result") {
                return Ok(result.clone());
            } else if let Some(error) = parsed.get("error") {
                return Err(format!("Python error: {}", error));
            } else {
                return Err("Invalid response format".to_string());
            }
        }
    }
}

/// Start the persistent Python core process
#[tauri::command]
async fn start_core(state: State<'_, AppState>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    if core.is_some() {
        return Err("Core process already running".to_string());
    }

    let mut process = CoreProcess::spawn()?;

    // Give Python a moment to initialize (import modules, load .env)
    std::thread::sleep(std::time::Duration::from_millis(500));

    // Verify the process is actually running with a ping
    match process.send_rpc("ping", json!({}), 0) {
        Ok(_) => {
            *core = Some(process);
            Ok("Core process started and verified".to_string())
        }
        Err(e) => {
            process.child.kill().ok();
            Err(format!("Core process started but failed health check: {}", e))
        }
    }
}

/// Stop the persistent Python core process
#[tauri::command]
async fn stop_core(state: State<'_, AppState>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    if let Some(mut process) = core.take() {
        process.child.kill().ok();
    }

    Ok("Core process stopped".to_string())
}

/// Ping the Python core process to verify it's working
#[tauri::command]
async fn ping_core(state: State<'_, AppState>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("ping", json!({}), 1)?;
    Ok(format!("Pong: {:?}", result))
}

/// Parse a URDF file using the Python core process
#[tauri::command]
async fn parse_urdf(state: State<'_, AppState>, path: String) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc("parse_urdf", json!({ "path": path }), 1)
}

/// Validate a URDF file (structural, physics, actuator, mesh checks)
#[tauri::command]
async fn validate_urdf(state: State<'_, AppState>, path: String) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc("validate_urdf", json!({ "path": path }), 1)
}

/// Validate URDF content from a string (for real-time editor validation)
#[tauri::command]
async fn validate_urdf_content(state: State<'_, AppState>, urdf_content: String) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc("validate_urdf_content", json!({ "urdf_content": urdf_content }), 1)
}

/// Load a robot model for simulation
#[tauri::command]
async fn sim_load(state: State<'_, AppState>, path: String, free_base: Option<bool>, seed: Option<u64>) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let mut params = json!({ "path": path, "free_base": free_base.unwrap_or(false) });
    if let Some(s) = seed {
        params["seed"] = json!(s);
    }
    process.send_rpc("sim_load", params, 1)
}

/// Step the simulation forward and return the resulting state
#[tauri::command]
async fn sim_step(state: State<'_, AppState>, n_steps: Option<u32>) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let params = json!({ "n_steps": n_steps.unwrap_or(1) });
    process.send_rpc("sim_step", params, 1)
}

/// Reset the simulation
#[tauri::command]
async fn sim_reset(state: State<'_, AppState>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("sim_reset", json!({}), 1)?;
    Ok(format!("Reset: {:?}", result))
}

/// Get the current simulation state (joint positions, velocities, etc.)
#[tauri::command]
async fn sim_get_state(state: State<'_, AppState>) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc("sim_get_state", json!({}), 1)
}

/// Set control inputs (joint targets, gripper, etc.)
#[tauri::command]
async fn sim_set_control(state: State<'_, AppState>, controls: serde_json::Value) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("sim_set_control", json!({ "controls": controls }), 1)?;
    Ok(format!("Controls set: {:?}", result))
}

/// Set gravity vector ([gx, gy, gz], URDF/MuJoCo Z-up, default [0,0,-9.81])
#[tauri::command]
async fn sim_set_gravity(state: State<'_, AppState>, gravity: Vec<f64>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("sim_set_gravity", json!({ "gravity": gravity }), 1)?;
    Ok(format!("Gravity set: {:?}", result))
}

/// Compile and install a Python step-callback script (Phase C script runner)
#[tauri::command]
async fn sim_set_script(state: State<'_, AppState>, code: String) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;
    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;
    process.send_rpc("sim_set_script", json!({ "code": code }), 1)
}

/// Render the simulation viewport to PNG and return base64
#[tauri::command]
async fn sim_render(state: State<'_, AppState>, width: Option<u32>, height: Option<u32>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let params = json!({
        "width": width.unwrap_or(640),
        "height": height.unwrap_or(480)
    });

    let result = process.send_rpc("sim_render", params, 1)?;

    // Result should contain base64 PNG data
    if let Some(base64) = result.as_str() {
        Ok(base64.to_string())
    } else {
        Ok(format!("{:?}", result))
    }
}

/// Use Claude AI to generate a robot model edit from natural language
#[tauri::command]
async fn ai_edit(app: AppHandle, state: State<'_, AppState>, prompt: String, urdf_content: String, kinematic_context: Option<String>, session_id: Option<String>) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc_streaming("ai_edit", json!({
        "prompt": prompt,
        "urdf_content": urdf_content,
        "kinematic_context": kinematic_context,
        "session_id": session_id.unwrap_or_else(|| "default".to_string())
    }), 1, &app)
}

/// Restore conversation history for an AI session from frontend localStorage
#[tauri::command]
async fn ai_set_history(state: State<'_, AppState>, session_id: String, history: serde_json::Value) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc("ai_set_history", json!({
        "session_id": session_id,
        "history": history
    }), 1)
}

/// Second-pass AI validation of assembled URDF — checks spatial correctness
#[tauri::command]
async fn ai_validate_assembly(app: AppHandle, state: State<'_, AppState>, urdf_content: String, original_prompt: String, session_id: Option<String>, screenshot_base64: Option<String>, screenshots: Option<Vec<String>>) -> Result<serde_json::Value, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    process.send_rpc_streaming("ai_validate_assembly", json!({
        "urdf_content": urdf_content,
        "original_prompt": original_prompt,
        "session_id": session_id.unwrap_or_else(|| "default".to_string()),
        "screenshot_base64": screenshot_base64,
        "screenshots": screenshots
    }), 1, &app)
}

/// Use Claude AI to generate inline completions (ghost text) for URDF/XML editing
#[tauri::command]
async fn ai_complete(state: State<'_, AppState>, urdf_content: String, cursor_line: u32, cursor_column: u32, prefix: String, kinematic_context: Option<String>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("ai_complete", json!({
        "urdf_content": urdf_content,
        "cursor_line": cursor_line,
        "cursor_column": cursor_column,
        "prefix": prefix,
        "kinematic_context": kinematic_context.unwrap_or_default()
    }), 1)?;

    // Extract the completion text from the result
    if let Some(completion) = result.as_str() {
        Ok(completion.to_string())
    } else {
        Ok(format!("{:?}", result))
    }
}

/// Write editor URDF to a staging file for `sim_load`. If `neighbor_urdf_path` is set (path to an
/// on-disk URDF), the staging file is written in the same directory so mesh `filename="meshes/..."`
/// resolves like the neighbor file. Otherwise uses the system temp directory.
#[tauri::command]
async fn write_sim_staging_urdf(content: String, neighbor_urdf_path: Option<String>) -> Result<String, String> {
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
    fs::write(&path, &content)
        .map_err(|e| format!("Failed to write file: {}", e))?;
    Ok(path)
}

/// Read file from disk
#[tauri::command]
async fn open_file(path: String) -> Result<String, String> {
    fs::read_to_string(&path)
        .map_err(|e| format!("Failed to read file: {}", e))
}

/// Read a binary file and return its contents as a Vec<u8> (for mesh loading)
#[tauri::command]
async fn read_binary_file(path: String) -> Result<Vec<u8>, String> {
    fs::read(&path)
        .map_err(|e| format!("Failed to read binary file: {}", e))
}

/// Open folder dialog and return the selected directory path
#[tauri::command]
async fn open_folder_dialog(app: tauri::AppHandle) -> Result<Option<String>, String> {
    let path: Option<tauri_plugin_dialog::FilePath> = app
        .dialog()
        .file()
        .blocking_pick_folder();

    Ok(path.map(|p| p.to_string()))
}

/// List files in a directory (recursive, max 2 levels deep)
#[tauri::command]
async fn list_directory(path: String) -> Result<Vec<serde_json::Value>, String> {
    let mut entries = Vec::new();
    list_dir_recursive(&std::path::Path::new(&path), &path, 0, 2, &mut entries)
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
    if depth > max_depth { return Ok(()); }
    let mut items: Vec<_> = fs::read_dir(dir)?.collect::<Result<Vec<_>, _>>()?;
    items.sort_by_key(|e| e.file_name());

    for entry in items {
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        let full_path = path.to_string_lossy().to_string();
        let is_dir = path.is_dir();

        // Skip hidden files and common non-relevant dirs
        if name.starts_with('.') || name == "node_modules" || name == "target" || name == "__pycache__" {
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
        .add_filter("Robot Files", &["urdf", "xacro", "mjcf", "sdf", "xml", "step", "stp", "iges"])
        .add_filter("All Files", &["*"])
        .blocking_pick_file();

    Ok(path.map(|p| p.to_string()))
}

/// Save file dialog and return selected save path
#[tauri::command]
async fn save_file_dialog(app: tauri::AppHandle, default_name: Option<String>) -> Result<Option<String>, String> {
    let mut dialog = app
        .dialog()
        .file()
        .add_filter("URDF Files", &["urdf"])
        .add_filter("MJCF Files", &["mjcf"])
        .add_filter("SDF Files", &["sdf"])
        .add_filter("XML Files", &["xml"])
        .add_filter("All Files", &["*"]);

    if let Some(name) = default_name {
        dialog = dialog.set_file_name(&name);
    }

    let path: Option<tauri_plugin_dialog::FilePath> = dialog.blocking_save_file();

    Ok(path.map(|p| p.to_string()))
}

/// Get recent files from app storage
#[tauri::command]
async fn get_recent_files() -> Result<Vec<String>, String> {
    // For now, return empty list. In a full implementation, this would read from
    // a JSON file in app data directory (app.path().app_data_dir())
    Ok(vec![])
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
    Ok(if branch.is_empty() { "HEAD".to_string() } else { branch })
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

/// Get git diff for a specific file
#[tauri::command]
async fn git_diff(file_path: String) -> Result<String, String> {
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["diff", &file_path])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to execute git diff: {}", e))?;

    Ok(String::from_utf8_lossy(&output.stdout).to_string())
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

/// Get git log
#[tauri::command]
async fn git_log(count: Option<u32>) -> Result<Vec<serde_json::Value>, String> {
    let count = count.unwrap_or(10);
    let root = git_repo_root();
    let output = Command::new("git")
        .args(["log", "--oneline", "-n", &count.to_string()])
        .current_dir(&root)
        .output()
        .map_err(|e| format!("Failed to execute git log: {}", e))?;

    if !output.status.success() {
        return Err("Failed to get git log".to_string());
    }

    let log_text = String::from_utf8_lossy(&output.stdout);
    let entries: Vec<serde_json::Value> = log_text
        .lines()
        .map(|line| {
            let parts: Vec<&str> = line.splitn(2, ' ').collect();
            if parts.len() == 2 {
                json!({ "hash": parts[0], "message": parts[1] })
            } else {
                json!({ "hash": line, "message": "" })
            }
        })
        .collect();

    Ok(entries)
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
        .manage(AppState {
            core: Mutex::new(None),
        })
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
            ping_core,
            parse_urdf,
            validate_urdf,
            validate_urdf_content,
            sim_load,
            sim_step,
            sim_reset,
            sim_get_state,
            sim_set_control,
            sim_set_gravity,
            sim_set_script,
            sim_render,
            ai_edit,
            ai_set_history,
            ai_validate_assembly,
            ai_complete,
            save_file,
            open_file,
            read_binary_file,
            write_sim_staging_urdf,
            remove_sim_staging_urdf,
            open_file_dialog,
            open_folder_dialog,
            list_directory,
            save_file_dialog,
            get_recent_files,
            git_branch,
            git_status,
            git_diff,
            git_stage,
            git_unstage,
            git_discard,
            git_commit,
            git_log,
            git_push,
            git_pull
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
