use serde_json::json;
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Child, Stdio};
use std::sync::Mutex;
use tauri::State;

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

        let mut child = Command::new("python")
            .arg("-m")
            .arg("core.server")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .current_dir(&project_root)
            .spawn()
            .map_err(|e| format!("Failed to spawn Python process: {}", e))?;

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

    /// Send a JSON-RPC request and read the response
    fn send_rpc(&mut self, method: &str, params: serde_json::Value, id: u32) -> Result<serde_json::Value, String> {
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
            return Err("No response from Python process".to_string());
        }

        // Parse response
        let response: serde_json::Value = serde_json::from_str(&response_line)
            .map_err(|e| format!("Failed to parse response JSON: {}", e))?;

        // Extract result or error
        if let Some(result) = response.get("result") {
            Ok(result.clone())
        } else if let Some(error) = response.get("error") {
            Err(format!("Python error: {}", error))
        } else {
            Err("Invalid response format".to_string())
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

/// Load a robot model for simulation
#[tauri::command]
async fn sim_load(state: State<'_, AppState>, path: String) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let result = process.send_rpc("sim_load", json!({ "path": path }), 1)?;
    Ok(format!("Model loaded: {:?}", result))
}

/// Step the simulation forward
#[tauri::command]
async fn sim_step(state: State<'_, AppState>, n_steps: Option<u32>) -> Result<String, String> {
    let mut core = state.core.lock().map_err(|e| format!("Failed to lock state: {}", e))?;

    let process = core.as_mut().ok_or("Core process not running. Call start_core first.")?;

    let params = json!({ "n_steps": n_steps.unwrap_or(1) });
    let result = process.send_rpc("sim_step", params, 1)?;
    Ok(format!("Stepped: {:?}", result))
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

    let result = process.send_rpc("sim_set_control", controls, 1)?;
    Ok(format!("Controls set: {:?}", result))
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
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            start_core,
            stop_core,
            ping_core,
            parse_urdf,
            sim_load,
            sim_step,
            sim_reset,
            sim_get_state,
            sim_set_control,
            sim_render
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
