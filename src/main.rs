extern crate kdri;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::{
    Json, Router,
    extract::State,
    http::StatusCode,
    response::Html,
    routing::{get, post},
};
use serde::{Deserialize, Serialize};
use tower_http::cors::CorsLayer;

struct AppState {
    connection: Mutex<Option<kdri::KettlerConnection>>,
}

#[derive(Serialize)]
struct RpmResponse {
    rpm: Option<u16>,
}

#[derive(Serialize)]
struct BrakeResponse {
    // vracíme jako číslo (UI slider 0..=20)
    brake: Option<u16>,
}

#[derive(Deserialize)]
struct SetBrakeRequest {
    brake: u16,
}

#[derive(Serialize)]
struct StatusResponse {
    success: bool,
    message: String,
}

async fn index() -> Html<String> {
    let html = std::fs::read_to_string("templates/index.html")
        .unwrap_or_else(|_| "Šablona nenalezena".to_string());
    Html(html)
}

async fn get_rpm(State(state): State<Arc<AppState>>) -> Json<RpmResponse> {
    let mut guard = state.connection.lock().unwrap();

    let rpm = if let Some(conn) = guard.as_mut() {
        let rpm = conn.get_rpm();
        println!("[API] GET /api/rpm -> {:?}", rpm);
        rpm
    } else {
        println!("[API] GET /api/rpm -> None (nepřipojeno)");
        None
    };

    Json(RpmResponse { rpm })
}

async fn get_brake(State(state): State<Arc<AppState>>) -> Json<BrakeResponse> {
    let mut guard = state.connection.lock().unwrap();

    let brake = if let Some(conn) = guard.as_mut() {
        let brake = conn.get_brake_level().map(|v| v as u16);
        println!("[API] GET /api/brake -> {:?}", brake);
        brake
    } else {
        println!("[API] GET /api/brake -> None (nepřipojeno)");
        None
    };

    Json(BrakeResponse { brake })
}

async fn set_brake(
    State(state): State<Arc<AppState>>,
    Json(payload): Json<SetBrakeRequest>,
) -> (StatusCode, Json<StatusResponse>) {
    let mut guard = state.connection.lock().unwrap();

    if let Some(conn) = guard.as_mut() {
        // Rozsah 1..=20
        let v = payload.brake.clamp(1, 20) as u8;

        // Musíme nastavit online=true, jinak zařízení nepřijme nové hodnoty
        conn.set_online(true);
        // Nastavíme brake mode na ConstantBrake pro manuální ovládání odporu
        conn.set_brake_mode(kdri::KettlerBrakeMode::ConstantBrake);
        // Nastavíme samotný brake level
        conn.set_brake_level(v);
        println!("[API] POST /api/brake <- {}", v);

        (
            StatusCode::OK,
            Json(StatusResponse {
                success: true,
                message: format!("Brake level nastaven na {}", v),
            }),
        )
    } else {
        println!("[API] POST /api/brake <- {} (nepřipojeno)", payload.brake);
        (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(StatusResponse {
                success: false,
                message: "Kettler není připojen".to_string(),
            }),
        )
    }
}

#[tokio::main]
async fn main() {
    let state = Arc::new(AppState {
        connection: Mutex::new(None),
    });

    // Spustíme připojování k Kettleru v background tasku
    let state_clone = Arc::clone(&state);
    tokio::spawn(async move {
        let addr_str = "20:6D:43:25:80:00";
        let device_name = "RACER RS 436D20";

        loop {
            println!("Připojuji se k \"{}\" (\"{}\")...", device_name, addr_str);

            // Blokující connect spustíme v spawn_blocking aby neblokoval tokio runtime
            let result = tokio::task::spawn_blocking(move || {
                let addr = kdri::BtAddr::from_str(addr_str).expect("Neplatná Bluetooth adresa");
                let device = kdri::KettlerDevice::new(device_name.to_string(), addr);
                device.connect()
            })
            .await;

            match result {
                Ok(Ok(connection)) => {
                    println!("Připojeno k Kettleru!");
                    let mut guard = state_clone.connection.lock().unwrap();
                    *guard = Some(connection);
                    break;
                }
                Ok(Err(e)) => {
                    eprintln!("Připojení selhalo: {:?}, zkouším znovu za 5s...", e);
                }
                Err(e) => {
                    eprintln!("Spawn blocking selhal: {:?}, zkouším znovu za 5s...", e);
                }
            }

            tokio::time::sleep(Duration::from_secs(5)).await;
        }
    });

    let app = Router::new()
        .route("/", get(index))
        .route("/api/rpm", get(get_rpm))
        .route("/api/brake", get(get_brake))
        .route("/api/brake", post(set_brake))
        .layer(CorsLayer::permissive())
        .with_state(state);

    let bind_addr = "0.0.0.0:3000";
    println!("Web server běží na http://127.0.0.1:3000");

    let listener = tokio::net::TcpListener::bind(bind_addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}
