//! HTTP backend for the FlightPlan editor.
//!
//! A thin axum + tokio wrapper around the local `flight_plan` Rust
//! crate (the kinematic core lives in `../rust`). The server exposes a
//! JSON REST API under `/api/...` and serves the Angular build
//! (`../frontend/dist/browser`) at the URL root so a single
//! `cargo run` is enough to develop locally.

mod handlers;
mod state;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result};
use axum::{
    routing::{get, patch, post},
    Router,
};
use clap::Parser;
use tower_http::cors::{Any, CorsLayer};
use tower_http::services::{ServeDir, ServeFile};
use tower_http::trace::TraceLayer;

use crate::state::AppState;

#[derive(Parser, Debug)]
#[command(
    name = "flight-plan-editor",
    about = "HTTP backend for the FlightPlan editor",
    version
)]
struct Args {
    /// TCP port to listen on.
    #[arg(long, default_value_t = 8000)]
    port: u16,

    /// Bind address.
    #[arg(long, default_value = "127.0.0.1")]
    host: String,

    /// Path to the Angular build (`dist/browser` directory or any
    /// folder containing `index.html` + JS chunks).
    #[arg(long)]
    dist: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,tower_http=info".into()),
        )
        .with_target(false)
        .init();

    let args = Args::parse();
    let dist_dir = resolve_dist_dir(args.dist.clone());
    tracing::info!("serving static files from {}", dist_dir.display());

    let state = AppState::new(dist_dir.clone());
    let app = router(state.clone(), dist_dir);

    let addr: SocketAddr = format!("{}:{}", args.host, args.port)
        .parse()
        .with_context(|| format!("invalid host:port '{}:{}'", args.host, args.port))?;
    tracing::info!("listening on http://{addr}");
    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .with_context(|| format!("bind {addr}"))?;
    axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await?;
    Ok(())
}

/// Build the axum `Router`. Kept separate so tests can mount it
/// without spawning a server.
fn router(state: AppState, dist_dir: PathBuf) -> Router {
    let api = Router::new()
        .route("/health", get(handlers::health))
        .route("/plans", get(handlers::list_plans).post(handlers::create_plan))
        .route(
            "/plans/:id",
            get(handlers::get_plan)
                .patch(handlers::update_plan)
                .delete(handlers::delete_plan),
        )
        .route("/plans/:id/visibility", post(handlers::set_visibility))
        .route("/plans/:id/waypoints", post(handlers::add_waypoint))
        .route("/plans/:id/connect", post(handlers::connect_waypoints))
        .route(
            "/plans/:id/waypoints/patch",
            patch(handlers::patch_waypoint).delete(handlers::delete_waypoint),
        )
        .route("/plans/:id/trace", get(handlers::get_trace))
        .route("/sim", get(handlers::get_sim).post(handlers::set_sim))
        .with_state(Arc::new(state));

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    // Static file handler for the Angular bundle.
    let index_file = dist_dir.join("index.html");
    let serve_dir = ServeDir::new(&dist_dir).fallback(ServeFile::new(&index_file));

    Router::new()
        .nest("/api", api)
        .fallback_service(serve_dir)
        .layer(TraceLayer::new_for_http())
        .layer(cors)
}

/// Resolve the directory that contains `index.html` (the Angular
/// build). Looks first at `--dist`, then at common defaults so the
/// server runs out of the box in standard layouts. The Angular
/// `:application` builder nests another `browser/` subdir when the
/// `outputPath` itself ends in `browser`, so we try both layouts.
fn resolve_dist_dir(dist: Option<PathBuf>) -> PathBuf {
    let here = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(d) = dist {
        candidates.push(d.clone());
        candidates.push(d.join("browser"));
    }
    candidates.extend([
        here.join("../frontend/dist/browser"),         // outputPath="dist/browser" → dist/browser/browser
        here.join("../frontend/dist/browser/browser"), // … (the actual layout with Angular 17 :application builder)
        here.join("../frontend/dist"),                 // legacy / dev-server
        here.join("dist/browser"),
        here.join("dist"),
    ]);
    for c in candidates {
        if c.join("index.html").is_file() {
            return c;
        }
    }
    // Fallback: best guess even if the file is missing, so logs are
    // still intelligible rather than a relative ".".
    here.join("../frontend/dist/browser/browser")
}

/// Wait for Ctrl-C (or SIGTERM on Unix) so the server can shut down
/// cleanly.
async fn shutdown_signal() {
    let ctrl_c = async {
        let _ = tokio::signal::ctrl_c().await;
    };
    #[cfg(unix)]
    let terminate = async {
        if let Ok(mut sig) =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        {
            sig.recv().await;
        }
    };
    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        _ = ctrl_c   => {},
        _ = terminate => {},
    }
    tracing::info!("shutdown signal received");
}
