//! Minimal HTTP/1.1 POST helper using embassy‑net TCP.
use core::fmt::Write;
use embassy_net::{tcp::TcpSocket, IpEndpoint};
use embassy_time::{Duration, with_timeout};

/// Perform an HTTP/1.1 POST with a pre‑built JSON `body`.
///
/// * Does not resolve DNS — pass a concrete `remote` endpoint.
/// * Writes a `Host` header with `host` and a `Content-Length`.
/// * Returns `Ok(())` when bytes are written; does not read the response body.
pub async fn post_json<'a, const N: usize>(
    sock: &mut TcpSocket<'a>,
    remote: IpEndpoint,
    host: &str,
    path: &str,
    body: &str,
) -> Result<(), &'static str> {
    let mut req: heapless::String<N> = heapless::String::new();
    let _ = write!(
        req,
        "POST {} HTTP/1.1\r\nHost: {}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
        path, host, body.len(), body
    );

    with_timeout(Duration::from_secs(5), sock.connect(remote))
        .await
        .map_err(|_| "connect timeout")?
        .map_err(|_| "connect failed")?;

    with_timeout(Duration::from_secs(5), sock.write(req.as_bytes()))
        .await
        .map_err(|_| "write timeout")?
        .map_err(|_| "write failed")?;

    Ok(())
}
