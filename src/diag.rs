//! Optional diagnostics on stderr.
//!
//! Silent by default. Set `ARCADIAOS_LOG=info` for plan summaries, charging
//! and fault handling, or `ARCADIAOS_LOG=debug` for every command and event.
//! Logging never influences a decision.

use std::fmt;

/// How much ArcadiaOS writes to stderr.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Nothing (the default).
    #[default]
    Off,
    /// Plan summaries, charging, rejections and failures.
    Info,
    /// Every command, completion and replan.
    Debug,
}

impl Level {
    /// Parse a level name; unknown or empty names mean [`Level::Off`].
    pub fn parse(s: &str) -> Self {
        match s.trim().to_ascii_lowercase().as_str() {
            "info" => Level::Info,
            "debug" | "trace" => Level::Debug,
            _ => Level::Off,
        }
    }

    /// The level requested through `ARCADIAOS_LOG`.
    pub fn from_env() -> Self {
        std::env::var("ARCADIAOS_LOG")
            .map(|v| Self::parse(&v))
            .unwrap_or_default()
    }
}

/// Writes diagnostics at or below its level, stamped with sim time.
#[derive(Debug, Clone, Copy, Default)]
pub struct Diag {
    level: Level,
}

impl Diag {
    /// A logger at `level`.
    pub fn new(level: Level) -> Self {
        Self { level }
    }

    /// The configured level.
    pub fn level(&self) -> Level {
        self.level
    }

    /// Whether messages at `level` are written.
    pub fn enabled(&self, level: Level) -> bool {
        level != Level::Off && level <= self.level
    }

    /// Write an info message; `msg` is only evaluated when enabled.
    pub fn info(&self, t: f64, msg: impl FnOnce() -> String) {
        self.write(Level::Info, t, msg);
    }

    /// Write a debug message; `msg` is only evaluated when enabled.
    pub fn debug(&self, t: f64, msg: impl FnOnce() -> String) {
        self.write(Level::Debug, t, msg);
    }

    fn write(&self, level: Level, t: f64, msg: impl FnOnce() -> String) {
        if self.enabled(level) {
            eprintln!("arcadiaos [{}] {}: {}", Stamp(t), tag(level), msg());
        }
    }
}

fn tag(level: Level) -> &'static str {
    match level {
        Level::Off => "off",
        Level::Info => "info",
        Level::Debug => "debug",
    }
}

/// Sim time as `sol.fraction` plus seconds, for readable logs.
struct Stamp(f64);

impl fmt::Display for Stamp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "t={:.0}s sol {:.3}",
            self.0,
            self.0 / arcadia_protocol::SOL_S
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_levels_leniently() {
        assert_eq!(Level::parse("info"), Level::Info);
        assert_eq!(Level::parse(" DEBUG "), Level::Debug);
        assert_eq!(Level::parse(""), Level::Off);
        assert_eq!(Level::parse("loud"), Level::Off);
    }

    #[test]
    fn enabled_respects_level() {
        let d = Diag::new(Level::Info);
        assert!(d.enabled(Level::Info));
        assert!(!d.enabled(Level::Debug));
        assert!(!Diag::default().enabled(Level::Info));
        assert!(!d.enabled(Level::Off));
    }
}
