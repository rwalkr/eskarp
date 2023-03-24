// #![no_std]

use defmt::*;
// use defmt_rtt as _;
use micromath::F32Ext;

use embedded_hal::blocking::delay::DelayMs;
use usbd_human_interface_device::device::mouse::WheelMouseReport;

pub type Result<T> = core::result::Result<T, iqs5xx::Error>;

#[derive(Format)]
pub enum MovementState {
    Idle,
    Contact {
        dx: i16,
        dy: i16,
        buttons: u8,
    },
    Coasting {
        v0: f32,
        px: f32,
        py: f32,
        v: f32,
        dx: f32,
        dy: f32,
        buttons: u8,
    },
}

pub trait Timer {
    fn start(&mut self, delay: u64);
    fn cancel(&mut self);
    fn reschedule(&mut self, interval: u64);
}

pub struct Movement<T: Timer> {
    move_state: MovementState,
    timer: T,
}

impl<T: Timer> Movement<T> {
    const COAST_TICK_INITIAL_DELAY: u64 = 15; // slightly longer than active report interval
    const COAST_TICK_INTERVAL: u64 = 10;
    const FRICTION: f32 = 0.90;
    const COAST_START_THRESHOLD: f32 = 16.;
    const COAST_END_THRESHOLD: f32 = 1.;

    pub fn new(timer: T) -> Self {
        Movement {
            move_state: MovementState::Idle,
            timer: timer,
        }
    }

    fn contact(&mut self, dx: i16, dy: i16, buttons: u8) -> Option<MouseReport> {
        let mx = map_v(dx);
        let my = map_v(dy);
        let report = MouseReport::default()
            .pos(mx as i8, my as i8)
            .buttons(buttons);
        self.move_state = MovementState::Contact { dx, dy, buttons };
        self.timer.start(Self::COAST_TICK_INITIAL_DELAY);
        Some(report)
    }

    fn stop(&mut self) {
        self.move_state = MovementState::Idle;
        self.timer.cancel();
    }

    fn tick(&mut self) -> Option<MouseReport> {
        info!("tick: {}", self.move_state);
        let (report, new_state) = match self.move_state {
            MovementState::Idle => (None, MovementState::Idle),
            MovementState::Contact { dx, dy, buttons } => {
                let vx = map_v(dx);
                let vy = map_v(dy);
                let v0 = (vx * vx + vy * vy).sqrt();
                // info!("  vx: {}, vy: {}, v0: {}", vx, vy, v0);
                if v0 >= Self::COAST_START_THRESHOLD {
                    let px = vx as f32 / v0;
                    let py = vy as f32 / v0;
                    let v = v0 * Self::FRICTION;
                    let (dx, rx) = update_v(v, px, 0.);
                    let (dy, ry) = update_v(v, py, 0.);
                    let report = MouseReport::default().pos(rx, ry).buttons(buttons);
                    (
                        Some(report),
                        MovementState::Coasting {
                            v0,
                            px,
                            py,
                            v,
                            dx,
                            dy,
                            buttons,
                        },
                    )
                } else {
                    (None, MovementState::Idle)
                }
            }
            MovementState::Coasting {
                v0,
                px,
                py,
                v,
                dx,
                dy,
                buttons,
            } => {
                if v >= Self::COAST_END_THRESHOLD {
                    let v = v * Self::FRICTION;
                    let (dx, rx) = update_v(v, px, dx);
                    let (dy, ry) = update_v(v, py, dy);
                    let report = if rx != 0 || ry != 0 {
                        Some(MouseReport::default().pos(rx, ry).buttons(buttons))
                    } else {
                        None
                    };
                    (
                        report,
                        MovementState::Coasting {
                            v0,
                            px,
                            py,
                            v,
                            dx,
                            dy,
                            buttons,
                        },
                    )
                } else {
                    (None, MovementState::Idle)
                }
            }
        };
        if let MovementState::Coasting { .. } = new_state {
            self.timer.reschedule(Self::COAST_TICK_INTERVAL);
        }
        self.move_state = new_state;
        report
    }
}

fn update_v(vel: f32, prop: f32, residual: f32) -> (f32, i8) {
    let new = residual + vel * prop;
    let rep = new as i8;
    let residual = new - rep as f32;
    (residual, rep)
}

fn map_v(d: i16) -> f32 {
    d.signum() as f32 * (0.7 * (d.abs() as f32)).powf(1.6)
}

pub struct Touchpad<I2C, RDY, RST, T: Timer> {
    iqs: iqs5xx::IQS5xx<I2C, RDY, RST>,
    resolution: (u16, u16),
    movement: Movement<T>,
    scroll_credit: i16,
}

#[derive(Clone, Debug, Default)]
struct MouseReport(WheelMouseReport);

impl MouseReport {
    pub fn buttons(mut self, buttons: u8) -> Self {
        self.0.buttons = buttons;
        self
    }

    pub fn pos(mut self, x: i8, y: i8) -> Self {
        self.0.x = x;
        self.0.y = y;
        self
    }

    pub fn wheel(mut self, v: i8) -> Self {
        self.0.vertical_wheel = v;
        self
    }

    pub fn pan(mut self, v: i8) -> Self {
        self.0.horizontal_wheel = v;
        self
    }
}

impl From<MouseReport> for WheelMouseReport {
    fn from(r: MouseReport) -> Self {
        r.0
    }
}

impl<I2C, RDY, RST, T> Touchpad<I2C, RDY, RST, T>
where
    I2C: embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    RDY: embedded_hal::digital::v2::InputPin,
    RST: embedded_hal::digital::v2::OutputPin,
    T: Timer,
{
    const CLICK_TIME_MS: u32 = 10;

    pub fn new(
        mut iqs: iqs5xx::IQS5xx<I2C, RDY, RST>,
        timer: T,
        delay: &mut dyn DelayMs<u32>,
    ) -> Result<Self> {
        info!("IQS reset!");
        iqs.reset(delay)?;
        info!("IQS reset done!");

        info!("IQS init!");
        // TODO: add timeout if RDY isn't connected
        iqs.poll_ready(delay)?;
        info!("IQS ready");
        iqs.init()?;
        info!("IQS OK");

        let res_x = iqs.read_resolution_x()?;
        let res_y = iqs.read_resolution_y()?;

        let tp = Touchpad {
            iqs,
            resolution: (res_x, res_y),
            movement: Movement::new(timer),
            scroll_credit: 0i16,
        };

        Ok(tp)
    }

    pub fn clear_irq<F: FnMut(&mut RDY) -> ()>(&mut self, clear_irq: F) {
        self.iqs.clear_irq(clear_irq);
    }

    pub fn process<F: FnMut(WheelMouseReport, u32) -> ()>(&mut self, mut send_report: F) {
        // read the report if available
        let res = self.iqs.try_transact(|iqs| iqs.get_report());
        match res {
            Ok(Some(tp_report)) => {
                let event = iqs5xx::Event::from(&tp_report);
                info!("Event: {}", event);
                let rhs_edge = self.resolution.0 - 100;
                let is_rhs = tp_report.num_fingers == 1 && tp_report.touches[0].abs_x > rhs_edge;
                match event {
                    iqs5xx::Event::Move { x: _, y } | iqs5xx::Event::PressHold { x: _, y }
                        if is_rhs =>
                    {
                        self.movement.stop();
                        // TODO: reset scroll_credit on other move events
                        self.scroll_credit += -y * 10;
                        const DIVISOR: i16 = 35;
                        let scroll = self.scroll_credit / DIVISOR;
                        self.scroll_credit -= scroll * DIVISOR;
                        send_report(MouseReport::default().wheel(scroll as i8).into(), 0);
                    }
                    iqs5xx::Event::Move { x, y } => {
                        let report = self.movement.contact(x, y, 0);
                        if let Some(r) = report {
                            send_report(r.into(), 0);
                        }
                    }
                    iqs5xx::Event::SingleTap { x, y } => {
                        self.movement.stop();
                        let b = if x > rhs_edge {
                            if y >= self.resolution.1 / 2 {
                                2
                            } else {
                                1
                            }
                        } else {
                            0
                        };
                        send_report(MouseReport::default().buttons(1 << b).into(), 0);
                        send_report(
                            MouseReport::default().buttons(0).into(),
                            Self::CLICK_TIME_MS,
                        );
                    }
                    iqs5xx::Event::PressHold { x, y } => {
                        let report = self.movement.contact(x, y, 1);
                        if let Some(r) = report {
                            send_report(r.into(), 0);
                        }
                    }
                    iqs5xx::Event::TwoFingerTap => {
                        self.movement.stop();
                        send_report(MouseReport::default().buttons(2).into(), 0);
                        send_report(
                            MouseReport::default().buttons(0).into(),
                            Self::CLICK_TIME_MS,
                        );
                    }
                    iqs5xx::Event::Scroll { x, y: _ } if x != 0 => {
                        self.movement.stop();
                        send_report(MouseReport::default().pan(x as i8).into(), 0);
                    }
                    iqs5xx::Event::Scroll { x: _, y } if y != 0 => {
                        self.movement.stop();
                        // TODO: reset scroll_credit on other move events
                        self.scroll_credit += -y * 10;
                        const DIVISOR: i16 = 35;
                        let scroll = self.scroll_credit / DIVISOR;
                        self.scroll_credit -= scroll * DIVISOR;
                        send_report(MouseReport::default().wheel(scroll as i8).into(), 0);
                    }
                    _ => {}
                };
            }
            Ok(None) => {}
            Err(_) => {
                info!("retry (R)");
            }
        }
    }

    pub fn tick<F: FnMut(WheelMouseReport) -> ()>(&mut self, mut send_report: F) {
        let report = self.movement.tick();
        if let Some(r) = report {
            send_report(r.into());
        }
    }
}
