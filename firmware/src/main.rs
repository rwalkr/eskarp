#![no_main]
#![no_std]

use panic_halt as _;
use rtic::app;

pub mod keyboard;

#[app(device = rp_pico::hal::pac,
      peripherals = true,
      dispatchers = [DMA_IRQ_0])]
mod app {
    use crate::keyboard::*;
    use embedded_hal::{
        digital::v2::{InputPin, OutputPin},
        serial::{Read, Write},
    };
    use embedded_time::duration::units::Extensions;
    use heapless::spsc::Queue;
    use keyberon::action::Action::*;
    use keyberon::action::{HoldTapConfig, d, k, l, m};
    use keyberon::debounce::Debouncer;
    use keyberon::hid;
    use keyberon::key_code::KeyCode::*;
    use keyberon::layout::{CustomEvent, Event};
    use keyberon::matrix::{Matrix, PressedKeys};
    use nb::block;
    use rp_pico::hal;
    use rp_pico::hal::gpio::dynpin::DynPin;
    use usb_device::bus::UsbBusAllocator;

    type UsbClass = hid::HidClass<'static, hal::usb::UsbBus, KbState>;
    type UsbDevice = usb_device::device::UsbDevice<'static, hal::usb::UsbBus>;
    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    type UartDevice = hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART0>;

    pub enum CustomKey {
        Media(MediaKey),
        Reset(either::Either<(), ()>),
    }
    type Action = keyberon::action::Action<CustomKey>;
    type Layout = keyberon::layout::Layout<CustomKey>;

    const fn make_keymap() -> keyberon::layout::Layers<CustomKey> {
        // aliases to keep keymap readable
        const K_NUBS: Action = k(NonUsBslash);
        const K_ENT: Action = k(Enter);
        const K_SPC: Action = k(Space);
        const K_BSP: Action = k(BSpace);
        const K_APP: Action = k(Application);
        const K_LBRK: Action = k(LBracket);
        const K_RBRK: Action = k(RBracket);
        const K_LBRA: Action = m(&[LShift, LBracket]);
        const K_RBRA: Action = m(&[LShift, RBracket]);
        const K_LPAR: Action = m(&[LShift, Kb9]);
        const K_RPAR: Action = m(&[LShift, Kb0]);
        const K_LT: Action = m(&[LShift, Comma]);
        const K_GT: Action = m(&[LShift, Dot]);
        const K_HASH: Action = k(NonUsHash);
        const K_PLUS: Action = k(KpPlus);
        const K_MINUS: Action = k(KpMinus);
        const K_MUL: Action = k(KpAsterisk);
        const K_DIV: Action = k(KpSlash);
        const K_PGUP: Action = k(PgUp);
        const K_PGDN: Action = k(PgDown);
        const K_INS: Action = k(Insert);
        const K_DEL: Action = k(Delete);
        const K_ESC: Action = k(Escape);
        const K_MUTE: Action = Action::Custom(CustomKey::Media(MediaKey::AudioMute));
        const K_VUP: Action = Action::Custom(CustomKey::Media(MediaKey::AudioVolUp));
        const K_VDN: Action = Action::Custom(CustomKey::Media(MediaKey::AudioVolDown));
        const K_PSCR: Action = k(PScreen);
        const K_SLCK: Action = k(ScrollLock);
        const K_PAUS: Action = k(Pause);
        const K_RSTL: Action = Action::Custom(CustomKey::Reset(either::Left(())));
        const K_RSTR: Action = Action::Custom(CustomKey::Reset(either::Right(())));
        const L_1: Action = HoldTap {
            timeout: 200,
            tap_hold_interval: 0,
            config: HoldTapConfig::Default,
            hold: &l(1),
            tap: &d(1),
        };
        const L_2: Action = HoldTap {
            timeout: 200,
            tap_hold_interval: 0,
            config: HoldTapConfig::Default,
            hold: &l(2),
            tap: &d(2),
        };
        const CUT: Action = m(&[LCtrl, X]);
        const COPY: Action = m(&[LCtrl, C]);
        const PASTE: Action = m(&[LCtrl, V]);

        #[rustfmt::skip]
        const KEYMAP: keyberon::layout::Layers<CustomKey> = &[
        &[
            &[k(Grave),  k(Kb1),  k(Kb2),   k(Kb3),  k(Kb4),  k(Kb5), NoOp,  /*|*/ NoOp,  k(Kb6), k(Kb7),  k(Kb8),   k(Kb9),   k(Kb0),    k(Minus), ],
            &[k(Tab),    k(Q),    k(W),     k(E),    k(R),    k(T),   NoOp,  /*|*/ NoOp,  k(Y),   k(U),    k(I),     k(O),     k(P),      k(Equal), ],
            &[k(LShift), k(A),    k(S),     k(D),    k(F),    k(G),   NoOp,  /*|*/ NoOp,  k(H),   k(J),    k(K),     k(L),     k(SColon), k(Quote), ],
            &[K_NUBS,    k(Z),    k(X),     k(C),    k(V),    k(B),   K_ENT, /*|*/ K_BSP, k(N),   k(M),    k(Comma), k(Dot),   k(Slash),  K_HASH,   ],
            &[NoOp,      NoOp,    k(LCtrl), k(LGui), k(LAlt), L_1,    K_SPC, /*|*/ K_SPC, L_1,    k(RAlt), K_APP,    k(RCtrl), NoOp,      NoOp,     ],
         ],
        &[
            &[K_ESC,     k(F1),   k(F2),    k(F3),   k(F4),   k(F5),  NoOp,  /*|*/ NoOp,  k(F6),  k(F7),   k(F8),    k(F9),    k(F10),    k(F11),   ],
            &[L_2,       CUT,     K_PLUS,   K_MINUS, K_LBRK,  K_RBRK, NoOp,  /*|*/ NoOp,  K_PGUP, k(Home), k(Up),    k(End),   NoOp,      k(F12),   ],
            &[NoOp,      COPY,    K_MUL,    K_DIV,   K_LBRA,  K_RBRA, NoOp,  /*|*/ NoOp,  K_PGDN, k(Left), k(Down),  k(Right), NoOp,      K_INS,    ],
            &[NoOp,      PASTE,   K_LT,     K_GT,    K_LPAR,  K_RPAR, NoOp,  /*|*/ K_DEL, NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      K_DEL,    ],
            &[NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    d(0),   NoOp,  /*|*/ NoOp,  d(0),   NoOp,    NoOp,     NoOp,     NoOp,      NoOp,     ],
        ],
        &[
            &[NoOp,      K_PSCR,  K_SLCK,   K_PAUS,  NoOp,    NoOp,   NoOp,  /*|*/ NoOp,  NoOp,  k(Kp7),   k(Kp8),   k(Kp9),   K_DIV,     NoOp,     ],
            &[NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    K_VUP,  NoOp,  /*|*/ NoOp,  NoOp,  k(Kp4),   k(Kp5),   k(Kp6),   K_MUL,     NoOp,     ],
            &[NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    K_VDN,  NoOp,  /*|*/ NoOp,  NoOp,  k(Kp1),   k(Kp2),   k(Kp3),   K_MINUS,   NoOp,     ],
            &[K_RSTL,    NoOp,    NoOp,     NoOp,    NoOp,    K_MUTE, NoOp,  /*|*/ K_BSP, NoOp,  NoOp,     k(Kp0),   NoOp,     K_PLUS,    K_RSTR,   ],
            &[NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    d(0),   K_SPC, /*|*/ K_ENT, d(0),  NoOp,     NoOp,     NoOp,     NoOp,      NoOp,     ],
        ],
        ];
        KEYMAP
    }
    pub static LAYERS: keyberon::layout::Layers<CustomKey> = make_keymap();

    const KBDSIZE_COLS: usize = 7;
    const KBDSIZE_ROWS: usize = 5;

    pub struct KeyboardState {
        is_left: bool,
        matrix: Matrix<DynPin, DynPin, KBDSIZE_COLS, KBDSIZE_ROWS>,
        debouncer: Debouncer<PressedKeys<KBDSIZE_COLS, KBDSIZE_ROWS>>,
    }

    impl KeyboardState {
        pub fn new(
            is_left: bool,
            mut rows: [DynPin; KBDSIZE_ROWS],
            mut cols: [DynPin; KBDSIZE_COLS],
        ) -> KeyboardState {
            for r in rows.iter_mut() {
                r.into_push_pull_output();
                r.set_high().unwrap();
            }
            for c in cols.iter_mut() {
                c.into_pull_up_input();
            }

            KeyboardState {
                is_left,
                matrix: Matrix::new(cols, rows).unwrap(),
                debouncer: Debouncer::new(PressedKeys::default(), PressedKeys::default(), 5),
            }
        }
    }

    const SCAN_TIME_US: u32 = 1_000;

    #[shared]
    struct Shared {
        layout: Layout,
        media_queue: Queue<MediaKeyHidReport, 8>,
        usb_dev: UsbDevice,
        usb_class: UsbClass,
        uart: UartDevice,
        rxbuf: [u8; 4],
    }

    #[local]
    struct Local {
        kbd_state: KeyboardState,
        transform: fn(Event) -> Event,
        timer: hal::Timer,
        alarm: hal::timer::Alarm0,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Set up the USB driver
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        let usb_bus = unsafe {
            // Note (safety): This is safe as interrupts haven't been started yet
            USB_BUS = Some(usb_bus);
            USB_BUS.as_ref().unwrap()
        };
        let usb_class = hid::HidClass::new(KbState::default(), usb_bus);
        let usb_dev = keyberon::new_device(usb_bus);

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt(&mut timer);

        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

        // Make a UART on the given pins
        let uart = hal::uart::UartPeripheral::new(c.device.UART0, &mut resets)
            .enable(
                hal::uart::common_configs::_38400_8_N_1,
                clocks.peripheral_clock.into(),
            )
            .unwrap();
        // TODO: enable UART IRQ when interrupt support is enabled in hal

        let kbd_side_pin = pins.gpio28.into_pull_up_input();
        let is_left = kbd_side_pin.is_low().unwrap();
        let transform: fn(Event) -> Event = if is_left {
            |e| e
        } else {
            |e| e.transform(|i, j| (i, j + KBDSIZE_COLS as u8))
        };

        let rows: [DynPin; KBDSIZE_ROWS] = [
            pins.gpio16.into(),
            pins.gpio17.into(),
            pins.gpio18.into(),
            pins.gpio19.into(),
            pins.gpio20.into(),
        ];
        let cols: [DynPin; KBDSIZE_COLS] = [
            pins.gpio2.into(),
            pins.gpio3.into(),
            pins.gpio4.into(),
            pins.gpio5.into(),
            pins.gpio6.into(),
            pins.gpio7.into(),
            pins.gpio8.into(),
        ];
        let kbd_state = KeyboardState::new(is_left, rows, cols);

        let shared = Shared {
            layout: Layout::new(LAYERS),
            media_queue: Queue::new(),
            usb_dev,
            usb_class,
            uart,
            rxbuf: [0; 4],
        };
        let local = Local {
            kbd_state,
            transform,
            timer,
            alarm,
        };
        (shared, local, init::Monotonics())
    }

    #[task(binds = USBCTRL_IRQ, priority = 2, shared = [usb_dev, usb_class])]
    fn usbctrl(c: usbctrl::Context) {
        let usb_dev = c.shared.usb_dev;
        let usb_class = c.shared.usb_class;
        (usb_dev, usb_class).lock(|usb_dev, usb_class| {
            usb_dev.poll(&mut [usb_class]);
        });
    }

    #[task(binds = TIMER_IRQ_0,
           local = [kbd_state, transform, timer, alarm, rset_count: u32 = 0],
           shared = [layout, media_queue, uart, rxbuf])]
    fn tick(c: tick::Context) {
        let timer = c.local.timer;
        let alarm = c.local.alarm;
        alarm.clear_interrupt(timer);
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());

        let kbd_state = c.local.kbd_state;
        let transform = c.local.transform;
        let rset_count = c.local.rset_count;
        (
            c.shared.uart,
            c.shared.rxbuf,
            c.shared.layout,
            c.shared.media_queue,
        )
            .lock(|uart, rxbuf, layout, media_queue| {
                uart_poll(layout, uart, rxbuf);

                handle_events(kbd_state, transform, layout, uart);
                let tick = layout.tick();
                match tick {
                    // reset if reset key pressed 5 times
                    CustomEvent::Release(CustomKey::Reset(k))
                        if k.is_left() == kbd_state.is_left =>
                    {
                        *rset_count += 1;
                        if *rset_count >= 5 {
                            *rset_count = 0;
                            do_reset();
                        }
                    }
                    CustomEvent::Press(CustomKey::Media(k)) => {
                        media_queue.enqueue(MediaKeyHidReport::from(k)).ok();
                    }
                    CustomEvent::Release(CustomKey::Media(_)) => {
                        media_queue.enqueue(MediaKeyHidReport::default()).ok();
                    }
                    _ => {}
                }
            });
        tick_keyberon::spawn().unwrap();
    }

    fn uart_poll(layout: &mut Layout, uart: &mut UartDevice, rxbuf: &mut [u8; 4]) {
        while let Ok(b) = uart.read() {
            rxbuf.rotate_left(1);
            rxbuf[3] = b;

            if rxbuf[3] == b'\n' {
                if let Ok(event) = de(&rxbuf[..]) {
                    layout.event(event);
                }
            }
        }
    }

    fn handle_events(
        kbd_state: &mut KeyboardState,
        transform: &fn(Event) -> Event,
        layout: &mut Layout,
        uart: &mut UartDevice,
    ) {
        let matrix_state = kbd_state.matrix.get().unwrap();
        let events = kbd_state.debouncer.events(matrix_state);
        for event in events {
            let event = transform(event);
            layout.event(event);
            for &b in &ser(event) {
                block!(uart.write(b)).unwrap();
            }
        }
    }

    #[task(shared = [layout, media_queue, usb_dev, usb_class])]
    fn tick_keyberon(c: tick_keyberon::Context) {
        let usb_dev = c.shared.usb_dev;
        let usb_class = c.shared.usb_class;
        let layout = c.shared.layout;
        let media_queue = c.shared.media_queue;
        (usb_dev, usb_class, layout, media_queue).lock(
            |usb_dev, usb_class, layout, media_queue| {
                if usb_dev.state() != usb_device::device::UsbDeviceState::Configured {
                    return;
                }

                while let Some(mk_report) = media_queue.dequeue() {
                    if usb_class.device_mut().set_mk_report(mk_report.clone()) {
                        send_report::spawn(either::Right(mk_report)).ok();
                    }
                }
                let report: KbHidReport = layout.keycodes().collect();
                if usb_class.device_mut().set_kb_report(report.clone()) {
                    send_report::spawn(either::Left(report)).ok();
                }
            },
        );
    }

    #[task(shared = [usb_class], capacity = 8)]
    fn send_report(
        c: send_report::Context,
        report: either::Either<KbHidReport, MediaKeyHidReport>,
    ) {
        let mut usb_class = c.shared.usb_class;
        usb_class.lock(|usb_class| {
            let res = match report.clone() {
                either::Left(mut r) => usb_class.write(r.as_bytes()),
                either::Right(mut r) => usb_class.write(r.as_bytes()),
            };
            if let Ok(0) = res {
                // no bytes written - rescedule to retry after next interrupt
                send_report::spawn(report).ok();
            }
        });
    }

    fn de(bytes: &[u8]) -> Result<Event, ()> {
        match *bytes {
            [b'P', i, j, b'\n'] => Ok(Event::Press(i, j)),
            [b'R', i, j, b'\n'] => Ok(Event::Release(i, j)),
            _ => Err(()),
        }
    }

    fn ser(e: Event) -> [u8; 4] {
        match e {
            Event::Press(i, j) => [b'P', i, j, b'\n'],
            Event::Release(i, j) => [b'R', i, j, b'\n'],
        }
    }

    fn do_reset() {
        hal::rom_data::reset_to_usb_boot(0, 0);
    }
}
