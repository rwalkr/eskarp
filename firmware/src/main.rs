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
    use keyberon::matrix::{Matrix};
    use nb::block;
    use rp_pico::hal;
    use rp_pico::hal::gpio::dynpin::DynPin;
    use usb_device::bus::UsbBusAllocator;

    type UsbClass = hid::HidClass<'static, hal::usb::UsbBus, KbState>;
    type UsbDevice = usb_device::device::UsbDevice<'static, hal::usb::UsbBus>;
    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    type UartPins = (
        hal::gpio::Pin<hal::gpio::pin::bank0::Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<hal::gpio::pin::bank0::Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    );
    type UartDevice = hal::uart::UartPeripheral<hal::uart::Enabled, hal::pac::UART0, UartPins>;

    pub enum CustomKey {
        Media(MediaKey),
        Reset(either::Either<(), ()>),
    }
    type Action = keyberon::action::Action<CustomKey>;
    type Layout = keyberon::layout::Layout<14, 5, 4, CustomKey>;
    type Layers = keyberon::layout::Layers<14, 5, 4, CustomKey>;

    const fn make_keymap() -> Layers {
        // aliases to keep keymap readable
        const K_NUBS: Action = k(NonUsBslash);
        const K_BKTK: Action = k(Grave);
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
        const UNDO: Action = m(&[LCtrl, Z]);
        const NK: Action = NoOp;

        #[rustfmt::skip]
        const KEYMAP: Layers = [
        [
            [K_ESC,     k(Kb1),  k(Kb2),   k(Kb3),  k(Kb4),  k(Kb5), NK,    /*|*/ NK,    k(Kb6), k(Kb7),  k(Kb8),   k(Kb9),   k(Kb0),    k(Minus), ],
            [k(Tab),    k(Q),    k(W),     k(E),    k(R),    k(T),   NK,    /*|*/ NK,    k(Y),   k(U),    k(I),     k(O),     k(P),      k(Equal), ],
            [k(LShift), k(A),    k(S),     k(D),    k(F),    k(G),   NK,    /*|*/ NK,    k(H),   k(J),    k(K),     k(L),     k(SColon), k(Quote), ],
            [k(LCtrl),  k(Z),    k(X),     k(C),    k(V),    k(B),   K_ENT, /*|*/ K_BSP, k(N),   k(M),    k(Comma), k(Dot),   k(Slash),  K_HASH,   ],
            [NK,        NK,      NoOp,     k(LGui), k(LAlt), L_1,    K_SPC, /*|*/ K_SPC, L_2,    k(RAlt), L_1,      k(RCtrl), NK,        NK,       ],
        ],
        // Nav / Select
        [
            [NoOp,      k(F1),   k(F2),    k(F3),   k(F4),   k(F5),  NK,    /*|*/ NK,    k(F6),  k(F7),   k(F8),    k(F9),    k(F10),    k(F11),   ],
            [NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    NoOp,   NK,    /*|*/ NK,    K_PGUP, k(Home), k(Up),    k(End),   K_APP,     k(F12),   ],
            [k(LShift), NoOp,    CUT,      COPY,    PASTE,   UNDO,   NK,    /*|*/ NK,    K_PGDN, k(Left), k(Down),  k(Right), NoOp,      K_INS,    ],
            [k(LCtrl),  NoOp,    NoOp,     NoOp,    NoOp,    NoOp,   NoOp,  /*|*/ K_DEL, NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      K_DEL,    ],
            [NK,        NK,      NoOp,     k(LGui), k(LAlt), d(0),   NoOp,  /*|*/ NoOp,  d(0),   k(RAlt), L_2,      k(RCtrl), NK,        NK,       ],
        ],
        // Symbols / Keypad 
        [
            [NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    NoOp,   NK,    /*|*/ NK,    NoOp,   k(Kb7),  k(Kb8),   k(Kb9),   K_DIV,     NoOp,     ],
            [NoOp,      NoOp,    K_PLUS,   K_MINUS, K_LBRK,  K_RBRK, NK,    /*|*/ NK,    K_NUBS, k(Kb4),  k(Kb5),   k(Kb6),   K_MUL,     NoOp,     ],
            [k(LShift), NoOp,    K_MUL,    K_DIV,   K_LBRA,  K_RBRA, NK,    /*|*/ NK,    K_BKTK, k(Kb1),  k(Kb2),   k(Kb3),   K_MINUS,   NoOp,     ],
            [k(LCtrl),  NoOp,    K_LT,     K_GT,    K_LPAR,  K_RPAR, NoOp,  /*|*/ K_BSP, NoOp,   NoOp,    k(Kb0),   NoOp,     K_PLUS,    NoOp,     ],
            [NK,        NK,      NoOp,     k(LGui), k(LAlt), d(0),   K_SPC, /*|*/ K_ENT, d(0),   k(RAlt), l(3),     k(RCtrl), NK,        NK,       ],
        ],
        // System
        [
            [K_RSTL,    K_PSCR,  K_SLCK,   K_PAUS,  NoOp,    NoOp,   NK,    /*|*/ NK,    NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      K_RSTR,   ],
            [NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    K_VUP,  NK,    /*|*/ NK,    NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      NoOp,     ],
            [NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    K_VDN,  NK,    /*|*/ NK,    NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      NoOp,     ],
            [NoOp,      NoOp,    NoOp,     NoOp,    NoOp,    K_MUTE, NoOp,  /*|*/ NoOp,  NoOp,   NoOp,    NoOp,     NoOp,     NoOp,      NoOp,     ],
            [NK,        NK,      NoOp,     NoOp,    NoOp,    d(0),   NoOp,  /*|*/ NoOp,  d(0),   NoOp,    d(0),     NoOp,     NK,        NK,       ],
        ],
        ];
        KEYMAP
    }
    pub static LAYERS: Layers = make_keymap();

    const KBDSIZE_COLS: usize = 7;
    const KBDSIZE_ROWS: usize = 5;

    pub struct KeyboardState {
        matrix: Matrix<DynPin, DynPin, KBDSIZE_COLS, KBDSIZE_ROWS>,
        debouncer: Debouncer<[[bool; KBDSIZE_COLS]; KBDSIZE_ROWS]>,
    }

    impl KeyboardState {
        pub fn new(
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
                matrix: Matrix::new(cols, rows).unwrap(),
                debouncer: Debouncer::new([[false; KBDSIZE_COLS]; KBDSIZE_ROWS], [[false; KBDSIZE_COLS]; KBDSIZE_ROWS], 5),
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
        is_left: bool,
        transform: fn(Event) -> Event,
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
        alarm.enable_interrupt();

        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio1.into_mode::<hal::gpio::FunctionUart>()
        );

        // Make a UART on the given pins
        let mut uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                hal::uart::common_configs::_38400_8_N_1,
                clocks.peripheral_clock.into(),
            )
            .unwrap();
        uart.enable_rx_interrupt();

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
        let kbd_state = KeyboardState::new(rows, cols);

        let shared = Shared {
            layout: Layout::new(&LAYERS),
            media_queue: Queue::new(),
            usb_dev,
            usb_class,
            uart,
            rxbuf: [0; 4],
        };
        let local = Local {
            kbd_state,
            is_left,
            transform,
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
           local = [kbd_state, transform, alarm],
           shared = [uart])]
    fn tick(c: tick::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());

        let transform = c.local.transform;
        let kbd_state = c.local.kbd_state;
        let mut uart = c.shared.uart;
        uart
            .lock(|uart| {
                let matrix_state = kbd_state.matrix.get().unwrap();
                let events = kbd_state.debouncer.events(matrix_state);
                for event in events {
                    let event = transform(event);
                    for &b in &ser(event) {
                        block!(uart.write(b)).unwrap();
                    }
                    handle_event::spawn(event).unwrap();
                }
            });
        tick_keyberon::spawn().unwrap();
    }

    #[task(binds = UART0_IRQ, shared = [uart, rxbuf])]
    fn uart_rx(c: uart_rx::Context) {
        (c.shared.uart,
         c.shared.rxbuf
        ).lock(|uart, rxbuf| {
            while let Ok(b) = uart.read() {
                rxbuf.rotate_left(1);
                rxbuf[3] = b;
    
                if rxbuf[3] == b'\n' {
                    if let Ok(event) = de(&rxbuf[..]) {
                        handle_event::spawn(event).unwrap();
                    }
                }
            }
        });
    }

    #[task(capacity = 8, shared = [layout])]
    fn handle_event(mut c: handle_event::Context, event: Event) {
        c.shared.layout.lock(|layout| layout.event(event));
    }

    #[task(shared = [layout, media_queue, usb_dev, usb_class],
           local = [is_left, rset_count: u32 = 0])]
    fn tick_keyberon(c: tick_keyberon::Context) {
        let usb_dev = c.shared.usb_dev;
        let usb_class = c.shared.usb_class;
        let layout = c.shared.layout;
        let media_queue = c.shared.media_queue;
        let is_left = *c.local.is_left;
        let rset_count = c.local.rset_count;
        (usb_dev, usb_class, layout, media_queue).lock(
            |usb_dev, usb_class, layout, media_queue| {
                let tick = layout.tick();
                match tick {
                    // reset if reset key pressed 5 times
                    CustomEvent::Release(CustomKey::Reset(k))
                        if k.is_left() == is_left =>
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
