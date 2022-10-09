use keyberon::hid::{HidDevice, Protocol, ReportType, Subclass};
use keyberon::key_code::KeyCode;

#[rustfmt::skip]
const REPORT_DESCRIPTOR : &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x95, 0x08,        //   Report Count (8)
    0x75, 0x01,        //   Report Size (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x29, 0xFF,        //   Usage Maximum (0xFF)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x95, 0x06,        //   Report Count (6)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x08,        //   Usage Page (LEDs)
    0x19, 0x01,        //   Usage Minimum (Num Lock)
    0x29, 0x05,        //   Usage Maximum (Kana)
    0x95, 0x05,        //   Report Count (5)
    0x75, 0x01,        //   Report Size (1)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x03,        //   Report Size (3)
    0x91, 0x03,        //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection
    0x05, 0x0C,                     // Usage Page (Consumer Devices)
    0x09, 0x01,                     // Usage (Consumer Control)
    0xA1, 0x01,                     // Collection (Application)
    0x85, 0x02,                     //      Report ID
    0x75, 0x10,                     //      Report Size (16)
    0x95, 0x01,                     //     Report Count (1)
    0x26, 0xFF, 0x07,               //      Logical Maximum (2047)
    0x19, 0x00,                     //      Usage Minimum (0)
    0x2A, 0xFF, 0x07,               //      Usage Maximum (2047)
    0x81, 0x00,                     //      Input (Data, Ary, Abs)
    0xC0,
];

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum MediaKey {
    Record = 0x0B2,
    FastForward = 0x0B3,
    Rewind = 0x0B4,
    NextTrack = 0x0B5,
    PrevTrack = 0x0B6,
    Stop = 0x0B7,
    Eject = 0x0B8,
    RandomPlay = 0x0B9,
    StopEject = 0x0CC,
    PlayPause = 0x0CD,
    AudioMute = 0x0E2,
    AudioVolUp = 0x0E9,
    AudioVolDown = 0x0EA,
}

#[derive(Default)]
pub struct KbState {
    mk_report: MediaKeyHidReport,
    kb_report: KbHidReport,
}

impl KbState {
    pub fn set_mk_report(&mut self, report: MediaKeyHidReport) -> bool {
        if report == self.mk_report {
            false
        } else {
            self.mk_report = report;
            true
        }
    }

    pub fn set_kb_report(&mut self, report: KbHidReport) -> bool {
        if report == self.kb_report {
            false
        } else {
            self.kb_report = report;
            true
        }
    }
}

impl HidDevice for KbState {
    fn subclass(&self) -> Subclass {
        Subclass::None
    }

    fn protocol(&self) -> Protocol {
        Protocol::Keyboard
    }
    fn report_descriptor(&self) -> &[u8] {
        REPORT_DESCRIPTOR
    }

    fn max_packet_size(&self) -> u16 {
        9 as u16
    }

    fn get_report(
        &mut self,
        report_type: ReportType,
        _report_id: u8,
    ) -> Result<&[u8], keyberon::hid::Error> {
        match report_type {
            ReportType::Input => Ok(self.kb_report.as_bytes()),
            _ => Err(keyberon::hid::Error),
        }
    }

    fn set_report(
        &mut self,
        _report_type: ReportType,
        _report_id: u8,
        _data: &[u8],
    ) -> Result<(), keyberon::hid::Error> {
        Ok(())
    }
}

#[derive(PartialEq, Copy, Clone)]
pub struct MediaKeyHidReport([u8; 3]);

impl Default for MediaKeyHidReport {
    fn default() -> Self {
        let mut res = MediaKeyHidReport([0; 3]);
        res.0[0] = 2;
        res
    }
}

impl MediaKeyHidReport {
    pub fn as_bytes(&mut self) -> &[u8] {
        &self.0
    }
}

impl From<&MediaKey> for MediaKeyHidReport {
    fn from(key: &MediaKey) -> Self {
        let mut rep = MediaKeyHidReport::default();
        rep.0[1] = *key as u8;
        rep.0[2] = ((*key as u16) >> 8) as u8;
        rep
    }
}

#[derive(Clone, Eq, PartialEq)]
pub struct KbHidReport([u8; 9]);

impl core::iter::FromIterator<KeyCode> for KbHidReport {
    fn from_iter<T>(iter: T) -> Self
    where
        T: IntoIterator<Item = KeyCode>,
    {
        let mut res = Self::default();
        for kc in iter {
            res.pressed(kc);
        }
        res
    }
}

impl Default for KbHidReport {
    fn default() -> Self {
        let mut res = KbHidReport([0; 9]);
        res.0[0] = 1;
        res
    }
}

impl KbHidReport {
    /// Returns the byte slice corresponding to the report.
    pub fn as_bytes(&mut self) -> &[u8] {
        &self.0
    }

    /// Add the given key code to the report. If the report is full,
    /// it will be set to `ErrorRollOver`.
    pub fn pressed(&mut self, kc: KeyCode) {
        use KeyCode::*;
        match kc {
            No => (),
            ErrorRollOver | PostFail | ErrorUndefined => self.set_all(kc),
            kc if kc.is_modifier() => self.0[1] |= kc.as_modifier_bit(),
            _ => self.0[3..]
                .iter_mut()
                .find(|c| **c == 0)
                .map(|c| *c = kc as u8)
                .unwrap_or_else(|| self.set_all(ErrorRollOver)),
        }
    }
    fn set_all(&mut self, kc: KeyCode) {
        for c in &mut self.0[2..] {
            *c = kc as u8;
        }
    }
}

#[derive(Clone)]
pub enum HIDReport {
    Keyboard(KbHidReport),
    MediaKey(MediaKeyHidReport),
}
