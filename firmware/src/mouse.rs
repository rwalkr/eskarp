use keyberon::hid::{HidDevice, Protocol, ReportType, Subclass};

#[rustfmt::skip]
const REPORT_DESCRIPTOR : &[u8] = &[
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x08,        //     Usage Maximum (0x08)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x75, 0x01,        //     Report Size (1)
    0x95, 0x08,        //     Report Count (8)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //     Usage (X)
    0x17, 0x81, 0xFF, 0xFF, 0xFF,  //     Logical Minimum (-128)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x31,        //     Usage (Y)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x38,        //     Usage (Wheel)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0C,        //     Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //     Usage (AC Pan)
    0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0xC0,              // End Collection
];

#[derive(Default)]
pub struct HidMouse {
    report: MouseReport,
}

impl HidMouse {
    pub fn set_report(&mut self, report: MouseReport) -> bool {
        if report == self.report {
            false
        } else {
            self.report = report;
            true
        }
    }
}

impl HidDevice for HidMouse {
    fn subclass(&self) -> Subclass {
        Subclass::None
    }

    fn protocol(&self) -> Protocol {
        Protocol::Mouse
    }
    fn report_descriptor(&self) -> &[u8] {
        REPORT_DESCRIPTOR
    }

    fn max_packet_size(&self) -> u16 {
        6 as u16
    }

    fn get_report(
        &mut self,
        report_type: ReportType,
        _report_id: u8,
    ) -> Result<&[u8], keyberon::hid::Error> {
        match report_type {
            ReportType::Input => Ok(self.report.as_bytes()),
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
pub struct MouseReport([u8; 6]);

impl Default for MouseReport {
    fn default() -> Self {
        let mut res = MouseReport([0; 6]);
        res.0[0] = 1;
        res
    }
}

impl MouseReport {
    /// Returns the byte slice corresponding to the report.
    pub fn as_bytes(&mut self) -> &[u8] {
        &self.0
    }

    pub fn buttons(mut self, buttons: u8) -> Self {
        self.0[1] = buttons;
        self
    }

    pub fn pos(mut self, x: i8, y: i8) -> Self {
        self.0[2] = x as u8;
        self.0[3] = y as u8;
        self
    }

    pub fn wheel(mut self, v: i8) -> Self {
        self.0[4] = v as u8;
        self
    }

    pub fn pan(mut self, v: i8) -> Self {
        self.0[5] = v as u8;
        self
    }
}
