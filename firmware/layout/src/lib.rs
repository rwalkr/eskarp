#![cfg_attr(not(test), no_std)]

use usbd_human_interface_device::page::{Consumer, Keyboard};

pub mod layout;

pub const KBDSIZE_COLS: usize = 7;
pub const KBDSIZE_COLS_2: usize = KBDSIZE_COLS * 2;
pub const KBDSIZE_ROWS: usize = 5;
pub const KBDSIZE_LAYERS: usize = 4;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum CustomKey {
    Media(Consumer),
    Reset(either::Either<(), ()>),
}
pub type Action = keyberon::action::Action<CustomKey, Keyboard>;
pub type Layout =
    keyberon::layout::Layout<KBDSIZE_COLS_2, KBDSIZE_ROWS, KBDSIZE_LAYERS, CustomKey, Keyboard>;
pub type Layers =
    keyberon::layout::Layers<KBDSIZE_COLS_2, KBDSIZE_ROWS, KBDSIZE_LAYERS, CustomKey, Keyboard>;
