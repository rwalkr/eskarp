use crate::Action;
use crate::CustomKey;
use crate::Layers;

use keyberon::action::{d, k, l, m, Action::*};
use usbd_human_interface_device::page::{Consumer, Keyboard, Keyboard::*};

pub fn make_keymap(layout: &'static str) -> Result<Layers, LayoutErr> {
    parse_layout(layout)
}

#[derive(PartialEq, Debug)]
pub enum LayoutErr {
    InvalidLayout,
    UnsupportedPrefix(&'static str),
    UnsupportedAction(&'static str),
    UnsupportedKey(Option<u8>),
    UnsupportedLayer(Option<u8>),
    UnsupportedDefLayer(Option<u8>),
    UnsupportedFn(&'static str),
}

fn parse_a(a: &'static str) -> Result<Action, LayoutErr> {
    // need to be able to make static references to these
    const K_PIPE: Action = m(&[LeftShift, NonUSBackslash].as_slice());
    const K_LBRA: Action = m(&[LeftShift, LeftBrace].as_slice());
    const K_RBRA: Action = m(&[LeftShift, RightBrace].as_slice());
    const K_LPAR: Action = m(&[LeftShift, Keyboard9].as_slice());
    const K_RPAR: Action = m(&[LeftShift, Keyboard0].as_slice());
    const K_LT: Action = m(&[LeftShift, Comma].as_slice());
    const K_GT: Action = m(&[LeftShift, Dot].as_slice());
    const K_TILD: Action = m(&[LeftShift, NonUSHash].as_slice());
    const K_AMP: Action = m(&[LeftShift, Apostrophe].as_slice());
    const K_COLN: Action = m(&[LeftShift, Semicolon].as_slice());
    const K_QUMK: Action = m(&[LeftShift, ForwardSlash].as_slice());
    const CUT: Action = m(&[LeftControl, X].as_slice());
    const COPY: Action = m(&[LeftControl, C].as_slice());
    const PASTE: Action = m(&[LeftControl, V].as_slice());
    const UNDO: Action = m(&[LeftControl, Z].as_slice());

    match a {
        "NUBS" => Ok(k(NonUSBackslash)),
        "PIPE" => Ok(K_PIPE),
        "BKTK" => Ok(k(Grave)),
        "ENT" => Ok(k(ReturnEnter)),
        "TAB" => Ok(k(Tab)),
        "SPC" => Ok(k(Space)),
        "BSP" => Ok(k(DeleteBackspace)),
        "APP" => Ok(k(Application)),
        "LBRK" => Ok(k(LeftBrace)),
        "RBRK" => Ok(k(RightBrace)),
        "LBRA" => Ok(K_LBRA),
        "RBRA" => Ok(K_RBRA),
        "LPAR" => Ok(K_LPAR),
        "RPAR" => Ok(K_RPAR),
        "LT" => Ok(K_LT),
        "GT" => Ok(K_GT),
        "DASH" => Ok(k(Minus)),
        "EQ" => Ok(k(Equal)),
        "SEMI" => Ok(k(Semicolon)),
        "QUOT" => Ok(k(Apostrophe)),
        "HASH" => Ok(k(NonUSHash)),
        "SLSH" => Ok(k(ForwardSlash)),
        "TILD" => Ok(K_TILD),
        "AMP" => Ok(K_AMP),
        "COLN" => Ok(K_COLN),
        "QUMK" => Ok(K_QUMK),
        "COMMA" => Ok(k(Comma)),
        "DOT" => Ok(k(Dot)),
        "PLUS" => Ok(k(KeypadAdd)),
        "MINUS" => Ok(k(KeypadSubtract)),
        "MUL" => Ok(k(KeypadMultiply)),
        "DIV" => Ok(k(KeypadDivide)),
        "LEFT" => Ok(k(LeftArrow)),
        "RIGHT" => Ok(k(RightArrow)),
        "UP" => Ok(k(UpArrow)),
        "DOWN" => Ok(k(DownArrow)),
        "HOME" => Ok(k(Home)),
        "END" => Ok(k(End)),
        "PGUP" => Ok(k(PageUp)),
        "PGDN" => Ok(k(PageDown)),
        "INS" => Ok(k(Insert)),
        "DEL" => Ok(k(DeleteForward)),
        "ESC" => Ok(k(Escape)),
        "MUTE" => Ok(Action::Custom(CustomKey::Media(Consumer::Mute))),
        "VUP" => Ok(Action::Custom(CustomKey::Media(Consumer::VolumeIncrement))),
        "VDN" => Ok(Action::Custom(CustomKey::Media(Consumer::VolumeDecrement))),
        "PSCR" => Ok(k(PrintScreen)),
        "SLCK" => Ok(k(ScrollLock)),
        "PAUS" => Ok(k(Pause)),
        "CLCK" => Ok(k(CapsLock)),
        "NLCK" => Ok(k(KeypadNumLockAndClear)),
        "LSHFT" => Ok(k(LeftShift)),
        "LCTRL" => Ok(k(LeftControl)),
        "RCTRL" => Ok(k(RightControl)),
        "LALT" => Ok(k(LeftAlt)),
        "RALT" => Ok(k(RightAlt)),
        "LGUI" => Ok(k(LeftGUI)),
        "RSTL" => Ok(Action::Custom(CustomKey::Reset(either::Left(())))),
        "RSTR" => Ok(Action::Custom(CustomKey::Reset(either::Right(())))),
        "CUT" => Ok(CUT),
        "COPY" => Ok(COPY),
        "PASTE" => Ok(PASTE),
        "UNDO" => Ok(UNDO),
        "KP0" => Ok(k(Keypad0)),
        "KP1" => Ok(k(Keypad1)),
        "KP2" => Ok(k(Keypad2)),
        "KP3" => Ok(k(Keypad3)),
        "KP4" => Ok(k(Keypad4)),
        "KP5" => Ok(k(Keypad5)),
        "KP6" => Ok(k(Keypad6)),
        "KP7" => Ok(k(Keypad7)),
        "KP8" => Ok(k(Keypad8)),
        "KP9" => Ok(k(Keypad9)),
        _ => Err(LayoutErr::UnsupportedAction(a)),
    }
}

fn parse_k(c: Option<u8>) -> Result<Action, LayoutErr> {
    // offset from base key in range
    match c {
        Some(c @ b'A'..=b'Z') => {
            let o = c - b'A';
            let i = u8::from(A) + o;
            Ok(k(Keyboard::from(i)))
        }
        Some(b'0') => Ok(k(Keyboard0)),
        Some(c @ b'1'..=b'9') => {
            let o = c - b'1';
            let i = u8::from(Keyboard1) + o;
            Ok(k(Keyboard::from(i)))
        }
        _ => Err(LayoutErr::UnsupportedKey(c)),
    }
}

fn parse_f(s: &'static str) -> Result<Action, LayoutErr> {
    let i = s.parse::<u8>().map_err(|_| LayoutErr::UnsupportedFn(s))?;
    if i == 0 {
        Err(LayoutErr::UnsupportedFn(s))
    } else if i <= 12 {
        Ok(k(Keyboard::from(u8::from(F1) + i - 1)))
    } else if i <= 24 {
        Ok(k(Keyboard::from(u8::from(F13) + i - 13)))
    } else {
        Err(LayoutErr::UnsupportedFn(s))
    }
}

fn parse_l(c: Option<u8>) -> Result<Action, LayoutErr> {
    match c {
        Some(c @ b'0'..=b'3') => {
            let i = c - b'0';
            Ok(l(i.into()))
        }
        _ => Err(LayoutErr::UnsupportedLayer(c)),
    }
}

fn parse_d(c: Option<u8>) -> Result<Action, LayoutErr> {
    match c {
        Some(c @ b'0'..=b'3') => {
            let i = c - b'0';
            Ok(d(i.into()))
        }
        _ => Err(LayoutErr::UnsupportedDefLayer(c)),
    }
}

fn parse_action(a: &'static str) -> Result<Action, LayoutErr> {
    match a {
        "_" | "." => Ok(NoOp),
        _ if a.starts_with("@") => parse_a(&a[1..]),
        _ if a.starts_with(":") => parse_k(a.bytes().nth(1)),
        _ if a.starts_with("F") => parse_f(&a[1..]),
        _ if a.starts_with("L") => parse_l(a.bytes().nth(1)),
        _ if a.starts_with("D") => parse_d(a.bytes().nth(1)),
        _ => Err(LayoutErr::UnsupportedPrefix(a)),
    }
}

pub fn parse_layout<const C: usize, const R: usize, const L: usize>(
    s: &'static str,
) -> Result<keyberon::layout::Layers<{ C }, { R }, { L }, CustomKey, Keyboard>, LayoutErr> {
    let mut l: keyberon::layout::Layers<{ C }, { R }, { L }, CustomKey, Keyboard> =
        [[[NoOp; C]; R]; L];

    let mut k = s.split_whitespace().filter(|i| *i != "|").peekable();

    if k.peek().is_none() {
        return Err(LayoutErr::InvalidLayout);
    }

    for li in 0..L {
        for ri in 0..R {
            for ci in 0..C {
                let ks = k.next().unwrap_or("_");
                l[li][ri][ci] = parse_action(ks)?;
            }
        }
    }

    Ok(l)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_parse_k() {
        assert_eq!(Ok(k(A)), parse_k(Some(b'A')));
        assert_eq!(Ok(k(B)), parse_k(Some(b'B')));
        assert_eq!(Ok(k(Z)), parse_k(Some(b'Z')));
        assert_eq!(Ok(k(Keyboard0)), parse_k(Some(b'0')));
        assert_eq!(Ok(k(Keyboard1)), parse_k(Some(b'1')));
        assert_eq!(Ok(k(Keyboard9)), parse_k(Some(b'9')));
        assert_eq!(Err(LayoutErr::UnsupportedKey(None)), parse_k(None));
        assert_eq!(
            Err(LayoutErr::UnsupportedKey(Some(b'#'))),
            parse_k(Some(b'#'))
        );
    }

    #[test]
    fn test_parse_f() {
        assert_eq!(Ok(k(F1)), parse_f("1"));
        assert_eq!(Ok(k(F2)), parse_f("2"));
        assert_eq!(Ok(k(F9)), parse_f("9"));
        assert_eq!(Ok(k(F10)), parse_f("10"));
        assert_eq!(Ok(k(F12)), parse_f("12"));
        assert_eq!(Ok(k(F13)), parse_f("13"));
        assert_eq!(Ok(k(F24)), parse_f("24"));
        assert_eq!(Err(LayoutErr::UnsupportedFn("0")), parse_f("0"));
        assert_eq!(Err(LayoutErr::UnsupportedFn("25")), parse_f("25"));
    }

    #[test]
    fn test_parse_a() {
        assert_eq!(Ok(k(NonUSBackslash)), parse_a("NUBS"));
        assert_eq!(Err(LayoutErr::UnsupportedAction("BLAH")), parse_a("BLAH"));
    }

    #[test]
    fn test_parse_l() {
        assert_eq!(Ok(l(1)), parse_l(Some(b'1')));
        assert_eq!(
            Err(LayoutErr::UnsupportedLayer(Some(b'4'))),
            parse_l(Some(b'4'))
        );
    }

    #[test]
    fn test_parse_d() {
        assert_eq!(Ok(d(1)), parse_d(Some(b'1')));
        assert_eq!(
            Err(LayoutErr::UnsupportedDefLayer(Some(b'4'))),
            parse_d(Some(b'4'))
        );
    }

    #[test]
    fn test_action() {
        assert_eq!(Ok(k(NonUSBackslash)), parse_action("@NUBS"));
        assert_eq!(Ok(k(A)), parse_action(":A"));
        assert_eq!(Ok(k(Keyboard0)), parse_action(":0"));
        assert_eq!(Ok(l(1)), parse_action("L1"));
        assert_eq!(Ok(l(1)), parse_action("L1"));
        assert_eq!(Ok(NoOp), parse_action("."));
        assert_eq!(Ok(NoOp), parse_action("_"));
        assert_eq!(Err(LayoutErr::UnsupportedPrefix("?")), parse_action("?"));
        assert_eq!(
            Err(LayoutErr::UnsupportedPrefix("?Foo")),
            parse_action("?Foo")
        );
    }

    #[test]
    fn test_empty_layout() {
        assert_eq!(Err(LayoutErr::InvalidLayout), make_keymap(""));
    }

    const LAYOUT: &str = "
    @ESC   :1     :2     :3     :4     :5     .      | .      :6     :7     :8     :9     :0     @DASH
@LALT  :Q     :W     :E     :R     :T     .      | .      :Y     :U     :I     :O     :P     @EQ
@LSHFT :A     :S     :D     :F     :G     .      | .      :H     :J     :K     :L     @SEMI  @QUOT
@LCTRL :Z     :X     :C     :V     :B     @ENT   | @BSP   :N     :M     @COMMA @DOT   @SLSH  @HASH
.      .      @LGUI  L2     @LALT  L1     @SPC   | @TAB   L2     @LALT  @LGUI  @LCTRL .      .

_      F1     F2     F3     F4     F5     .      | .      F6     F7     F8     F9     F10    @APP
@LALT  F11    F12    F13    F14    F15    .      | .      @PGUP  @HOME  @UP    @END   _      _
@LSHFT _      @CUT   @COPY  @PASTE _      .      | .      @PGDN  @LEFT  @DOWN  @RIGHT _      @INS
@LCTRL _      _      _      @UNDO  _      _      | @DEL   _      _      _      _      _      @DEL
.      .      @LGUI  D0     @LALT  D0     _      | _      L2     @LALT  @LGUI  @LCTRL .      @RSTR
    ";

    #[test]
    fn test_full_layout() {
        let l = make_keymap(LAYOUT).unwrap();
        assert_eq!(k(Q), l[0][1][1]);
        assert_eq!(k(NonUSHash), l[0][3][13]);
        assert_eq!(NoOp, l[1][0][0]);
        assert_eq!(k(F1), l[1][0][1]);
        assert_eq!(k(LeftControl), l[1][4][11]);
        assert_eq!(
            Action::Custom(CustomKey::Reset(either::Right(()))),
            l[1][4][13]
        );
    }
}
