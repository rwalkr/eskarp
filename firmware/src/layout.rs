use crate::app::Action;
use crate::app::CustomKey;
use crate::app::Layers;

use keyberon::action::{d, k, l, m, Action::*};
use usbd_human_interface_device::page::{Consumer, Keyboard::*};

// aliases to keep keymap readable
const K_0: Action = k(Keyboard0);
const K_1: Action = k(Keyboard1);
const K_2: Action = k(Keyboard2);
const K_3: Action = k(Keyboard3);
const K_4: Action = k(Keyboard4);
const K_5: Action = k(Keyboard5);
const K_6: Action = k(Keyboard6);
const K_7: Action = k(Keyboard7);
const K_8: Action = k(Keyboard8);
const K_9: Action = k(Keyboard9);
const K_NUBS: Action = k(NonUSBackslash);
const K_PIPE: Action = m(&[LeftShift, NonUSBackslash].as_slice());
const K_BKTK: Action = k(Grave);
const K_ENT: Action = k(ReturnEnter);
const K_SPC: Action = k(Space);
const K_BSP: Action = k(DeleteBackspace);
const K_APP: Action = k(Application);
const K_LBRK: Action = k(LeftBrace);
const K_RBRK: Action = k(RightBrace);
const K_LBRA: Action = m(&[LeftShift, LeftBrace].as_slice());
const K_RBRA: Action = m(&[LeftShift, RightBrace].as_slice());
const K_LPAR: Action = m(&[LeftShift, Keyboard9].as_slice());
const K_RPAR: Action = m(&[LeftShift, Keyboard0].as_slice());
const K_LT: Action = m(&[LeftShift, Comma].as_slice());
const K_GT: Action = m(&[LeftShift, Dot].as_slice());
const K_EQ: Action = k(Equal);
const K_SEMI: Action = k(Semicolon);
const K_QUOT: Action = k(Apostrophe);
const K_HASH: Action = k(NonUSHash);
const K_SLSH: Action = k(ForwardSlash);
const K_TILD: Action = m(&[LeftShift, NonUSHash].as_slice());
const K_AMP: Action = m(&[LeftShift, Apostrophe].as_slice());
const K_COLN: Action = m(&[LeftShift, Semicolon].as_slice());
const K_QUMK: Action = m(&[LeftShift, ForwardSlash].as_slice());
const K_PLUS: Action = k(KeypadAdd);
const K_MINUS: Action = k(KeypadSubtract);
const K_MUL: Action = k(KeypadMultiply);
const K_DIV: Action = k(KeypadDivide);
const K_LEFT: Action = k(LeftArrow);
const K_RIGHT: Action = k(RightArrow);
const K_UP: Action = k(UpArrow);
const K_DOWN: Action = k(DownArrow);
const K_PGUP: Action = k(PageUp);
const K_PGDN: Action = k(PageDown);
const K_INS: Action = k(Insert);
const K_DEL: Action = k(DeleteForward);
const K_ESC: Action = k(Escape);
const K_MUTE: Action = Action::Custom(CustomKey::Media(Consumer::Mute));
const K_VUP: Action = Action::Custom(CustomKey::Media(Consumer::VolumeIncrement));
const K_VDN: Action = Action::Custom(CustomKey::Media(Consumer::VolumeDecrement));
const K_PSCR: Action = k(PrintScreen);
const K_SLCK: Action = k(ScrollLock);
const K_PAUS: Action = k(Pause);
const K_LSHFT: Action = k(LeftShift);
const K_LCTRL: Action = k(LeftControl);
const K_RCTRL: Action = k(RightControl);
const K_LALT: Action = k(LeftAlt);
const K_RALT: Action = k(RightAlt);
const K_LGUI: Action = k(LeftGUI);
const K_RSTL: Action = Action::Custom(CustomKey::Reset(either::Left(())));
const K_RSTR: Action = Action::Custom(CustomKey::Reset(either::Right(())));
const L_1: Action = l(1);
const L_2: Action = l(2);
const CUT: Action = m(&[LeftControl, X].as_slice());
const COPY: Action = m(&[LeftControl, C].as_slice());
const PASTE: Action = m(&[LeftControl, V].as_slice());
const UNDO: Action = m(&[LeftControl, Z].as_slice());
const NK: Action = NoOp;

pub const fn make_keymap() -> Layers {

    #[rustfmt::skip]
    const KEYMAP: Layers = [
        [
            [K_ESC,   K_1,     K_2,     K_3,     K_4,     K_5,     NK,    /*|*/ NK,      K_6,     K_7,     K_8,      K_9,     K_0,     k(Minus),],
            [K_LALT,  k(Q),    k(W),    k(E),    k(R),    k(T),    NK,    /*|*/ NK,      k(Y),    k(U),    k(I),     k(O),    k(P),    K_EQ,    ],
            [K_LSHFT, k(A),    k(S),    k(D),    k(F),    k(G),    NK,    /*|*/ NK,      k(H),    k(J),    k(K),     k(L),    K_SEMI,  K_QUOT,  ],
            [K_LCTRL, k(Z),    k(X),    k(C),    k(V),    k(B),    K_ENT, /*|*/ K_BSP,   k(N),    k(M),    k(Comma), k(Dot),  K_SLSH,  K_HASH,  ],
            [NK,      NK,      K_LGUI,  L_2,     K_LALT,  L_1,     K_SPC, /*|*/ k(Tab),  L_2,     K_LALT,  K_LGUI,   K_LCTRL, NK,      NK,      ],
        ],
        // Nav / Select
        [
            [NoOp,    k(F1),   k(F2),   k(F3),   k(F4),   k(F5),   NK,    /*|*/ NK,      k(F6),   k(F7),   k(F8),    k(F9),   k(F10),  K_APP,   ],
            [K_LALT,  k(F11),  k(F12),  k(F13),  k(F14),  k(F15),  NK,    /*|*/ NK,      K_PGUP,  k(Home), K_UP,     k(End),  NoOp,    NoOp,    ],
            [K_LSHFT, NoOp,    CUT,     COPY,    PASTE,   NoOp,    NK,    /*|*/ NK,      K_PGDN,  K_LEFT,  K_DOWN,   K_RIGHT, NoOp,    K_INS,   ],
            [K_LCTRL, NoOp,    NoOp,    NoOp,    UNDO,    NoOp,    NoOp,  /*|*/ K_DEL,   NoOp,    NoOp,    NoOp,     NoOp,    NoOp,    K_DEL,   ],
            [NK,      NK,      K_LGUI,  d(0),    K_LALT,  d(0),    NoOp,  /*|*/ NoOp,    L_2,     K_LALT,  K_LGUI,   K_LCTRL, NK,      NK,      ],
        ],
        // Symbols / Keypad 
        [
            [NoOp,    NoOp,    K_COLN,  K_AMP,   K_LBRK,  K_RBRK,  NK,    /*|*/ NK,      K_TILD,  K_7,     K_8,      K_9,     NoOp,    NoOp,    ],
            [K_LALT,  NoOp,    K_NUBS,  K_SLSH,  K_LBRA,  K_RBRA,  NK,    /*|*/ NK,      K_NUBS,  K_4,     K_5,      K_6,     K_MINUS, K_DIV,   ],
            [K_LSHFT, NoOp,    K_HASH,  K_QUMK,  K_LPAR,  K_RPAR,  NK,    /*|*/ NK,      K_BKTK,  K_1,     K_2,      K_3,     K_PLUS,  K_MUL,   ],
            [K_LCTRL, NoOp,    NoOp,    NoOp,    K_LT,    K_GT,    K_ENT, /*|*/ K_BSP,   K_PIPE,  NoOp,    K_0,      k(Dot),  K_EQ,    NoOp,    ],
            [NK,      NK,      K_LGUI,  d(0),    K_LALT,  d(0),    K_SPC, /*|*/ L_1,     d(0),    K_RALT,  l(3),     K_RCTRL, NK,      NK,      ],
        ],
        // System
        [
            [K_RSTL,  K_PSCR,  K_SLCK,  K_PAUS,  NoOp,    NoOp,    NK,    /*|*/ NK,      NoOp,    NoOp,    NoOp,     NoOp,    NoOp,    K_RSTR,  ],
            [NoOp,    NoOp,    NoOp,    NoOp,    NoOp,    K_VUP,   NK,    /*|*/ NK,      NoOp,    NoOp,    NoOp,     NoOp,    NoOp,    NoOp,    ],
            [NoOp,    NoOp,    NoOp,    NoOp,    NoOp,    K_VDN,   NK,    /*|*/ NK,      NoOp,    NoOp,    NoOp,     NoOp,    NoOp,    NoOp,    ],
            [NoOp,    NoOp,    NoOp,    NoOp,    NoOp,    K_MUTE,  NoOp,  /*|*/ NoOp,    NoOp,    NoOp,    NoOp,     NoOp,    NoOp,    NoOp,    ],
            [NK,      NK,      NoOp,    NoOp,    NoOp,    d(0),    NoOp,  /*|*/ NoOp,    d(0),    NoOp,    d(0),     NoOp,    NK,      NK,      ],
        ],
    ];
    KEYMAP
}
