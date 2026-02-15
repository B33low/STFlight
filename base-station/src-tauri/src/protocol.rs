// src-tauri/src/protocol.rs
pub const AA: u8 = 0xAA;
pub const BB: u8 = 0x55;

#[derive(Debug, Clone)]
pub struct Frame {
    pub msg: MsgType,
    pub kind: MsgKind,
    pub id: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
enum St {
    WaitAA,
    Wait55,
    ReadMsg,
    ReadKind,
    ReadId,
    ReadLen,
    ReadPayload,
}
#[repr(u8)]
#[derive(Debug, Clone, Copy,PartialEq)]
pub enum MsgType {
    BusMsgPublish,
    BusMsgWrite,
    BusMsgInject,
    BusMsgReadReq,
}
#[repr(u8)]
#[derive(Debug, Clone, Copy,PartialEq)]
pub enum MsgKind {
    BusKindState,
    BusKindParam,
    BusKindStream,
}


impl MsgType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::BusMsgPublish),
            2 => Some(Self::BusMsgWrite),
            3 => Some(Self::BusMsgInject),
            4 => Some(Self::BusMsgReadReq),
            _ => None,
        }
    }
}

impl MsgKind {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::BusKindState),
            2 => Some(Self::BusKindParam),
            3 => Some(Self::BusKindStream),
            _ => None,
        }
    }
}


pub struct BusFrameParser {
    st: St,
    msg: u8,
    kind: u8,
    id: u8,
    len: u8,
    payload: Vec<u8>,
}

impl Default for BusFrameParser {
    fn default() -> Self {
        Self {
            st: St::WaitAA,
            msg: 0,
            kind: 0,
            id: 0,
            len: 0,
            payload: Vec::new(),
        }
    }
}

impl BusFrameParser {
    pub fn feed(&mut self, data: &[u8]) -> Vec<Frame> {
        let mut frames = Vec::new();

        for &b in data {
            match self.st {
                St::WaitAA => {
                    self.st = if b == AA { St::Wait55 } else { St::WaitAA };
                }
                St::Wait55 => {
                    if b == BB {
                        self.st = St::ReadMsg;
                    } else {
                        self.st = if b == AA { St::Wait55 } else { St::WaitAA };
                    }
                }
                St::ReadMsg => {
                    self.msg = b;
                    self.st = St::ReadKind;
                }
                St::ReadKind => {
                    self.kind = b;
                    self.st = St::ReadId;
                }
                St::ReadId => {
                    self.id = b;
                    self.st = St::ReadLen;
                }
                St::ReadLen => {
                    self.len = b;
                    self.payload.clear();
                    if self.len == 0 {
                        if let (Some(msg), Some(kind)) = (MsgType::from_u8(self.msg), MsgKind::from_u8(self.kind)) {
                        frames.push(Frame {
                            msg,
                            kind,
                            id: self.id,
                            payload: vec![],
                        });
                        }
                        self.st = St::WaitAA;
                    } else {
                        self.st = St::ReadPayload;
                    }
                }
                St::ReadPayload => {
                    self.payload.push(b);
                    if self.payload.len() >= self.len as usize {
                        if let (Some(msg), Some(kind)) = (MsgType::from_u8(self.msg), MsgKind::from_u8(self.kind)) {
                        frames.push(Frame {
                            msg,
                            kind,
                            id: self.id,
                            payload: std::mem::take(&mut self.payload),
                        });}
                        self.st = St::WaitAA;
                    }
                }
            }
        }

        frames
    }
}
