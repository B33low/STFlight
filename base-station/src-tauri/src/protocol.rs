// src-tauri/src/protocol.rs
pub const AA: u8 = 0xAA;
pub const BB: u8 = 0x55;

#[derive(Debug, Clone)]
pub struct Frame {
  pub msg: u8,
  pub kind: u8,
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
            frames.push(Frame { msg: self.msg, kind: self.kind, id: self.id, payload: vec![] });
            self.st = St::WaitAA;
          } else {
            self.st = St::ReadPayload;
          }
        }
        St::ReadPayload => {
          self.payload.push(b);
          if self.payload.len() >= self.len as usize {
            frames.push(Frame {
              msg: self.msg,
              kind: self.kind,
              id: self.id,
              payload: std::mem::take(&mut self.payload),
            });
            self.st = St::WaitAA;
          }
        }
      }
    }

    frames
  }
}
