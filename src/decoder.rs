use crate::command::{CRC16, FRAME_END, FRAME_START_SHORT};
use crate::{DecodeError, MAX_FRAME_LEN};

/// Bytes a frame adds around its payload: start byte, length, CRC16, end byte.
const FRAME_OVERHEAD: usize = 1 + 1 + 2 + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum State {
    /// Waiting for a start byte.
    Idle,
    /// Start byte seen, waiting for the payload length.
    Length,
    /// Accumulating the payload; `remaining` bytes are still expected.
    Payload { remaining: usize },
    /// Waiting for the high byte of the checksum.
    CrcHigh,
    /// Waiting for the low byte of the checksum.
    CrcLow { high: u8 },
    /// Checksum matched, waiting for the end byte.
    End,
}

/// Outcome of processing a single byte.
enum Step {
    /// The frame is not complete yet.
    Pending,
    /// A frame was completed; its payload is `buf[start + 2..pos - 3]`.
    Frame,
    /// The current candidate frame was rejected and its bytes queued for
    /// rescanning.
    Rejected(DecodeError),
}

/// A streaming frame decoder for the VESC communication protocol.
///
/// The decoder extracts frames from a byte stream incrementally: every input
/// byte is processed once, so feeding data in small chunks costs no more than
/// feeding it at once. It validates only the framing (start byte, length,
/// checksum, end byte) and hands out raw payloads, which can be parsed with
/// [`CommandReply::from_payload`] or by custom code for commands this crate
/// doesn't know about.
///
/// When a candidate frame turns out to be invalid, the decoder rescans the
/// bytes it buffered for that frame, starting right after the rejected start
/// byte, so a real frame hidden behind a false start byte is not lost. Note
/// that a false start byte followed by a large length makes the decoder wait
/// for that many bytes before it can reject the frame; call [`reset`] (for
/// example on a receive timeout) to drop such a candidate early.
///
/// The `BUFLEN` parameter sets the internal buffer size, which bounds the
/// largest frame that can be decoded. Longer frames are rejected as
/// [`DecodeError::InvalidFrame`].
///
/// [`CommandReply::from_payload`]: crate::CommandReply::from_payload
/// [`reset`]: Self::reset
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Decoder<const BUFLEN: usize = MAX_FRAME_LEN> {
    state: State,
    buf: [u8; BUFLEN],
    /// Index of the current candidate frame's start byte.
    start: usize,
    /// Index of the next byte to process. `buf[start..pos]` holds the
    /// processed bytes of the current candidate frame.
    pos: usize,
    /// End of buffered data. `buf[pos..end]` holds bytes queued for
    /// rescanning after a rejected frame.
    end: usize,
}

impl Default for Decoder<MAX_FRAME_LEN> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const BUFLEN: usize> Decoder<BUFLEN> {
    /// Creates a new decoder with an empty internal buffer.
    pub fn new() -> Self {
        const { assert!(BUFLEN >= FRAME_OVERHEAD, "decoder buffer too small for a frame") };
        Self {
            state: State::Idle,
            buf: [0; BUFLEN],
            start: 0,
            pos: 0,
            end: 0,
        }
    }

    /// Discards any partially received frame and buffered data.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.start = 0;
        self.pos = 0;
        self.end = 0;
    }

    /// Feeds data into the decoder until a frame is completed, a candidate
    /// frame is rejected, or the input is exhausted.
    ///
    /// Returns the number of bytes consumed from `data` together with:
    ///
    /// - `Some(Ok(payload))` when a valid frame was decoded;
    /// - `Some(Err(_))` when a candidate frame was rejected because of a
    ///   checksum mismatch or a malformed frame. The decoder recovers by
    ///   itself; the error is informational;
    /// - `None` when all of `data` was consumed and no further frame can be
    ///   decoded from buffered data.
    ///
    /// Call `feed` again with the unconsumed rest of the input until it
    /// returns `None`, even if the rest is empty: frames can be decoded from
    /// data buffered for rescanning without any new input.
    ///
    /// ```
    /// use vesc::{CommandReply, Decoder};
    ///
    /// let mut decoder = Decoder::default();
    /// let mut data: &[u8] = &[2, 7, 50, 0, 0, 1, 128, 0, 0, 4, 210, 1, 176, 254, 22, 3];
    /// loop {
    ///     let (consumed, frame) = decoder.feed(data);
    ///     data = &data[consumed..];
    ///     match frame {
    ///         Some(Ok(payload)) => {
    ///             let reply = CommandReply::from_payload(payload);
    ///             assert!(matches!(reply, Ok(CommandReply::GetValuesSelective(_))));
    ///         }
    ///         Some(Err(_)) => (), // a corrupted frame was skipped
    ///         None => break,
    ///     }
    /// }
    /// ```
    pub fn feed(&mut self, data: &[u8]) -> (usize, Option<Result<&[u8], DecodeError>>) {
        let mut consumed = 0;
        loop {
            if self.pos == self.end {
                let Some(&byte) = data.get(consumed) else {
                    return (consumed, None);
                };
                consumed += 1;
                self.push(byte);
            }
            match self.step() {
                Step::Pending => (),
                Step::Frame => {
                    let payload = &self.buf[self.start + 2..self.pos - 3];
                    return (consumed, Some(Ok(payload)));
                }
                Step::Rejected(err) => return (consumed, Some(Err(err))),
            }
        }
    }

    /// Appends a new input byte. Must only be called when no bytes are
    /// pending (`pos == end`).
    fn push(&mut self, byte: u8) {
        if self.state == State::Idle {
            self.start = 0;
            self.pos = 0;
            self.end = 0;
        } else if self.end == BUFLEN {
            // The candidate frame started in the middle of the buffer while
            // rescanning; move it to the front to make room.
            self.buf.copy_within(self.start..self.end, 0);
            self.pos -= self.start;
            self.end -= self.start;
            self.start = 0;
        }
        self.buf[self.end] = byte;
        self.end += 1;
    }

    /// Processes the byte at `pos`.
    fn step(&mut self) -> Step {
        let byte = self.buf[self.pos];
        self.pos += 1;

        match self.state {
            State::Idle => {
                if byte == FRAME_START_SHORT {
                    self.start = self.pos - 1;
                    self.state = State::Length;
                } else {
                    self.start = self.pos;
                }
            }
            State::Length => {
                let len = byte as usize;
                if len + FRAME_OVERHEAD > BUFLEN {
                    return self.reject(DecodeError::InvalidFrame);
                }
                self.state = match len {
                    0 => State::CrcHigh,
                    remaining => State::Payload { remaining },
                };
            }
            State::Payload { remaining } => {
                self.state = match remaining - 1 {
                    0 => State::CrcHigh,
                    remaining => State::Payload { remaining },
                };
            }
            State::CrcHigh => {
                self.state = State::CrcLow { high: byte };
            }
            State::CrcLow { high } => {
                let expected = u16::from_be_bytes([high, byte]);
                let actual = CRC16.checksum(&self.buf[self.start + 2..self.pos - 2]);
                if actual != expected {
                    return self.reject(DecodeError::ChecksumMismatch { expected, actual });
                }
                self.state = State::End;
            }
            State::End => {
                if byte != FRAME_END {
                    return self.reject(DecodeError::InvalidFrame);
                }
                self.state = State::Idle;
                return Step::Frame;
            }
        }
        Step::Pending
    }

    /// Rejects the current candidate frame and queues every byte after its
    /// start byte for rescanning.
    fn reject(&mut self, err: DecodeError) -> Step {
        self.buf.copy_within(self.start + 1..self.end, 0);
        self.end -= self.start + 1;
        self.start = 0;
        self.pos = 0;
        self.state = State::Idle;
        Step::Rejected(err)
    }
}
