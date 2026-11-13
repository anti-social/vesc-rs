//! The VESC® firmware is an open source motor controller firmware, and this
//! library implements the necessary structures and functions to [`encode`]
//! commands and decode replies with a [`Decoder`].
//!
//! # Examples
//!
//! ## Encoding a Command
//!
//! ```no_run
//! use vesc::{Command, ValuesMask};
//!
//! let mut buf = [0u8; 64];
//! let command = Command::GetValuesSelective(ValuesMask::RPM | ValuesMask::VOLTAGE_IN);
//! let frame_len = vesc::encode(command, &mut buf).unwrap();
//! let frame = &buf[..frame_len];
//! ```
//!
//! ## Decoding a Stream
//!
//! ```no_run
//! use vesc::{Decoder, CommandReply};
//!
//! let mut decoder = Decoder::default();
//! let mut data: &[u8] = &[2, 7, 50, 0, 0, 1, 128, 0, 0, 4, 210, 1, 176, 254, 22, 3];
//!
//! loop {
//!     let (consumed, frame) = decoder.feed(data);
//!     data = &data[consumed..];
//!     match frame {
//!         Some(Ok(payload)) => match CommandReply::from_payload(payload) {
//!             Ok(CommandReply::GetValuesSelective(values)) => {
//!                 let rpm = values.rpm;
//!                 let voltage_in = values.voltage_in;
//!             }
//!             _ => (),
//!         },
//!         Some(Err(_)) => (), // a corrupted frame was skipped
//!         None => break,
//!     }
//! }
//! ```
#![cfg_attr(not(feature = "std"), no_std)]

mod command;
mod decoder;
mod packer;

pub use command::{
    //
    AmpHours,
    AvgCurrent,
    AvgVoltage,
    BatteryLevel,
    Command,
    CommandReply,
    Current,
    CurrentRel,
    DecodeError,
    Distance,
    Duty,
    DutyCycle,
    EncodeError,
    FaultCode,
    FwInfo,
    FwVersion,
    HwType,
    MAX_FRAME_LEN,
    NrfFlags,
    Position,
    QmlAppFlags,
    QmlHw,
    SetupValues,
    Speed,
    Stats,
    StatsMask,
    Temperature,
    Values,
    ValuesMask,
    ValuesSetupMask,
    Voltage,
    WattHours,
    WattHoursLeft,
    encode,
};
pub use decoder::Decoder;
