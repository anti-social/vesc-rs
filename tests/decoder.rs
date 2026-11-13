use googletest::prelude::*;
use vesc::{
    AmpHours, AvgCurrent, AvgVoltage, CommandReply, DecodeError, Decoder, DutyCycle, FaultCode,
    Position, Temperature, Values, Voltage, WattHours,
};

const GET_VALUES: [u8; 79] = [
    2, 74, 4, 1, 20, 0, 0, 0, 0, 0, 37, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 32, 0, 110, 0, 0, 3, 251,
    1, 125, 0, 0, 0, 17, 0, 0, 0, 0, 0, 0, 2, 137, 0, 0, 0, 0, 255, 255, 111, 75, 0, 2, 159, 199,
    0, 4, 106, 124, 40, 1, 1, 21, 252, 76, 252, 13, 0, 0, 0, 229, 0, 0, 8, 214, 0, 58, 151, 3,
];

const SELECTIVE_1: [u8; 28] = [
    2, 23, 50, 0, 2, 161, 138, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 128, 255, 255, 158, 70, 0, 1, 63,
    148, 3,
];

const SELECTIVE_20: [u8; 28] = [
    2, 23, 50, 0, 2, 161, 138, 0, 0, 0, 0, 0, 4, 0, 0, 3, 221, 1, 119, 255, 255, 170, 43, 0, 20,
    45, 58, 3,
];

/// Feeds `input` until the decoder reports `None`, parsing every decoded
/// frame. Framing and parsing errors are both collected.
fn decode_all(
    decoder: &mut Decoder,
    mut input: &[u8],
) -> Vec<core::result::Result<CommandReply, DecodeError>> {
    let mut replies = vec![];
    loop {
        let (consumed, frame) = decoder.feed(input);
        input = &input[consumed..];
        match frame {
            Some(frame) => replies.push(frame.and_then(CommandReply::from_payload)),
            None => break,
        }
    }
    assert!(input.is_empty());
    replies
}

fn decode_valid(decoder: &mut Decoder, input: &[u8]) -> Vec<CommandReply> {
    decode_all(decoder, input)
        .into_iter()
        .filter_map(|r| r.ok())
        .collect()
}

fn is_get_values<'a>() -> impl Matcher<&'a CommandReply> {
    pat!(&CommandReply::GetValues(pat!(Values {
        temp_mosfet: eq(Temperature::from_raw(276)),
        temp_motor: eq(Temperature::from_raw(0)),
        avg_current_motor: eq(AvgCurrent::from_raw(37)),
        avg_current_input: eq(AvgCurrent::from_raw(3)),
        avg_current_d: eq(AvgCurrent::from_raw(0)),
        avg_current_q: eq(AvgCurrent::from_raw(32)),
        duty_cycle: eq(DutyCycle::from_raw(110)),
        rpm: eq(1019),
        voltage_in: eq(Voltage::from_raw(381)),
        amp_hours: eq(AmpHours::from_raw(17)),
        amp_hours_charged: eq(AmpHours::from_raw(0)),
        watt_hours: eq(WattHours::from_raw(649)),
        watt_hours_charged: eq(WattHours::from_raw(0)),
        tachometer: eq(-37045),
        tachometer_abs: eq(171975),
        fault_code: eq(FaultCode::None),
        pid_pos: eq(Position::from_raw(74087464)),
        controller_id: eq(1),
        temp_mosfet1: eq(Temperature::from_raw(277)),
        temp_mosfet2: eq(Temperature::from_raw(-948)),
        temp_mosfet3: eq(Temperature::from_raw(-1011)),
        avg_voltage_d: eq(AvgVoltage::from_raw(229)),
        avg_voltage_q: eq(AvgVoltage::from_raw(2262)),
        status: eq(0),
    })))
}

fn is_selective_1<'a>() -> impl Matcher<&'a CommandReply> {
    pat!(&CommandReply::GetValuesSelective(pat!(Values {
        avg_current_input: eq(AvgCurrent::from_raw(0)),
        rpm: eq(0),
        voltage_in: eq(Voltage::from_raw(384)),
        tachometer: eq(-25018),
        fault_code: eq(FaultCode::None),
        controller_id: eq(1),
        ..
    })))
}

fn is_selective_20<'a>() -> impl Matcher<&'a CommandReply> {
    pat!(&CommandReply::GetValuesSelective(pat!(Values {
        avg_current_input: eq(AvgCurrent::from_raw(4)),
        rpm: eq(989),
        voltage_in: eq(Voltage::from_raw(375)),
        tachometer: eq(-21973),
        fault_code: eq(FaultCode::None),
        controller_id: eq(20),
        ..
    })))
}

#[test]
fn decoder_decodes_single_packet() {
    let mut decoder = Decoder::default();

    let (consumed, frame) = decoder.feed(&SELECTIVE_20);

    assert_that!(consumed, eq(28));
    let payload = frame.unwrap().unwrap();
    assert_that!(payload, eq(&SELECTIVE_20[2..25]));
    assert_that!(
        &CommandReply::from_payload(payload).unwrap(),
        is_selective_20()
    );
    assert_eq!(decoder.feed(&[]), (0, None));
}

#[test]
fn decoder_decodes_packet_fed_in_chunks() {
    let mut decoder = Decoder::default();

    let mut chunks_iter = GET_VALUES.chunks(5).peekable();
    while let Some(chunk) = chunks_iter.next() {
        let (consumed, frame) = decoder.feed(chunk);
        assert_that!(consumed, eq(chunk.len()));
        if chunks_iter.peek().is_some() {
            assert_that!(frame, none());
        } else {
            let reply = CommandReply::from_payload(frame.unwrap().unwrap()).unwrap();
            assert_that!(&reply, is_get_values());
        }
    }
}

#[test]
fn decoder_returns_none_until_packet_is_complete() {
    let mut decoder = Decoder::default();

    let (head, last) = GET_VALUES.split_at(GET_VALUES.len() - 1);
    for chunk in head.chunks(head.len() / 5) {
        assert_eq!(decoder.feed(chunk), (chunk.len(), None));
    }

    let (consumed, frame) = decoder.feed(last);
    assert_that!(consumed, eq(1));
    let reply = CommandReply::from_payload(frame.unwrap().unwrap()).unwrap();
    assert_that!(&reply, is_get_values());
}

#[test]
fn decoder_decodes_two_packets_from_single_feed() {
    let input = [SELECTIVE_1, SELECTIVE_20].concat();
    let mut decoder = Decoder::default();

    assert_that!(
        decode_valid(&mut decoder, &input),
        elements_are![is_selective_1(), is_selective_20()]
    );
}

#[test]
fn decoder_decodes_two_packets_from_separate_feeds() {
    let mut decoder = Decoder::default();

    assert_that!(
        decode_valid(&mut decoder, &SELECTIVE_1),
        elements_are![is_selective_1()]
    );
    assert_that!(
        decode_valid(&mut decoder, &SELECTIVE_20),
        elements_are![is_selective_20()]
    );
}

#[test]
fn decoder_ignores_incomplete_packet() {
    let mut decoder = Decoder::default();
    let input = &SELECTIVE_20[..SELECTIVE_20.len() - 1];

    assert_eq!(decoder.feed(input), (input.len(), None));
}

#[test]
fn decoder_rejects_packet_with_bad_checksum() {
    let input = [
        2, 23, 50, 0, 2, 161, 138, 0, 0, 0, 0, 0, 10, 255, 255, 246, 213, 1, 118, 255, 255, 181,
        218, 0, 21, 94, 130, 3,
    ];
    let mut decoder = Decoder::default();

    assert_that!(
        decode_all(&mut decoder, &input),
        elements_are![err(matches_pattern!(DecodeError::ChecksumMismatch { .. }))]
    );
}

#[test]
fn decoder_rejects_packet_with_bad_end_byte() {
    let mut input = SELECTIVE_20;
    *input.last_mut().unwrap() = 4;
    let mut decoder = Decoder::default();

    assert_that!(
        decode_all(&mut decoder, &input),
        elements_are![err(eq(&DecodeError::InvalidFrame))]
    );
}

#[test]
fn decoder_passes_unknown_packet_through() {
    let input = [2, 3, 222, 4, 0, 178, 81, 3];
    let mut decoder = Decoder::default();

    let (consumed, frame) = decoder.feed(&input);

    assert_that!(consumed, eq(8));
    let payload = frame.unwrap().unwrap();
    assert_that!(payload, eq(&[222, 4, 0]));
    assert_that!(
        CommandReply::from_payload(payload),
        err(eq(&DecodeError::UnknownPacket { id: 222 }))
    );
}

#[test]
fn decoder_skips_junk_bytes_between_packets() {
    let input = [&[10, 34, 12][..], &SELECTIVE_1, &[4, 178, 255], &SELECTIVE_20].concat();
    let mut decoder = Decoder::default();

    assert_that!(
        decode_valid(&mut decoder, &input),
        elements_are![is_selective_1(), is_selective_20()]
    );
}

#[test]
fn decoder_recovers_from_false_start_byte() {
    let input = [&[2, 5, 50, 0, 0, 0, 1][..], &SELECTIVE_20].concat();
    let mut decoder = Decoder::default();

    // The false frame swallows the real start byte as its checksum. Rescanning
    // after the mismatch finds the real frame.
    assert_that!(
        decode_all(&mut decoder, &input),
        elements_are![
            err(matches_pattern!(DecodeError::ChecksumMismatch { .. })),
            ok(is_selective_20()),
        ]
    );
}

#[test]
fn decoder_decodes_frame_buffered_behind_false_start_byte() {
    // The false frame's length covers the whole real frame, which is then
    // decoded from buffered data without any new input.
    let input = [&[2, 30][..], &SELECTIVE_20, &[0; 4]].concat();
    let mut decoder = Decoder::default();

    let (consumed, frame) = decoder.feed(&input);
    assert_that!(consumed, eq(input.len()));
    assert_that!(
        frame,
        some(err(matches_pattern!(DecodeError::ChecksumMismatch { .. })))
    );

    let (consumed, frame) = decoder.feed(&[]);
    assert_that!(consumed, eq(0));
    let reply = CommandReply::from_payload(frame.unwrap().unwrap()).unwrap();
    assert_that!(&reply, is_selective_20());
}

#[test]
fn decoder_waits_for_false_frame_length_before_recovering() {
    let bad_checksum = [
        2, 23, 50, 0, 2, 161, 138, 0, 0, 0, 0, 0, 10, 255, 255, 246, 213, 1, 118, 255, 255, 181,
        218, 0, 21, 94, 130, 3,
    ];
    let valid = [
        2, 23, 50, 0, 2, 161, 138, 0, 0, 0, 0, 0, 10, 255, 255, 246, 213, 1, 118, 255, 255, 181,
        218, 0, 20, 94, 130, 3,
    ];
    let input = [
        &[2, 45, 4][..], // junk: a false start covering the next frame's start
        &GET_VALUES,
        &bad_checksum,
        &valid,
    ]
    .concat();
    let mut decoder = Decoder::default();

    // Rescanning `bad_checksum` locks onto its `2, 161` bytes, a false frame
    // that is longer than the rest of the input, so `valid` stays buffered.
    assert_that!(
        decode_valid(&mut decoder, &input),
        elements_are![is_get_values()]
    );

    // Once enough bytes arrive to reject the false frame, `valid` is decoded
    // from the rescanned data.
    assert_that!(
        decode_valid(&mut decoder, &[0; 161]),
        elements_are![pat!(&CommandReply::GetValuesSelective(pat!(Values {
            avg_current_input: eq(AvgCurrent::from_raw(10)),
            rpm: eq(-2347),
            voltage_in: eq(Voltage::from_raw(374)),
            tachometer: eq(-18982),
            fault_code: eq(FaultCode::None),
            controller_id: eq(20),
            ..
        })))]
    );
}

#[test]
fn decoder_reset_drops_pending_false_frame() {
    let mut decoder = Decoder::default();

    // A false start with a long length would otherwise hold back the frame.
    assert_eq!(decoder.feed(&[2, 200, 1, 2, 3]), (5, None));
    decoder.reset();

    assert_that!(
        decode_valid(&mut decoder, &SELECTIVE_20),
        elements_are![is_selective_20()]
    );
}

#[test]
fn decoder_rejects_frame_longer_than_buffer() {
    let mut decoder = Decoder::<16>::new();

    let (consumed, frame) = decoder.feed(&SELECTIVE_20);

    assert_that!(consumed, eq(2));
    assert_that!(frame, some(err(eq(&DecodeError::InvalidFrame))));
}
