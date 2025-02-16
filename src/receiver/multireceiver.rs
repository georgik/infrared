#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::InputPin;

use crate::receiver::{time::InfraMonotonic, DecoderFactory, NoPin, Receiver};

/// Multi Receiver
pub struct MultiReceiver<
    const N: usize,
    Receivers: ReceiverWrapper<N, Time>,
    Input,
    Time: InfraMonotonic = u32,
> {
    receivers: Receivers::Receivers,
    input: Input,
}

impl<const N: usize, Receivers: ReceiverWrapper<N, Mono>, Input, Mono: InfraMonotonic>
    MultiReceiver<N, Receivers, Input, Mono>
{
    pub fn new(res: u32, input: Input) -> Self {
        MultiReceiver {
            input,
            receivers: Receivers::make(res),
        }
    }

    pub fn event_generic(
        &mut self,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; N] {
        Receivers::event(&mut self.receivers, dt, edge)
    }

    pub fn event_generic_iter(
        &mut self,
        dt: Mono::Duration,
        flank: bool,
    ) -> impl Iterator<Item = MultiReceiverCommand> {
        let arr = self.event_generic(dt, flank);
        arr.into_iter().flatten()
    }
}

#[cfg(feature = "embedded-hal")]
impl<const N: usize, Receivers, Pin: InputPin, Mono: InfraMonotonic>
    MultiReceiver<N, Receivers, Pin, Mono>
where
    Receivers: ReceiverWrapper<N, Mono>,
{
    pub fn event(
        &mut self,
        dt: Mono::Duration,
    ) -> Result<[Option<MultiReceiverCommand>; N], Pin::Error> {
        let edge = self.input.is_low()?;
        Ok(self.event_generic(dt, edge))
    }

    pub fn event_iter(
        &mut self,
        dt: Mono::Duration,
    ) -> Result<impl Iterator<Item = MultiReceiverCommand>, Pin::Error> {
        let arr = self.event(dt)?;
        Ok(arr.into_iter().flatten())
    }

    pub fn pin(&mut self) -> &mut Pin {
        &mut self.input
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// MultiReceiver Command
pub enum MultiReceiverCommand {
    #[cfg(feature = "nec")]
    Nec(crate::protocol::nec::NecCommand),
    #[cfg(feature = "nec")]
    Nec16(crate::protocol::nec::Nec16Command),
    #[cfg(feature = "nec")]
    NecSamsung(crate::protocol::nec::SamsungNecCommand),
    #[cfg(feature = "nec")]
    NecApple(crate::protocol::nec::AppleNecCommand),
    #[cfg(feature = "nec")]
    NecDebug(crate::protocol::nec::NecDebugCmd),
    #[cfg(feature = "rc5")]
    Rc5(crate::protocol::rc5::Rc5Command),
    #[cfg(feature = "rc6")]
    Rc6(crate::protocol::rc6::Rc6Command),
    #[cfg(feature = "denon")]
    Denon(crate::protocol::denon::DenonCommand),
}

#[cfg(feature = "nec")]
impl From<crate::protocol::nec::NecCommand> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::nec::NecCommand) -> MultiReceiverCommand {
        MultiReceiverCommand::Nec(cmd)
    }
}
#[cfg(feature = "nec")]
impl From<crate::protocol::nec::Nec16Command> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::nec::Nec16Command) -> MultiReceiverCommand {
        MultiReceiverCommand::Nec16(cmd)
    }
}
#[cfg(feature = "nec")]
impl From<crate::protocol::nec::SamsungNecCommand> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::nec::SamsungNecCommand) -> MultiReceiverCommand {
        MultiReceiverCommand::NecSamsung(cmd)
    }
}
#[cfg(feature = "nec")]
impl From<crate::protocol::nec::AppleNecCommand> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::nec::AppleNecCommand) -> MultiReceiverCommand {
        MultiReceiverCommand::NecApple(cmd)
    }
}
#[cfg(feature = "nec")]
impl From<crate::protocol::nec::NecDebugCmd> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::nec::NecDebugCmd) -> MultiReceiverCommand {
        MultiReceiverCommand::NecDebug(cmd)
    }
}
#[cfg(feature = "rc5")]
impl From<crate::protocol::rc5::Rc5Command> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::rc5::Rc5Command) -> MultiReceiverCommand {
        MultiReceiverCommand::Rc5(cmd)
    }
}
#[cfg(feature = "rc6")]
impl From<crate::protocol::rc6::Rc6Command> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::rc6::Rc6Command) -> MultiReceiverCommand {
        MultiReceiverCommand::Rc6(cmd)
    }
}
#[cfg(feature = "denon")]
impl From<crate::protocol::denon::DenonCommand> for MultiReceiverCommand {
    fn from(cmd: crate::protocol::denon::DenonCommand) -> MultiReceiverCommand {
        MultiReceiverCommand::Denon(cmd)
    }
}

pub trait ReceiverWrapper<const N: usize, Mono: InfraMonotonic> {
    type Receivers;

    fn make(res: u32) -> Self::Receivers;

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        flank: bool,
    ) -> [Option<MultiReceiverCommand>; N];
}

impl<P1, P2, Mono: InfraMonotonic> ReceiverWrapper<2, Mono> for (P1, P2)
where
    P1: DecoderFactory<Mono>,
    P2: DecoderFactory<Mono>,
    P1::Cmd: Into<MultiReceiverCommand>,
    P2::Cmd: Into<MultiReceiverCommand>,
{
    type Receivers = (Receiver<P1, NoPin, Mono>, Receiver<P2, NoPin, Mono>);

    fn make(res: u32) -> Self::Receivers {
        (Receiver::new(res), Receiver::new(res))
    }

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; 2] {
        [
            rs.0.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.1.event(dt, edge).unwrap_or_default().map(Into::into),
        ]
    }
}

impl<P1, P2, P3, Mono: InfraMonotonic> ReceiverWrapper<3, Mono> for (P1, P2, P3)
where
    P1: DecoderFactory<Mono>,
    P2: DecoderFactory<Mono>,
    P3: DecoderFactory<Mono>,
    P1::Cmd: Into<MultiReceiverCommand>,
    P2::Cmd: Into<MultiReceiverCommand>,
    P3::Cmd: Into<MultiReceiverCommand>,
{
    type Receivers = (
        Receiver<P1, NoPin, Mono>,
        Receiver<P2, NoPin, Mono>,
        Receiver<P3, NoPin, Mono>,
    );

    fn make(res: u32) -> Self::Receivers {
        (Receiver::new(res), Receiver::new(res), Receiver::new(res))
    }

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; 3] {
        [
            rs.0.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.1.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.2.event(dt, edge).unwrap_or_default().map(Into::into),
        ]
    }
}

impl<P1, P2, P3, P4, Mono: InfraMonotonic> ReceiverWrapper<4, Mono> for (P1, P2, P3, P4)
where
    P1: DecoderFactory<Mono>,
    P2: DecoderFactory<Mono>,
    P3: DecoderFactory<Mono>,
    P4: DecoderFactory<Mono>,
    P1::Cmd: Into<MultiReceiverCommand>,
    P2::Cmd: Into<MultiReceiverCommand>,
    P3::Cmd: Into<MultiReceiverCommand>,
    P4::Cmd: Into<MultiReceiverCommand>,
{
    type Receivers = (
        Receiver<P1, NoPin, Mono>,
        Receiver<P2, NoPin, Mono>,
        Receiver<P3, NoPin, Mono>,
        Receiver<P4, NoPin, Mono>,
    );

    fn make(res: u32) -> Self::Receivers {
        (
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
        )
    }

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; 4] {
        [
            rs.0.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.1.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.2.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.3.event(dt, edge).unwrap_or_default().map(Into::into),
        ]
    }
}

impl<P1, P2, P3, P4, P5, Mono: InfraMonotonic> ReceiverWrapper<5, Mono> for (P1, P2, P3, P4, P5)
where
    P1: DecoderFactory<Mono>,
    P2: DecoderFactory<Mono>,
    P3: DecoderFactory<Mono>,
    P4: DecoderFactory<Mono>,
    P5: DecoderFactory<Mono>,
    P1::Cmd: Into<MultiReceiverCommand>,
    P2::Cmd: Into<MultiReceiverCommand>,
    P3::Cmd: Into<MultiReceiverCommand>,
    P4::Cmd: Into<MultiReceiverCommand>,
    P5::Cmd: Into<MultiReceiverCommand>,
{
    type Receivers = (
        Receiver<P1, NoPin, Mono>,
        Receiver<P2, NoPin, Mono>,
        Receiver<P3, NoPin, Mono>,
        Receiver<P4, NoPin, Mono>,
        Receiver<P5, NoPin, Mono>,
    );

    fn make(res: u32) -> Self::Receivers {
        (
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
        )
    }

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; 5] {
        [
            rs.0.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.1.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.2.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.3.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.4.event(dt, edge).unwrap_or_default().map(Into::into),
        ]
    }
}

impl<P1, P2, P3, P4, P5, P6, Mono: InfraMonotonic> ReceiverWrapper<6, Mono>
    for (P1, P2, P3, P4, P5, P6)
where
    P1: DecoderFactory<Mono>,
    P2: DecoderFactory<Mono>,
    P3: DecoderFactory<Mono>,
    P4: DecoderFactory<Mono>,
    P5: DecoderFactory<Mono>,
    P6: DecoderFactory<Mono>,

    P1::Cmd: Into<MultiReceiverCommand>,
    P2::Cmd: Into<MultiReceiverCommand>,
    P3::Cmd: Into<MultiReceiverCommand>,
    P4::Cmd: Into<MultiReceiverCommand>,
    P5::Cmd: Into<MultiReceiverCommand>,
    P6::Cmd: Into<MultiReceiverCommand>,
{
    type Receivers = (
        Receiver<P1, NoPin, Mono>,
        Receiver<P2, NoPin, Mono>,
        Receiver<P3, NoPin, Mono>,
        Receiver<P4, NoPin, Mono>,
        Receiver<P5, NoPin, Mono>,
        Receiver<P6, NoPin, Mono>,
    );

    fn make(res: u32) -> Self::Receivers {
        (
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
            Receiver::new(res),
        )
    }

    fn event(
        rs: &mut Self::Receivers,
        dt: Mono::Duration,
        edge: bool,
    ) -> [Option<MultiReceiverCommand>; 6] {
        [
            rs.0.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.1.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.2.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.3.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.4.event(dt, edge).unwrap_or_default().map(Into::into),
            rs.5.event(dt, edge).unwrap_or_default().map(Into::into),
        ]
    }
}
