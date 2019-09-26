use core::ops::Range;
use crate::{Receiver, ReceiverState};
#[cfg(feature="protocol-dev")]
use crate::ReceiverDebug;


#[derive(Debug)]
pub struct Rc5Command {
    pub addr: u8,
    pub cmd: u8,
    pub start: u8,
    pub toggle: u8,
}

impl Rc5Command {
    pub const fn new(data: u16) -> Self {

        //                   SS_TAAA_AACC_CCCC
        let addr_mask = 0b_0000_0111_1100_0000;
        let cmd_mask  = 0b_0000_0000_0011_1111;

        let addr = ((data & addr_mask) >> 6) as u8;
        let cmd = (data & cmd_mask) as u8;
        let start = (data >> 12) as u8;
        let toggle = ((data >> 11) & 1) as u8;

        Self {addr, cmd, start, toggle}
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Rc5Error {
    Header(u16),
    Data(u16),
}

pub struct Rc5Receiver {
    samplerate: u32,
    state: Rc5State,
    pinval: bool,
    bitbuf: u16,
    last: u32,
    pub rc5cntr: u32,

    #[cfg(feature="protocol-dev")]
    pub debug: ReceiverDebug<Rc5State, Option<u32>>,
}

impl Rc5Receiver {
    pub fn new(samplerate: u32) -> Self {
        Self {
            samplerate,
            last: 0,
            state: Rc5State::Idle,
            pinval: false,
            bitbuf: 0,
            rc5cntr: 0,
            #[cfg(feature="protocol-dev")]
            debug: ReceiverDebug {
                state: Rc5State::Idle,
                state_new: Rc5State::Idle,
                delta: 0,
                extra: None,
            }
        }
    }

    pub fn interval_to_units(&self, interval: u16) -> Option<u32> {
        for i in 1..=2 {
            if rc5_multiplier(self.samplerate, i).contains(&(u32::from(interval))) {
                return Some(i);
            }
        }
        None
    }

    // Time since last edge
    fn delta(&self, sampletime: u32) -> u16 {
        let delta = sampletime.wrapping_sub(self.last);
        if delta >= core::u16::MAX.into() {
            return 0;
        }
        delta as u16
    }

    fn receiver_state(&self) -> Rc5Res {
        use ReceiverState::*;
        match self.state {
            Rc5State::Idle => Idle,
            Rc5State::Done => Done(Rc5Command::new(self.bitbuf)),
            Rc5State::Error(err) => Error(err),
            _ => Receiving
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Rc5State {
    Idle,
    Data(u8),
    Done,
    Error(Rc5Error),
    Disabled,
}

const RISING: bool = true;
const FALLING: bool = false;

type Rc5Res = ReceiverState<Rc5Command, Rc5Error>;

impl Receiver for Rc5Receiver {
    type Cmd = Rc5Command;
    type Err = Rc5Error;

    fn sample(&mut self, pinval: bool, sampletime: u32) -> Rc5Res {

        let edge = self.pinval != pinval;
        if edge {
            return self.sample_edge(pinval, sampletime);
        }

        self.receiver_state()
    }

    fn sample_edge(&mut self, rising: bool, sampletime: u32) -> Rc5Res {
        use Rc5State::*;

        let delta = self.delta(sampletime);
        self.last = sampletime;
        self.pinval = rising;

        // Number of rc5 units since last pin edge
        let rc5units = self.interval_to_units(delta);

        if let Some(units) = rc5units {
            self.rc5cntr += units;
        }

        let odd = self.rc5cntr & 1 == 0;

        let newstate = match (self.state, rising, rc5units) {
            (Idle,          FALLING, _)              => Idle,
            (Idle,          RISING,  _)              => {self.bitbuf |= 1 << 13; Data(12)},

            (Data(0),       RISING,  Some(_)) if odd => {self.bitbuf |= 1; Done},
            (Data(0),       FALLING, Some(_)) if odd => Done,

            (Data(bit),     RISING,  Some(_)) if odd => {self.bitbuf |= 1 << bit; Data(bit-1)},
            (Data(bit),     FALLING, Some(_)) if odd => Data(bit-1),

            (Data(bit),     _,      Some(_))         => Data(bit),
            (Data(_),       _,      None)            => Error(Rc5Error::Data(delta)),
            (Done,          _,      _)               => Done,
            (Error(err),    _,      _)               => Error(err),
            (Disabled,      _,      _)               => Disabled,
        };

        #[cfg(feature="protocol-dev")]
        {
            self.debug.state = self.state;
            self.debug.state_new = newstate;
            self.debug.delta = delta;
            self.debug.extra = rc5units;
        }

        self.state = newstate;
        self.receiver_state()
    }

    fn reset(&mut self) {
        self.state = Rc5State::Idle;
        self.pinval = false;
        self.bitbuf = 0;
        self.rc5cntr = 0;
    }

    fn disable(&mut self) {
        self.state = Rc5State::Disabled;
    }
}

const fn rc5_multiplier(samplerate: u32, multiplier: u32) -> Range<u32> {
    let base = (samplerate * 889 * multiplier) / 1_000_000;
    range(base, 10)
}

const fn range(len: u32, percent: u32) -> Range<u32> {
    let tol = (len * percent) / 100;

     Range {
        start: len - tol - 2,
        end: len + tol + 2,
    }
}
#[cfg(test)]
mod tests {
    use crate::rc5::Rc5Receiver;
    use crate::prelude::*;

    #[test]
    fn command() {

        let dists = [0, 37, 34, 72, 72, 73, 70, 72, 36, 37, 34, 36, 36, 36, 71, 73, 35, 37, 70, 37];

        let mut recv = Rc5Receiver::new(40_000);
        let mut edge = false;
        let mut tot = 0;
        let mut state = ReceiverState::Idle;

        for dist in dists.iter() {
            edge = !edge;
            tot += dist;
            state = recv.sample_edge(edge, tot);
        }

        if let ReceiverState::Done(cmd) = state {
            assert_eq!(cmd.addr, 20);
            assert_eq!(cmd.cmd, 9);
        } else {
            assert!(false);
        }
    }
}
