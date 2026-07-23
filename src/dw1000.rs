/*!
 * A Decawave DW1000 driver implementation.
 */
use embedded_hal::{
    digital::{InputPin, OutputPin},
    spi::{Operation, SpiDevice},
};
use embedded_hal_async::{delay::DelayNs, digital::Wait};
use thiserror::Error;

use crate::registers::{Register, RegisterType, ReservedField, dw1000::*};

/// This is the size of the preamble detection sliding-window register.
/// In general, the largest value will get the best results, but the preamble size
/// should be ~4x bigger at least or detection may fail entirely.
pub enum PacSize {
    /// 8-symbol window.
    Pac8,
    /// 16-symbol window.
    Pac16,
    /// 32-symbol window
    Pac32,
    /// 64-symbol window
    Pac64,
}

/// The DW1000's communications protocol. Two dw1000s with the same protocol
/// are guaranteed able to communicate.
///
/// Some profiles are intercompatible (e.g. 6.8mbps and 850kbps are mostly interchangeable - the receiver automatically detects which is in use)
/// but for best performance you should always use identical profiles on receive and transmit side.
pub struct Profile {
    /// The channel. DW1000 supports many channels.
    pub channel: Channel,
    /// The length of the preamble.
    pub preamble: Preamble,
    /// The pulse-repetition frequency - this is effectively
    /// the subcarrier frequency afaict.
    pub prf: Prf,
    /// The data rate in use. Note that 110kbps gets much better range and ranging accuracy,
    /// but is painstakingly slow and will not work with smaller preambles (a minimum of 1024, experimentally)
    pub drate: Bitrate,
    /// The preamble code to use. Generally you don't need to worry about this -
    /// 9 works as a reasonable default; other codes (and their channels/prfs) can be
    /// found on page 202 of the user manual.
    pub preamble_code: u8,
    /// Size of the PAC sliding window.
    pub pac_size: PacSize,
    /// TX power byte. The upper three bits control the coarse gain, and smaller values
    /// for that subfield result in a higher power; the lower five bits control the
    /// fine power, and larger values for that subfield result in a higher power.
    /// The highest possible power is 0x1F; the lowest is 0xE0.
    pub power: u8,
}

impl Default for Profile {
    fn default() -> Self {
        Self {
            channel: Channel::Chan5,
            preamble: Preamble::Preamble64,
            prf: Prf::Mhz64,
            drate: Bitrate::Mbps6_8,
            preamble_code: 9,
            pac_size: PacSize::Pac8,
            power: 0x75,
        }
    }
}

impl SysStatus {
    /// Bitmask for RX events.
    pub fn rx_mask() -> Self {
        Self {
            rxdfr: true,
            ldeerr: true,
            rxphe: true,
            rxfce: true,
            rxrfsl: true,
            rxpto: true,
            ..Default::default()
        }
    }

    /// Bitmask for TX events.
    pub fn tx_mask() -> Self {
        Self {
            txberr: true,
            txfrb: true,
            txfrs: true,
            txphs: true,
            txprs: true,
            txpute: true,
            ..Default::default()
        }
    }
}

impl<const N: usize> core::ops::BitOr for ReservedField<N> {
    type Output = ReservedField<N>;

    fn bitor(self, _rhs: Self) -> Self::Output {
        ReservedField
    }
}

macro_rules! bitor_bool_impl_self {
    ($self:ident, $rhs:ident, $($field_name:ident)*) => {
        Self {
            $(
                $field_name: $self.$field_name | $rhs.$field_name,
            )*
        }
    };
}

impl core::ops::BitOr for SysStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        bitor_bool_impl_self!(self, rhs,
            irqs
            cplock
            esyncr
            aat
            txfrb
            txprs
            txphs
            txfrs
            rxprd
            rxsfdd
            ldedone
            _res1
            rxovrr
            rxpto
            gpioirq
            slp2init
            rfpll_ll
            clkpll_ll
            rxsfd_to
            hpdwarn
            txberr
            affrej
            hsrbp
            icrbp
            rxrscs
            rxprej
            txpute
            rxphd
            rxphe
            rxdfr
            rxfcg
            rxfce
            rxrfsl
            rxrfto
            ldeerr
        )
    }
}

#[derive(Debug, Error)]
/// DW1000 errors.
pub enum Error<SpiError: embedded_hal::spi::Error, GpioError: embedded_hal::digital::Error> {
    #[error("invalid device id 0x{0:08X}, expected 0xDECA0130")]
    /// The DEV_ID register contained an invalid value.
    BadDeviceId(u32),
    #[error("spi driver error")]
    /// The SPI driver reported an error.
    SpiError(#[from] SpiError),
    #[error("operation timed out")]
    /// Something timed out (generic)
    Timeout,
    #[error("invalid PHY header")]
    /// The received PHY header was invalid.
    PhyHeaderError,
    #[error("frame CRC check failed")]
    /// A frame failed CRC checking.
    FcsError,
    #[error("lost reed-solomon sync")]
    /// The Reed-Solomon error correction subsystem lost sync.
    ReedSolomonSyncLoss,
    #[error("issue with Leading Edge Detection (LDE)")]
    /// The leading-edge detector failed.
    LdeErr,
    #[error("preamble timeout")]
    /// Timed out waiting for a preamble. You have to explicitly opt-in
    /// to this error.
    PreambleTimeout,
    #[error("timed out waiting for SFD sequence")]
    /// The preamble was detected but the SFD sequence was not.
    ///
    /// This may indicate your SFD timeout value is too low - see register DRX_SFDTOC (page 134-135 of user manual)
    SfdTimeout,
    #[error("frame filtering rejection")]
    /// The frame filtering code rejected a frame.
    FrameFilterReject,
    #[error("received frame length {0}, expected >=2")]
    /// The received frame was < 2 bytes. This indicates some sort of corruption
    /// as the received length contains the 2 byte CRC - either the transmitter
    /// didn't generate a CRC, or the frame was garbled.
    RxLenTooSmall(usize),
    #[error("error with gpio subsystem")]
    /// The GPIO subsystem did something weird.
    GpioError(GpioError),
    #[error("An unknown error occurred")]
    /// The DW1000 did something weird we can't identify
    Unknown,
}

impl<SpiError: embedded_hal::spi::Error, GpioError: embedded_hal::digital::Error>
    Error<SpiError, GpioError>
{
    fn check_rx_sysstatus(stat: SysStatus) -> Result<(), Self> {
        if stat.rxphe {
            return Err(Error::PhyHeaderError);
        }
        if stat.rxfce {
            return Err(Error::FcsError);
        }
        if stat.rxrfsl {
            return Err(Error::ReedSolomonSyncLoss);
        }
        if stat.rxrfto {
            return Err(Error::Timeout);
        }
        if stat.ldeerr {
            return Err(Error::LdeErr);
        }
        if stat.rxpto {
            return Err(Error::PreambleTimeout);
        }
        if stat.rxsfd_to {
            return Err(Error::SfdTimeout);
        }
        if stat.affrej {
            return Err(Error::FrameFilterReject);
        }
        Ok(())
    }
}

/// A receive stream in RXAUTR=1 mode (e.g. receiver automatically re-enables itself)
/// it is not possible to transmit while in receive streaming mode.
pub struct RxStream<
    'a,
    Device: SpiDevice,
    NrstPin: OutputPin,
    IrqPin: Wait + InputPin,
    Delays: DelayNs,
> {
    chip: &'a mut Dw1000<Device, NrstPin, IrqPin, Delays>,
}

impl<'a, Device: SpiDevice, NrstPin: OutputPin, IrqPin: Wait + InputPin, Delays: DelayNs> Drop
    for RxStream<'a, Device, NrstPin, IrqPin, Delays>
{
    fn drop(&mut self) {
        self.chip.reset_syscfg().unwrap();
        self.chip.idle().unwrap();
    }
}

impl<'a, Device: SpiDevice, NrstPin: OutputPin, IrqPin: Wait + InputPin, Delays: DelayNs>
    RxStream<'a, Device, NrstPin, IrqPin, Delays>
{
    /// The error type.
    pub type Error = Error<Device::Error, NrstPin::Error>;

    /// Grab the next frame from the double-buffer, waiting if the double-buffer is empty.
    /// This does not support breaking up the access into two parts (status and data) -
    /// they must be done in quick succession or the buffer will overrun and cause data corruption.
    pub async fn next(&mut self, buf: &mut [u8]) -> Result<(usize, u64), Self::Error> {
        self.chip.interrupt_wait(|_| true).await?;
        let stat = self.chip.read_reg(SYS_STATUS)?;
        Error::check_rx_sysstatus(stat)?;
        if stat.rxdfr {
            self.chip.write_reg(SYS_STATUS, SysStatus::rx_mask())?;
            let finfo = self.chip.read_reg(RX_FINFO)?;
            if finfo.rxflen.0 < 2 {
                return Err(Error::RxLenTooSmall(finfo.rxflen.0 as usize));
            }
            let len = (finfo.rxflen.0 as usize - 2).min(buf.len());
            self.chip.read_rx_buffer(&mut buf[0..len])?;
            let time = self.chip.get_rx_time()?;
            self.chip.hrbpt()?;
            Ok((len, time))
        } else {
            Err(Error::Unknown)
        }
    }
}

/// A DecaWave DW1000 generic over the spi, gpio, and timing interfaces.
///
/// Note that this does not use asynchronous spi: all spi operations are blocking.
/// This is because DW1000 requires fairly fast spi turnarounds and async is slow.
pub struct Dw1000<Device: SpiDevice, NrstPin: OutputPin, IrqPin: Wait, Delays: DelayNs> {
    spi: Device,
    nrst: NrstPin,
    irq: IrqPin,
    profile: Profile,
    delays: Delays,
}

impl<Device: SpiDevice, NrstPin: OutputPin, IrqPin: Wait + InputPin, Delays: DelayNs>
    Dw1000<Device, NrstPin, IrqPin, Delays>
{
    /// The error type returned by most of the functions of the driver.
    /// It's fairly complicated because it's generic over the embedded_hal errors.
    pub type Error = Error<Device::Error, NrstPin::Error>;

    /// Construct a new dw1000 given an SPI Device, an NRST pin, and a profile.
    /// This will NOT initialize the dw1000! Call dw1000.reset() and then dw1000.init() for that.
    ///
    /// Note that dw1000.init() performs slow-clock calls and cannot function above 2mhz spi - make sure
    /// the spi driver is set to 1-2mhz before calling the reset()/init() sequence, and increase as high as 20mhz
    /// for actual operation.
    pub fn new(
        spi: Device,
        mut nrst: NrstPin,
        irq: IrqPin,
        profile: Profile,
        delays: Delays,
    ) -> Result<Self, Self::Error> {
        nrst.set_high().map_err(Error::GpioError)?;
        Ok(Self {
            spi,
            irq,
            nrst,
            profile,
            delays,
        })
    }

    /// Get a reference to the internal spi device.
    pub fn spi<'a>(&'a mut self) -> &'a mut Device {
        &mut self.spi
    }

    /// Verify that the device id is correct (0xDECA0130)
    pub fn check_dev_id(&mut self) -> Result<(), Self::Error> {
        let devid = self.read_reg(registers::DEV_ID)?;
        if devid != 0xDECA0130 {
            Err(Error::BadDeviceId(devid))
        } else {
            Ok(())
        }
    }

    async fn interrupt_wait(
        &mut self,
        mut f: impl FnMut(SysStatus) -> bool,
    ) -> Result<(), Self::Error> {
        loop {
            self.irq.wait_for_high().await.unwrap();
            if f(self.get_status()?) {
                break;
            }
        }
        Ok(())
    }

    /// Reset the device. This pulls NRST low for 10ms and then waits
    /// 100ms for bring-up.
    pub async fn reset(&mut self) -> Result<(), Self::Error> {
        self.nrst.set_low().map_err(Error::GpioError)?;
        self.delays.delay_ms(10).await;
        self.nrst.set_high().map_err(Error::GpioError)?;
        self.delays.delay_ms(100).await;
        Ok(())
    }

    /// Issue HRBPT to switch to the other buffer in the RX double-buffered set.
    pub fn hrbpt(&mut self) -> Result<(), Self::Error> {
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                hrbpt: true,
                ..Default::default()
            },
        )
    }

    fn make_header<'a>(write: bool, file: u8, addr: u16, buffer: &'a mut [u8; 3]) -> &'a [u8] {
        if addr == 0 {
            *buffer = [if write { 1 << 7 } else { 0 } | file, 0, 0];
            &buffer[0..1]
        } else if addr < 128 {
            *buffer = [
                if write { 1 << 7 } else { 0 } | (1 << 6) | file,
                addr as u8,
                0,
            ];
            &buffer[0..2]
        } else {
            *buffer = [
                if write { 1 << 7 } else { 0 } | (1 << 6) | file,
                ((addr & 0b00000_00011_11111) as u8) | (1 << 7),
                (((addr & 0b11111_11100_00000) >> 7) as u8),
            ];
            &buffer[0..3]
        }
    }

    /// Read a register from the DW1000 register file.
    /// Note that this blocks.
    pub fn read_reg<const LEN: usize, Reg: Register<LEN, Dw1000RegSet>>(
        &mut self,
        _reg: Reg,
    ) -> Result<Reg::Type, Self::Error> {
        let mut buffer = [0; LEN];
        self.spi.transaction(&mut [
            Operation::Write(Self::make_header(false, Reg::FILE, Reg::REG, &mut [0; 3])),
            Operation::Read(&mut buffer),
        ])?;
        let ret = Ok(Reg::Type::deserialize(&buffer));
        ret
    }

    /// Write a value to a register in the dw1000 register bank.
    /// This blocks.
    pub fn write_reg<const LEN: usize, Reg: Register<LEN, Dw1000RegSet>>(
        &mut self,
        _reg: Reg,
        data: Reg::Type,
    ) -> Result<(), Self::Error> {
        let buffer = data.serialize();
        self.spi.transaction(&mut [
            Operation::Write(Self::make_header(true, Reg::FILE, Reg::REG, &mut [0; 3])),
            Operation::Write(&buffer),
        ])?;
        Ok(())
    }

    /// Write the TX buffer register. This cannot be efficiently encoded in the current
    /// register model, and rather than making it much more complicated, we've opted
    /// to just add special exceptions for the buffers.
    pub fn write_tx_buffer(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [
            Operation::Write(Self::make_header(true, 0x09, 0x00, &mut [0; 3])),
            Operation::Write(data),
        ])?;
        Ok(())
    }

    /// Read the RX buffer register. This cannot be encoded in the current
    /// register model, and rather than making it much more complicated, we've opted
    /// to just add special exceptions for the buffers.
    pub fn read_rx_buffer(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [
            Operation::Write(Self::make_header(false, 0x11, 0x00, &mut [0; 3])),
            Operation::Read(data),
        ])?;
        Ok(())
    }

    fn generate_syscfg(&self) -> SysCfg {
        SysCfg {
            dis_drxb: true,
            dis_stxp: true,
            hirq_pol: true,
            rxautr: false,
            rxm110k: matches!(self.profile.drate, Bitrate::Kbps110),
            ..Default::default()
        }
    }

    fn reset_syscfg(&mut self) -> Result<(), Self::Error> {
        self.write_reg(SYS_CFG, self.generate_syscfg())
    }

    /// Initialize the DW1000. This writes all system configuration and
    /// "magic" tuning parameters. After this returns Ok(()), the DW1000
    /// is able to transmit, receive, and range - e.g., no more setup work is
    /// necessary for any of the basic operating modes.
    ///
    /// This blocks for potentially several microseconds as it needs to
    /// write many, many registers. It's recommended to execute this before
    /// starting any background async tasks that it might starve out.
    ///
    /// For implementers: the magic values here are mostly undocumented; we've taken
    /// a "just trust it" approach, which has worked well enough so far.
    pub async fn init(&mut self) -> Result<(), Self::Error> {
        self.write_reg(SYS_CFG, self.generate_syscfg())?;
        self.write_reg(SYS_MASK, SysStatus::rx_mask() | SysStatus::tx_mask())?;
        self.write_reg(
            CHAN_CTRL,
            ChanCtrl {
                rx_chan: self.profile.channel,
                tx_chan: self.profile.channel,
                tx_pcode: self.profile.preamble_code.into(),
                rx_pcode: self.profile.preamble_code.into(),
                rxprf: self.profile.prf,
                ..Default::default()
            },
        )?;
        self.write_reg(
            DRX_TUNE0B,
            match self.profile.drate {
                Bitrate::Kbps110 => 0x000A,
                Bitrate::Kbps850 => 0x0001,
                Bitrate::Mbps6_8 => 0x0001,
            },
        )?;
        self.write_reg(
            DRX_TUNE1A,
            match self.profile.prf {
                Prf::Mhz4 => 0x0000,
                Prf::Mhz16 => 0x0087,
                Prf::Mhz64 => 0x008D,
            },
        )?;
        self.write_reg(
            DRX_TUNE1B,
            match self.profile.drate {
                Bitrate::Kbps110 => 0x0064,
                Bitrate::Kbps850 => 0x0020,
                Bitrate::Mbps6_8 => 0x0020,
            },
        )?;
        self.write_reg(
            DRX_TUNE2,
            match self.profile.pac_size {
                PacSize::Pac8 => match self.profile.prf {
                    Prf::Mhz4 => 0,
                    Prf::Mhz16 => 0x311A002D,
                    Prf::Mhz64 => 0x313B006B,
                },
                PacSize::Pac16 => match self.profile.prf {
                    Prf::Mhz4 => 0,
                    Prf::Mhz16 => 0x331A0052,
                    Prf::Mhz64 => 0x333B00BE,
                },
                PacSize::Pac32 => match self.profile.prf {
                    Prf::Mhz4 => 0,
                    Prf::Mhz16 => 0x351A009A,
                    Prf::Mhz64 => 0x353B015E,
                },
                PacSize::Pac64 => match self.profile.prf {
                    Prf::Mhz4 => 0,
                    Prf::Mhz16 => 0x371A011D,
                    Prf::Mhz64 => 0x373B0296,
                },
            },
        )?;
        self.write_reg(
            DRX_TUNE4H,
            match self.profile.preamble {
                Preamble::Preamble64 => 0x0010,
                _ => 0x0028,
            },
        )?;
        self.write_reg(
            AGC_TUNE1,
            match self.profile.prf {
                Prf::Mhz4 => 0,
                Prf::Mhz16 => 0x8870,
                Prf::Mhz64 => 0x889B,
            },
        )?;
        self.write_reg(AGC_TUNE2, 0x2502A907)?;
        self.write_reg(AGC_TUNE3, 0x0035)?;
        let pgdelay_values = [0xC9, 0xC2, 0xC5, 0x95, 0xC0, 0x00, 0x93];
        self.write_reg(
            TC_PGDELAY,
            pgdelay_values[self.profile.channel as usize - 1],
        )?;
        let fs_pllcfg_values = [
            0x09000407, 0x08400508, 0x08401009, 0x08400508, 0x0800041D, 0x0, 0x0800041D,
        ];
        self.write_reg(
            FS_PLLCFG,
            fs_pllcfg_values[self.profile.channel as usize - 1],
        )?;
        let fs_plltune_values = [0x1E, 0x26, 0x56, 0x26, 0xBE, 0x00, 0xBE];
        self.write_reg(
            FS_PLLTUNE,
            fs_plltune_values[self.profile.channel as usize - 1],
        )?;
        self.write_reg(LDE_CFG1, 0xD)?;
        self.write_reg(
            LDE_CFG2,
            match self.profile.prf {
                Prf::Mhz4 => 0,
                Prf::Mhz16 => 0x1607,
                Prf::Mhz64 => 0x0607,
            },
        )?;
        let lde_repc_values = [
            0x5998, 0x5998, 0x51EA, 0x428E, 0x451E, 0x2E14, 0x8000, 0x51EA, 0x28F4, 0x3332, 0x3AE0,
            0x3D70, 0x3AE0, 0x35C2, 0x2B84, 0x35C2, 0x3332, 0x35C2, 0x35C2, 0x47AE, 0x3AE0, 0x3850,
            0x30A2, 0x3850,
        ];
        let mut lde_repc = lde_repc_values[self.profile.preamble_code as usize - 1];
        if matches!(self.profile.drate, Bitrate::Kbps110) {
            lde_repc >>= 3;
            lde_repc &= 0xFFFF;
        }
        self.write_reg(LDE_REPC, lde_repc)?;
        let power = self.profile.power as u32;
        self.write_reg(
            TX_POWER,
            (power << 24) | (power << 16) | (power << 8) | power,
        )?;

        self.load_lde().await?;
        Ok(())
    }

    /// Load LDE microcode. This is used heavily by the RX subsystem.
    /// Note that receives cannot actually run without it! They fail in many interesting ways
    /// without a running LDE, regardless of whether you're doing ranging or not.
    ///
    /// This is called by init(), so you probably don't need to call it yourself.
    pub async fn load_lde(&mut self) -> Result<(), Self::Error> {
        self.write_reg(PMSC_CTRL0, 0x0301)?;
        self.write_reg(
            OTP_CTRL,
            OtpCtrl {
                ldeload: true,
                ..Default::default()
            },
        )?;
        self.delays.delay_us(200).await;
        self.write_reg(PMSC_CTRL0, 0x0200)?;
        Ok(())
    }

    /// Set the TX function control register. This uses several
    /// parameters from the DW1000 profile; it's just a convenience function
    /// to simplify encoding those.
    pub fn set_tx_fctrl(&mut self, dlen: usize) -> Result<(), Self::Error> {
        self.write_reg(
            TX_FCTRL,
            TxFctrl {
                tflen: (dlen as u16).into(),
                txbr: self.profile.drate,
                tr: false,
                txprf: self.profile.prf,
                preamble: self.profile.preamble,
                txboffs: 0.into(),
                ifsdelay: 0.into(),
                ..Default::default()
            },
        )?;
        Ok(())
    }

    /// Force the DW1000 into idle mode.
    pub fn idle(&mut self) -> Result<(), Self::Error> {
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                trxoff: true,
                ..Default::default()
            },
        )?;
        Ok(())
    }

    /// Transmit a message. This waits until the transmission completes,
    /// so once this returns Ok(()), you can immediately transmit (or receive).
    pub async fn transmit(&mut self, message: &[u8]) -> Result<(), Self::Error> {
        self.idle()?;
        self.set_tx_fctrl(message.len() + 2)?;
        self.write_tx_buffer(message)?;
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                txstrt: true,
                ..Default::default()
            },
        )?;
        self.interrupt_wait(|sys_status| sys_status.txfrs).await?;
        self.write_reg(SYS_STATUS, SysStatus::tx_mask())?;
        Ok(())
    }

    /// Delayed transmission. This waits some dw1000 clock cycles (each of which is ~15.65ps) before transmitting.
    /// It uses the DW1000's internal delay support and so is extremely precise, suitable
    /// for ranging purposes.
    pub async fn transmit_delayed(
        &mut self,
        message: &[u8],
        dtime: u64,
    ) -> Result<(), Self::Error> {
        self.idle()?;
        self.set_tx_fctrl(message.len() + 2)?;
        self.write_reg(DX_TIME, Timestamp { time: dtime.into() })?;
        self.write_tx_buffer(message)?;
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                txstrt: true,
                txdlys: true,
                ..Default::default()
            },
        )?;
        self.interrupt_wait(|sys_status| sys_status.txfrs).await?;
        self.write_reg(SYS_STATUS, SysStatus::tx_mask())?;
        Ok(())
    }

    /// Read the status register.
    pub fn get_status(&mut self) -> Result<SysStatus, Self::Error> {
        self.read_reg(registers::SYS_STATUS)
    }

    /// Read the RX time register. This returns a value in dw1000 clock cycles,
    /// each of which is ~15.65ps
    pub fn get_rx_time(&mut self) -> Result<u64, Self::Error> {
        Ok(self.read_reg(registers::RX_TIME)?.time.0)
    }

    /// Read the TX time register. This returns a value in dw1000 clock cycles,
    /// each of which is ~15.65ps
    pub fn get_tx_time(&mut self) -> Result<u64, Self::Error> {
        Ok(self.read_reg(registers::TX_TIME)?.time.0)
    }

    /// Receive a frame. This waits until a frame is received. You will probably want
    /// to use an external timeout (e.g. embassy's `with_timeout` function) to ensure
    /// it doesn't spin forever. Returns the length of the received frame - you can
    /// get the actual data with the `read_rx_buffer` function.
    pub async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        // go to idle
        self.idle()?;
        // clear rx status
        self.write_reg(SYS_STATUS, SysStatus::rx_mask())?;
        // start rx
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                rxenab: true,
                ..Default::default()
            },
        )?;
        let ret = loop {
            //defmt::info!("state is {}", self.read_reg(SYS_STATE)?);
            //defmt::info!("status is {}", self.read_reg(SYS_STATUS)?);
            self.interrupt_wait(|_| true).await?;
            let stat = self.read_reg(SYS_STATUS)?;
            Error::check_rx_sysstatus(stat)?;
            if stat.rxdfr {
                let finfo = self.read_reg(RX_FINFO)?;
                if finfo.rxflen.0 < 2 {
                    return Err(Error::RxLenTooSmall(finfo.rxflen.0 as usize));
                }
                let len = (finfo.rxflen.0 as usize - 2).min(buffer.len());
                self.read_rx_buffer(&mut buffer[0..len])?;
                self.write_reg(SYS_STATUS, SysStatus::rx_mask())?;
                break Ok(len);
            }
            if matches!(self.read_reg(SYS_STATE)?.rx_state, RxState::Idle) {
                defmt::info!("rx idled, restarting");
                self.write_reg(
                    SYS_CTRL,
                    SysCtrl {
                        rxenab: true,
                        ..Default::default()
                    },
                )?;
            }
        };
        // go to idle
        //self.idle().await;
        // clear rx status
        self.write_reg(SYS_STATUS, SysStatus::rx_mask())?;
        ret
    }

    /// Get an RXAUTR stream - the receiver automatically turns itself
    /// back on after every reception (this is a DW1000 feature),
    /// avoiding dropped frames. This is much more efficient and reliable
    /// than manually polling the receiver.
    pub async fn rx_stream<'a>(
        &'a mut self,
    ) -> Result<RxStream<'a, Device, NrstPin, IrqPin, Delays>, Self::Error> {
        let mut cfg = self.generate_syscfg();
        cfg.rxautr = true;
        cfg.dis_drxb = false;
        self.write_reg(SYS_CFG, cfg)?;
        self.write_reg(
            SYS_CTRL,
            SysCtrl {
                rxenab: true,
                ..Default::default()
            },
        )?;
        Ok(RxStream { chip: self })
    }

    /// Enter the standard sleep mode.
    pub fn sleep_wakeup(
        &mut self,
        pin: bool,
        spi: bool,
        timer: Option<u16>,
    ) -> Result<(), Self::Error> {
        self.write_reg(
            AON_CFG0,
            AonCfg0 {
                sleep_en: true,
                wake_cnt: timer.is_some(),
                wake_pin: pin,
                wake_spi: spi,
                sleep_tim: timer.unwrap_or(0).into(),
                ..Default::default()
            },
        )?;
        self.write_reg(
            AON_CTRL,
            AonCtrl {
                upl_cfg: true,
                ..Default::default()
            },
        )?;
        Ok(())
    }

    /// Sleep with SPI wakeup enabled.
    pub fn sleep(&mut self) -> Result<(), Self::Error> {
        self.sleep_wakeup(false, true, None)
    }

    /// Wake up a sleeping DW1000 by asserting the SPI CS line for 500µs.
    pub fn wakeup(&mut self) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [Operation::DelayNs(500000)])?;
        Ok(())
    }
}

pub use crate::registers::dw1000 as registers;
