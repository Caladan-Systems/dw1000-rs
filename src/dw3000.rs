/*
 * FILE: dw3000.rs
 * AUTHOR: Tyler Clarke
 *
 * Drivers for the DW3000.
 */

use crate::registers::Register;
use crate::registers::RegisterType;
use crate::registers::dw3000::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;
use embedded_hal_async::delay::DelayNs;

pub use crate::registers::dw3000 as registers;

impl PreambleCode {
    pub fn prf(&self) -> Prf {
        use PreambleCode::*;
        match self {
            Code3 | Code4 => Prf::Prf16,
            _ => Prf::Prf64,
        }
    }
}

pub struct Profile {
    pub channel: Channel,
    pub sfd_type: SfdType,
    pub preamble_code: PreambleCode,
    pub bitrate: Bitrate,
    pub psr: TxPsr,
    pub fine_power: u8,
    pub coarse_power: u8,
}

impl Profile {
    pub fn default_5() -> Self {
        Self {
            channel: Channel::Channel5,
            sfd_type: SfdType::Ieee802154z,
            preamble_code: PreambleCode::Code4,
            bitrate: Bitrate::Mbps681,
            psr: TxPsr::Preamble64,
            fine_power: 63,
            coarse_power: 3,
        }
    }
}

pub struct Dw3000<Device: SpiDevice, NrstPin: OutputPin, Delays: DelayNs> {
    spi: Device,
    nrst: NrstPin,
    profile: Profile,
    delays: Delays,
}

impl registers::SysStatus {
    /// Bitmask for RX events.
    pub fn rx_mask() -> Self {
        Self {
            rxfce: true,
            rxfcg: true,
            rxfr: true,
            rxfsl: true,
            rxfto: true,
            rxovrr: true,
            rxphd: true,
            rxphe: true,
            rxprd: true,
            rxprej: true,
            rxpto: true,
            rxsfdd: true,
            rxsto: true,
            ..Default::default()
        }
    }
}

#[derive(Debug)]
pub enum ReceiveError {
    Timeout, // timeout expired. this can be software controlled, or controlled by the timer on the dwm1000 (see SYS_STATUS.RX_RFTO)
    PhyHeaderError,
    FcsError,
    ReedSolomonSyncLoss,
    LdeErr,
    PreambleTimeout,
    SfdTimeout, // distinct from a timeout: this is when the preamble is detected but no valid SFD follows
    FrameFilterReject,
}

impl SysStatus {
    fn tx_mask() -> Self {
        Self {
            txfrb: true,
            txfrs: true,
            txphs: true,
            txprs: true,
            ..Default::default()
        }
    }
}

impl<Device: SpiDevice, NrstPin: OutputPin, Delays: DelayNs> Dw3000<Device, NrstPin, Delays> {
    pub async fn new(spi: Device, mut nrst: NrstPin, profile: Profile, delays: Delays) -> Self {
        nrst.set_high().unwrap();
        Self {
            spi,
            nrst,
            profile,
            delays,
        }
    }

    fn full_header(write: bool, file: u8, reg: u8) -> [u8; 2] {
        [
            if write { 0b1100_0000 } else { 0b0100_0000 } | (file << 1) | (reg >> 6),
            (reg << 2),
        ]
    }

    pub async fn reset(&mut self) {
        self.nrst.set_low().unwrap();
        self.delays.delay_ms(10).await;
        self.nrst.set_high().unwrap();
        self.delays.delay_ms(100).await;
    }

    pub async fn read_reg<const LEN: usize, Reg: Register<LEN, Dw3000RegSet>>(
        &mut self,
        _reg: Reg,
    ) -> Result<Reg::Type, Device::Error>
    where
        [u8; Reg::Type::LEN]: Sized,
    {
        let mut databuf = [0; LEN];
        self.spi.transaction(&mut [
            Operation::Write(&Self::full_header(false, Reg::FILE, Reg::REG as u8)),
            Operation::Read(&mut databuf),
        ])?;
        Ok(Reg::Type::deserialize(&databuf))
    }

    pub async fn write_reg<const LEN: usize, Reg: Register<LEN, Dw3000RegSet>>(
        &mut self,
        _reg: Reg,
        value: Reg::Type,
    ) -> Result<(), Device::Error>
    where
        [u8; Reg::Type::LEN]: Sized,
    {
        self.spi.transaction(&mut [
            Operation::Write(&Self::full_header(true, Reg::FILE, Reg::REG as u8)),
            Operation::Write(&value.serialize()),
        ])
    }

    pub async fn command(&mut self, command: FastCommand) {
        self.spi
            .transaction(&mut [Operation::Write(&[0b10_00000_1 | (command.as_bits() << 1)])]);
    }

    pub async fn write_tx_buffer(&mut self, buffer: &[u8]) {
        self.spi.transaction(&mut [
            Operation::Write(&Self::full_header(true, 0x14, 0x00)),
            Operation::Write(&buffer),
        ]);
    }

    pub async fn read_rx_buffer(&mut self, buffer: &mut [u8]) {
        self.spi.transaction(&mut [
            Operation::Write(&Self::full_header(false, 0x12, 0x00)),
            Operation::Read(buffer),
        ]);
    }

    pub async fn read_status_clear(&mut self) -> registers::SysStatus {
        let status = self.read_reg(registers::SYS_STATUS).await;
        self.write_reg(registers::SYS_STATUS, status).await;
        status
    }

    pub async fn read_status(&mut self) -> registers::SysStatus {
        self.read_reg(registers::SYS_STATUS).await
    }

    pub async fn check_dev_id(&mut self) -> Result<(), u32> {
        let devid = self.read_reg(registers::DEV_ID).await;
        if devid != 0xDECA0302 {
            Err(devid)
        } else {
            Ok(())
        }
    }

    pub async fn idle(&mut self) {
        self.command(FastCommand::TRXOFF).await;
    }

    /// Tune channel-specific magic parameters.
    pub async fn tune_channel(&mut self, channel: Channel) {
        let (cfg0, cfg1, lut0, lut1, lut2, lut3, lut4, lut5, lut6, pllcfg, txctrl2) = match channel
        {
            Channel::Channel5 => (
                0x10000240, 0x1b6da489, 0x0001C0FD, 0x0001C43E, 0x0001C6BE, 0x0001C77E, 0x0001CF36,
                0x0001CFB5, 0x0001CFF5, 0x1F3C, 0x1C071134,
            ),
            Channel::Channel9 => (
                0x10000240, 0x1b6da489, 0x0002A8FE, 0x0002AC36, 0x0002A5FE, 0x0002AF3E, 0x0002AF7d,
                0x0002AFB5, 0x0002AFB5, 0x0F3C, 0x1C010034,
            ),
        };
        self.write_reg(registers::DGC_CFG0, cfg0).await;
        self.write_reg(registers::DGC_CFG1, cfg1).await;
        self.write_reg(registers::DGC_LUT_0, lut0).await;
        self.write_reg(registers::DGC_LUT_1, lut1).await;
        self.write_reg(registers::DGC_LUT_2, lut2).await;
        self.write_reg(registers::DGC_LUT_3, lut3).await;
        self.write_reg(registers::DGC_LUT_4, lut4).await;
        self.write_reg(registers::DGC_LUT_5, lut5).await;
        self.write_reg(registers::DGC_LUT_6, lut6).await;
        self.write_reg(registers::PLL_CFG, pllcfg).await;
        self.write_reg(registers::RF_TX_CTRL_2, txctrl2).await;
    }

    /// Tune fixed parameters.
    pub async fn tune(&mut self) {
        self.write_reg(registers::DTUNE3, 0xAF5F35CC).await;
        self.write_reg(registers::RF_TX_CTRL_1, 0x0E).await;
        self.write_reg(registers::LDO_RLOAD, 0x14).await;
        self.write_reg(registers::PLL_CAL, 0x0081).await; // magic, see https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54?permalink_comment_id=4472410#gistcomment-4472410
    }

    /// Set transmitter power.
    pub async fn set_tx_power(&mut self, fine: u8, coarse: u8) {
        let byte = (fine << 2) | coarse;
        self.write_reg(
            registers::TW_POWER,
            TxPower {
                phr_pwr: byte.into(),
                shr_pwr: byte.into(),
                sts_pwr: byte.into(),
                data_pwr: byte.into(),
            },
        )
        .await;
    }

    /// Receiver calibration.
    pub async fn rx_cal(&mut self) {
        let old_ldo_ctrl = self.read_reg(registers::LDO_CTRL).await;
        self.write_reg(registers::LDO_CTRL, 0x105).await;
        self.write_reg(
            registers::RX_CAL,
            RxCal {
                cal_en: false,
                cal_mode: registers::CalMode::Normal,
                ..Default::default()
            },
        )
        .await;

        self.delays.delay_ms(20).await;

        self.write_reg(
            registers::RX_CAL,
            RxCal {
                cal_en: true,
                cal_mode: registers::CalMode::Calibration,
                ..Default::default()
            },
        )
        .await;

        defmt::info!(
            "current state of cal reg: {}",
            defmt::Debug2Format(&self.read_reg(registers::RX_CAL).await)
        );

        while self.read_reg(registers::RX_CAL).await.cal_en {}

        defmt::info!(
            "current state of cal reg: {}",
            defmt::Debug2Format(&self.read_reg(registers::RX_CAL).await)
        );

        self.write_reg(
            registers::RX_CAL,
            RxCal {
                cal_en: true,
                cal_mode: registers::CalMode::Normal,
                ..Default::default()
            },
        )
        .await;

        defmt::info!(
            "current state of cal res: 0x{:08X}, 0x{:08X}",
            self.read_reg(registers::RX_CAL_RESI).await,
            self.read_reg(registers::RX_CAL_RESQ).await
        );

        self.write_reg(registers::LDO_CTRL, old_ldo_ctrl).await;

        //loop {}
    }

    /// Write the configuration.
    pub async fn init(&mut self) {
        self.write_reg(
            registers::SYS_CFG,
            SysCfg {
                ffen: false,
                dis_fcs_tx: false,
                dis_fce: false,
                dis_drxb: true,
                phr_mode: registers::PhrMode::Standard,
                phr_6m8: false,
                spi_crcen: false,
                cia_ipatov: false,
                cia_sts: false,
                rxwtoe: false,
                rxautr: false,
                auto_ack: false,
                cp_spc: registers::CpSpc::NoSts,
                cp_sdc: false,
                pdoa_mode: registers::PdoaMode::Disable,
                fast_aat: true,
                ..Default::default()
            },
        )
        .await;

        self.write_reg(
            registers::CHAN_CTRL,
            ChanCtrl {
                rf_chan: self.profile.channel,
                sfd_type: self.profile.sfd_type,
                tx_pcode: self.profile.preamble_code,
                rx_pcode: self.profile.preamble_code,
                ..Default::default()
            },
        )
        .await;

        self.write_reg(
            registers::DGC_CFG,
            DgcCfg {
                rx_tune_en: matches!(self.profile.preamble_code.prf(), Prf::Prf64),
                ..Default::default()
            },
        )
        .await;

        self.write_reg(
            registers::DTUNE0,
            DTune0 {
                dt0b4: false,
                pac: registers::Pac::Pac8, /*match self.profile.bitrate {
                                               Bitrate::Kbps850 => registers::Pac::Pac8,
                                               Bitrate::Mbps681 => match self.profile.psr {
                                                   TxPsr::Preamble32 => registers::Pac::Pac4,
                                                   _ => registers::Pac::Pac8,
                                               },
                                           },*/
            },
        )
        .await;

        self.set_tx_power(self.profile.fine_power, self.profile.coarse_power)
            .await;

        self.rx_cal().await;

        self.tune_channel(self.profile.channel).await;
        self.tune().await;
    }

    async fn frame_setup(&mut self, frame: &[u8]) {
        self.write_reg(
            registers::TX_FCTRL,
            TxFctrl {
                txflen: (frame.len() as u16 + 2).into(),
                txbr: self.profile.bitrate,
                tr: false,
                txpsr: self.profile.psr,
                txb_offset: 0.into(),
                fine_plen: 0.into(),
                ..Default::default()
            },
        )
        .await;
        self.write_tx_buffer(frame).await;
    }

    /// Transmit a frame
    pub async fn transmit(&mut self, frame: &[u8]) {
        self.frame_setup(frame).await;
        self.command(FastCommand::TX).await;
        loop {
            let status = self.read_status().await;
            if status.txfrs {
                self.write_reg(SYS_STATUS, SysStatus::tx_mask()).await;
                break;
            }
        }
    }

    pub async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, ReceiveError> {
        // go to idle
        self.idle().await;
        // clear rx status
        self.write_reg(registers::SYS_STATUS, registers::SysStatus::rx_mask())
            .await;
        // start rx
        self.command(FastCommand::RX).await;
        let ret = loop {
            let stat = self.read_reg(registers::SYS_STATUS).await;
            if stat.rxfr {
                let finfo = self.read_reg(registers::RX_FINFO).await;
                let len = (finfo.rxflen.0 as usize - 2).min(buffer.len());
                self.read_rx_buffer(&mut buffer[0..len]).await;
                break Ok(len);
            }
            if stat.rxphe {
                break Err(ReceiveError::PhyHeaderError);
            }
            if stat.rxfce {
                break Err(ReceiveError::FcsError);
            }
            if stat.rxfsl {
                break Err(ReceiveError::ReedSolomonSyncLoss);
            }
            if stat.rxfto {
                break Err(ReceiveError::Timeout);
            }
            if stat.rxpto {
                break Err(ReceiveError::PreambleTimeout);
            }
            if stat.rxsto {
                break Err(ReceiveError::SfdTimeout);
            }
        };
        // go to idle
        self.idle().await;
        // clear rx status
        self.write_reg(registers::SYS_STATUS, registers::SysStatus::rx_mask())
            .await;
        ret
    }

    pub async fn get_tx_time(&mut self) -> u64 {
        self.read_reg(registers::TX_TIME).await.time.0
    }

    pub async fn get_rx_time(&mut self) -> u64 {
        self.read_reg(registers::RX_TIME).await.time.0
    }
}

macro_rules! fastcommands {
    {$($cmd_name:ident => $num:literal),*} => {
        #[allow(dead_code)]
        #[allow(non_camel_case_types)]
        #[derive(Copy, Clone)]
        pub enum FastCommand {
            $($cmd_name,)*
        }

        #[allow(dead_code)]
        impl FastCommand {
            fn as_bits(&self) -> u8 {
                match *self {
                    $(
                        FastCommand::$cmd_name => $num,
                    )*
                }
            }
        }
    };
}

fastcommands! {
    TRXOFF => 0x0,
    TX => 0x1,
    RX => 0x2,
    DTX => 0x3,
    DRX => 0x4,
    DTX_TS => 0x5,
    DRX_TS => 0x6,
    DTX_RS => 0x7,
    DRX_RS => 0x8,
    DTX_REF => 0x9,
    DRX_REF => 0xA,
    CCA_TX => 0xB,
    TX_W4R => 0xC,
    DTX_W4R => 0xD,
    DTX_TS_W4R => 0xE,
    DTX_RS_W4R => 0xF,
    DTX_REF_W4R => 0x10,
    CCA_TX_W4R => 0x11,
    CLR_IRQS => 0x12,
    DB_TOGGLE => 0x13
}
