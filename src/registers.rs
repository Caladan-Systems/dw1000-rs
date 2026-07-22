/*!
 * A convenient set of traits and macros for implementing the Decawave register files.
 */

#![allow(missing_docs)]

/// A type that can be encoded as a DW1000 register with a constant length.
pub trait RegisterType<const LEN: usize> {
    /// This should always be (manually, in the impl) set to the same value as
    /// the generic parameter with the same name.
    ///
    /// The length of the register in bytes.
    const LEN: usize;

    /// Deserialize a buffer containing enough bytes for this register type.
    /// Note that this is infallible.
    fn deserialize(buffer: &[u8; LEN]) -> Self;

    /// Serialize this into a buffer. This is infallible.
    fn serialize(&self) -> [u8; LEN];
}

/// An actual register in a file with an offset (FILE:REG).
pub trait Register<const LEN: usize, RegSet> {
    /// The register file.
    const FILE: u8;
    /// The offset into that register file.
    const REG: u16;

    /// The type of this register (e.g. u16, SysStatus).
    type Type: RegisterType<LEN>;
}

#[allow(dead_code)]
trait RegisterFields {
    fn bitfield_len() -> usize;

    fn from_u64(data: u64) -> Self;

    fn to_u64(&self) -> u64;

    fn parse(bytes: &[u8], offset: &mut usize) -> Self
    where
        Self: Sized,
    {
        let inner_offset = *offset % 8 as usize;
        let bytes = &bytes[*offset as usize >> 3..];
        let mut ret = 0;
        let me_size = (Self::bitfield_len() + inner_offset - 1) / 8 + 1; // number of bytes we need to parse
        for i in 0..me_size {
            let shift = bytes[i] as u64;
            let shift = shift << (i as u64 * 8);
            let shift = shift >> inner_offset;
            ret |= shift;
        }
        ret &= (1 << Self::bitfield_len()) - 1;
        *offset += Self::bitfield_len();
        Self::from_u64(ret)
    }

    fn write_into(&self, bytes: &mut [u8], offset: &mut usize) {
        let bytes = &mut bytes[*offset as usize >> 3..];
        let initial_offset = *offset % 8;
        let me_size = (Self::bitfield_len() + initial_offset - 1) / 8 + 1; // number of bytes needed
        // to hold this number *at the offset*
        let num = self.to_u64() & ((1 << Self::bitfield_len()) - 1);
        for (i, byte) in bytes[0..me_size].iter_mut().enumerate() {
            let shift = initial_offset as i32 - i as i32 * 8;
            let sliced = if shift >= 0 {
                num << shift
            } else {
                num >> -shift
            } as u8;
            *byte |= sliced;
        }
        *offset += Self::bitfield_len();
    }
}

/// A number that can be stored in a u64
pub trait U64Number {
    /// Convert this number into a u64
    fn into_u64(&self) -> u64;

    /// Turn a u64 into this number (truncating if necessary)
    fn from_u64(thing: u64) -> Self;
}

macro_rules! u64_number_impl {
    ($numtype:ty) => {
        impl U64Number for $numtype {
            fn into_u64(&self) -> u64 {
                *self as u64
            }

            fn from_u64(thing: u64) -> Self {
                thing as Self
            }
        }
    };
}

u64_number_impl!(u64);
u64_number_impl!(u32);
u64_number_impl!(u16);
u64_number_impl!(u8);

#[derive(Debug, Copy, Clone, defmt::Format)]
/// A pure numerical field with an outer container type (the smallest pure number type that can
/// contain the register field, usually) and an actual length in bits. The field will be masked
/// by the bit length so you can e.g. use NumberField<u64, 40> as a u64-backed storage for a
/// 40-bit number.
pub struct NumberField<Type: U64Number, const BIT_COUNT: usize>(pub Type);

impl<Type: U64Number, const BIT_COUNT: usize> From<Type> for NumberField<Type, BIT_COUNT> {
    fn from(value: Type) -> Self {
        NumberField(value)
    }
}

#[derive(Debug, Copy, Clone, defmt::Format)]
/// Just takes up bits, doesn't actually encode anything. There are many reserved
/// bitfields in the dw1000 register file; this is used to pad those.
pub struct ReservedField<const BIT_COUNT: usize>;

impl<const BIT_COUNT: usize, Type: U64Number> RegisterFields for NumberField<Type, BIT_COUNT> {
    fn bitfield_len() -> usize {
        BIT_COUNT
    }

    fn from_u64(data: u64) -> Self {
        NumberField(Type::from_u64(data))
    }

    fn to_u64(&self) -> u64 {
        self.0.into_u64()
    }
}

impl<const BIT_COUNT: usize> RegisterFields for ReservedField<BIT_COUNT> {
    fn bitfield_len() -> usize {
        1
    }

    fn from_u64(_data: u64) -> Self {
        Self
    }

    fn to_u64(&self) -> u64 {
        0
    }

    fn parse(_bytes: &[u8], offset: &mut usize) -> Self {
        // no-op
        *offset += BIT_COUNT;
        Self
    }

    fn write_into(&self, _bytes: &mut [u8], offset: &mut usize) {
        // no-op
        *offset += BIT_COUNT;
    }
}

impl RegisterFields for bool {
    fn bitfield_len() -> usize {
        1
    }

    fn from_u64(data: u64) -> Self {
        data != 0
    }

    fn to_u64(&self) -> u64 {
        if *self { 1 } else { 0 }
    }
}

macro_rules! numeric_reg_type {
    ($number:ty : $size:literal ) => {
        impl RegisterType<$size> for $number {
            const LEN: usize = $size;

            fn deserialize(buffer: &[u8; $size]) -> Self {
                Self::from_le_bytes(*buffer)
            }

            fn serialize(&self) -> [u8; $size] {
                self.to_le_bytes()
            }
        }
    };
}

macro_rules! register {
    ($name:ident: $regtype: ty = $file:literal $reg:literal) => {
        #[allow(non_camel_case_types)]
        #[allow(dead_code)]
        #[doc = concat!("Register ", stringify!($name), " at ", stringify!($file), ":", stringify!($reg))]
        pub struct $name;
        impl Register<{ <$regtype as RegisterType<_>>::LEN }, Basis> for $name {
            const FILE: u8 = $file;
            const REG: u16 = $reg;

            type Type = $regtype;
        }
    };
}

macro_rules! reg_type {
     (struct $name:ident($reg_size:literal) { $($bitfield_name:ident : $bitfield_type:ty = $bitfield_default:tt),* }) => {
         #[allow(dead_code)]
         #[derive(Debug, Copy, Clone, defmt::Format)]
         pub struct $name {
             $(
                 pub $bitfield_name: $bitfield_type,
             )*
         }

         impl Default for $name {
             fn default() -> Self {
                 Self {
                     $(
                         $bitfield_name: $bitfield_default,
                     )*
                 }
             }
         }

         impl RegisterType<$reg_size> for $name {
             const LEN: usize = $reg_size;

             #[allow(unused_variables)]
             #[allow(unused_mut)]
             fn deserialize(buffer: &[u8; $reg_size]) -> Self {
                 let mut offset = 0;
                 Self {
                     $(
                         $bitfield_name: <$bitfield_type as RegisterFields>::parse(buffer, &mut offset),
                     )*
                 }
             }

             #[allow(unused_variables)]
             #[allow(unused_mut)]
             fn serialize(&self) -> [u8; $reg_size] {
                 let mut offset = 0;
                 let mut buffer = [0; $reg_size];
                 $(
                     self.$bitfield_name.write_into(&mut buffer, &mut offset);
                 )*
                 buffer
             }
         }
     };
 }

macro_rules! reg_enum {
     (enum $name:ident($base_type:ty, $field_len:literal) { $($field_name:ident => $field_value:literal ),* }) => {
         #[allow(dead_code)]
         #[derive(Debug, Copy, Clone,  defmt::Format)]
         pub enum $name {
             $(
                 $field_name,
             )*
         }

         #[allow(dead_code)]
         impl $name {
             pub fn into(&self) -> $base_type {
                 match *self {
                     $(
                         $name::$field_name => $field_value,
                     )*
                 }
             }

             pub fn from(value: $base_type) -> Option<Self> {
                 match value {
                     $(
                         $field_value => Some($name::$field_name),
                     )*
                     _ => None
                 }
             }
         }

         impl RegisterFields for $name {
             fn bitfield_len() -> usize {
                 $field_len
             }

             fn from_u64(data: u64) -> Self {
                 Self::from(data as $base_type).unwrap()
             }

             fn to_u64(&self) -> u64 {
                 self.into() as u64
             }
         }

         impl RegisterType<$field_len> for $name {
             const LEN: usize = $field_len;

             fn deserialize(buffer: &[u8; $field_len]) -> Self {
                 let mut offset = 0;
                 Self::parse(buffer, &mut offset)
             }

             fn serialize(&self) -> [u8; $field_len] {
                 let mut offset = 0;
                 let mut buffer = [0; $field_len];
                 self.write_into(&mut buffer, &mut offset);
                 buffer
             }
         }
     };
 }

numeric_reg_type!(u64: 8);
numeric_reg_type!(u32: 4);
numeric_reg_type!(u16: 2);
numeric_reg_type!(u8: 1);
numeric_reg_type!(i64: 8);
numeric_reg_type!(i32: 4);
numeric_reg_type!(i16: 2);
numeric_reg_type!(i8: 1);

pub mod dw3000 {
    /// Marker for the dw3000 register set.
    pub struct Dw3000RegSet;
    type Basis = Dw3000RegSet;

    use crate::registers::*;
    reg_enum!(enum TxPsr(u8, 4) {
        Preamble64 => 0b0001,
        Preamble1024 => 0b0010,
        Preamble4096 => 0b0011,
        Preamble32 => 0b0100,
        Preamble128 => 0b0101,
        Preamble1536 => 0b0110,
        Preamble256 => 0b1001,
        Preamble2048 => 0b1010,
        Preamble512 => 0b1101
    });

    reg_enum!(enum Bitrate(u8, 1) {
        Kbps850 => 0,
        Mbps681 => 1
    });

    reg_type!(struct TxFctrl(6) {
        txflen: NumberField<u16, 10> = (10.into()),
        txbr: Bitrate = (Bitrate::Kbps850),
        tr: bool = false,
        txpsr: TxPsr = (TxPsr::Preamble64),
        txb_offset: NumberField<u16, 10> = (0.into()),
        _res: ReservedField<14> = (ReservedField),
        fine_plen: NumberField<u16, 8> = (0.into())
    });

    reg_enum!(enum Channel(u8, 1) {
        Channel5 => 0,
        Channel9 => 1
    });

    reg_enum!(enum SfdType(u8, 2) {
        Ieee802154Short => 0b00,
        Decawave8 => 0b01,
        Decawave16 => 0b10,
        Ieee802154z => 0b11
    });

    reg_enum!(enum PreambleCode(u8, 5) {
        Code3 => 3,
        Code4 => 4,
        Code9 => 9,
        Code10 => 10,
        Code11 => 11,
        Code12 => 12
    });

    reg_type!(struct ChanCtrl(2) {
        rf_chan: Channel = (Channel::Channel5),
        sfd_type: SfdType = (SfdType::Ieee802154Short),
        tx_pcode: PreambleCode = (PreambleCode::Code9),
        rx_pcode: PreambleCode = (PreambleCode::Code9),
        _res: ReservedField<3> = (ReservedField)
    });

    reg_type!(struct SysStatus(6) {
        irqs: bool = false,
        cplock: bool = false,
        spicrce: bool = false,
        aat: bool = false,
        txfrb: bool = false,
        txprs: bool = false,
        txphs: bool = false,
        txfrs: bool = false,
        rxprd: bool = false,
        rxsfdd: bool = false,
        ciadone: bool = false,
        rxphd: bool = false,
        rxphe: bool = false,
        rxfr: bool = false,
        rxfcg: bool = false,
        rxfce: bool = false,
        rxfsl: bool = false,
        rxfto: bool = false,
        ciaerr: bool = false,
        vwarn: bool = false,
        rxovrr: bool = false,
        rxpto: bool = false,
        _res1: ReservedField<1> = ReservedField,
        spirdy: bool = false,
        rcint: bool = false,
        pllhilo: bool = false,
        rxsto: bool = false,
        hpdwarn: bool = false,
        cperr: bool = false,
        arfe: bool = false,
        _res2: ReservedField<3> = ReservedField,
        rxprej: bool = false,
        _res3: ReservedField<2> = ReservedField,
        vt_dt: bool = false,
        gpioirq: bool = false,
        aes_done: bool = false,
        aes_err: bool = false,
        cmd_err: bool = false,
        spi_ovf: bool = false,
        spi_unf: bool = false,
        spierr: bool = false,
        cca_fail: bool = false
    });

    reg_enum!(enum CalMode(u8, 2) {
        Normal => 0,
        Calibration => 1
    });

    reg_type!(struct RxCal(4) {
        cal_mode: CalMode = (CalMode::Normal),
        _res1: ReservedField<2> = ReservedField,
        cal_en: bool = false,
        _res2: ReservedField<11> = ReservedField,
        comp_dly: NumberField<u8, 4> = (0x2.into())
    });

    reg_enum!(enum PhrMode(u8, 1) {
        Standard => 0,
        Long => 1
    });

    reg_enum!(enum CpSpc(u8, 2) {
        NoSts => 0,
        BetweenSdfPhr => 1,
        AfterData => 2,
        AfterSdfNoPhr => 3
    });

    reg_enum!(enum PdoaMode(u8, 2) {
        Disable => 0,
        Mode1 => 1,
        Mode3 => 3
    });

    reg_type!(struct SysCfg(4) {
        ffen: bool = false,
        dis_fcs_tx: bool = false,
        dis_fce: bool = false,
        dis_drxb: bool = true,
        phr_mode: PhrMode = (PhrMode::Standard),
        phr_6m8: bool = false,
        spi_crcen: bool = false,
        cia_ipatov: bool = false,
        cia_sts: bool = false,
        rxwtoe: bool = false,
        rxautr: bool = false,
        auto_ack: bool = false,
        cp_spc: CpSpc = (CpSpc::NoSts),
        _res1: ReservedField<1> = ReservedField,
        cp_sdc: bool = false,
        pdoa_mode: PdoaMode = (PdoaMode::Disable),
        fast_aat: bool = false
    });

    reg_enum!(enum Prf(u8, 2) {
        Prf16 => 0b01,
        Prf64 => 0b10
    });

    reg_type!(struct RxFinfo(4) {
        rxflen: NumberField<u16, 10> = (0.into()),
        _res1: ReservedField<1> = ReservedField,
        rxnspl: NumberField<u8, 2> = (0.into()),
        rxbr: Bitrate = (Bitrate::Kbps850),
        _res2: ReservedField<1> = ReservedField,
        rng: bool = false,
        rxprf: Prf = (Prf::Prf16),
        rxpsr: NumberField<u8, 2> = (0.into()),
        rxpacc: NumberField<u16, 12> = (0.into())
    });

    reg_enum!(enum Pac(u8, 2) {
        Pac8 => 0,
        Pac16 => 1,
        Pac32 => 2,
        Pac4 => 3
    });

    reg_type!(struct DTune0(2) {
        pac: Pac = (Pac::Pac8),
        dt0b4: bool = false
    });

    reg_type!(struct DgcCfg(2) {
        rx_tune_en: bool = true,
        _res1: ReservedField<8> = ReservedField,
        thr_64: NumberField<u8, 6> = (0x32.into())
    });

    reg_type!(struct TxPower(4) {
        data_pwr: NumberField<u8, 8> = (0.into()),
        phr_pwr: NumberField<u8, 8> = (0.into()),
        shr_pwr: NumberField<u8, 8> = (0.into()),
        sts_pwr: NumberField<u8, 8> = (0.into())
    });

    reg_type!(struct Timestamp(5) {
        time: NumberField<u64, 40> = (0.into())
    });

    register!(DEV_ID: u32 = 0x00 0x00);
    register!(EUI64: u64 = 0x00 0x04);
    register!(SYS_CFG: SysCfg = 0x00 0x10);
    register!(TX_FCTRL: TxFctrl = 0x00 0x24);
    register!(DX_TIME: u32 = 0x00 0x2C);
    register!(SYS_STATUS: SysStatus = 0x00 0x44);
    register!(RX_FINFO: RxFinfo = 0x00 0x4C);
    register!(RX_TIME: Timestamp = 0x00 0x64);
    register!(TX_TIME: Timestamp = 0x00 0x74);

    register!(TW_POWER: TxPower = 0x01 0x0C);
    register!(CHAN_CTRL: ChanCtrl = 0x01 0x14);

    register!(DGC_CFG: DgcCfg = 0x03 0x18);
    register!(DGC_CFG0: u32 = 0x03 0x1C);
    register!(DGC_CFG1: u32 = 0x03 0x20);
    register!(DGC_LUT_0: u32 = 0x03 0x38);
    register!(DGC_LUT_1: u32 = 0x03 0x3C);
    register!(DGC_LUT_2: u32 = 0x03 0x40);
    register!(DGC_LUT_3: u32 = 0x03 0x44);
    register!(DGC_LUT_4: u32 = 0x03 0x48);
    register!(DGC_LUT_5: u32 = 0x03 0x4C);
    register!(DGC_LUT_6: u32 = 0x03 0x50);

    register!(RX_CAL: RxCal = 0x04 0x0C);
    register!(RX_CAL_RESI: u32 = 0x04 0x14);
    register!(RX_CAL_RESQ: u32 = 0x04 0x1C);
    register!(RX_CAL_STS: u8 = 0x04 0x20);

    register!(DTUNE0: DTune0 = 0x06 0x00);
    register!(RX_SFD_TOC: u16 = 0x06 0x02);
    register!(DTUNE3: u32 = 0x06 0x0C);

    register!(RF_TX_CTRL_1: u8 = 0x07 0x1A);
    register!(RF_TX_CTRL_2: u32 = 0x07 0x1C);
    register!(LDO_CTRL: u32 = 0x07 0x48);
    register!(LDO_RLOAD: u8 = 0x07 0x51);

    register!(PLL_CFG: u16 = 0x09 0x00);
    register!(PLL_CAL: u16 = 0x09 0x08);

    register!(BIAS_CTRL: u16 = 0x11 0x1F);
}

pub mod dw1000 {
    use crate::registers::*;
    /// Marker for the dw1000 register set.
    pub struct Dw1000RegSet;
    type Basis = Dw1000RegSet;

    reg_type!(struct PanAdr(4) {
        short_addr: NumberField<u16, 16> = (0.into()),
        pan_id: NumberField<u16, 16> = (0.into())
    });

    reg_enum!(enum PhrMode(u8, 2) {
        Standard => 0b00,
        LongFrames => 0b11
    });

    reg_type!(struct SysCfg(4) {
        ffen: bool = false,
        ffbc: bool = true,
        ffab: bool = true,
        ffad: bool = true,
        ffaa: bool = true,
        ffam: bool = true,
        ffar: bool = true,
        ffa4: bool = true,
        ffa5: bool = true,
        hirq_pol: bool = true,
        spi_edge: bool = true,
        dis_fce: bool = false,
        dis_drxb: bool = true,
        dis_phe: bool = false,
        dis_rsde: bool = false,
        fcs_init2f: bool = false,
        phr_mode: PhrMode = (PhrMode::Standard),
        dis_stxp: bool = true,
        _res1: ReservedField<3> = ReservedField,
        rxm110k: bool = false,
        _res2: ReservedField<5> = ReservedField,
        rxwtoe: bool = false,
        rxautr: bool = false,
        autoack: bool = false,
        aackpend: bool = false
    });

    reg_type!(struct Timestamp(5) {
        time: NumberField<u64, 40> = (0.into())
    });

    reg_enum!(enum Bitrate(u8, 2) {
        Kbps110 => 0b00,
        Kbps850 => 0b01,
        Mbps6_8 => 0b10
    });

    reg_enum!(enum Prf(u8, 2) {
        Mhz4 => 0b00,
        Mhz16 => 0b01,
        Mhz64 => 0b10
    });

    reg_enum!(enum Preamble(u8, 4) {
        Preamble64 => 0b0001,
        Preamble128 => 0b0101,
        Preamble256 => 0b1001,
        Preamble512 => 0b1101,
        Preamble1024 => 0b0010,
        Preamble1536 => 0b0110,
        Preamble2048 => 0b1010,
        Preamble4096 => 0b0011
    });

    reg_type!(struct TxFctrl(5) {
        tflen: NumberField<u16, 10> = (0.into()),
        _res1: ReservedField<3> = ReservedField,
        txbr: Bitrate = (Bitrate::Kbps850),
        tr: bool = false,
        txprf: Prf = (Prf::Mhz64),
        preamble: Preamble = (Preamble::Preamble128),
        txboffs: NumberField<u16, 10> = (0.into()),
        ifsdelay: NumberField<u8, 8> = (0.into())
    });

    reg_type!(struct SysCtrl(4) {
        sfcst: bool = false,
        txstrt: bool = false,
        txdlys: bool = false,
        cansfcs: bool = false,
        _res1: ReservedField<2> = ReservedField,
        trxoff: bool = false,
        wait4resp: bool = false,
        rxenab: bool = false,
        rxdlye: bool = false,
        _res2: ReservedField<14> = ReservedField,
        hrbpt: bool = false
    });

    reg_type!(struct SysStatus(5) {
        irqs: bool = false,
        cplock: bool = false,
        esyncr: bool = false,
        aat: bool = false,
        txfrb: bool = false,
        txprs: bool = false,
        txphs: bool = false,
        txfrs: bool = false,
        rxprd: bool = false,
        rxsfdd: bool = false,
        ldedone: bool = false,
        rxphd: bool = false,
        rxphe: bool = false,
        rxdfr: bool = false,
        rxfcg: bool = false,
        rxfce: bool = false,
        rxrfsl: bool = false,
        rxrfto: bool = false,
        ldeerr: bool = false,
        _res1: ReservedField<1> = ReservedField,
        rxovrr: bool = false,
        rxpto: bool = false,
        gpioirq: bool = false,
        slp2init: bool = false,
        rfpll_ll: bool = false,
        clkpll_ll: bool = false,
        rxsfd_to: bool = false,
        hpdwarn: bool = false,
        txberr: bool = false,
        affrej: bool = false,
        hsrbp: bool = false,
        icrbp: bool = false,
        rxrscs: bool = false,
        rxprej: bool = false,
        txpute: bool = false
    });

    reg_type!(struct RxFinfo(4) {
        rxflen: NumberField<u16, 10> = (0.into()),
        _res1: ReservedField<1> = ReservedField,
        rxnxspl: NumberField<u8, 2> = (0.into()),
        rxbr: Bitrate = (Bitrate::Kbps850),
        rng: bool = false,
        rxprfr: Prf = (Prf::Mhz64),
        rxpsr: NumberField<u8, 2> = (0.into()),
        rxpacc: NumberField<u16, 12> = (0.into())
    });

    reg_enum!(enum Channel(u8, 4) {
        Chan1 => 1,
        Chan2 => 2,
        Chan3 => 3,
        Chan4 => 4,
        Chan5 => 5,
        Chan7 => 7
    });

    reg_type!(struct ChanCtrl(4) {
        tx_chan: Channel = (Channel::Chan5),
        rx_chan: Channel = (Channel::Chan5),
        _res1: ReservedField<9> = ReservedField,
        dwsfd: bool = false,
        rxprf: Prf = (Prf::Mhz64),
        tnssfd: bool = false,
        rnssfd: bool = false,
        tx_pcode: NumberField<u8, 5> = (0.into()),
        rx_pcode: NumberField<u8, 5> = (0.into())
    });

    reg_type!(struct RfConf(4) {
        _res1: ReservedField<8> = ReservedField,
        txfen: NumberField<u8, 5> = (0.into()),
        pllfen: NumberField<u8, 3> = (0.into()),
        ldofen: NumberField<u8, 5> = (0.into()),
        txrxsw: NumberField<u8, 2> = (0.into())
    });

    reg_type!(struct EvcCtrl(4) {
        evc_en: bool = false,
        evc_clr: bool = false
    });

    reg_enum!(enum ClockSelect(u8, 2) {
        Auto => 0b00,
        ForcedXti => 0b01,
        ForcedPll => 0b10
    });

    reg_type!(struct PmscCtrl0(4) {
        sysclks: ClockSelect = (ClockSelect::Auto),
        rxclks: ClockSelect = (ClockSelect::Auto),
        txclks: ClockSelect = (ClockSelect::Auto),
        face: bool = false,
        _res1: ReservedField<3> = ReservedField,
        adcce: bool = false,
        _res2: ReservedField<4> = ReservedField,
        amce: bool = false,
        gpce: bool = false,
        gprn: bool = false,
        gpdce: bool = false,
        gpdrn: bool = false,
        _res3: ReservedField<3> = ReservedField,
        khzclken: bool = false,
        _res4: ReservedField<4> = ReservedField,
        softreset: NumberField<u8, 4> = (0.into())
    });

    reg_type!(struct OtpCtrl(2) {
        otprden: bool = false,
        otpread: bool = false,
        _res1: ReservedField<1> = ReservedField,
        otpmrwr: bool = false,
        _res2: ReservedField<2> = ReservedField,
        otpprog: bool = false,
        otpmr: NumberField<u8, 4> = (0.into()),
        _res3: ReservedField<4> = ReservedField,
        ldeload: bool = false
    });

    reg_enum!(enum TxState(u8, 4) {
        Idle => 0x0,
        Preamble => 0x1,
        Sfd => 0x2,
        Phr => 0x3,
        Sde => 0x4,
        Data => 0x5,
        RspData => 0x6
    });

    reg_enum!(enum RxState(u8, 8) {
        Idle => 0x00,
        StartAnalog => 0x01,
        RxReady => 0x04,
        PreambleFind => 0x05,
        PreambleTimeout => 0x06,
        SfdFound => 0x07,
        ConfigurePhrRx => 0x08,
        PhrRxStart => 0x09,
        DataRateReady => 0x0A,
        DataRxSeq => 0x0C,
        ConfigData => 0x0D,
        PhrNotOk => 0x0E,
        LastSymbol => 0x0F,
        WaitRsdDone => 0x10,
        RsdNotOk => 0x12,
        Reconfig110 => 0x13,
        Wait110Phr => 0x14
    });

    reg_enum!(enum PmscState(u8, 8) {
        Init => 0x00,
        Idle => 0x01,
        TxWait => 0x02,
        RxWait => 0x03,
        Tx => 0x4,
        Rx => 0x5
    });

    reg_type!(struct SysState(3) {
        tx_state: TxState = (TxState::Idle),
        _res1: ReservedField<4> = ReservedField,
        rx_state: RxState = (RxState::Idle),
        pmsc_state: PmscState = (PmscState::Idle)
    });

    reg_type!(struct AonCtrl(1) {
        restore: bool = false,
        save: bool = false,
        upl_cfg: bool = false,
        dca_read: bool = false,
        _res1: ReservedField<3> = ReservedField,
        dca_enab: bool = false
    });

    reg_type!(struct AonCfg0(4) {
        sleep_en: bool = false,
        wake_pin: bool = false,
        wake_spi: bool = false,
        wake_cnt: bool = false,
        lpdiv_en: bool = false,
        lpclkdiva: NumberField<u16, 11> = (0.into()),
        sleep_tim: NumberField<u16, 16> = (0.into())
    });

    reg_type!(struct RxFqual(8) {
        std_noise: NumberField<u16, 16> = (0.into()),
        ff_ampl2: NumberField<u16, 16> = (0.into()),
        ff_ampl3: NumberField<u16, 16> = (0.into()),
        cir_pwr: NumberField<u16, 16> = (0.into())
    });

    reg_type!(struct RxTtcko(5) {
        rxtofs: NumberField<u32, 19> = (0.into()),
        _res1: ReservedField<5> = ReservedField,
        rsmpdel: NumberField<u8, 8> = (0.into()),
        rcphase: NumberField<u8, 7> = (0.into())
    });

    reg_type!(struct AckRespT(4) {
        w4r_tim: NumberField<u32, 20> = (0.into()),
        _res1: ReservedField<4> = ReservedField,
        ack_tim: NumberField<u8, 8> = (0.into())
    });

    reg_type!(struct RxSniff(4) {
        sniff_ont: NumberField<u8, 4> = (0.into()),
        _res1: ReservedField<4> = ReservedField,
        sniff_offt: NumberField<u8, 8> = (0.into())
    });

    reg_type!(struct AgcCtrl1(2) {
        dis_am: bool = false
    });

    reg_type!(struct AgcStat1(3) {
        _res1: ReservedField<6> = ReservedField,
        edg1: NumberField<u8, 5> = (0.into()),
        edv2: NumberField<u16, 9> = (0.into())
    });

    reg_type!(struct EcCtrl(4) {
        ostsm: bool = false,
        osrsm: bool = false,
        _res1: ReservedField<1> = ReservedField,
        wait: NumberField<u8, 7> = (0.into()),
        ostrm: bool = false
    });

    reg_type!(struct EcGolp(4) {
        offset_ext: NumberField<u8, 6> = (0.into())
    });

    reg_enum!(enum Msgp(u8, 2) {
        Gpio => 0b00,
        Led => 0b01
    });

    reg_type!(struct GpioMode(4) {
        _res1: ReservedField<6> = ReservedField,
        msgp0: Msgp = (Msgp::Gpio),
        msgp1: Msgp = (Msgp::Gpio),
        msgp2: Msgp = (Msgp::Gpio),
        msgp3: Msgp = (Msgp::Gpio),
        msgp4: Msgp = (Msgp::Gpio),
        msgp5: Msgp = (Msgp::Gpio),
        msgp6: Msgp = (Msgp::Gpio),
        msgp7: Msgp = (Msgp::Gpio),
        msgp8: Msgp = (Msgp::Gpio)
    });

    reg_type!(struct GpioDir(4) {
        gdp0: bool = false,
        gdp1: bool = false,
        gdp2: bool = false,
        gdp3: bool = false,
        gdm0: bool = false,
        gdm1: bool = false,
        gdm2: bool = false,
        gdm3: bool = false,
        gdp4: bool = false,
        gdp5: bool = false,
        gdp6: bool = false,
        gdp7: bool = false,
        gdm4: bool = false,
        gdm5: bool = false,
        gdm6: bool = false,
        gdm7: bool = false,
        gdp8: bool = false,
        _res1: ReservedField<3> = ReservedField,
        gdm8: bool = false
    });

    reg_type!(struct GpioDout(4) {
        gop0: bool = false,
        gop1: bool = false,
        gop2: bool = false,
        gop3: bool = false,
        gom0: bool = false,
        gom1: bool = false,
        gom2: bool = false,
        gom3: bool = false,
        gop4: bool = false,
        gop5: bool = false,
        gop6: bool = false,
        gop7: bool = false,
        gom4: bool = false,
        gom5: bool = false,
        gom6: bool = false,
        gom7: bool = false,
        gop8: bool = false,
        _res1: ReservedField<3> = ReservedField,
        gom8: bool = false
    });

    reg_type!(struct GpioIrqe(4) {
        girqe0: bool = false,
        girqe1: bool = false,
        girqe2: bool = false,
        girqe3: bool = false,
        girqe4: bool = false,
        girqe5: bool = false,
        girqe6: bool = false,
        girqe7: bool = false,
        girqe8: bool = false
    });

    reg_type!(struct GpioIsen(4) {
        gisen0: bool = false,
        gisen1: bool = false,
        gisen2: bool = false,
        gisen3: bool = false,
        gisen4: bool = false,
        gisen5: bool = false,
        gisen6: bool = false,
        gisen7: bool = false,
        gisen8: bool = false
    });

    reg_type!(struct GpioImode(4) {
        gimode0: bool = false,
        gimode1: bool = false,
        gimode2: bool = false,
        gimode3: bool = false,
        gimode4: bool = false,
        gimode5: bool = false,
        gimode6: bool = false,
        gimode7: bool = false,
        gimode8: bool = false
    });

    reg_type!(struct GpioIbes(4) {
        gibes0: bool = false,
        gibes1: bool = false,
        gibes2: bool = false,
        gibes3: bool = false,
        gibes4: bool = false,
        gibes5: bool = false,
        gibes6: bool = false,
        gibes7: bool = false,
        gibes8: bool = false
    });

    reg_type!(struct GpioIclr(4) {
        giclr0: bool = false,
        giclr1: bool = false,
        giclr2: bool = false,
        giclr3: bool = false,
        giclr4: bool = false,
        giclr5: bool = false,
        giclr6: bool = false,
        giclr7: bool = false,
        giclr8: bool = false
    });

    reg_type!(struct GpioIdbe(4) {
        gidbe0: bool = false,
        gidbe1: bool = false,
        gidbe2: bool = false,
        gidbe3: bool = false,
        gidbe4: bool = false,
        gidbe5: bool = false,
        gidbe6: bool = false,
        gidbe7: bool = false,
        gidbe8: bool = false
    });

    reg_type!(struct GpioRaw(4) {
        grawp0: bool = false,
        grawp1: bool = false,
        grawp2: bool = false,
        grawp3: bool = false,
        grawp4: bool = false,
        grawp5: bool = false,
        grawp6: bool = false,
        grawp7: bool = false,
        grawp8: bool = false
    });

    register!(DEV_ID: u32 = 0x00 0x00);
    register!(EUI: u64 = 0x01 0x00);
    register!(PANADR: PanAdr = 0x03 0x00);
    register!(SYS_CFG: SysCfg = 0x04 0x00);
    register!(SYS_TIME: Timestamp = 0x06 0x00);
    register!(TX_FCTRL: TxFctrl = 0x08 0x00);
    // TX_BUFFER (0x09 0x00) omitted because it's a special case
    register!(DX_TIME: Timestamp = 0x0A 0x00);
    register!(RX_FWTO: u16 = 0x0C 0x00);
    register!(SYS_CTRL: SysCtrl = 0x0D 0x00);
    register!(SYS_MASK: SysStatus = 0x0E 0x00);
    register!(SYS_STATUS: SysStatus = 0x0F 0x00);
    register!(RX_FINFO: RxFinfo = 0x10 0x00);
    // RX_BUFFER (0x11 0x00) omitted because it's a special case
    register!(RX_FQUAL: RxFqual = 0x12 0x00);
    register!(RX_TTCKI: u32 = 0x13 0x00);
    register!(RX_TTCKO: RxTtcko = 0x14 0x00);
    register!(RX_TIME: Timestamp = 0x15 0x00);
    register!(TX_TIME: Timestamp = 0x17 0x00);
    register!(TX_ANTD: u16 = 0x18 0x00);
    register!(SYS_STATE: SysState = 0x19 0x00);
    register!(ACK_RESP_T: AckRespT = 0x1A 0x00);
    register!(RX_SNIFF: RxSniff = 0x1D 0x00);
    register!(TX_POWER: u32 = 0x1E 0x00);
    register!(CHAN_CTRL: ChanCtrl = 0x1F 0x00);
    // USR_SFD (0x21 0x00) omitted because it's a special case
    register!(AGC_CTRL1: AgcCtrl1 = 0x23 0x02);
    register!(AGC_TUNE1: u16 = 0x23 0x04);
    register!(AGC_TUNE2: u32 = 0x23 0x0C);
    register!(AGC_TUNE3: u16 = 0x23 0x12);
    register!(AGC_STAT1: AgcStat1 = 0x23 0x1E);
    register!(EC_CTRL: EcCtrl = 0x24 0x00);
    register!(EC_RXTC: u32 = 0x24 0x04);
    register!(EC_GOLP: EcGolp = 0x24 0x08);
    // ACC_MEM (0x25 0x00) omitted because it's a special case
    register!(GPIO_MODE: GpioMode = 0x26 0x00);
    register!(GPIO_DIR: GpioDir = 0x26 0x08);
    register!(GPIO_DOUT: GpioDout = 0x26 0x0C);
    register!(GPIO_IRQE: GpioIrqe = 0x26 0x10);
    register!(GPIO_ISEN: GpioIsen = 0x26 0x14);
    register!(GPIO_IMODE: GpioImode = 0x26 0x18);
    register!(GPIO_IBES: GpioIbes = 0x26 0x1C);
    register!(GPIO_ICLR: GpioIclr = 0x26 0x20);
    register!(GPIO_IDBE: GpioIdbe = 0x26 0x24);
    register!(GPIO_RAW: GpioRaw = 0x26 0x28);
    register!(DRX_TUNE0B: u16 = 0x27 0x02);
    register!(DRX_TUNE1A: u16 = 0x27 0x04);
    register!(DRX_TUNE1B: u16 = 0x27 0x06);
    register!(DRX_TUNE2: u32 = 0x27 0x08);
    register!(DRX_SFDTOC: u16 = 0x27 0x20);
    register!(DRX_PRETOC: u16 = 0x27 0x24);
    register!(DRX_TUNE4H: u16 = 0x27 0x26);
    register!(RF_CONF: RfConf = 0x28 0x00);
    register!(RF_RXCTRLH: u8 = 0x28 0x0B);
    register!(RF_TXCTRL: u32 = 0x28 0x0C);
    register!(RF_STATUS: u32 = 0x28 0x2C);
    register!(TC_SARC: u16 = 0x2A 0x00);
    register!(TC_SARL: u16 = 0x2A 0x03);
    register!(TC_SARW: u16 = 0x2A 0x06);
    register!(TC_PGDELAY: u8 = 0x2A 0x0B);
    register!(TC_PGTEST: u8 = 0x2A 0x0C);
    register!(FS_PLLCFG: u32 = 0x2B 0x07);
    register!(FS_PLLTUNE: u8 = 0x2B 0x0B);
    register!(FS_XTALT: u8 = 0x2B 0x0E);
    register!(AON_CTRL: AonCtrl = 0x2C 0x02);
    register!(AON_CFG0: AonCfg0 = 0x2C 0x06);
    register!(OTP_CTRL: OtpCtrl = 0x2D 0x06);
    register!(LDE_CFG1: u16 = 0x2E 0x0806);
    register!(LDE_CFG2: u16 = 0x2E 0x1806);
    register!(LDE_REPC: u16 = 0x2E 0x2804);
    register!(EVC_CTRL: EvcCtrl = 0x2F 0x00);
    register!(EVC_FCG: u16 = 0x2F 0x08);

    register!(PMSC_CTRL0: u16 = 0x36 0x00);
}
