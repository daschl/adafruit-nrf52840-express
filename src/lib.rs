#![no_std]

use nrf52840_hal::{
    gpio::{p0, p1, Floating, Input, Level, Output, PushPull},
    pac::CorePeripherals,
    pac::Peripherals,
    prelude::OutputPin,
};
use nrf52840_hal::{
    gpio::{Pin, PullUp},
    pac,
    prelude::InputPin,
    spim::Frequency,
    spim::{self, MODE_0},
    Spim,
};

pub mod prelude {
    pub use nrf52840_hal::prelude::*;
}

#[allow(non_snake_case)]
pub struct Board {
    /// Directly usable pins on the Board
    pub pins: Pins,
    /// LEDs on the Board
    pub leds: Leds,
    /// Buttons on the Board
    pub buttons: Buttons,
    /// NFC Ports
    pub nfc: NFC,
    /// The Flash SPI for the 2MB Flash
    pub flash: Spim<pac::SPIM2>,
    /// The Chip Select pin for the Flash SPI
    pub flash_cs: Pin<Output<PushPull>>,
    /// Cache and branch predictor maintenance operations
    pub CBP: pac::CBP,
    /// CPUID
    pub CPUID: pac::CPUID,
    /// Debug Control Block
    pub DCB: pac::DCB,
    /// Data Watchpoint and Trace unit
    pub DWT: pac::DWT,
    /// Flash Patch and Breakpoint unit
    pub FPB: pac::FPB,
    /// Floating Point Unit
    pub FPU: pac::FPU,
    /// Instrumentation Trace Macrocell
    pub ITM: pac::ITM,
    /// Memory Protection Unit
    pub MPU: pac::MPU,
    /// Nested Vector Interrupt Controller
    pub NVIC: pac::NVIC,
    /// System Control Block
    pub SCB: pac::SCB,
    /// SysTick: System Timer
    pub SYST: pac::SYST,
    /// Trace Port Interface Unit
    pub TPIU: pac::TPIU,

    pub FICR: pac::FICR,

    pub UICR: pac::UICR,

    pub CLOCK: pac::CLOCK,

    pub POWER: pac::POWER,

    pub RADIO: pac::RADIO,

    pub UART0: pac::UART0,

    pub UARTE0: pac::UARTE0,

    pub SPI0: pac::SPI0,

    pub SPIM0: pac::SPIM0,

    pub SPIS0: pac::SPIS0,

    pub TWI0: pac::TWI0,

    pub TWIM0: pac::TWIM0,

    pub TWIS0: pac::TWIS0,

    pub SPI1: pac::SPI1,

    pub SPIM1: pac::SPIM1,

    pub SPIS1: pac::SPIS1,

    pub TWI1: pac::TWI1,

    pub TWIM1: pac::TWIM1,

    pub TWIS1: pac::TWIS1,

    pub NFCT: pac::NFCT,

    pub GPIOTE: pac::GPIOTE,

    pub SAADC: pac::SAADC,

    pub TIMER0: pac::TIMER0,

    pub TIMER1: pac::TIMER1,

    pub TIMER2: pac::TIMER2,

    pub RTC0: pac::RTC0,

    pub TEMP: pac::TEMP,

    pub RNG: pac::RNG,

    pub ECB: pac::ECB,

    pub AAR: pac::AAR,

    pub CCM: pac::CCM,

    pub WDT: pac::WDT,

    pub RTC1: pac::RTC1,

    pub QDEC: pac::QDEC,

    pub COMP: pac::COMP,

    pub LPCOMP: pac::LPCOMP,

    pub EGU0: pac::EGU0,

    pub SWI0: pac::SWI0,

    pub EGU1: pac::EGU1,

    pub SWI1: pac::SWI1,

    pub EGU2: pac::EGU2,

    pub SWI2: pac::SWI2,

    pub EGU3: pac::EGU3,

    pub SWI3: pac::SWI3,

    pub EGU4: pac::EGU4,

    pub SWI4: pac::SWI4,

    pub EGU5: pac::EGU5,

    pub SWI5: pac::SWI5,

    pub TIMER3: pac::TIMER3,

    pub TIMER4: pac::TIMER4,

    pub PWM0: pac::PWM0,

    pub PDM: pac::PDM,

    pub ACL: pac::ACL,

    pub NVMC: pac::NVMC,

    pub PPI: pac::PPI,

    pub MWU: pac::MWU,

    pub PWM1: pac::PWM1,

    pub PWM2: pac::PWM2,

    pub SPI2: pac::SPI2,

    pub SPIS2: pac::SPIS2,

    pub RTC2: pac::RTC2,

    pub I2S: pac::I2S,

    pub USBD: pac::USBD,

    pub UARTE1: pac::UARTE1,

    pub QSPI: pac::QSPI,

    pub PWM3: pac::PWM3,

    pub SPIM3: pac::SPIM3,

    pub CC_HOST_RGF: pac::CC_HOST_RGF,

    pub CRYPTOCELL: pac::CRYPTOCELL,
}

impl Board {
    pub fn take() -> Option<Self> {
        Some(Self::new(CorePeripherals::take()?, Peripherals::take()?))
    }

    pub unsafe fn steal() -> Self {
        Self::new(CorePeripherals::steal(), Peripherals::steal())
    }

    fn new(cp: CorePeripherals, p: Peripherals) -> Self {
        let pins0 = p0::Parts::new(p.P0);
        let pins1 = p1::Parts::new(p.P1);

        let flash_spim = Spim::new(
            p.SPIM2,
            spim::Pins {
                sck: pins0.p0_19.into_push_pull_output(Level::Low).degrade(),
                mosi: Some(pins0.p0_17.into_push_pull_output(Level::Low).degrade()),
                miso: Some(pins0.p0_22.into_floating_input().degrade()),
            },
            Frequency::K500,
            MODE_0,
            0,
        );

        let flash_cs = pins0.p0_20.into_push_pull_output(Level::High).degrade();

        Self {
            pins: Pins {
                a0: pins0.p0_04,
                a1: pins0.p0_05,
                a2: pins0.p0_30,
                a3: pins0.p0_28,
                a4: pins0.p0_02,
                a5: pins0.p0_03,
                sck: pins0.p0_14,
                mosi: pins0.p0_13,
                miso: pins0.p0_15,
                scl: pins0.p0_11,
                sda: pins0.p0_12,
                tx: pins0.p0_25,
                rx: pins0.p0_24,
                d13: pins1.p1_09,
                d12: pins0.p0_08,
                d11: pins0.p0_06,
                d10: pins0.p0_27,
                d9: pins0.p0_26,
                d6: pins0.p0_07,
                d5: pins1.p1_08,
            },
            leds: Leds {
                led1: Led::new(pins1.p1_15.degrade()),
                led2: Led::new(pins1.p1_10.degrade()),
                neo_pixel: Led::new(pins0.p0_16.degrade()),
            },
            buttons: Buttons {
                button_1: Button::new(pins1.p1_02.degrade()),
            },
            nfc: NFC {
                nfc_1: pins0.p0_09,
                nfc_2: pins0.p0_10,
            },
            flash: flash_spim,
            flash_cs,

            // Core Peripherals
            CBP: cp.CBP,
            CPUID: cp.CPUID,
            DCB: cp.DCB,
            DWT: cp.DWT,
            FPB: cp.FPB,
            FPU: cp.FPU,
            ITM: cp.ITM,
            MPU: cp.MPU,
            NVIC: cp.NVIC,
            SCB: cp.SCB,
            SYST: cp.SYST,
            TPIU: cp.TPIU,

            // nRF52840 Peripherals
            FICR: p.FICR,
            UICR: p.UICR,
            CLOCK: p.CLOCK,
            POWER: p.POWER,
            RADIO: p.RADIO,
            UART0: p.UART0,
            UARTE0: p.UARTE0,
            SPI0: p.SPI0,
            SPIM0: p.SPIM0,
            SPIS0: p.SPIS0,
            TWI0: p.TWI0,
            TWIM0: p.TWIM0,
            TWIS0: p.TWIS0,
            SPI1: p.SPI1,
            SPIM1: p.SPIM1,
            SPIS1: p.SPIS1,
            TWI1: p.TWI1,
            TWIM1: p.TWIM1,
            TWIS1: p.TWIS1,
            NFCT: p.NFCT,
            GPIOTE: p.GPIOTE,
            SAADC: p.SAADC,
            TIMER0: p.TIMER0,
            TIMER1: p.TIMER1,
            TIMER2: p.TIMER2,
            RTC0: p.RTC0,
            TEMP: p.TEMP,
            RNG: p.RNG,
            ECB: p.ECB,
            AAR: p.AAR,
            CCM: p.CCM,
            WDT: p.WDT,
            RTC1: p.RTC1,
            QDEC: p.QDEC,
            COMP: p.COMP,
            LPCOMP: p.LPCOMP,
            EGU0: p.EGU0,
            SWI0: p.SWI0,
            EGU1: p.EGU1,
            SWI1: p.SWI1,
            EGU2: p.EGU2,
            SWI2: p.SWI2,
            EGU3: p.EGU3,
            SWI3: p.SWI3,
            EGU4: p.EGU4,
            SWI4: p.SWI4,
            EGU5: p.EGU5,
            SWI5: p.SWI5,
            TIMER3: p.TIMER3,
            TIMER4: p.TIMER4,
            PWM0: p.PWM0,
            PDM: p.PDM,
            ACL: p.ACL,
            NVMC: p.NVMC,
            PPI: p.PPI,
            MWU: p.MWU,
            PWM1: p.PWM1,
            PWM2: p.PWM2,
            SPI2: p.SPI2,
            SPIS2: p.SPIS2,
            RTC2: p.RTC2,
            I2S: p.I2S,
            USBD: p.USBD,
            UARTE1: p.UARTE1,
            QSPI: p.QSPI,
            PWM3: p.PWM3,
            SPIM3: p.SPIM3,
            CC_HOST_RGF: p.CC_HOST_RGF,
            CRYPTOCELL: p.CRYPTOCELL,
        }
    }
}

pub struct Pins {
    /// Analog 0
    pub a0: p0::P0_04<Input<Floating>>,
    /// Analog 1
    pub a1: p0::P0_05<Input<Floating>>,
    /// Analog 2
    pub a2: p0::P0_30<Input<Floating>>,
    /// Analog 3
    pub a3: p0::P0_28<Input<Floating>>,
    /// Analog 4
    pub a4: p0::P0_02<Input<Floating>>,
    /// Analog 5
    pub a5: p0::P0_03<Input<Floating>>,
    /// SPI SCK
    pub sck: p0::P0_14<Input<Floating>>,
    /// SPI MOSI
    pub mosi: p0::P0_13<Input<Floating>>,
    /// SPI MISO
    pub miso: p0::P0_15<Input<Floating>>,
    /// I2C SCL
    pub scl: p0::P0_11<Input<Floating>>,
    /// I2C SDA
    pub sda: p0::P0_12<Input<Floating>>,
    /// UART TX
    pub tx: p0::P0_25<Input<Floating>>,
    /// UART RX
    pub rx: p0::P0_24<Input<Floating>>,
    /// Digital In/Out 13
    pub d13: p1::P1_09<Input<Floating>>,
    /// Digital In/Out 12
    pub d12: p0::P0_08<Input<Floating>>,
    /// Digital In/Out 11
    pub d11: p0::P0_06<Input<Floating>>,
    /// Digital In/Out 10
    pub d10: p0::P0_27<Input<Floating>>,
    /// Digital In/Out 9
    pub d9: p0::P0_26<Input<Floating>>,
    /// Digital In/Out 6
    pub d6: p0::P0_07<Input<Floating>>,
    /// Digital In/Out 5
    pub d5: p1::P1_08<Input<Floating>>,
}

pub struct Leds {
    /// Red LED at P1.15
    pub led1: Led,
    /// Blue LED at P1.10
    pub led2: Led,
    /// RGB NeoPixel at P0.16
    pub neo_pixel: Led,
}

pub struct Led(Pin<Output<PushPull>>);

impl Led {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Led(pin.into_push_pull_output(Level::High))
    }

    pub fn enable(&mut self) {
        self.0.set_high().unwrap();
    }

    pub fn disable(&mut self) {
        self.0.set_low().unwrap();
    }
}

pub struct Buttons {
    pub button_1: Button,
}

pub struct Button(Pin<Input<PullUp>>);

impl Button {
    fn new<Mode>(pin: Pin<Mode>) -> Self {
        Button(pin.into_pullup_input())
    }

    /// Button is pressed
    pub fn is_pressed(&self) -> bool {
        self.0.is_low().unwrap()
    }

    /// Button is released
    pub fn is_released(&self) -> bool {
        self.0.is_high().unwrap()
    }
}

pub struct NFC {
    /// NFC 1, exposed only via test point on bottom of the board
    pub nfc_1: p0::P0_09<Input<Floating>>,
    /// NFC 2
    pub nfc_2: p0::P0_10<Input<Floating>>,
}
