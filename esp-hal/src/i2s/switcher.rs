//! I2S half-duplex switcher for sharing clock pins between RX and TX.

use crate::{
    dma::DmaDescriptor,
    gpio::{AnyPin, OutputPin, interconnect::{PeripheralInput, PeripheralOutput}},
    i2s::master::{I2s, I2sRx, I2sTx},
    Blocking,
};

// This trait is made `pub(crate)` in `master.rs` – see note below.
use crate::i2s::master::private::Signals;

/// A wrapper that holds an I2S instance and separate DMA descriptors for RX and TX.
pub struct I2sSwitcher<'d> {
    i2s: I2s<'d, Blocking>,
    rx_descriptors: &'static mut [DmaDescriptor],
    tx_descriptors: &'static mut [DmaDescriptor],
}

impl<'d> I2sSwitcher<'d> {
    /// Creates a new switcher.
    pub fn new(
        i2s: I2s<'d, Blocking>,
        rx_descriptors: &'static mut [DmaDescriptor],
        tx_descriptors: &'static mut [DmaDescriptor],
    ) -> Self {
        Self {
            i2s,
            rx_descriptors,
            tx_descriptors,
        }
    }

    /// Convert into an I2S receiver (RX) with the given data input pin.
    pub fn into_rx(mut self, din: impl PeripheralInput<'d>) -> I2sRx<'d, Blocking> {
        self.init_shared_clocks();
        self.i2s.i2s_rx.with_din(din).build(self.rx_descriptors)
    }

    /// Convert into an I2S transmitter (TX) with the given data output pin.
    pub fn into_tx(mut self, dout: impl OutputPin) -> I2sTx<'d, Blocking> {
        self.init_shared_clocks();
        self.i2s.i2s_tx.with_dout(dout).build(self.tx_descriptors)
    }

    /// Initialise shared clock pins (BCLK, WS, MCLK) for both RX and TX.
    fn init_shared_clocks(&mut self) {
        if let (Some(bclk), Some(ws)) = (self.i2s.bclk.take(), self.i2s.ws.take()) {
            // Connect to TX outputs (master mode)
            bclk.connect_peripheral_to_output(self.i2s.i2s_tx.i2s.bclk_signal());
            ws.connect_peripheral_to_output(self.i2s.i2s_tx.i2s.ws_signal());

            // Enable input buffers so RX can also receive the clocks
            bclk.set_input_enable(true);
            ws.set_input_enable(true);

            // Connect RX input signals to the same pins
            self.i2s.i2s_tx.i2s.bclk_rx_signal().connect_to(&bclk);
            self.i2s.i2s_tx.i2s.ws_rx_signal().connect_to(&ws);
        }

        if let Some(mclk) = self.i2s.mclk.take() {
            #[cfg(not(esp32))]
            {
                mclk.set_output_enable(true);
                self.i2s.i2s_tx.i2s.mclk_signal().connect_to(&mclk);
            }
            #[cfg(esp32)]
            {
                use crate::gpio::OutputSignal;
                use crate::peripherals::IO_MUX;

                let clk_signal = mclk.signal();
                mclk.set_output_enable(true);

                let selector = match self.i2s.i2s_tx.i2s.0 {
                    crate::i2s::any::Inner::I2s0(_) => 0x0,
                    crate::i2s::any::Inner::I2s1(_) => 0xF,
                };

                IO_MUX::regs().pin_ctrl().modify(|_, w| unsafe {
                    match clk_signal {
                        OutputSignal::CLK_OUT1 => w.clk1().bits(selector),
                        OutputSignal::CLK_OUT2 => w.clk2().bits(selector),
                        OutputSignal::CLK_OUT3 => w.clk3().bits(selector),
                        _ => unreachable!(),
                    }
                });

                clk_signal.connect_to(&mclk);
            }
        }
    }
}
